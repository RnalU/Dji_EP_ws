// ================= New Optimized Implementation =====================

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

extern "C" {
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
#include <libavutil/hwcontext.h>
}

#include <thread>
#include <atomic>
#include <string>
#include <chrono>

/*
 * Low-latency RTSP (H.264) node using FFmpeg (no GStreamer).
 * - Optional hardware acceleration (auto/vaapi/cuda/v4l2m2m/none)
 * - Minimal buffering, aggressive frame dropping option
 * - Single worker thread: decode -> color convert -> publish
 * - PTS mapped to ROS time for synchronization
 */

class RtspFFmpegNode {
public:
    RtspFFmpegNode(ros::NodeHandle &nh, ros::NodeHandle &pnh) : nh_(nh), pnh_(pnh) {
        // Parameters
        pnh_.param<std::string>("rtsp_url", rtsp_url_, std::string("rtsp://127.0.0.1:8554/live"));
        pnh_.param<std::string>("output_topic", output_topic_, std::string("/vision/ffmpeg/image_raw"));
        pnh_.param<bool>("force_tcp", force_tcp_, false);
        pnh_.param<int>("reconnect_delay_ms", reconnect_delay_ms_, 1200);
        pnh_.param<double>("publish_fps", publish_fps_, 30.0);
        pnh_.param<std::string>("hw_accel", hw_accel_type_, std::string("auto"));
        pnh_.param<int>("video_width", force_width_, 0);
        pnh_.param<int>("video_height", force_height_, 0);
        pnh_.param<bool>("low_delay_flags", low_delay_flags_, true);
        pnh_.param<int>("max_open_retry", max_open_retry_, 15);
        pnh_.param<bool>("aggressive_skip", aggressive_skip_, false); // drop non-ref frames & loop filter

        image_pub_ = nh_.advertise<sensor_msgs::Image>(output_topic_, 1);
        running_ = true;
        worker_ = std::thread(&RtspFFmpegNode::decodePublishLoop, this);
    }

    ~RtspFFmpegNode() { stop(); }

    void stop() {
        if (!running_) return;
        running_ = false;
        if (worker_.joinable()) worker_.join();
        closeStream();
    }

private:
    // ---------- FFmpeg helpers ----------
    void logVersions() {
        ROS_INFO_STREAM("libavformat version=" << avformat_version());
        ROS_INFO_STREAM("libavcodec version=" << avcodec_version());
    }

    AVPixelFormat findHwFormat(AVCodecContext *ctx, const enum AVPixelFormat *pix_fmts) {
        for (const enum AVPixelFormat *p = pix_fmts; *p != AV_PIX_FMT_NONE; ++p) {
            if (*p == hw_pix_fmt_) return *p;
        }
        return AV_PIX_FMT_NONE;
    }

    bool initHWDevice(AVCodecID codec_id) {
        if (hw_accel_type_ == "none") return false;
        if (hw_accel_type_ == "auto") {
            const char* try_list[] = {"vaapi", "cuda", "v4l2m2m"};
            for (auto name : try_list) if (createHWDevice(name)) return true;
            return false;
        }
        return createHWDevice(hw_accel_type_.c_str());
    }

    bool createHWDevice(const char *type) {
        AVHWDeviceType dev_type = av_hwdevice_find_type_by_name(type);
        if (dev_type == AV_HWDEVICE_TYPE_NONE) { ROS_WARN_STREAM("HW device type not found: " << type); return false; }
        int ret = av_hwdevice_ctx_create(&hw_device_ctx_, dev_type, nullptr, nullptr, 0);
        if (ret < 0) { ROS_WARN_STREAM("Failed to create HW device: " << type << " ret=" << ret); return false; }
        for (int i=0;;++i) {
            const AVCodecHWConfig *config = avcodec_get_hw_config(decoder_, i);
            if (!config) break;
            if ((config->methods & AV_CODEC_HW_CONFIG_METHOD_HW_DEVICE_CTX) && config->device_type == dev_type) {
                hw_pix_fmt_ = config->pix_fmt; ROS_INFO_STREAM("Using HW decoding: " << type); return true; }
        }
        ROS_WARN_STREAM("No matching HW config: " << type);
        av_buffer_unref(&hw_device_ctx_);
        return false;
    }

    void closeStream() {
        if (sws_ctx_) { sws_freeContext(sws_ctx_); sws_ctx_ = nullptr; }
        if (codec_ctx_) { avcodec_free_context(&codec_ctx_); codec_ctx_ = nullptr; }
        if (fmt_ctx_) { avformat_close_input(&fmt_ctx_); fmt_ctx_ = nullptr; }
        av_frame_free(&frame_);
        av_frame_free(&hw_frame_);
        if (hw_device_ctx_) av_buffer_unref(&hw_device_ctx_);
    }

    bool openStream() {
        logVersions();
        AVDictionary *opts = nullptr;
        if (force_tcp_) av_dict_set(&opts, "rtsp_transport", "tcp", 0);
        av_dict_set(&opts, "stimeout", "2000000", 0);          // 2s timeout microseconds
        av_dict_set(&opts, "buffer_size", "102400", 0);
        av_dict_set(&opts, "max_delay", "500000", 0);           // 0.5s
        av_dict_set(&opts, "reorder_queue_size", "0", 0);
        av_dict_set(&opts, "probesize", "8192", 0);
        av_dict_set(&opts, "analyzeduration", "0", 0);
        av_dict_set(&opts, "fflags", "nobuffer+discardcorrupt", 0);
        av_dict_set(&opts, "flags", "low_delay", 0);
        av_dict_set(&opts, "flush_packets", "1", 0);

        int ret = avformat_open_input(&fmt_ctx_, rtsp_url_.c_str(), nullptr, &opts);
        av_dict_free(&opts);
        if (ret < 0) { ROS_WARN_STREAM("avformat_open_input failed ret=" << ret); return false; }
        ret = avformat_find_stream_info(fmt_ctx_, nullptr);
        if (ret < 0) { ROS_WARN("find_stream_info failed"); return false; }

        video_stream_index_ = -1;
        for (unsigned i=0; i<fmt_ctx_->nb_streams; ++i) {
            if (fmt_ctx_->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) { video_stream_index_ = (int)i; break; }
        }
        if (video_stream_index_ < 0) { ROS_ERROR("Video stream not found"); return false; }

        AVCodecParameters *codecpar = fmt_ctx_->streams[video_stream_index_]->codecpar;
        decoder_ = avcodec_find_decoder(codecpar->codec_id);
        if (!decoder_) { ROS_ERROR("Decoder not found"); return false; }
        codec_ctx_ = avcodec_alloc_context3(decoder_);
        if (!codec_ctx_) { ROS_ERROR("Alloc codec_ctx failed"); return false; }
        if (avcodec_parameters_to_context(codec_ctx_, codecpar) < 0) { ROS_ERROR("Copy codec params failed"); return false; }
        if (low_delay_flags_) { codec_ctx_->flags |= AV_CODEC_FLAG_LOW_DELAY; codec_ctx_->flags2 |= AV_CODEC_FLAG2_FAST; }
        codec_ctx_->thread_count = 1; // latency
        if (aggressive_skip_) { codec_ctx_->skip_frame = AVDISCARD_NONREF; codec_ctx_->skip_loop_filter = AVDISCARD_ALL; }
        codec_ctx_->err_recognition |= AV_EF_CRCCHECK | AV_EF_BITSTREAM | AV_EF_BUFFER;

        bool hw = initHWDevice(codecpar->codec_id);
        if (hw && hw_device_ctx_) {
            codec_ctx_->hw_device_ctx = av_buffer_ref(hw_device_ctx_);
            codec_ctx_->get_format = [](AVCodecContext *ctx, const AVPixelFormat *pix_fmts) {
                return reinterpret_cast<RtspFFmpegNode*>(ctx->opaque)->findHwFormat(ctx, pix_fmts);
            };
            codec_ctx_->opaque = this;
        }
        if (avcodec_open2(codec_ctx_, decoder_, nullptr) < 0) { ROS_ERROR("Failed to open decoder"); return false; }

        frame_ = av_frame_alloc(); hw_frame_ = av_frame_alloc();
        if (!frame_ || !hw_frame_) { ROS_ERROR("Alloc frame failed"); return false; }

        if (force_width_ > 0 && force_height_ > 0) { target_width_ = force_width_; target_height_ = force_height_; }
        else { target_width_ = codec_ctx_->width; target_height_ = codec_ctx_->height; }

        ROS_INFO_STREAM("Opened stream src=" << codec_ctx_->width << "x" << codec_ctx_->height
                        << " target=" << target_width_ << "x" << target_height_
                        << (hw?" (HW)":" (SW)") << (aggressive_skip_?" skip=aggressive":""));
        return true;
    }

    // ---------- Main loop ----------
    void decodePublishLoop() {
        int open_retry = 0;
        ros::Time base_wall_time; bool base_wall_initialized = false; int64_t first_pts = AV_NOPTS_VALUE;
        size_t stat_frames = 0; double stat_latency_sum = 0.0; ros::Time stat_start = ros::Time::now();
        ros::Time last_pub(0); double min_interval = publish_fps_ > 0 ? 1.0 / publish_fps_ : 0.0;

        while (running_ && ros::ok()) {
            if (!fmt_ctx_) {
                if (open_retry >= max_open_retry_) { ROS_ERROR("Max open retries reached; exiting loop"); break; }
                if (!openStream()) { ++open_retry; std::this_thread::sleep_for(std::chrono::milliseconds(reconnect_delay_ms_)); continue; }
                open_retry = 0; base_wall_initialized = false; first_pts = AV_NOPTS_VALUE;
            }

            AVPacket pkt; av_init_packet(&pkt);
            int ret = av_read_frame(fmt_ctx_, &pkt);
            if (ret < 0) {
                ROS_WARN_THROTTLE(2.0, "Read frame failed or stream end; reconnecting");
                av_packet_unref(&pkt); closeStream(); std::this_thread::sleep_for(std::chrono::milliseconds(reconnect_delay_ms_)); continue; }
            if (pkt.stream_index != video_stream_index_) { av_packet_unref(&pkt); continue; }
            if (avcodec_send_packet(codec_ctx_, &pkt) < 0) { ROS_WARN_THROTTLE(2.0, "send_packet failed"); av_packet_unref(&pkt); continue; }
            av_packet_unref(&pkt);

            while (running_) {
                ret = avcodec_receive_frame(codec_ctx_, frame_);
                if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) break;
                if (ret < 0) { ROS_WARN_THROTTLE(2.0, "receive_frame failed"); break; }

                AVFrame *use_frame = frame_;
                if (frame_->format == hw_pix_fmt_) {
                    if (av_hwframe_transfer_data(hw_frame_, frame_, 0) == 0) use_frame = hw_frame_;
                    else ROS_WARN_THROTTLE(5.0, "hwframe transfer failed, fallback");
                }
                if (!sws_ctx_) {
                    sws_ctx_ = sws_getContext(use_frame->width, use_frame->height, (AVPixelFormat)use_frame->format,
                                              target_width_, target_height_, AV_PIX_FMT_BGR24, SWS_FAST_BILINEAR, nullptr, nullptr, nullptr);
                    if (!sws_ctx_) { ROS_ERROR("sws_getContext failed"); continue; }
                }
                if (bgr_buffer_.empty() || bgr_buffer_.cols != target_width_ || bgr_buffer_.rows != target_height_) {
                    bgr_buffer_.create(target_height_, target_width_, CV_8UC3);
                }
                uint8_t *dst_data[4] = { bgr_buffer_.data, nullptr, nullptr, nullptr }; int dst_linesize[4] = {(int)bgr_buffer_.step[0],0,0,0};
                sws_scale(sws_ctx_, use_frame->data, use_frame->linesize, 0, use_frame->height, dst_data, dst_linesize);
                ros::Time decode_end = ros::Time::now();

                AVStream *vst = fmt_ctx_->streams[video_stream_index_];
                int64_t best_pts = (frame_->best_effort_timestamp != AV_NOPTS_VALUE)? frame_->best_effort_timestamp : frame_->pts;
                ros::Time pts_time = decode_end; // fallback
                if (best_pts != AV_NOPTS_VALUE) {
                    if (!base_wall_initialized) { double first_pts_sec = best_pts * av_q2d(vst->time_base); base_wall_time = decode_end - ros::Duration(first_pts_sec); base_wall_initialized = true; first_pts = best_pts; }
                    double pts_sec = best_pts * av_q2d(vst->time_base); pts_time = base_wall_time + ros::Duration(pts_sec);
                }

                if (min_interval > 0.0 && !last_pub.isZero()) {
                    double since = (decode_end - last_pub).toSec();
                    if (since < (min_interval * 0.9)) continue; // drop frame
                }

                try {
                    cv_bridge::CvImage cv_img; cv_img.header.stamp = pts_time; cv_img.header.frame_id = "rtsp_ffmpeg"; cv_img.encoding = "bgr8"; cv_img.image = bgr_buffer_;
                    sensor_msgs::Image msg; cv_img.toImageMsg(msg); image_pub_.publish(msg); last_pub = decode_end;
                    double latency_pts = (decode_end - pts_time).toSec(); stat_latency_sum += latency_pts; stat_frames++;
                    ros::Time now = decode_end; if ((now - stat_start).toSec() >= 5.0) { double fps = stat_frames / (now - stat_start).toSec(); double avg_ms = stat_frames? (stat_latency_sum/stat_frames)*1000.0:0; ROS_INFO_STREAM("Stats: FPS=" << fps << " avg_pts_latency(ms)=" << avg_ms); stat_frames = 0; stat_latency_sum = 0.0; stat_start = now; }
                } catch (const std::exception &e) {
                    ROS_WARN_STREAM("Publish exception: " << e.what());
                }
            }
        }
    }

private:
    // ROS
    ros::NodeHandle nh_; ros::NodeHandle pnh_;
    ros::Publisher image_pub_;

    // Params
    std::string rtsp_url_, output_topic_; bool force_tcp_{false}; int reconnect_delay_ms_{1200}; double publish_fps_{30.0};
    std::string hw_accel_type_{"auto"}; int force_width_{0}, force_height_{0}; bool low_delay_flags_{true}; int max_open_retry_{15}; bool aggressive_skip_{false};

    // Thread control
    std::thread worker_; std::atomic<bool> running_{false};

    // FFmpeg objects
    AVFormatContext *fmt_ctx_{nullptr}; AVCodecContext *codec_ctx_{nullptr}; const AVCodec *decoder_{nullptr}; AVBufferRef *hw_device_ctx_{nullptr};
    AVFrame *frame_{nullptr}; AVFrame *hw_frame_{nullptr}; SwsContext *sws_ctx_{nullptr};
    int video_stream_index_{-1}; int target_width_{0}, target_height_{0}; AVPixelFormat hw_pix_fmt_{AV_PIX_FMT_NONE};

    // Reused buffer
    cv::Mat bgr_buffer_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "rtsp_ffmpeg_node");
    ros::NodeHandle nh; ros::NodeHandle pnh("~");
    av_log_set_level(AV_LOG_ERROR); // reduce noise
    RtspFFmpegNode node(nh, pnh);
    ros::spin();
    node.stop();
    return 0;
}
