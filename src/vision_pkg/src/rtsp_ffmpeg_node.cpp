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
#include <mutex>
#include <condition_variable>
#include <queue>
#include <string>
#include <chrono>

/*
 * 低延迟 RTSP 拉流 (H.264) 节点, 不使用 GStreamer.
 * 特性:
 *  - 使用 FFmpeg libav* API 直接解码, 可选硬件加速 (vaapi / cuda / v4l2m2m / drm)
 *  - 丢弃过时帧, 只保留最新帧, 减少延迟
 *  - 自动重连
 *  - 发布 ROS sensor_msgs/Image (bgr8)
 */

struct DecodedFrame {
    cv::Mat bgr;
    ros::Time capture_stamp; // 解码完成时间近似
};

class RtspFFmpegNode {
public: //  rtsp://127.0.0.1:8554/live
    RtspFFmpegNode(ros::NodeHandle &nh, ros::NodeHandle &pnh): nh_(nh), pnh_(pnh) {
        pnh_.param<std::string>("rtsp_url", rtsp_url_, std::string("rtsp://127.0.0.1:8554/live"));
        pnh_.param<std::string>("output_topic", output_topic_, std::string("/vision/ffmpeg/image_raw"));
        pnh_.param<bool>("force_tcp", force_tcp_, false); // 对应于设置 rtsp_transport=tcp
        pnh_.param<int>("reconnect_delay_ms", reconnect_delay_ms_, 1500);
        pnh_.param<int>("queue_size", queue_size_, 1);
        pnh_.param<double>("publish_fps", publish_fps_, 30.0);
        pnh_.param<std::string>("hw_accel", hw_accel_type_, std::string("auto")); // auto / none / vaapi / cuda / v4l2m2m
        pnh_.param<int>("video_width", force_width_, 0); // 0 不强制
        pnh_.param<int>("video_height", force_height_, 0);
        pnh_.param<bool>("low_delay_flags", low_delay_flags_, true); // 设置解码器低延迟参数
        pnh_.param<int>("max_open_retry", max_open_retry_, 10);

        image_pub_ = nh_.advertise<sensor_msgs::Image>(output_topic_, 1);
        running_ = true;

        decode_thread_ = std::thread(&RtspFFmpegNode::decodeLoop, this);
        publish_thread_ = std::thread(&RtspFFmpegNode::publishLoop, this);
    }

    ~RtspFFmpegNode() { stop(); }

    void stop() {
        if(!running_) return;
        running_ = false;
        cv_.notify_all();
        if(decode_thread_.joinable()) decode_thread_.join();
        if(publish_thread_.joinable()) publish_thread_.join();
        closeStream();
    }

private:
    void logFFmpegVersions() {
        ROS_INFO_STREAM("libavformat version: " << avformat_version());
        ROS_INFO_STREAM("libavcodec  version: " << avcodec_version());
    }

    AVPixelFormat findHwFormat(AVCodecContext *ctx, const enum AVPixelFormat *pix_fmts) {
        for (const enum AVPixelFormat *p = pix_fmts; *p != AV_PIX_FMT_NONE; p++) {
            if (*p == hw_pix_fmt_) return *p;
        }
        return AV_PIX_FMT_NONE;
    }

    bool initHWDevice(AVCodecID codec_id) {
        if (hw_accel_type_ == "none") return false;
        if (hw_accel_type_ == "auto") {
            // 简单启发式: 先尝试 vaapi, 再 v4l2m2m, 再 cuda
            const char* try_list[] = {"vaapi", "v4l2m2m", "cuda"};
            for (auto name: try_list) {
                if (createHWDevice(name)) return true;
            }
            return false;
        }
        return createHWDevice(hw_accel_type_.c_str());
    }

    bool createHWDevice(const char* type) {
        AVHWDeviceType dev_type = av_hwdevice_find_type_by_name(type);
        if (dev_type == AV_HWDEVICE_TYPE_NONE) {
            ROS_WARN_STREAM("未找到硬件设备类型: " << type);
            return false;
        }
        int ret = av_hwdevice_ctx_create(&hw_device_ctx_, dev_type, NULL, NULL, 0);
        if (ret < 0) {
            ROS_WARN_STREAM("创建硬件设备失败: " << type << " ret=" << ret);
            return false;
        }
        // 选择对应的 hw pix fmt
        for (int i=0;;i++) {
            const AVCodecHWConfig *config = avcodec_get_hw_config(decoder_, i);
            if (!config) break;
            if ((config->methods & AV_CODEC_HW_CONFIG_METHOD_HW_DEVICE_CTX) && config->device_type == dev_type) {
                hw_pix_fmt_ = config->pix_fmt;
                ROS_INFO_STREAM("使用硬件解码: " << type);
                return true;
            }
        }
        ROS_WARN_STREAM("未找到匹配硬件配置: " << type);
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
        logFFmpegVersions();
        AVDictionary *opts = nullptr;
        if (force_tcp_) av_dict_set(&opts, "rtsp_transport", "tcp", 0);
        av_dict_set(&opts, "stimeout", "2000000", 0); // 2s 超时 (us)
        av_dict_set(&opts, "buffer_size", "102400", 0);
        av_dict_set(&opts, "max_delay", "500000", 0); // 0.5s
        av_dict_set(&opts, "reorder_queue_size", "0", 0);
        av_dict_set(&opts, "fflags", "nobuffer", 0);
        av_dict_set(&opts, "flags", "low_delay", 0);

        int ret = avformat_open_input(&fmt_ctx_, rtsp_url_.c_str(), nullptr, &opts);
        av_dict_free(&opts);
        if (ret < 0) {
            ROS_WARN_STREAM("avformat_open_input 失败 ret=" << ret);
            return false;
        }
        ret = avformat_find_stream_info(fmt_ctx_, nullptr);
        if (ret < 0) {
            ROS_WARN("find_stream_info 失败");
            return false;
        }
        video_stream_index_ = -1;
        for (unsigned i=0;i<fmt_ctx_->nb_streams;i++) {
            if (fmt_ctx_->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {
                video_stream_index_ = static_cast<int>(i);
                break;
            }
        }
        if (video_stream_index_ < 0) {
            ROS_ERROR("未找到视频流");
            return false;
        }
        AVCodecParameters *codecpar = fmt_ctx_->streams[video_stream_index_]->codecpar;
        const AVCodec *codec = avcodec_find_decoder(codecpar->codec_id);
        if (!codec) { ROS_ERROR("未找到解码器"); return false; }
        decoder_ = codec;
        codec_ctx_ = avcodec_alloc_context3(codec);
        if (!codec_ctx_) { ROS_ERROR("分配 codec_ctx 失败"); return false; }
        if (avcodec_parameters_to_context(codec_ctx_, codecpar) < 0) { ROS_ERROR("参数复制失败"); return false; }

        if (low_delay_flags_) {
            codec_ctx_->flags |= AV_CODEC_FLAG_LOW_DELAY;
            codec_ctx_->flags2 |= AV_CODEC_FLAG2_FAST;
        }
        codec_ctx_->thread_count = 1; // 避免线程延迟缓冲

        bool hw_enabled = initHWDevice(codecpar->codec_id);
        if (hw_enabled && hw_device_ctx_) {
            codec_ctx_->hw_device_ctx = av_buffer_ref(hw_device_ctx_);
            codec_ctx_->get_format = [](AVCodecContext *ctx, const enum AVPixelFormat *pix_fmts){
                RtspFFmpegNode *self = reinterpret_cast<RtspFFmpegNode*>(ctx->opaque);
                return self->findHwFormat(ctx, pix_fmts);
            };
            codec_ctx_->opaque = this;
        }

        if (avcodec_open2(codec_ctx_, codec, nullptr) < 0) {
            ROS_ERROR("打开解码器失败");
            return false;
        }

        frame_ = av_frame_alloc();
        hw_frame_ = av_frame_alloc();
        if (!frame_ || !hw_frame_) { ROS_ERROR("分配帧失败"); return false; }

        // 强制缩放尺寸
        if (force_width_ > 0 && force_height_ > 0) {
            target_width_ = force_width_;
            target_height_ = force_height_;
        } else {
            target_width_ = codec_ctx_->width;
            target_height_ = codec_ctx_->height;
        }

        ROS_INFO_STREAM("打开成功: 原始尺寸=" << codec_ctx_->width << "x" << codec_ctx_->height
                        << " 目标尺寸=" << target_width_ << "x" << target_height_
                        << (hw_enabled?" (HW)":" (SW)"));
        return true;
    }

    void decodeLoop() {
        int open_retry = 0;
        while (running_ && ros::ok()) {
            if (!fmt_ctx_) {
                if (open_retry >= max_open_retry_) {
                    ROS_ERROR("超过最大打开重试次数, 停止解码循环");
                    break;
                }
                if (!openStream()) {
                    open_retry++;
                    std::this_thread::sleep_for(std::chrono::milliseconds(reconnect_delay_ms_));
                    continue;
                }
                open_retry = 0;
            }

            AVPacket pkt; av_init_packet(&pkt);
            int ret = av_read_frame(fmt_ctx_, &pkt);
            if (ret < 0) {
                ROS_WARN("读取帧失败/流结束, 重连");
                av_packet_unref(&pkt);
                closeStream();
                std::this_thread::sleep_for(std::chrono::milliseconds(reconnect_delay_ms_));
                continue;
            }
            if (pkt.stream_index != video_stream_index_) { av_packet_unref(&pkt); continue; }

            if (avcodec_send_packet(codec_ctx_, &pkt) < 0) {
                ROS_WARN("发送 packet 失败" );
                av_packet_unref(&pkt);
                continue;
            }
            av_packet_unref(&pkt);

            while (ret >= 0) {
                ret = avcodec_receive_frame(codec_ctx_, frame_);
                if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) break;
                if (ret < 0) { ROS_WARN("接收 frame 失败"); break; }

                AVFrame *use_frame = frame_;
                AVFrame *mapped = nullptr;
                if (frame_->format == hw_pix_fmt_) {
                    // 提取硬件帧
                    if (av_hwframe_transfer_data(hw_frame_, frame_, 0) == 0) {
                        use_frame = hw_frame_;
                    } else {
                        ROS_WARN_THROTTLE(5.0, "硬件帧拷贝失败, 回退原始帧");
                    }
                }

                // 建立转换上下文 (YUV -> BGR)
                if (!sws_ctx_) {
                    sws_ctx_ = sws_getContext(use_frame->width, use_frame->height, (AVPixelFormat)use_frame->format,
                                              target_width_, target_height_, AV_PIX_FMT_BGR24, SWS_BILINEAR, nullptr, nullptr, nullptr);
                    if (!sws_ctx_) {
                        ROS_ERROR("创建 sws_ctx 失败");
                        continue;
                    }
                }

                cv::Mat bgr(target_height_, target_width_, CV_8UC3);
                uint8_t *dst_data[4] = { bgr.data, nullptr, nullptr, nullptr };
                int dst_linesize[4] = { (int)bgr.step[0], 0, 0, 0 };
                sws_scale(sws_ctx_, use_frame->data, use_frame->linesize, 0, use_frame->height, dst_data, dst_linesize);

                DecodedFrame df;
                df.bgr = std::move(bgr);
                df.capture_stamp = ros::Time::now();

                {
                    std::lock_guard<std::mutex> lk(mtx_);
                    latest_ = std::move(df);
                    have_frame_ = true;
                }
                cv_.notify_one();
            }
        }
    }

    void publishLoop() {
        ros::Rate rate(publish_fps_ > 0 ? publish_fps_ : 30.0);
        size_t frame_count = 0; double accum_latency = 0.0; ros::Time stat_start = ros::Time::now();
        while (running_ && ros::ok()) {
            DecodedFrame to_pub; bool got = false;
            {
                std::unique_lock<std::mutex> lk(mtx_);
                cv_.wait_for(lk, std::chrono::milliseconds(500), [&]{return have_frame_ || !running_;});
                if (!running_) break;
                if (have_frame_) { to_pub = std::move(latest_); have_frame_ = false; got = true; }
            }
            if (!got) { rate.sleep(); continue; }
            try {
                cv_bridge::CvImage cv_img;
                cv_img.header.stamp = ros::Time::now();
                cv_img.header.frame_id = "rtsp_ffmpeg";
                cv_img.encoding = "bgr8";
                cv_img.image = to_pub.bgr; // 共享数据
                sensor_msgs::Image msg; cv_img.toImageMsg(msg);
                image_pub_.publish(msg);
                double latency = (cv_img.header.stamp - to_pub.capture_stamp).toSec();
                accum_latency += latency; frame_count++;
                ros::Time now = ros::Time::now();
                if ((now - stat_start).toSec() >= 5.0) {
                    double fps = frame_count / (now - stat_start).toSec();
                    ROS_INFO_STREAM("FFmpeg节点统计: FPS=" << fps << " 平均发布延迟(ms)=" << (frame_count? (accum_latency/frame_count)*1000.0:0));
                    frame_count = 0; accum_latency = 0.0; stat_start = now;
                }
            } catch (std::exception &e) {
                ROS_WARN_STREAM("发布异常: " << e.what());
            }
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_; ros::NodeHandle pnh_;
    ros::Publisher image_pub_;

    std::string rtsp_url_; std::string output_topic_;
    bool force_tcp_{true}; int reconnect_delay_ms_{1500}; int queue_size_{1};
    double publish_fps_{30.0}; std::string hw_accel_type_{"auto"};
    int force_width_{0}, force_height_{0}; bool low_delay_flags_{true}; int max_open_retry_{10};

    std::thread decode_thread_; std::thread publish_thread_; std::atomic<bool> running_{false};
    std::mutex mtx_; std::condition_variable cv_; bool have_frame_{false}; DecodedFrame latest_;

    // FFmpeg 相关
    AVFormatContext *fmt_ctx_{nullptr}; AVCodecContext *codec_ctx_{nullptr};
    const AVCodec *decoder_{nullptr}; AVBufferRef *hw_device_ctx_{nullptr};
    AVFrame *frame_{nullptr}; AVFrame *hw_frame_{nullptr};
    struct SwsContext *sws_ctx_{nullptr};
    int video_stream_index_{-1};
    int target_width_{0}, target_height_{0};
    AVPixelFormat hw_pix_fmt_{AV_PIX_FMT_NONE};
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "rtsp_ffmpeg_node");
    ros::NodeHandle nh; ros::NodeHandle pnh("~");
    av_log_set_level(AV_LOG_ERROR); // 降低噪声
    RtspFFmpegNode node(nh, pnh);
    ros::spin();
    node.stop();
    return 0;
}
