#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Header.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <sstream>

struct FramePack {
    cv::Mat frame;
    ros::Time stamp_capture;
};

class RTSPGStreamerNode {
public:
    RTSPGStreamerNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
        : nh_(nh), pnh_(pnh), running_(true), have_frame_(false) {

        // 读取参数
        pnh_.param<std::string>("rtsp_url", rtsp_url_, "rtsp://127.0.0.1:8554/live");
        pnh_.param<int>("latency", latency_ms_, 100);
        pnh_.param<bool>("force_tcp", force_tcp_, true);
        pnh_.param<bool>("use_hw_decoder", use_hw_decoder_, false);
        pnh_.param<int>("width", width_, 0);     // 0 表示不强制
        pnh_.param<int>("height", height_, 0);
        pnh_.param<double>("fps", target_fps_, 25.0);
        pnh_.param<double>("reconnect_delay", reconnect_delay_, 2.0);
        pnh_.param<int>("open_fail_retry", open_fail_retry_, 5);

        image_pub_ = nh_.advertise<sensor_msgs::Image>("image_raw", 1);

        ROS_INFO_STREAM("RTSP GStreamer Node 启动参数:"
                        << "\n  url=" << rtsp_url_
                        << "\n  latency_ms=" << latency_ms_
                        << "\n  force_tcp=" << (force_tcp_?"true":"false")
                        << "\n  hw_decoder=" << (use_hw_decoder_?"true":"false")
                        << "\n  size=" << width_ << "x" << height_
                        << "\n  fps=" << target_fps_);

        capture_thread_ = std::thread(&RTSPGStreamerNode::captureLoop, this);
        publisher_thread_ = std::thread(&RTSPGStreamerNode::publishLoop, this);
    }

    ~RTSPGStreamerNode() {
        stop();
    }

    void stop() {
        if (!running_) return;
        running_ = false;
        cv_.notify_all();
        if (capture_thread_.joinable()) capture_thread_.join();
        if (publisher_thread_.joinable()) publisher_thread_.join();
        if (cap_.isOpened()) cap_.release();
        ROS_INFO("RTSP GStreamer Node 已停止");
    }

private:
    std::string buildPipeline() {
        // 选择解码器
        // 简单示例: 仅当 use_hw_decoder_ 为 true 时尝试常见硬件名称，否则 avdec_h264
        std::string decoder = "avdec_h264";
        if (use_hw_decoder_) {
            // 用户可按平台手动改成 nvv4l2decoder / vaapih264dec / nvh264dec
            decoder = "avdec_h264"; // 占位：未检测具体插件
        }

        std::ostringstream ss;
        ss << "rtspsrc location=" << rtsp_url_;
        if (force_tcp_) ss << " protocols=tcp";
        ss << " latency=" << latency_ms_;
        ss << " ! rtph264depay ! h264parse ! " << decoder
           << " ! videoconvert";

        if (width_ > 0 && height_ > 0) {
            ss << " ! videoscale ! video/x-raw,width=" << width_ << ",height=" << height_ << ",format=BGR";
        } else {
            ss << " ! video/x-raw,format=BGR";
        }

        // 控制 appsink 丢帧避免堆积
        ss << " ! appsink drop=1 sync=false max-buffers=1";
        return ss.str();
    }

    bool openCapture() {
        std::string pipeline = buildPipeline();
        ROS_INFO_STREAM("尝试打开 GStreamer pipeline:\n" << pipeline);
        cap_.open(pipeline, cv::CAP_GSTREAMER);
        if (!cap_.isOpened()) {
            ROS_WARN("VideoCapture 打开失败");
            return false;
        }
        // 预热读取
        for (int i = 0; i < 5; ++i) {
            cv::Mat tmp;
            if (cap_.read(tmp) && !tmp.empty()) {
                ROS_INFO_STREAM("打开成功 分辨率=" << tmp.cols << "x" << tmp.rows);
                return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        ROS_WARN("预热读取失败，释放并重试");
        cap_.release();
        return false;
    }

    void captureLoop() {
        int retry_count = 0;
        while (running_) {
            if (!cap_.isOpened()) {
                if (retry_count >= open_fail_retry_) {
                    ROS_ERROR("超过最大打开重试次数，停止捕获线程");
                    break;
                }
                if (openCapture()) {
                    retry_count = 0;
                } else {
                    ++retry_count;
                    std::this_thread::sleep_for(std::chrono::milliseconds(
                        static_cast<int>(reconnect_delay_ * 1000)));
                    continue;
                }
            }

            cv::Mat frame;
            if (!cap_.read(frame) || frame.empty()) {
                ROS_WARN("读取帧失败，重新建立连接");
                cap_.release();
                std::this_thread::sleep_for(std::chrono::milliseconds(
                    static_cast<int>(reconnect_delay_ * 1000)));
                continue;
            }

            {
                std::lock_guard<std::mutex> lock(mutex_);
                latest_.frame = std::move(frame);
                latest_.stamp_capture = ros::Time::now();
                have_frame_ = true;
            }
            cv_.notify_one();
        }
    }

    void publishLoop() {
        ros::Rate r(target_fps_ > 0 ? target_fps_ : 30.0);
        size_t frame_counter = 0;
        ros::Time stat_start = ros::Time::now();
        double accum_latency = 0.0;

        while (running_ && ros::ok()) {
            // 等待有帧
            {
                std::unique_lock<std::mutex> lock(mutex_);
                cv_.wait_for(lock, std::chrono::milliseconds(500), [&]{
                    return have_frame_ || !running_;
                });
                if (!running_) break;
                if (!have_frame_) {
                    // 超时继续
                    lock.unlock();
                    r.sleep();
                    continue;
                }
                // 拷贝一份 (避免发布时阻塞捕获线程)
                current_pub_ = latest_;
                have_frame_ = false;
            }

            // 发布
            try {
                cv_bridge::CvImage cv_img;
                cv_img.header.stamp = ros::Time::now();
                cv_img.header.frame_id = "rtsp_camera";
                cv_img.encoding = "bgr8";
                cv_img.image = current_pub_.frame;
                sensor_msgs::Image msg;
                cv_img.toImageMsg(msg);
                image_pub_.publish(msg);

                // 统计
                double latency = (cv_img.header.stamp - current_pub_.stamp_capture).toSec();
                accum_latency += latency;
                ++frame_counter;

                ros::Time now = ros::Time::now();
                if ((now - stat_start).toSec() >= 5.0) {
                    double fps = frame_counter / (now - stat_start).toSec();
                    double avg_latency_ms = (frame_counter > 0 ? (accum_latency / frame_counter) * 1000.0 : 0.0);
                    ROS_INFO_STREAM("统计窗口: FPS=" << fps
                                    << " 平均延迟(ms)=" << avg_latency_ms
                                    << " 帧数=" << frame_counter);
                    frame_counter = 0;
                    accum_latency = 0.0;
                    stat_start = now;
                }

            } catch (const std::exception& e) {
                ROS_WARN_STREAM("发布异常: " << e.what());
            }

            r.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Publisher image_pub_;

    std::string rtsp_url_;
    int latency_ms_{100};
    bool force_tcp_{true};
    bool use_hw_decoder_{false};
    int width_{0};
    int height_{0};
    double target_fps_{25.0};
    double reconnect_delay_{2.0};
    int open_fail_retry_{5};

    cv::VideoCapture cap_;

    std::thread capture_thread_;
    std::thread publisher_thread_;
    std::atomic<bool> running_;

    std::mutex mutex_;
    std::condition_variable cv_;
    FramePack latest_;
    FramePack current_pub_;
    bool have_frame_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "rtsp_gstreamer_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // 打印 OpenCV 与 GStreamer 支持
    try {
        std::string info = cv::getBuildInformation();
        bool gst = (info.find("GStreamer") != std::string::npos);
        ROS_INFO_STREAM("OpenCV GStreamer 支持: " << (gst?"YES":"NO"));
    } catch (...) {
        ROS_WARN("无法获取 OpenCV 构建信息");
    }

    RTSPGStreamerNode node(nh, pnh);
    ros::spin();
    node.stop();
    return 0;
}