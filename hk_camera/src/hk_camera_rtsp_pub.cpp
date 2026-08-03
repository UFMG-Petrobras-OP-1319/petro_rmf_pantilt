// Image publisher node for HIKVISION network cameras (IP camera / RTSP).
//
// Note: this node does not use the MVS SDK. HIKVISION network cameras speak RTSP/ONVIF
// and are not GigE Vision devices, so MV_CC_EnumDevices can never enumerate them.
// Use the hk_camera node for industrial cameras (MV-CS / MV-CA and similar).

#include <algorithm>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <string>

#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>

namespace
{
// Number of consecutive failures after which the lockout hint is logged once
constexpr int kFailuresBeforeHint = 3;

// Percent-encode special characters in the user name / password, otherwise the URL is parsed wrong
std::string url_encode(const std::string &value)
{
    static const char *hex = "0123456789ABCDEF";
    std::string out;
    for (unsigned char c : value)
    {
        if (isalnum(c) || c == '-' || c == '_' || c == '.' || c == '~')
        {
            out += static_cast<char>(c);
        }
        else
        {
            out += '%';
            out += hex[c >> 4];
            out += hex[c & 0x0F];
        }
    }
    return out;
}

// Never print the plaintext password in the logs
std::string mask_password(const std::string &url)
{
    const auto at = url.rfind('@');
    const auto scheme = url.find("://");
    if (at == std::string::npos || scheme == std::string::npos)
    {
        return url;
    }
    const auto colon = url.find(':', scheme + 3);
    if (colon == std::string::npos || colon > at)
    {
        return url;
    }
    return url.substr(0, colon + 1) + "****" + url.substr(at);
}
} // namespace

class RtspCameraNode : public rclcpp::Node
{
public:
    RtspCameraNode()
        : Node("hk_camera_rtsp")
    {
        host_ = declare_parameter<std::string>("host", "192.168.1.64");
        port_ = declare_parameter<int>("port", 554);
        username_ = declare_parameter<std::string>("username", "admin");
        password_ = declare_parameter<std::string>("password", "");
        // 101 = main stream (high resolution), 102 = sub stream (lower resolution, less latency)
        channel_ = declare_parameter<int>("channel", 101);
        // When a full URL is given, the host/username/... parameters above are ignored
        rtsp_url_ = declare_parameter<std::string>("rtsp_url", "");
        frame_id_ = declare_parameter<std::string>("frame_id", "hk_camera");
        use_tcp_ = declare_parameter<bool>("use_tcp", true);
        reconnect_delay_s_ = declare_parameter<double>("reconnect_delay", 3.0);
        // Upper bound for the backoff below. Retrying a rejected login every few seconds
        // keeps a camera lockout alive instead of letting it expire, so the wait grows.
        max_reconnect_delay_s_ = declare_parameter<double>("max_reconnect_delay", 60.0);
        // 0 means publish at full speed, following the camera's own frame rate
        publish_rate_ = declare_parameter<double>("publish_rate", 0.0);

        if (rtsp_url_.empty())
        {
            rtsp_url_ = "rtsp://" + url_encode(username_) + ":" + url_encode(password_) +
                        "@" + host_ + ":" + std::to_string(port_) +
                        "/Streaming/Channels/" + std::to_string(channel_);
        }

        // FFMPEG defaults to UDP, which tears the image on packet loss; TCP is more stable
        setenv("OPENCV_FFMPEG_CAPTURE_OPTIONS",
               use_tcp_ ? "rtsp_transport;tcp" : "rtsp_transport;udp", 1);

        publisher_ = image_transport::create_publisher(this, "/hk_camera/rgb");

        if (publish_rate_ > 0.0)
        {
            rate_ = std::make_unique<rclcpp::Rate>(publish_rate_);
        }

        RCLCPP_INFO(get_logger(), "RTSP URL: %s", mask_password(rtsp_url_).c_str());
        if (password_.empty())
        {
            RCLCPP_WARN(get_logger(),
                        "The password parameter is empty. HIKVISION network cameras require "
                        "Digest auth, so the connection will fail. Start with password:=<password>.");
        }
    }

    // Blocking main loop: connect -> stream -> reconnect on failure
    void spin()
    {
        cv_bridge::CvImage frame;
        frame.encoding = sensor_msgs::image_encodings::BGR8;
        frame.header.frame_id = frame_id_;

        while (rclcpp::ok())
        {
            if (!capture_.isOpened() && !connect())
            {
                sleep_for_reconnect();
                continue;
            }

            cv::Mat image;
            if (!capture_.read(image) || image.empty())
            {
                RCLCPP_WARN(get_logger(), "Failed to read a frame, reconnecting...");
                capture_.release();
                sleep_for_reconnect();
                continue;
            }

            frame.image = image;
            frame.header.stamp = now();
            publisher_.publish(frame.toImageMsg());

            rclcpp::spin_some(get_node_base_interface());
            if (rate_)
            {
                rate_->sleep();
            }
        }
        capture_.release();
    }

private:
    bool connect()
    {
        RCLCPP_INFO(get_logger(), "Connecting to %s ...", mask_password(rtsp_url_).c_str());
        capture_.open(rtsp_url_, cv::CAP_FFMPEG);
        if (!capture_.isOpened())
        {
            RCLCPP_ERROR(get_logger(),
                         "Connection failed. Check the user name / password, that the camera is "
                         "reachable, and that the channel exists.");
            return false;
        }
        // Keep only the newest frame so latency does not pile up
        capture_.set(cv::CAP_PROP_BUFFERSIZE, 1);
        consecutive_failures_ = 0;
        RCLCPP_INFO(get_logger(), "Connected: %.0fx%.0f @ %.1f fps",
                    capture_.get(cv::CAP_PROP_FRAME_WIDTH),
                    capture_.get(cv::CAP_PROP_FRAME_HEIGHT),
                    capture_.get(cv::CAP_PROP_FPS));
        return true;
    }

    // Wait before the next attempt, backing off exponentially while failures continue.
    // A fixed short interval is actively harmful when the credentials are wrong: HIKVISION
    // cameras lock the client out after a few failed logins, and every further attempt
    // renews that lock, so a brief mistake turns into a permanent lockout.
    void sleep_for_reconnect()
    {
        if (!rclcpp::ok())
        {
            return;
        }

        const double delay = std::min(reconnect_delay_s_ * std::pow(2.0, consecutive_failures_),
                                      max_reconnect_delay_s_);
        ++consecutive_failures_;

        if (consecutive_failures_ == kFailuresBeforeHint)
        {
            RCLCPP_WARN(get_logger(),
                        "%d connection attempts failed in a row. If this is an authentication "
                        "error, the camera has probably locked this host out; check with "
                        "curl --digest -u <user>:<password> http://%s/ISAPI/Security/userCheck "
                        "and stop this node while the lock lasts.",
                        consecutive_failures_, host_.c_str());
        }

        RCLCPP_INFO(get_logger(), "Retrying in %.1fs ...", delay);
        rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(delay * 1000)));
    }

    std::string host_;
    int port_;
    std::string username_;
    std::string password_;
    int channel_;
    std::string rtsp_url_;
    std::string frame_id_;
    bool use_tcp_;
    double reconnect_delay_s_;
    double max_reconnect_delay_s_;
    double publish_rate_;
    int consecutive_failures_{0};

    cv::VideoCapture capture_;
    image_transport::Publisher publisher_;
    std::unique_ptr<rclcpp::Rate> rate_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RtspCameraNode>();
    node->spin();
    rclcpp::shutdown();
    return 0;
}
