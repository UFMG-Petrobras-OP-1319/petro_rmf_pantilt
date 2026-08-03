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
#include "hk_camera_interfaces/srv/take_photo.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>

namespace
{
// Number of consecutive failures after which the lockout hint is logged once
constexpr int kFailuresBeforeHint = 3;

// Sentinel meaning "publish the frame as it arrives"; the others are cv::flip codes
constexpr int kNoFlip = -2;

// Translate the flip_mode parameter into the code cv::flip expects
int flip_code_from(const std::string &mode)
{
    if (mode == "none" || mode.empty())
    {
        return kNoFlip;
    }
    if (mode == "horizontal")
    {
        return 1;
    }
    if (mode == "vertical")
    {
        return 0;
    }
    // "180" works from a YAML file, where it is quoted. On the command line
    // -p flip_mode:=180 is parsed as an integer and the node rejects it, so
    // rotate_180 is the spelling that works everywhere.
    if (mode == "rotate_180" || mode == "180" || mode == "both")
    {
        return -1;
    }
    return kNoFlip;
}

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
        // Rotate or mirror every frame before publishing, for a camera that is not
        // mounted upright: "none", "horizontal", "vertical", or "rotate_180" for an
        // upside-down mount. Doing this on the camera instead costs no CPU here.
        flip_mode_ = declare_parameter<std::string>("flip_mode", "none");
        flip_code_ = flip_code_from(flip_mode_);
        if (flip_code_ == kNoFlip && flip_mode_ != "none" && !flip_mode_.empty())
        {
            RCLCPP_WARN(get_logger(),
                        "Unknown flip_mode '%s'; expected none, horizontal, vertical or rotate_180. "
                        "Publishing frames unchanged.",
                        flip_mode_.c_str());
        }
        else if (flip_code_ != kNoFlip)
        {
            RCLCPP_INFO(get_logger(), "Flipping every frame: %s", flip_mode_.c_str());
        }
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

        // Writes the most recent published frame to disk on demand. The frame is already
        // decoded and flipped here, so a photo costs no extra connection to the camera and
        // matches exactly what /hk_camera/rgb showed.
        photo_service_ = create_service<hk_camera_interfaces::srv::TakePhoto>(
            "~/take_photo",
            std::bind(&RtspCameraNode::handle_take_photo, this,
                      std::placeholders::_1, std::placeholders::_2));

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
                // Drop the cached frame so take_photo reports the outage instead of
                // quietly writing a picture from before the stream died
                last_frame_.release();
                sleep_for_reconnect();
                continue;
            }

            if (flip_code_ == kNoFlip)
            {
                frame.image = image;
            }
            else
            {
                // Not done in place: cv::flip does not support aliasing src and dst
                cv::flip(image, flipped_, flip_code_);
                frame.image = flipped_;
            }
            frame.header.stamp = now();
            publisher_.publish(frame.toImageMsg());

            // Owned copy: frame.image aliases either the capture buffer or flipped_, and
            // both are overwritten by the next iteration. The service callback runs on this
            // same thread (inside spin_some, below), so no lock is needed.
            frame.image.copyTo(last_frame_);
            last_frame_stamp_ = frame.header.stamp;

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
        // Slice the wait so service calls are still answered while the stream is down.
        // The backoff reaches a minute, and a take_photo client should be told the stream
        // is gone rather than blocking until the camera comes back.
        const auto deadline = std::chrono::steady_clock::now() +
                              std::chrono::milliseconds(static_cast<int>(delay * 1000));
        while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline)
        {
            rclcpp::spin_some(get_node_base_interface());
            rclcpp::sleep_for(std::chrono::milliseconds(100));
        }
    }

    void handle_take_photo(
        const std::shared_ptr<hk_camera_interfaces::srv::TakePhoto::Request> request,
        std::shared_ptr<hk_camera_interfaces::srv::TakePhoto::Response> response)
    {
        const std::string &path = request->save_path;

        if (path.empty())
        {
            response->success = false;
            response->message = "save_path is empty.";
            RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
            return;
        }

        if (last_frame_.empty())
        {
            response->success = false;
            response->message = capture_.isOpened()
                                    ? "No frame received from the camera yet."
                                    : "The camera stream is disconnected.";
            RCLCPP_ERROR(get_logger(), "take_photo failed: %s", response->message.c_str());
            return;
        }

        // imwrite returns false for an unwritable path and throws for an extension it has
        // no encoder for, so both have to be handled to avoid reporting a success that
        // left nothing on disk
        bool written = false;
        try
        {
            written = cv::imwrite(path, last_frame_);
        }
        catch (const cv::Exception &e)
        {
            response->success = false;
            response->message = "Could not encode the image for '" + path + "': " + e.what();
            RCLCPP_ERROR(get_logger(), "take_photo failed: %s", response->message.c_str());
            return;
        }

        if (!written)
        {
            response->success = false;
            response->message = "Could not write '" + path +
                                "'. Check that the directory exists and is writable.";
            RCLCPP_ERROR(get_logger(), "take_photo failed: %s", response->message.c_str());
            return;
        }

        response->success = true;
        response->message = "Image saved to " + path;
        RCLCPP_INFO(get_logger(), "Photo (%dx%d, captured at %.3fs) saved to %s",
                    last_frame_.cols, last_frame_.rows,
                    rclcpp::Time(last_frame_stamp_).seconds(), path.c_str());
    }

    std::string host_;
    int port_;
    std::string username_;
    std::string password_;
    int channel_;
    std::string rtsp_url_;
    std::string frame_id_;
    std::string flip_mode_;
    int flip_code_{kNoFlip};
    bool use_tcp_;
    double reconnect_delay_s_;
    double max_reconnect_delay_s_;
    double publish_rate_;
    int consecutive_failures_{0};

    cv::VideoCapture capture_;
    cv::Mat flipped_;
    // Most recently published frame, kept for the take_photo service. Empty until the
    // first frame arrives and released again whenever the stream drops.
    cv::Mat last_frame_;
    builtin_interfaces::msg::Time last_frame_stamp_;
    image_transport::Publisher publisher_;
    rclcpp::Service<hk_camera_interfaces::srv::TakePhoto>::SharedPtr photo_service_;
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
