// PTZ (pan-tilt-zoom) control node for HIKVISION network cameras (IP cameras).
//
// Commands are sent over HIKVISION ISAPI (HTTP + Digest auth), which is an independent
// connection from the RTSP video stream. Industrial cameras (MV-CS / MV-CA and similar)
// have no gimbal, so this node only applies to network cameras.
//
// Topics:
//   ~/cmd_vel      geometry_msgs/Twist    continuous motion: angular.z=pan, angular.y=tilt, linear.x=zoom
//   ~/goto_preset  std_msgs/Int32         move to a stored preset
//   ~/absolute     geometry_msgs/Vector3  absolute position: x=azimuth (deg), y=elevation (deg), z=zoom

#include <algorithm>
#include <cmath>
#include <chrono>
#include <string>

#include "curl/curl.h"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;

namespace
{
// The ISAPI speed range is -100 ~ 100
constexpr int kSpeedLimit = 100;

// Map the normalized -1.0~1.0 input onto the ISAPI speed range and clamp it
int to_isapi_speed(double normalized, double scale)
{
    const double value = std::round(normalized * scale * kSpeedLimit);
    return static_cast<int>(std::clamp(value, -1.0 * kSpeedLimit, 1.0 * kSpeedLimit));
}

// The XML the camera returns is only interesting on failure; discard it otherwise
size_t collect_response(char *ptr, size_t size, size_t nmemb, void *userdata)
{
    auto *body = static_cast<std::string *>(userdata);
    body->append(ptr, size * nmemb);
    return size * nmemb;
}
} // namespace

class PtzNode : public rclcpp::Node
{
public:
    PtzNode()
        : Node("hk_camera_ptz")
    {
        host_ = declare_parameter<std::string>("host", "192.168.1.64");
        port_ = declare_parameter<int>("port", 80);
        username_ = declare_parameter<std::string>("username", "admin");
        password_ = declare_parameter<std::string>("password", "");
        // ISAPI PTZ channel number, normally 1 (not the 101/102 RTSP stream number)
        channel_ = declare_parameter<int>("channel", 1);
        pan_scale_ = declare_parameter<double>("pan_scale", 1.0);
        tilt_scale_ = declare_parameter<double>("tilt_scale", 1.0);
        zoom_scale_ = declare_parameter<double>("zoom_scale", 1.0);
        // Stop automatically when no new cmd_vel arrives within this time, so the gimbal
        // does not keep spinning if the teleop side goes away
        command_timeout_ = declare_parameter<double>("command_timeout", 0.5);
        http_timeout_ = declare_parameter<double>("http_timeout", 2.0);

        base_url_ = "http://" + host_ + ":" + std::to_string(port_) +
                    "/ISAPI/PTZCtrl/channels/" + std::to_string(channel_);

        curl_global_init(CURL_GLOBAL_DEFAULT);
        curl_ = curl_easy_init();
        if (curl_ == nullptr)
        {
            RCLCPP_FATAL(get_logger(), "curl initialization failed");
            throw std::runtime_error("curl_easy_init failed");
        }

        cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "~/cmd_vel", 10,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) { on_cmd_vel(*msg); });
        preset_sub_ = create_subscription<std_msgs::msg::Int32>(
            "~/goto_preset", 10,
            [this](const std_msgs::msg::Int32::SharedPtr msg) { on_goto_preset(*msg); });
        absolute_sub_ = create_subscription<geometry_msgs::msg::Vector3>(
            "~/absolute", 10,
            [this](const geometry_msgs::msg::Vector3::SharedPtr msg) { on_absolute(*msg); });

        if (command_timeout_ > 0.0)
        {
            watchdog_ = create_wall_timer(100ms, [this] { check_timeout(); });
        }

        RCLCPP_INFO(get_logger(), "PTZ control endpoint: %s", base_url_.c_str());
        if (password_.empty())
        {
            RCLCPP_WARN(get_logger(),
                        "The password parameter is empty. ISAPI requires Digest auth, so every "
                        "command will return 401. Start with password:=<password>.");
        }
    }

    ~PtzNode() override
    {
        // Always stop before exiting, otherwise the gimbal keeps the last commanded speed
        if (moving_)
        {
            send_continuous(0, 0, 0);
        }
        if (curl_ != nullptr)
        {
            curl_easy_cleanup(curl_);
        }
        curl_global_cleanup();
    }

private:
    void on_cmd_vel(const geometry_msgs::msg::Twist &msg)
    {
        // Positive means pan right and tilt up, matching the ISAPI sign convention
        const int pan = to_isapi_speed(msg.angular.z, pan_scale_);
        const int tilt = to_isapi_speed(msg.angular.y, tilt_scale_);
        const int zoom = to_isapi_speed(msg.linear.x, zoom_scale_);

        last_cmd_time_ = now();

        // Teleop-style nodes resend the same command at a fixed rate. ISAPI continuous
        // motion latches until a stop is sent, so resending only wastes HTTP round-trips.
        if (pan == last_pan_ && tilt == last_tilt_ && zoom == last_zoom_)
        {
            return;
        }
        send_continuous(pan, tilt, zoom);
    }

    void on_goto_preset(const std_msgs::msg::Int32 &msg)
    {
        const std::string url = base_url_ + "/presets/" + std::to_string(msg.data) + "/goto";
        if (put(url, ""))
        {
            RCLCPP_INFO(get_logger(), "Moving to preset %d", msg.data);
        }
    }

    void on_absolute(const geometry_msgs::msg::Vector3 &msg)
    {
        // ISAPI works in units of 0.1 degrees: azimuth 0~3600, elevation -900~900
        const int azimuth = static_cast<int>(std::lround(std::fmod(msg.x, 360.0) * 10.0));
        const int elevation =
            static_cast<int>(std::clamp(std::lround(msg.y * 10.0), -900L, 900L));
        // Zoom is 1~1000; the message carries a multiplier, so 1.0x maps to 10
        const int zoom = static_cast<int>(std::clamp(std::lround(msg.z * 10.0), 10L, 1000L));

        const std::string body =
            "<?xml version=\"1.0\" encoding=\"UTF-8\"?>"
            "<PTZData><AbsoluteHigh>"
            "<elevation>" + std::to_string(elevation) + "</elevation>"
            "<azimuth>" + std::to_string(azimuth < 0 ? azimuth + 3600 : azimuth) + "</azimuth>"
            "<absoluteZoom>" + std::to_string(zoom) + "</absoluteZoom>"
            "</AbsoluteHigh></PTZData>";

        if (put(base_url_ + "/absolute", body))
        {
            RCLCPP_INFO(get_logger(), "Absolute move: azimuth %.1f deg, elevation %.1f deg, zoom %.1fx",
                        msg.x, msg.y, msg.z);
        }
    }

    // Stop automatically when no new command has arrived for a while
    void check_timeout()
    {
        if (!moving_)
        {
            return;
        }
        if ((now() - last_cmd_time_).seconds() < command_timeout_)
        {
            return;
        }
        RCLCPP_WARN(get_logger(), "No cmd_vel for %.1fs, stopping the gimbal", command_timeout_);
        send_continuous(0, 0, 0);
    }

    void send_continuous(int pan, int tilt, int zoom)
    {
        const std::string body =
            "<?xml version=\"1.0\" encoding=\"UTF-8\"?>"
            "<PTZData>"
            "<pan>" + std::to_string(pan) + "</pan>"
            "<tilt>" + std::to_string(tilt) + "</tilt>"
            "<zoom>" + std::to_string(zoom) + "</zoom>"
            "</PTZData>";

        if (!put(base_url_ + "/continuous", body))
        {
            return;
        }
        last_pan_ = pan;
        last_tilt_ = tilt;
        last_zoom_ = zoom;
        moving_ = (pan != 0 || tilt != 0 || zoom != 0);
        RCLCPP_DEBUG(get_logger(), "pan=%d tilt=%d zoom=%d", pan, tilt, zoom);
    }

    bool put(const std::string &url, const std::string &body)
    {
        std::string response;
        curl_slist *headers = curl_slist_append(nullptr, "Content-Type: application/xml");

        curl_easy_reset(curl_);
        curl_easy_setopt(curl_, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl_, CURLOPT_CUSTOMREQUEST, "PUT");
        curl_easy_setopt(curl_, CURLOPT_POSTFIELDS, body.c_str());
        curl_easy_setopt(curl_, CURLOPT_POSTFIELDSIZE, static_cast<long>(body.size()));
        curl_easy_setopt(curl_, CURLOPT_HTTPHEADER, headers);
        // HIKVISION defaults to Digest; some older firmware only supports Basic, so let
        // curl negotiate the scheme itself
        curl_easy_setopt(curl_, CURLOPT_HTTPAUTH, CURLAUTH_ANY);
        curl_easy_setopt(curl_, CURLOPT_USERNAME, username_.c_str());
        curl_easy_setopt(curl_, CURLOPT_PASSWORD, password_.c_str());
        curl_easy_setopt(curl_, CURLOPT_TIMEOUT_MS,
                         static_cast<long>(http_timeout_ * 1000));
        curl_easy_setopt(curl_, CURLOPT_WRITEFUNCTION, collect_response);
        curl_easy_setopt(curl_, CURLOPT_WRITEDATA, &response);

        const CURLcode code = curl_easy_perform(curl_);
        long status = 0;
        curl_easy_getinfo(curl_, CURLINFO_RESPONSE_CODE, &status);
        curl_slist_free_all(headers);

        if (code != CURLE_OK)
        {
            RCLCPP_ERROR(get_logger(), "Request to %s failed: %s", url.c_str(),
                         curl_easy_strerror(code));
            return false;
        }
        if (status != 200)
        {
            RCLCPP_ERROR(get_logger(), "Camera returned HTTP %ld: %s", status, response.c_str());
            return false;
        }
        return true;
    }

    std::string host_;
    int port_;
    std::string username_;
    std::string password_;
    int channel_;
    double pan_scale_;
    double tilt_scale_;
    double zoom_scale_;
    double command_timeout_;
    double http_timeout_;
    std::string base_url_;

    CURL *curl_{nullptr};
    int last_pan_{0};
    int last_tilt_{0};
    int last_zoom_{0};
    bool moving_{false};
    rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr preset_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr absolute_sub_;
    rclcpp::TimerBase::SharedPtr watchdog_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PtzNode>());
    rclcpp::shutdown();
    return 0;
}
