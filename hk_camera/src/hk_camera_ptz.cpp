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
//   pan_camera_cmd   std_msgs/Float64     joint setpoint for pan_joint (rad), as the fleet sends it
//   tilt_camera_cmd  std_msgs/Float64     joint setpoint for tilt_joint (rad)
//   ~/joint_states   sensor_msgs/JointState  (published) pan_joint/tilt_joint read back from the camera
//
// The joint topics speak the same joint space as the simulated robot model, so the fleet
// and the robot controller drive and check the real gimbal exactly as they do in
// simulation. ptz_geometry.hpp maps that space onto ISAPI positions.

#include <algorithm>
#include <cmath>
#include <chrono>
#include <optional>
#include <string>

#include "curl/curl.h"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "ptz_geometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"
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

        // Joint-space bridge. Empty topic names disable it.
        pan_cmd_topic_ = declare_parameter<std::string>("pan_cmd_topic", "pan_camera_cmd");
        tilt_cmd_topic_ = declare_parameter<std::string>("tilt_cmd_topic", "tilt_camera_cmd");
        pan_joint_name_ = declare_parameter<std::string>("pan_joint_name", "pan_joint");
        tilt_joint_name_ = declare_parameter<std::string>("tilt_joint_name", "tilt_joint");
        // How the joints relate to ISAPI azimuth/elevation on this mount; calibrate on
        // the robot (see ptz_geometry.hpp)
        mapping_.pan_sign = declare_parameter<double>("pan_sign", 1.0);
        mapping_.pan_offset_deg = declare_parameter<double>("pan_offset_deg", 0.0);
        mapping_.tilt_sign = declare_parameter<double>("tilt_sign", 1.0);
        mapping_.tilt_offset_deg = declare_parameter<double>("tilt_offset_deg", 0.0);
        // Position feedback: how often to ask the camera where it is. 0 disables it.
        status_rate_ = declare_parameter<double>("status_rate", 5.0);
        joint_states_topic_ =
            declare_parameter<std::string>("joint_states_topic", "~/joint_states");

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

        if (!pan_cmd_topic_.empty() && !tilt_cmd_topic_.empty())
        {
            pan_cmd_sub_ = create_subscription<std_msgs::msg::Float64>(
                pan_cmd_topic_, 10,
                [this](const std_msgs::msg::Float64::SharedPtr msg) { on_joint_cmd(pan_target_, *msg); });
            tilt_cmd_sub_ = create_subscription<std_msgs::msg::Float64>(
                tilt_cmd_topic_, 10,
                [this](const std_msgs::msg::Float64::SharedPtr msg) { on_joint_cmd(tilt_target_, *msg); });
            // The fleet publishes pan and tilt as two messages; collecting them for one
            // cycle sends a single absolute move instead of two
            bridge_timer_ = create_wall_timer(50ms, [this] { send_joint_target(); });
        }

        if (status_rate_ > 0.0)
        {
            joint_state_pub_ =
                create_publisher<sensor_msgs::msg::JointState>(joint_states_topic_, 10);
            status_timer_ = create_wall_timer(
                std::chrono::duration<double>(1.0 / status_rate_), [this] { poll_status(); });
        }

        RCLCPP_INFO(get_logger(), "PTZ control endpoint: %s", base_url_.c_str());
        RCLCPP_INFO(get_logger(),
                    "Joint bridge: %s / %s -> %s, %s (pan_sign %.0f, pan_offset %.1f deg, "
                    "tilt_sign %.0f, tilt_offset %.1f deg); feedback %s at %.1f Hz",
                    pan_cmd_topic_.empty() ? "off" : pan_cmd_topic_.c_str(),
                    tilt_cmd_topic_.empty() ? "off" : tilt_cmd_topic_.c_str(),
                    pan_joint_name_.c_str(), tilt_joint_name_.c_str(), mapping_.pan_sign,
                    mapping_.pan_offset_deg, mapping_.tilt_sign, mapping_.tilt_offset_deg,
                    status_rate_ > 0.0 ? joint_states_topic_.c_str() : "off", status_rate_);
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

    void on_joint_cmd(std::optional<double> &target, const std_msgs::msg::Float64 &msg)
    {
        target = msg.data;
        target_pending_ = true;
    }

    // Send the latest joint setpoints as one absolute move. An axis the fleet has not
    // commanded yet stays where the camera reports it; without a report the move waits.
    void send_joint_target()
    {
        if (!target_pending_)
        {
            return;
        }
        hk_camera::JointPosition joints;
        if (pan_target_)
        {
            joints.pan_rad = *pan_target_;
        }
        else if (measured_)
        {
            joints.pan_rad = hk_camera::isapi_to_joints(*measured_, mapping_).pan_rad;
        }
        else
        {
            return;
        }
        if (tilt_target_)
        {
            joints.tilt_rad = *tilt_target_;
        }
        else if (measured_)
        {
            joints.tilt_rad = hk_camera::isapi_to_joints(*measured_, mapping_).tilt_rad;
        }
        else
        {
            return;
        }
        target_pending_ = false;

        // Keep the zoom the camera is at; absoluteZoom is required in the request
        const double zoom = measured_ ? measured_->zoom : 1.0;
        const auto position = hk_camera::joints_to_isapi(joints, mapping_, zoom);
        if (put(base_url_ + "/absolute", absolute_body(position)))
        {
            RCLCPP_INFO(get_logger(),
                        "Joint move: pan %.3f rad, tilt %.3f rad -> azimuth %.1f deg, "
                        "elevation %.1f deg",
                        joints.pan_rad, joints.tilt_rad, position.azimuth_deg,
                        position.elevation_deg);
        }
    }

    // Ask the camera where it is and publish that as joint positions
    void poll_status()
    {
        std::string response;
        const long status = perform("GET", base_url_ + "/status", "", response);
        if (status != 200)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "PTZ status request failed (HTTP %ld); no joint feedback",
                                 status);
            return;
        }
        const auto position = hk_camera::parse_ptz_status(response);
        if (!position)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "Could not read AbsoluteHigh from the PTZ status: %s",
                                 response.c_str());
            return;
        }
        measured_ = *position;
        const auto joints = hk_camera::isapi_to_joints(*position, mapping_);

        sensor_msgs::msg::JointState msg;
        msg.header.stamp = now();
        msg.name = {pan_joint_name_, tilt_joint_name_};
        msg.position = {joints.pan_rad, joints.tilt_rad};
        joint_state_pub_->publish(msg);
    }

    static std::string absolute_body(const hk_camera::IsapiPosition &position)
    {
        // ISAPI works in units of 0.1 degrees: azimuth 0~3600, elevation -900~900;
        // zoom is 1~1000 for a 0.1x~100x multiplier
        const long azimuth = std::lround(position.azimuth_deg * 10.0) % 3600;
        const long elevation = std::clamp(std::lround(position.elevation_deg * 10.0), -900L, 900L);
        const long zoom = std::clamp(std::lround(position.zoom * 10.0), 10L, 1000L);
        return "<?xml version=\"1.0\" encoding=\"UTF-8\"?>"
               "<PTZData><AbsoluteHigh>"
               "<elevation>" + std::to_string(elevation) + "</elevation>"
               "<azimuth>" + std::to_string(azimuth) + "</azimuth>"
               "<absoluteZoom>" + std::to_string(zoom) + "</absoluteZoom>"
               "</AbsoluteHigh></PTZData>";
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
        const long status = perform("PUT", url, body, response);
        if (status < 0)
        {
            RCLCPP_ERROR(get_logger(), "Request to %s failed: %s", url.c_str(), response.c_str());
            return false;
        }
        if (status != 200)
        {
            RCLCPP_ERROR(get_logger(), "Camera returned HTTP %ld: %s", status, response.c_str());
            return false;
        }
        return true;
    }

    // One ISAPI request. Returns the HTTP status, or -1 when the request did not complete
    // (response then holds curl's error).
    long perform(const char *method, const std::string &url, const std::string &body,
                 std::string &response)
    {
        curl_slist *headers = curl_slist_append(nullptr, "Content-Type: application/xml");

        curl_easy_reset(curl_);
        curl_easy_setopt(curl_, CURLOPT_URL, url.c_str());
        if (std::string(method) == "GET")
        {
            curl_easy_setopt(curl_, CURLOPT_HTTPGET, 1L);
        }
        else
        {
            curl_easy_setopt(curl_, CURLOPT_CUSTOMREQUEST, method);
            curl_easy_setopt(curl_, CURLOPT_POSTFIELDS, body.c_str());
            curl_easy_setopt(curl_, CURLOPT_POSTFIELDSIZE, static_cast<long>(body.size()));
        }
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
            response = curl_easy_strerror(code);
            return -1;
        }
        return status;
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
    std::string pan_cmd_topic_;
    std::string tilt_cmd_topic_;
    std::string pan_joint_name_;
    std::string tilt_joint_name_;
    std::string joint_states_topic_;
    double status_rate_;
    hk_camera::JointMapping mapping_;

    CURL *curl_{nullptr};
    std::optional<double> pan_target_;
    std::optional<double> tilt_target_;
    bool target_pending_{false};
    std::optional<hk_camera::IsapiPosition> measured_;
    int last_pan_{0};
    int last_tilt_{0};
    int last_zoom_{0};
    bool moving_{false};
    rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr preset_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr absolute_sub_;
    rclcpp::TimerBase::SharedPtr watchdog_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr pan_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr tilt_cmd_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::TimerBase::SharedPtr bridge_timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PtzNode>());
    rclcpp::shutdown();
    return 0;
}
