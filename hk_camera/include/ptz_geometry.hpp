// Conversions between the gimbal's joint angles and HIKVISION ISAPI PTZ positions.
//
// The fleet and the robot controller talk about the gimbal in joint space, the same as
// the simulated robot model: pan_joint and tilt_joint in radians. ISAPI talks about it in
// degrees: azimuth 0~360 and elevation -90~90, both in the camera's own frame. How the
// two relate depends on how the camera is mounted (which way azimuth grows, where its
// zero points), so the relation is one sign and one offset per axis, read from the
// node parameters and calibrated on the robot:
//
//   azimuth   = pan_sign  * deg(pan_joint)  + pan_offset_deg     (wrapped to 0~360)
//   elevation = tilt_sign * deg(tilt_joint) + tilt_offset_deg    (clamped to -90~90)
//
// Kept free of ROS and curl so it can be unit-tested on its own.

#pragma once

#include <algorithm>
#include <cmath>
#include <optional>
#include <string>

namespace hk_camera
{

struct JointMapping
{
    double pan_sign{1.0};
    double pan_offset_deg{0.0};
    double tilt_sign{1.0};
    double tilt_offset_deg{0.0};
};

struct IsapiPosition
{
    double azimuth_deg{0.0};
    double elevation_deg{0.0};
    double zoom{1.0};
};

struct JointPosition
{
    double pan_rad{0.0};
    double tilt_rad{0.0};
};

inline double deg_to_rad(double deg) { return deg * M_PI / 180.0; }
inline double rad_to_deg(double rad) { return rad * 180.0 / M_PI; }

// Wrap an angle in degrees to [0, 360)
inline double wrap_360(double deg)
{
    const double wrapped = std::fmod(deg, 360.0);
    return wrapped < 0.0 ? wrapped + 360.0 : wrapped;
}

// Wrap an angle in radians to (-pi, pi]
inline double wrap_pi(double rad)
{
    const double wrapped = std::remainder(rad, 2.0 * M_PI);
    return wrapped <= -M_PI ? wrapped + 2.0 * M_PI : wrapped;
}

inline IsapiPosition joints_to_isapi(const JointPosition &joints, const JointMapping &mapping,
                                     double zoom)
{
    IsapiPosition position;
    position.azimuth_deg = wrap_360(mapping.pan_sign * rad_to_deg(joints.pan_rad) +
                                    mapping.pan_offset_deg);
    position.elevation_deg = std::clamp(
        mapping.tilt_sign * rad_to_deg(joints.tilt_rad) + mapping.tilt_offset_deg, -90.0, 90.0);
    position.zoom = zoom;
    return position;
}

// The signs are +-1, so each one is its own inverse
inline JointPosition isapi_to_joints(const IsapiPosition &position, const JointMapping &mapping)
{
    JointPosition joints;
    joints.pan_rad =
        wrap_pi(deg_to_rad(mapping.pan_sign * (position.azimuth_deg - mapping.pan_offset_deg)));
    joints.tilt_rad =
        deg_to_rad(mapping.tilt_sign * (position.elevation_deg - mapping.tilt_offset_deg));
    return joints;
}

// Integer value of the first <tag>...</tag> at or after `from`, if there is one
inline std::optional<long> xml_integer(const std::string &xml, const std::string &tag,
                                       std::size_t from = 0)
{
    const std::string open = "<" + tag + ">";
    const std::size_t start = xml.find(open, from);
    if (start == std::string::npos)
    {
        return std::nullopt;
    }
    const std::size_t value_start = start + open.size();
    const std::size_t end = xml.find("</" + tag + ">", value_start);
    if (end == std::string::npos)
    {
        return std::nullopt;
    }
    try
    {
        std::size_t used = 0;
        const std::string text = xml.substr(value_start, end - value_start);
        const long value = std::stol(text, &used);
        if (used != text.size())
        {
            return std::nullopt;
        }
        return value;
    }
    catch (const std::exception &)
    {
        return std::nullopt;
    }
}

// Read the position out of a GET /ISAPI/PTZCtrl/channels/<n>/status reply:
//   <PTZStatus><AbsoluteHigh><elevation>..</elevation><azimuth>..</azimuth>
//   <absoluteZoom>..</absoluteZoom></AbsoluteHigh></PTZStatus>
// Angles come in tenths of a degree and zoom in tenths of the multiplier.
inline std::optional<IsapiPosition> parse_ptz_status(const std::string &xml)
{
    const std::size_t block = xml.find("<AbsoluteHigh>");
    if (block == std::string::npos)
    {
        return std::nullopt;
    }
    const auto elevation = xml_integer(xml, "elevation", block);
    const auto azimuth = xml_integer(xml, "azimuth", block);
    const auto zoom = xml_integer(xml, "absoluteZoom", block);
    if (!elevation || !azimuth)
    {
        return std::nullopt;
    }
    IsapiPosition position;
    position.elevation_deg = *elevation / 10.0;
    position.azimuth_deg = *azimuth / 10.0;
    position.zoom = zoom ? *zoom / 10.0 : 1.0;
    return position;
}

} // namespace hk_camera
