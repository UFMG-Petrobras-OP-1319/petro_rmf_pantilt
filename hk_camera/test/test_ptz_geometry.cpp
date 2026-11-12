#include <gtest/gtest.h>
#include <limits>

#include "ptz_geometry.hpp"

using hk_camera::IsapiPosition;
using hk_camera::JointMapping;
using hk_camera::JointPosition;

TEST(PtzGeometry, IdentityMappingConvertsDegrees)
{
    const auto position = hk_camera::joints_to_isapi({M_PI / 4.0, M_PI / 6.0}, JointMapping{}, 1.0);
    EXPECT_NEAR(position.azimuth_deg, 45.0, 1e-9);
    EXPECT_NEAR(position.elevation_deg, 30.0, 1e-9);
}

TEST(PtzGeometry, NegativePanWrapsIntoTheIsapiRange)
{
    const auto position = hk_camera::joints_to_isapi({-M_PI / 2.0, 0.0}, JointMapping{}, 1.0);
    EXPECT_NEAR(position.azimuth_deg, 270.0, 1e-9);
}

TEST(PtzGeometry, UnreachableElevationIsRejectedInsteadOfClamped)
{
    EXPECT_THROW(hk_camera::joints_to_isapi({0.0, 2.0}, JointMapping{}, 1.0),
                 std::out_of_range);
}

TEST(PtzGeometry, ScoutLogicalZeroMapsToApprovedRawHome)
{
    JointMapping mapping;
    mapping.pan_offset_deg = 160.0;
    mapping.tilt_offset_deg = 15.0;
    mapping.limits = {0.0, 355.0, 0.0, 90.0, 1.0, 64.0};
    const auto raw = hk_camera::joints_to_isapi({0.0, 0.0}, mapping, 1.0);
    EXPECT_NEAR(raw.azimuth_deg, 160.0, 1e-9);
    EXPECT_NEAR(raw.elevation_deg, 15.0, 1e-9);
    EXPECT_NEAR(raw.zoom, 1.0, 1e-9);
    const auto logical = hk_camera::isapi_to_joints(raw, mapping);
    EXPECT_NEAR(logical.pan_rad, 0.0, 1e-9);
    EXPECT_NEAR(logical.tilt_rad, 0.0, 1e-9);
}

TEST(PtzGeometry, ScoutMappingAppliesSignsAndRejectsUnreachableTilt)
{
    JointMapping mapping;
    mapping.pan_sign = -1.0;
    mapping.pan_offset_deg = 160.0;
    mapping.tilt_sign = -1.0;
    mapping.tilt_offset_deg = 15.0;
    mapping.limits = {0.0, 355.0, 0.0, 90.0, 1.0, 64.0};
    const auto raw = hk_camera::joints_to_isapi(
        {hk_camera::deg_to_rad(5.0), hk_camera::deg_to_rad(5.0)}, mapping, 16.0);
    EXPECT_NEAR(raw.azimuth_deg, 155.0, 1e-9);
    EXPECT_NEAR(raw.elevation_deg, 10.0, 1e-9);
    EXPECT_THROW(hk_camera::joints_to_isapi({0.0, hk_camera::deg_to_rad(16.0)}, mapping, 1.0),
                 std::out_of_range);
}

TEST(PtzGeometry, PanPathRejectsWrapAcrossMechanicalGap)
{
    const hk_camera::IsapiLimits limits{0.0, 355.0, 0.0, 90.0, 1.0, 64.0};
    EXPECT_TRUE(hk_camera::pan_path_is_safe(160.0, 165.0, limits));
    EXPECT_TRUE(hk_camera::pan_path_is_safe(350.0, 340.0, limits));
    EXPECT_FALSE(hk_camera::pan_path_is_safe(350.0, 10.0, limits));
    EXPECT_FALSE(hk_camera::pan_path_is_safe(10.0, 350.0, limits));
}

TEST(PtzGeometry, ScoutLimitsRejectPanElevationAndZoomOutsideCapability)
{
    const hk_camera::IsapiLimits limits{0.0, 355.0, 0.0, 90.0, 1.0, 64.0};
    EXPECT_FALSE(hk_camera::valid_isapi_position({356.0, 15.0, 1.0}, limits));
    EXPECT_FALSE(hk_camera::valid_isapi_position({160.0, -0.1, 1.0}, limits));
    EXPECT_FALSE(hk_camera::valid_isapi_position({160.0, 15.0, 0.9}, limits));
    EXPECT_TRUE(hk_camera::valid_isapi_position({160.0, 15.0, 64.0}, limits));
}

TEST(PtzGeometry, RoundTripWithSignsAndOffsets)
{
    JointMapping mapping;
    mapping.pan_sign = -1.0;
    mapping.pan_offset_deg = 12.5;
    mapping.tilt_sign = -1.0;
    mapping.tilt_offset_deg = 15.0;
    for (double pan : {-3.0, -1.2, 0.0, 0.7, 3.0})
    {
        for (double tilt : {-0.26, 0.0, 0.5, 1.2})
        {
            const auto position = hk_camera::joints_to_isapi({pan, tilt}, mapping, 1.0);
            const auto joints = hk_camera::isapi_to_joints(position, mapping);
            EXPECT_NEAR(joints.pan_rad, pan, 1e-9) << "pan " << pan;
            EXPECT_NEAR(joints.tilt_rad, tilt, 1e-9) << "tilt " << tilt;
        }
    }
}

TEST(PtzGeometry, PanReadBackIsWrappedToPlusMinusPi)
{
    const auto joints = hk_camera::isapi_to_joints({350.0, 0.0, 1.0}, JointMapping{});
    EXPECT_NEAR(joints.pan_rad, hk_camera::deg_to_rad(-10.0), 1e-9);
}

TEST(PtzGeometry, ParsesTheStatusReply)
{
    const std::string xml =
        "<?xml version=\"1.0\" encoding=\"UTF-8\"?>"
        "<PTZStatus version=\"2.0\" xmlns=\"http://www.hikvision.com/ver20/XMLSchema\">"
        "<AbsoluteHigh><elevation>-150</elevation><azimuth>2705</azimuth>"
        "<absoluteZoom>20</absoluteZoom></AbsoluteHigh></PTZStatus>";
    const auto position = hk_camera::parse_ptz_status(xml);
    ASSERT_TRUE(position.has_value());
    EXPECT_NEAR(position->elevation_deg, -15.0, 1e-9);
    EXPECT_NEAR(position->azimuth_deg, 270.5, 1e-9);
    EXPECT_NEAR(position->zoom, 2.0, 1e-9);
}

TEST(PtzGeometry, RejectsAStatusWithoutAPositionOrZoom)
{
    EXPECT_FALSE(hk_camera::parse_ptz_status("<PTZStatus></PTZStatus>").has_value());
    const std::string status_without_zoom =
        "<AbsoluteHigh><elevation>0</elevation><azimuth>1600</azimuth></AbsoluteHigh>";
    EXPECT_TRUE(hk_camera::parse_ptz_status(status_without_zoom).has_value());
    EXPECT_FALSE(hk_camera::parse_ptz_status(status_without_zoom, true).has_value());
    EXPECT_FALSE(hk_camera::parse_ptz_status(
                     "<AbsoluteHigh><elevation>x</elevation><azimuth>10</azimuth></AbsoluteHigh>")
                     .has_value());
}

TEST(PtzGeometry, ExtendedCommandPreservesDirectZoomUnits)
{
    const auto measured = hk_camera::parse_ptz_status(
        "<AbsoluteHigh><elevation>0</elevation><azimuth>1600</azimuth>"
        "<absoluteZoom>160</absoluteZoom></AbsoluteHigh>");
    ASSERT_TRUE(measured);
    const std::string body = hk_camera::absolute_ex_body(*measured);
    EXPECT_NE(body.find("<azimuth>160</azimuth>"), std::string::npos);
    EXPECT_NE(body.find("<absoluteZoom>16</absoluteZoom>"), std::string::npos);
    EXPECT_EQ(body.find("<absoluteZoom>160</absoluteZoom>"), std::string::npos);
}

TEST(PtzGeometry, ExtendedCommandPreservesFractionalZoomAndAngles)
{
    const std::string body = hk_camera::absolute_ex_body({159.99, 0.1, 30.6});
    EXPECT_NE(body.find("<azimuth>159.99</azimuth>"), std::string::npos);
    EXPECT_NE(body.find("<elevation>0.1</elevation>"), std::string::npos);
    EXPECT_NE(body.find("<absoluteZoom>30.6</absoluteZoom>"), std::string::npos);
}

TEST(PtzGeometry, ExtendedCommandRejectsNonfiniteTargets)
{
    const double nan = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(hk_camera::absolute_ex_body({nan, 0.0, 16.0}), std::invalid_argument);
    EXPECT_THROW(hk_camera::absolute_ex_body({160.0, nan, 16.0}), std::invalid_argument);
    EXPECT_THROW(hk_camera::absolute_ex_body({160.0, 0.0, nan}), std::invalid_argument);
    EXPECT_THROW(hk_camera::absolute_ex_body({160.0, 0.0, 0.0}), std::invalid_argument);
}
