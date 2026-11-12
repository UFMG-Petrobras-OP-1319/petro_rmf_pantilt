# Scout inspection return and zoom

The Scout DS-2DE2A404IW-DE3 accepted the extended ISAPI absolute endpoint in the
approved 2026-11-11 camera-only checks. GET absoluteEx/capabilities reported pan
0..355 degrees, elevation 0..90 degrees and direct zoom 1..64. GET absoluteEx
returned zoom 16 while the legacy status returned 160. An extended command to
pan 160 preserved zoom 16; subsequent operator-requested adjustments reached
175/15/1, 165/15/1 and 160/15/1. These are camera coordinates, not a chassis
calibration. The operator confirmed that raw 0/0 points toward the rear.

The legacy absolute endpoint changed zoom during the initial zero-position test
despite copying its status value. Its exact clamp/firmware behavior remains
unresolved. Do not use that endpoint for this Scout's automatic return.

Set `joint_absolute_api: extended` for this camera. The default remains legacy
for compatibility with deployments whose firmware has not been checked.
Scout's companion launch patch selects extended explicitly. No automatic fallback
is attempted if an extended request fails.

Extended joint commands require a fresh measured position (at most one second
old). They preserve the measured direct zoom. A target without fresh feedback
is refused, rather than held for an eventual reconnect.

For an inspection return that also restores a particular zoom, the new topic
`~/joint_absolute` accepts geometry_msgs/Vector3: x and y are pan/tilt JOINT
RADIANS after the configured sign/offset mapping; z is the direct zoom value.
The driver resolves both axes and zoom into one extended API request. It clears
pending split-axis targets so an old bridge-timer command cannot overwrite it.

The new stamped feedback topic `~/joint_position` reports the same joint radians
and direct zoom. Its message type is geometry_msgs/Vector3Stamped. This is a PTZ
state tuple, not a spatial translation vector. Existing JointState feedback and
pan_camera_cmd/tilt_camera_cmd interfaces remain available.

The picture_taker companion can use these atomic command/feedback topics to
confirm both the resting direction and zoom before returning a successful result.
Its return_to_forward and forward_pose_calibrated flags remain false until the
operator explicitly selects the home pose. No startup movement is added.

The patched executable is built in an isolated /tmp directory inside the robot
container; it is not started or installed over the working runtime. The camera
checks exercise the API directly, so a full driver/capture lifecycle hardware
test still requires approval after the home pose is selected.
