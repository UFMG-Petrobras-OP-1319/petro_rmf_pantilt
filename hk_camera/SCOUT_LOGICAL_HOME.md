# Scout PTZ logical home

This driver keeps Hikvision firmware coordinates unchanged. For Scout 1, the
robot launch supplies pan/tilt signs `+1/+1`, raw offsets `160°/15°`, and raw
capability limits pan `0..355°`, elevation `0..90°`, and direct zoom `1..64`.
Logical home remains `(pan=0 rad, tilt=0 rad, zoom=1x)`.

The same mapping converts commands and measured feedback. Targets outside camera
limits are rejected; elevation is never silently clamped. Extended `absoluteEx`
requests serialize direct zoom multipliers and never fall back to legacy
`absolute`. The atomic `~/joint_absolute` tuple is logical pan radians, logical
tilt radians, and direct zoom. The stamped `~/joint_position` tuple publishes
those same values as feedback; its `Vector3` is a PTZ tuple, not a Cartesian
translation.

A `~/return_home` `std_srvs/Trigger` request queues the saved home parameters
(`home_pan_rad`, `home_tilt_rad`, `home_zoom`) and replies once accepted. It does
not wait for movement. The caller must monitor fresh stamped `~/joint_position`
feedback until pan, tilt, and zoom settle. Nothing invokes this service on node
startup, reconnect, or installation. The service requires fresh camera feedback,
checks the configured capability range, and rejects the mechanical pan wrap gap.

Do not use the camera's “Back to origin” preset, rewrite firmware origin, or use
firmware raw `(0,0)` as the software home. The selected 160°/15° pose is a
resting view; it is not a chassis-level calibration. No camera commands are sent
by these source changes.
