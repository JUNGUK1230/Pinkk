# Pinkk MyCobot ROS2 bridge

For the current setup status and the exact next-session procedure, see
[`HAND_EYE_HANDOFF_KO.md`](HAND_EYE_HANDOFF_KO.md).

The first integration stage is deliberately read-only. The robot PC reads
`MyCobot280.get_angles()` and publishes `/joint_states`; it never calls a motion
API. MoveIt and RViz run on the laptop without fake ros2_control hardware.

Use the same network settings on both machines:

```bash
export ROS_DOMAIN_ID=36
export ROS_LOCALHOST_ONLY=0
```

Robot PC:

```bash
ros2 launch pinkk_mycobot_bridge joint_state_bridge.launch.py
```

Laptop:

```bash
ros2 launch pinkk_mycobot_bridge planning_only.launch.py
```

Do not run `sync_plan`, `sync_plan_arduino`, Jupyter robot code, or another
program that opens `/dev/ttyUSB0` at the same time as the bridge.

## ChArUco target TF

The camera node uses the existing 11x8 calib.io board settings and publishes
only valid detections (at least 35 corners and at most 0.7 px reprojection
error) as `camera_optical_frame -> charuco_board`.

```bash
ros2 launch pinkk_mycobot_bridge charuco_tf_bridge.launch.py
```

The OpenCV camera coordinate convention is the ROS optical-frame convention:
x right, y down, z forward. Do not rename the parent frame to `camera_link`
unless that frame is also defined with optical axes.
