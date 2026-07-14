# Pinkk MyCobot ROS2 bridge

The first integration stage is deliberately read-only. The robot PC reads
`MyCobot280.get_angles()` and publishes `/joint_states`; it never calls a motion
API. MoveIt and RViz run on the laptop without fake ros2_control hardware.

Use the same network settings on both machines:

```bash
export ROS_DOMAIN_ID=42
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
