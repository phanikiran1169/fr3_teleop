# fr3_teleop

ROS 2 package for FR3 keyboard teleoperation via MoveIt Servo. Runs on the inference laptop as part of the 2-PC deployment architecture.

## Contents

| File | Purpose |
|------|---------|
| `config/servo_config.yaml` | MoveIt Servo parameters (FR3 frames, scaling, topics) |
| `launch/teleop.launch.py` | Launches `servo_node` and `robot_state_publisher` |

## Build

```bash
cd ~/franka_ws
colcon build --packages-select fr3_teleop
```

## Dependencies

- `ros-humble-moveit-servo`
- `ros-humble-teleop-twist-keyboard`
- `franka_description` 0.3.0 (source-built)
- `franka_fr3_moveit_config` (source-built, from franka_ros2 v0.1.15)

## Configuration

`config/servo_config.yaml` is adapted from [MoveIt Servo's Panda example](https://github.com/moveit/moveit2/blob/humble/moveit_ros/moveit_servo/config/panda_simulated_config.yaml). Key FR3-specific values:

| Parameter | Value | Notes |
|-----------|-------|-------|
| `move_group_name` | `fr3_arm` | From FR3 SRDF |
| `planning_frame` | `fr3_link0` | FR3 base frame |
| `ee_frame_name` | `fr3_link8` | FR3 end-effector frame |
| `command_out_topic` | `/fr3_arm_controller/joint_trajectory` | Matches controller from `fr3_ros_controllers.yaml` |
| `linear_scale` / `rotational_scale` | `0.1` | Conservative for safety |

## References

- [MoveIt Servo tutorial (Humble)](https://moveit.picknik.ai/humble/doc/examples/realtime_servo/realtime_servo_tutorial.html)
- [MoveIt Servo parameters](https://moveit.picknik.ai/humble/api/html/servo__parameters_8h_source.html)
