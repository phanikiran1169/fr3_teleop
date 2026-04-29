# fr3_teleop

ROS 2 package for FR3 teleoperation via MoveIt Servo. Supports keyboard input (`teleop_twist_keyboard`) and PS5 DualSense input (custom node). Runs on the inference laptop as part of the 2-PC deployment architecture.

## Contents

| File | Purpose |
|------|---------|
| `config/servo_config.yaml` | MoveIt Servo parameters (FR3 frames, scaling, topics) |
| `launch/teleop.launch.py` | Launches `servo_node` and `robot_state_publisher` |
| `launch/move_group.launch.py` | Inference-PC `move_group` for the home-pose button |
| `launch/joystick.launch.py` | One-command bring-up for PS5 teleop |
| `fr3_teleop/joystick_teleop.py` | PS5 DualSense → MoveIt Servo bridge |

## Build

```bash
cd ~/franka_ws
colcon build --packages-select fr3_teleop
```

## Dependencies

- `ros-humble-moveit-servo`
- `ros-humble-teleop-twist-keyboard`
- `ros-humble-joy`
- `ros-humble-pymoveit2`
- `franka_description` 0.3.0 (source-built)
- `franka_fr3_moveit_config` (source-built, from franka_ros2 v0.1.15)
- `franka_msgs` (source-built, from franka_ros2 v0.1.15) — gripper actions

## Configuration

`config/servo_config.yaml` is adapted from [MoveIt Servo's Panda example](https://github.com/moveit/moveit2/blob/humble/moveit_ros/moveit_servo/config/panda_simulated_config.yaml). Key FR3-specific values:

| Parameter | Value | Notes |
|-----------|-------|-------|
| `move_group_name` | `fr3_arm` | From FR3 SRDF |
| `planning_frame` | `fr3_link0` | FR3 base frame |
| `ee_frame_name` | `fr3_link8` | FR3 end-effector frame |
| `command_out_topic` | `/fr3_arm_controller/joint_trajectory` | Matches controller from `fr3_ros_controllers.yaml` |
| `linear_scale` / `rotational_scale` | `0.1` | Conservative for safety |

## PS5 joystick teleop

Single-command bring-up:

```bash
ros2 launch fr3_teleop joystick.launch.py
```

This launches `teleop.launch.py` (servo + RSP), `move_group.launch.py` (planner for the home-pose button), `joy_node`, and `joystick_teleop`. The teleop node auto-calls `/servo_node/start_servo` once the service is available.

Disable `move_group` (and the home-pose button) with `with_move_group:=false`. Select a different `/dev/input/js<N>` with `joy_dev_id:=<N>`.

### PS5 button mapping

Sticks and triggers (continuous):

| Input | Action |
|---|---|
| Left stick | Translate X (forward/back) and Y (left/right) in `fr3_link0` |
| Right stick | Rotate yaw (X axis) and pitch (Y axis) |
| L2 / R2 | Roll (analog, left/right) |
| L1 / R1 | Translate Z down / up |
| D-pad X / Y | Joint 1 jog / joint 7 jog |

Buttons (discrete):

| Button | Action |
|---|---|
| Cross | Toggle gripper (open ↔ close) |
| Circle | Plan and execute home pose via `move_group` |
| Triangle | Speed level + |
| Square | Speed level − |
| Create | E-stop |
| Options | E-stop |
| PS | Resume after e-stop |

Speed levels (4 steps): linear `[0.05, 0.1, 0.2, 0.3]` m/s; angular `[0.3, 0.5, 0.8, 1.2]` rad/s. Default is level 1.

## References

- [MoveIt Servo tutorial (Humble)](https://moveit.picknik.ai/humble/doc/examples/realtime_servo/realtime_servo_tutorial.html)
- [MoveIt Servo parameters](https://moveit.picknik.ai/humble/api/html/servo__parameters_8h_source.html)
