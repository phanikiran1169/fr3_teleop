#!/usr/bin/env python3
# joystick_teleop.py: PS5 DualSense → MoveIt Servo bridge for the FR3.
# Subscribes to /joy and republishes as TwistStamped + JointJog at 20 Hz.

# PS5 DualSense → /joy mapping (captured 2026-04-28 with ros-humble-joy 3.3.0):
#
# AXES (8)
#   axes[0]  Left stick X       right = -, left = +
#   axes[1]  Left stick Y       forward = +, back = -
#   axes[2]  L2 trigger         idle +1 → press -1
#   axes[3]  Right stick X      right = -, left = +
#   axes[4]  Right stick Y      forward = +, back = -
#   axes[5]  R2 trigger         idle +1 → press -1
#   axes[6]  D-pad X            right = -, left = +
#   axes[7]  D-pad Y            up = +, down = -
#
# BUTTONS (13)
#   buttons[0]   Cross
#   buttons[1]   Circle
#   buttons[2]   Triangle
#   buttons[3]   Square
#   buttons[4]   L1
#   buttons[5]   R1
#   buttons[6]   L2 click
#   buttons[7]   R2 click
#   buttons[8]   Create
#   buttons[9]   Options
#   buttons[10]  PS
#   buttons[11]  L3
#   buttons[12]  R3
#
# Function mapping
#   Left stick    → translate X (Y axis) and Y (X axis) in fr3_link0
#   Right stick   → rotate Z (yaw) and Y (pitch)
#   L2 / R2       → rotate -X / +X (roll, analog)
#   L1 / R1       → translate -Z / +Z
#   D-pad X       → joint 1 jog (base rotation)
#   D-pad Y       → joint 7 jog (wrist roll)
#   Cross         → toggle gripper (open ↔ close)
#   Circle        → home pose (planned via move_group)
#   Triangle      → speed +
#   Square        → speed -
#   Create        → e-stop (calls /servo_node/stop_servo)
#   Options       → e-stop (alternate)
#   PS            → resume (calls /servo_node/start_servo)

import math
import sys
import threading
import time

import rclpy
from control_msgs.msg import JointJog
from geometry_msgs.msg import TwistStamped
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger

try:
    from franka_msgs.action import Grasp, Move
    GRIPPER_AVAILABLE = True
except ImportError:
    GRIPPER_AVAILABLE = False

try:
    from pymoveit2 import MoveIt2, MoveIt2State
    MOVEIT_AVAILABLE = True
except ImportError:
    MOVEIT_AVAILABLE = False


# Axis indices.
AX_L_X = 0
AX_L_Y = 1
AX_L2 = 2
AX_R_X = 3
AX_R_Y = 4
AX_R2 = 5
AX_D_X = 6
AX_D_Y = 7

# Button indices.
BTN_CROSS = 0
BTN_CIRCLE = 1
BTN_TRIANGLE = 2
BTN_SQUARE = 3
BTN_L1 = 4
BTN_R1 = 5
BTN_CREATE = 8
BTN_OPTIONS = 9
BTN_PS = 10

# Frame and joints.
PLANNING_FRAME = 'fr3_link0'
END_EFFECTOR_LINK = 'fr3_link8'
ARM_GROUP = 'fr3_arm'
JOINT_NAMES_ARM = ['fr3_joint1', 'fr3_joint2', 'fr3_joint3',
                   'fr3_joint4', 'fr3_joint5', 'fr3_joint6', 'fr3_joint7']
JOINT_NAMES_JOG = ['fr3_joint1', 'fr3_joint7']

# SRDF "ready" pose for FR3, matching MoveIt2's panda/franka convention.
HOME_POSITIONS = [0.0, -math.pi / 4, 0.0, -3 * math.pi / 4,
                  0.0, math.pi / 2, math.pi / 4]
HOME_VEL_SCALE = 0.1
HOME_TIMEOUT_S = 30.0

# Servo command topics (match fr3_teleop's servo_node).
TWIST_TOPIC = '/servo_node/delta_twist_cmds'
JOG_TOPIC = '/servo_node/delta_joint_cmds'
START_SERVO_SVC = '/servo_node/start_servo'
STOP_SERVO_SVC = '/servo_node/stop_servo'

# Gripper actions (franka_gripper).
GRIPPER_MOVE_ACTION = '/fr3_gripper/move'
GRIPPER_GRASP_ACTION = '/fr3_gripper/grasp'
GRIPPER_OPEN_WIDTH = 0.08      # m, full open for Franka Hand
GRIPPER_CLOSE_WIDTH = 0.0      # m
GRIPPER_SPEED = 0.1            # m/s
GRIPPER_GRASP_FORCE = 20.0     # N
GRIPPER_GRASP_TOLERANCE = 0.01  # m

PUBLISH_HZ = 20.0
DEADZONE = 0.08
LINEAR_SPEED_LEVELS = [0.05, 0.1, 0.2, 0.3]   # m/s
ANGULAR_SPEED_LEVELS = [0.3, 0.5, 0.8, 1.2]   # rad/s
DEFAULT_SPEED_IDX = 1
JOINT_JOG_SCALE = 0.5
JOY_STALE_S = 0.5


def apply_deadzone(v, deadzone=DEADZONE):
    if abs(v) < deadzone:
        return 0.0
    return v


class JoystickTeleop(Node):
    def __init__(self):
        super().__init__('joystick_teleop')
        self._cb = ReentrantCallbackGroup()

        self._latest_joy = None
        self._latest_joy_time = None
        self._stale_warned = False
        self._prev_buttons = []

        self._speed_idx = DEFAULT_SPEED_IDX
        self._user_estop = False
        self._home_pause = False
        self._gripper_open_state = True
        self._home_in_progress = False

        self._twist_pub = self.create_publisher(TwistStamped, TWIST_TOPIC, 10)
        self._jog_pub = self.create_publisher(JointJog, JOG_TOPIC, 10)
        self.create_subscription(Joy, '/joy', self._on_joy, 10,
                                 callback_group=self._cb)

        self._start_servo = self.create_client(Trigger, START_SERVO_SVC,
                                               callback_group=self._cb)
        self._stop_servo = self.create_client(Trigger, STOP_SERVO_SVC,
                                              callback_group=self._cb)

        if GRIPPER_AVAILABLE:
            self._gripper_move = ActionClient(self, Move, GRIPPER_MOVE_ACTION,
                                              callback_group=self._cb)
            self._gripper_grasp = ActionClient(self, Grasp, GRIPPER_GRASP_ACTION,
                                               callback_group=self._cb)
        else:
            self.get_logger().warn(
                'franka_msgs not importable; gripper button disabled.')

        if MOVEIT_AVAILABLE:
            self._moveit = MoveIt2(
                node=self,
                joint_names=JOINT_NAMES_ARM,
                base_link_name=PLANNING_FRAME,
                end_effector_name=END_EFFECTOR_LINK,
                group_name=ARM_GROUP,
                callback_group=self._cb,
                use_move_group_action=True,
            )
            self._moveit.max_velocity = HOME_VEL_SCALE
            self._moveit.max_acceleration = HOME_VEL_SCALE
        else:
            self._moveit = None
            self.get_logger().warn(
                'pymoveit2 not importable; home button disabled.')

        # Servo's incoming_command_timeout (default 100 ms) decelerates the
        # arm if commands stop, so publish on every tick even when sticks
        # are at rest.
        self.create_timer(1.0 / PUBLISH_HZ, self._tick,
                          callback_group=self._cb)

        self.get_logger().info(
            f'joystick_teleop ready. Speed level {self._speed_idx} '
            f'(lin {LINEAR_SPEED_LEVELS[self._speed_idx]} m/s, '
            f'ang {ANGULAR_SPEED_LEVELS[self._speed_idx]} rad/s). '
            'Hold EAD before pressing PS to resume.')

        threading.Thread(target=self._auto_start_servo, daemon=True).start()

    def _auto_start_servo(self):
        if not self._start_servo.wait_for_service(timeout_sec=10.0):
            self.get_logger().warn(
                'start_servo service not available; call it manually.')
            return
        if self._user_estop or self._home_in_progress or self._home_pause:
            self.get_logger().info(
                'Auto-start cancelled (e-stop or home active).')
            return
        future = self._start_servo.call_async(Trigger.Request())
        deadline = time.monotonic() + 5.0
        while not future.done():
            if time.monotonic() >= deadline:
                self.get_logger().warn('start_servo did not respond in 5 s.')
                return
            time.sleep(0.05)
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().error(f'start_servo raised: {e}')
            return
        if self._user_estop or self._home_in_progress or self._home_pause:
            self.get_logger().info(
                'E-stop or home activated during auto-start; stopping servo.')
            if self._stop_servo.service_is_ready():
                self._stop_servo.call_async(Trigger.Request())
            return
        if result.success:
            self.get_logger().info('Servo started.')
        else:
            self.get_logger().warn(f'start_servo: {result.message}')

    def _on_joy(self, msg):
        self._latest_joy = msg
        self._latest_joy_time = time.monotonic()
        if self._stale_warned:
            self.get_logger().info('/joy stream resumed.')
            self._stale_warned = False
        if len(self._prev_buttons) != len(msg.buttons):
            self._prev_buttons = [0] * len(msg.buttons)
        for i, val in enumerate(msg.buttons):
            if val and not self._prev_buttons[i]:
                self._on_button_pressed(i)
        self._prev_buttons = list(msg.buttons)

    def _on_button_pressed(self, idx):
        if idx == BTN_CROSS:
            self._gripper_toggle()
        elif idx == BTN_CIRCLE:
            self._go_home()
        elif idx == BTN_TRIANGLE:
            self._speed_change(+1)
        elif idx == BTN_SQUARE:
            self._speed_change(-1)
        elif idx in (BTN_CREATE, BTN_OPTIONS):
            self._estop_now()
        elif idx == BTN_PS:
            self._resume()

    def _speed_change(self, delta):
        new_idx = max(0, min(len(LINEAR_SPEED_LEVELS) - 1, self._speed_idx + delta))
        if new_idx == self._speed_idx:
            self.get_logger().info(
                f'Speed already at {"max" if delta > 0 else "min"} '
                f'(lin {LINEAR_SPEED_LEVELS[new_idx]} m/s, '
                f'ang {ANGULAR_SPEED_LEVELS[new_idx]} rad/s).')
            return
        self._speed_idx = new_idx
        self.get_logger().info(
            f'Speed level {self._speed_idx} '
            f'(lin {LINEAR_SPEED_LEVELS[self._speed_idx]} m/s, '
            f'ang {ANGULAR_SPEED_LEVELS[self._speed_idx]} rad/s).')

    def _estop_now(self):
        if self._user_estop:
            return
        self._user_estop = True
        self.get_logger().warn('E-STOP. Press PS to resume.')
        if self._stop_servo.service_is_ready():
            self._stop_servo.call_async(Trigger.Request())

    def _resume(self):
        if not self._user_estop:
            return
        if self._home_in_progress or self._home_pause:
            self._user_estop = False
            self.get_logger().info(
                'E-stop cleared; servo will resume after home completes.')
            return
        if self._start_servo.service_is_ready():
            future = self._start_servo.call_async(Trigger.Request())
            future.add_done_callback(self._on_resume_done)
        else:
            self.get_logger().warn('start_servo service not ready.')

    def _on_resume_done(self, future):
        try:
            result = future.result()
            if result.success:
                self._user_estop = False
                self.get_logger().info('Servo resumed.')
            else:
                self.get_logger().error(f'Resume failed: {result.message}')
        except Exception as e:
            self.get_logger().error(f'Resume call raised: {e}')

    def _send_gripper_goal(self, action_client, goal, label):
        send_future = action_client.send_goal_async(goal)

        def _on_goal_response(fut):
            try:
                handle = fut.result()
            except Exception as e:
                self.get_logger().error(f'{label}: send_goal raised: {e}')
                return
            if not handle.accepted:
                self.get_logger().warn(f'{label}: goal rejected.')
                return
            result_future = handle.get_result_async()
            result_future.add_done_callback(
                lambda f: self._on_gripper_result(f, label))

        send_future.add_done_callback(_on_goal_response)

    def _on_gripper_result(self, future, label):
        try:
            result = future.result().result
        except Exception as e:
            self.get_logger().error(f'{label}: result raised: {e}')
            return
        if getattr(result, 'success', False):
            self.get_logger().info(f'{label}: success.')
        else:
            err = getattr(result, 'error', '')
            self.get_logger().warn(f'{label}: failed ({err}).')

    def _gripper_open(self):
        if not GRIPPER_AVAILABLE:
            return
        if not self._gripper_move.server_is_ready():
            self.get_logger().warn('gripper move action not ready.')
            return
        goal = Move.Goal()
        goal.width = GRIPPER_OPEN_WIDTH
        goal.speed = GRIPPER_SPEED
        self._send_gripper_goal(self._gripper_move, goal, 'gripper open')

    def _gripper_close(self):
        if not GRIPPER_AVAILABLE:
            return
        if not self._gripper_grasp.server_is_ready():
            self.get_logger().warn('gripper grasp action not ready.')
            return
        goal = Grasp.Goal()
        goal.width = GRIPPER_CLOSE_WIDTH
        goal.speed = GRIPPER_SPEED
        goal.force = GRIPPER_GRASP_FORCE
        goal.epsilon.inner = GRIPPER_GRASP_TOLERANCE
        goal.epsilon.outer = GRIPPER_GRASP_TOLERANCE
        self._send_gripper_goal(self._gripper_grasp, goal, 'gripper close')

    def _gripper_toggle(self):
        if self._gripper_open_state:
            self._gripper_close()
            self._gripper_open_state = False
        else:
            self._gripper_open()
            self._gripper_open_state = True

    def _go_home(self):
        if self._moveit is None:
            self.get_logger().warn(
                'pymoveit2 not available; home button has no effect.')
            return
        if self._home_in_progress:
            self.get_logger().warn('Home already in progress; ignoring.')
            return
        self._home_in_progress = True
        threading.Thread(target=self._go_home_blocking, daemon=True).start()

    def _go_home_blocking(self):
        try:
            # pymoveit2 leaks its execution mutex when /move_action isn't
            # ready, which would deadlock the drain loop on query_state.
            move_ac = self._moveit._MoveIt2__move_action_client
            if not move_ac.wait_for_server(timeout_sec=2.0):
                self.get_logger().error(
                    'Home: /move_action server not ready; '
                    'is move_group running?')
                return

            self.get_logger().info('Home: stopping servo.')
            if self._stop_servo.service_is_ready():
                self._stop_servo.call_async(Trigger.Request())
            self._home_pause = True

            self.get_logger().info('Home: planning + executing.')
            self._moveit.motion_suceeded = False
            self._moveit.move_to_configuration(HOME_POSITIONS)

            saw_non_idle = False
            deadline = time.monotonic() + HOME_TIMEOUT_S
            while True:
                state = self._moveit.query_state()
                if state != MoveIt2State.IDLE:
                    saw_non_idle = True
                if state == MoveIt2State.IDLE:
                    if not saw_non_idle:
                        self.get_logger().error('Home: goal never dispatched.')
                        return
                    if self._moveit.motion_suceeded:
                        self.get_logger().info('Home: reached.')
                    else:
                        self.get_logger().error('Home: execution failed.')
                    return
                if time.monotonic() >= deadline:
                    self.get_logger().error(
                        f'Home: exceeded {HOME_TIMEOUT_S} s; cancelling.')
                    try:
                        self._moveit.cancel_execution()
                    except Exception:
                        pass
                    return
                time.sleep(0.05)
        finally:
            self._home_in_progress = False
            self._home_pause = False
            if not self._user_estop and self._start_servo.service_is_ready():
                self._start_servo.call_async(Trigger.Request())
                self.get_logger().info('Home: servo resumed.')

    def _tick(self):
        lin = LINEAR_SPEED_LEVELS[self._speed_idx]
        ang = ANGULAR_SPEED_LEVELS[self._speed_idx]
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = PLANNING_FRAME

        jog = JointJog()
        jog.header.stamp = twist.header.stamp
        jog.joint_names = list(JOINT_NAMES_JOG)
        jog.velocities = [0.0, 0.0]

        joy_fresh = (self._latest_joy is not None
                     and self._latest_joy_time is not None
                     and (time.monotonic() - self._latest_joy_time) < JOY_STALE_S)
        if (self._latest_joy is not None
                and not joy_fresh
                and not self._stale_warned):
            self.get_logger().warn(
                f'/joy stale (>{JOY_STALE_S}s). Holding zero twist until it resumes.')
            self._stale_warned = True

        if joy_fresh and not (self._user_estop or self._home_pause):
            ax = self._latest_joy.axes
            btn = self._latest_joy.buttons
            if len(ax) >= 8 and len(btn) >= 11:
                tx = apply_deadzone(ax[AX_L_Y]) * lin
                ty = apply_deadzone(ax[AX_L_X]) * lin
                tz = (btn[BTN_R1] - btn[BTN_L1]) * lin

                rz = apply_deadzone(ax[AX_R_X]) * ang
                ry = apply_deadzone(ax[AX_R_Y]) * ang
                # Triggers idle at +1, full press at -1.
                l2 = (1.0 - ax[AX_L2]) / 2.0
                r2 = (1.0 - ax[AX_R2]) / 2.0
                rx = (r2 - l2) * ang

                twist.twist.linear.x = tx
                twist.twist.linear.y = ty
                twist.twist.linear.z = tz
                twist.twist.angular.x = rx
                twist.twist.angular.y = ry
                twist.twist.angular.z = rz

                jog.velocities = [
                    apply_deadzone(ax[AX_D_X]) * JOINT_JOG_SCALE,
                    apply_deadzone(ax[AX_D_Y]) * JOINT_JOG_SCALE,
                ]

        self._twist_pub.publish(twist)
        self._jog_pub.publish(jog)


def main():
    rclpy.init()
    node = JoystickTeleop()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
