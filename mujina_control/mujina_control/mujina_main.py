# The MIT License (MIT)
#
# Copyright (c) 2024 Kento Kawaharazuka
# Copyright (c) 2025 CoRE-MA-KING
# Copyright (c) 2026 RT Corporation
#
# Permission is hereby granted, free of charge, to any person obtaining a copy of
# this software and associated documentation files (the "Software"), to deal in
# the Software without restriction, including without limitation the rights to
# use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
# the Software, and to permit persons to whom the Software is furnished to do so,
# subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
# FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
# COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
# IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
# CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

# mypy: disable-error-code=attr-defined
import argparse
import os
import sys
import threading
import time
from functools import partial
from typing import Literal, Tuple, Union  # noqa: F401

import mujoco
from mujina_control.mujina_utils.mujina_onnx import OnnxPredictor
import mujoco.viewer
import rclpy
import torch
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Twist
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Imu, JointState, Joy

from mujina_control.interface.robot_mode_command import RobotModeCommand
from mujina_control.motor_lib.motor_lib import CanMotorController
from mujina_msgs.msg import MotorState, RobotMode  # noqa: F401
from mujina_msgs.msg._motor_log import MotorLog

from .mujina_utils import mujina_utils
from .mujina_utils.parameters import parameters as P
# from .tmotor_lib.tmotor_lib import CanMotorController


# Current status of robot(motor)
class RobotState:
    def __init__(self, n_motor=12):
        self.angle = [0.0] * n_motor
        self.velocity = [0.0] * n_motor
        self.current = [0.0] * n_motor
        self.temperature = [0.0] * n_motor
        self.lock = threading.Lock()


# Peripheral state of robot(sensor)
class PeripheralState:
    def __init__(self):
        self.sensor_last_received_time = None
        self.body_quat = [0.0, 0.0, 0.0, 1.0]
        self.body_gyro = [0.0] * 3
        self.body_acc = [0.0] * 3
        self.control_enable = False
        self.control_command = [0.0] * 3
        self.lock = threading.Lock()


# Joint Command and Mode Management
class RobotCommand:
    def __init__(self, n_motor=12):
        self.angle = [0.0] * n_motor
        self.velocity = [0.0] * n_motor
        self.kp = []
        self.kd = []
        self.coef = 1.0
        for name in P.JOINT_NAME:
            for key in P.control.stiffness.keys():
                if key in name:
                    self.kp.append(P.control.stiffness[key] * self.coef)
                    self.kd.append(P.control.damping[key] * self.coef)
        assert len(self.kp) == n_motor
        assert len(self.kd) == n_motor
        self.torque = [0.0] * n_motor

        self.robot_mode = RobotModeCommand.STANDBY
        self.mode_transition_command = None

        self.initial_angle = [0.0] * n_motor
        self.final_angle = [0.0] * n_motor
        self.interpolating_time = 0.0
        self.remaining_time = 0.0
        self.initialized = False

        self.lock = threading.Lock()


# ############### function ##################


def command_callback(
    command: RobotModeCommand,
    robot_state: RobotState,
    robot_command: RobotCommand,
):
    print('command_callback')
    with robot_command.lock:
        robot_mode = robot_command.robot_mode
        if not robot_command.initialized:
            pass
    # Emergency stopは状態によらず即座に停止する
    if command == RobotModeCommand.EMERGENCY_STOP:
        with robot_command.lock:
            robot_command.robot_mode = RobotModeCommand.EMERGENCY_STOP
            robot_command.kp = [0.0] * 12
            robot_command.kd = [P.control.emergency_stop_damping] * 12
        return

    if robot_mode == RobotModeCommand.STANDBY:
        if command == RobotModeCommand.STANDUP:
            with robot_command.lock:
                # 前の遷移が完了している場合にのみ実行
                if robot_command.remaining_time < 0.1:
                    robot_command.robot_mode = (
                        RobotModeCommand.TRANSITION_TO_STANDUP
                    )
                    with robot_state.lock:
                        robot_command.initial_angle = robot_state.angle[:]
                        robot_command.final_angle = P.DEFAULT_ANGLE[:]
                        robot_command.interpolating_time = (
                            P.MODE_TRANSITION_TIME
                        )
                        robot_command.remaining_time = (
                            robot_command.interpolating_time
                        )
    elif robot_mode == RobotModeCommand.STANDUP:
        if command == RobotModeCommand.STANDBY:
            with robot_command.lock:
                # 前の遷移が完了している場合にのみ実行
                if robot_command.remaining_time < 0.1:
                    robot_command.robot_mode = (
                        RobotModeCommand.TRANSITION_TO_STANDBY
                    )
                    with robot_state.lock:
                        robot_command.initial_angle = robot_state.angle[:]
                        robot_command.final_angle = P.STANDBY_ANGLE[:]
                        robot_command.interpolating_time = (
                            P.MODE_TRANSITION_TIME
                        )
                        robot_command.remaining_time = (
                            robot_command.interpolating_time
                        )
        elif command == RobotModeCommand.WALK:
            with robot_command.lock:
                # 前の遷移が完了している場合にのみ実行
                if robot_command.remaining_time < 0.1:
                    # 現状WALKは即遷移
                    robot_command.robot_mode = RobotModeCommand.WALK
                    robot_command.interpolating_time = P.MODE_TRANSITION_TIME
                    robot_command.remaining_time = (
                        robot_command.interpolating_time
                    )
    elif robot_mode == RobotModeCommand.WALK:
        if command == RobotModeCommand.STANDUP:
            with robot_command.lock:
                # 前の遷移が完了している場合にのみ実行
                if robot_command.remaining_time < 0.1:
                    robot_command.robot_mode = (
                        RobotModeCommand.TRANSITION_TO_STANDUP
                    )
                    with robot_state.lock:
                        robot_command.initial_angle = robot_state.angle[:]
                        robot_command.final_angle = P.DEFAULT_ANGLE[:]
                        robot_command.interpolating_time = (
                            P.MODE_TRANSITION_TIME
                        )
                        robot_command.remaining_time = (
                            robot_command.interpolating_time
                        )


def fused_imu_callback(msg: Imu, params: PeripheralState):
    peripheral_state = params
    peripheral_state.sensor_last_received_time = time.time()
    quat = [
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w,
    ]
    with peripheral_state.lock:
        peripheral_state.body_quat = quat
        peripheral_state.body_gyro = [
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
        ]
        peripheral_state.body_acc = [
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            -msg.linear_acceleration.z,
        ]


class BaseNode(Node):
    def __init__(
        self,
        robot_state: RobotState,
        robot_command: RobotCommand,
        peripheral_state: PeripheralState,
    ):
        super().__init__('mujina_base_communication_node')
        # Create callback groups
        self.timer_callback_group = MutuallyExclusiveCallbackGroup()
        self.subscriber_callback_group = MutuallyExclusiveCallbackGroup()

        self.robot_state = robot_state
        self.robot_command = robot_command
        self.peripheral_state = peripheral_state

        with self.robot_state.lock:
            self.robot_state.angle = [float(x) for x in P.STANDBY_ANGLE]
            self.robot_state.velocity = [float(x) for x in P.STANDBY_ANGLE]
            self.robot_state.current = [float(x) for x in P.STANDBY_ANGLE]
            self.robot_state.temperature = [float(x) for x in P.STANDBY_ANGLE]

        with self.robot_command.lock:
            self.robot_command.robot_mode = RobotModeCommand.STANDBY
            self.robot_command.angle = [float(x) for x in P.STANDBY_ANGLE]
            self.robot_command.initial_angle = [
                float(x) for x in P.STANDBY_ANGLE
            ]
            self.robot_command.final_angle = [
                float(x) for x in P.STANDBY_ANGLE
            ]
            self.robot_command.interpolating_time = P.MODE_TRANSITION_TIME
            self.robot_command.remaining_time = (
                self.robot_command.interpolating_time
            )

        self.create_timer(
            0.1,  # 10Hz
            self.pub_robot_mode_timer_cb,
            callback_group=self.timer_callback_group,
        )
        self.create_timer(
            0.1,  # 10Hz
            self.pub_motor_state_timer_cb,
            callback_group=self.timer_callback_group,
        )

        self.robot_mode_pub = self.create_publisher(
            RobotMode, 'robot_mode', 10
        )
        self.motor_state_pub = self.create_publisher(
            MotorState, 'motor_state', 10
        )

        # Create subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            partial(self.cmd_vel_cb, params=(self.peripheral_state)),
            10,
            callback_group=self.subscriber_callback_group,
        )

        self.joint_states_pub = self.create_publisher(
            JointState, '/joint_states', 2
        )
        self.mujina_log_pub = self.create_publisher(
            MotorLog, '/motor_log', 2
        )
        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            partial(
                self.joy_callback,
                params=(self.peripheral_state, self.robot_command),
            ),
            1,
        )
        self.fused_imu_sub = self.create_subscription(
            Imu,
            'imu/data',
            partial(fused_imu_callback, params=(self.peripheral_state)),
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
        )

        # サブスクライバーを作成
        self.mode_transition_command_sub = self.create_subscription(
            RobotMode,  # メッセージの型
            'robot_mode_transition_command',  # トピック名
            partial(
                self.mode_transition_command_callback,
                params=(self.robot_state, self.robot_command),
            ),
            1,  # キューサイズ
        )

    def pub_motor_state_timer_cb(self):
        # Publish motor state
        motor_state = MotorState()
        motor_state.header.stamp = self.get_clock().now().to_msg()
        with self.robot_state.lock:
            motor_state.position = self.robot_state.angle
            motor_state.velocity = self.robot_state.velocity
            motor_state.current = self.robot_state.current
            motor_state.temperature = self.robot_state.temperature

        self.motor_state_pub.publish(motor_state)

    def pub_robot_mode_timer_cb(self):
        # Publish robot mode
        robot_mode = RobotMode()
        robot_mode.header.stamp = self.get_clock().now().to_msg()
        robot_mode.mode = self.robot_command.robot_mode.value

        self.robot_mode_pub.publish(robot_mode)

    def cmd_vel_cb(self, msg: Twist, params: PeripheralState):
        peripheral_state = params
        with peripheral_state.lock:
            peripheral_state.control_enable = True
            peripheral_state.control_command = [
                msg.linear.x,
                msg.linear.y,
                msg.angular.z,
            ]

    def joy_callback(
        self, msg: Joy, params: tuple[PeripheralState, RobotCommand]
    ):
        peripheral_state, robot_command = params
        with peripheral_state.lock:
            peripheral_state.control_enable = True
            peripheral_state.control_command = [
                msg.axes[1],
                msg.axes[0],
                msg.axes[3],
            ]
            one_pushed = msg.buttons[0]
            two_pushed = msg.buttons[1]
            three_pushed = msg.buttons[2]

        with robot_command.lock:
            robot_mode = robot_command.robot_mode
        if three_pushed == 1:
            if robot_mode != RobotModeCommand.EMERGENCY_STOP:
                self.get_logger().info('Emergency Stop!')
                command_callback(
                    RobotModeCommand.EMERGENCY_STOP,
                    self.robot_state,
                    robot_command,
                )
            return
        elif one_pushed == 1:
            if robot_mode == RobotModeCommand.STANDBY:
                self.get_logger().info('Changing mode from STANDBY to STANDUP')
                command_callback(
                    RobotModeCommand.STANDUP, self.robot_state, robot_command
                )
                return
            elif robot_mode == RobotModeCommand.STANDUP:
                self.get_logger().info('Changing mode from STANDUP to STANDBY')
                command_callback(
                    RobotModeCommand.STANDBY, self.robot_state, robot_command
                )
                return
        elif two_pushed == 1:
            if robot_mode == RobotModeCommand.STANDUP:
                self.get_logger().info('Changing mode from STANDUP to WALK')
                command_callback(
                    RobotModeCommand.WALK, self.robot_state, robot_command
                )
                return
            elif robot_mode == RobotModeCommand.WALK:
                self.get_logger().info('Changing mode from WALK to STANDUP')
                command_callback(
                    RobotModeCommand.STANDUP, self.robot_state, robot_command
                )
                return

    def mode_transition_command_callback(
        self, msg: RobotMode, params: Tuple[RobotState, RobotCommand]
    ):
        if msg.mode not in RobotModeCommand:
            self.get_logger().error(
                f'Invalid mode command received: {msg.mode}'
            )
            return

        robot_state, robot_command = params
        with robot_command.lock:
            robot_mode = robot_command.robot_mode

        if robot_mode == RobotModeCommand.STANDBY:
            if msg.mode == RobotModeCommand.STANDUP:
                self.get_logger().info('Changing mode from STANDBY to STANDUP')
                command_callback(
                    RobotModeCommand.STANDUP, robot_state, robot_command
                )
                return
        elif robot_mode == RobotModeCommand.STANDUP:
            if msg.mode == RobotModeCommand.STANDBY:
                self.get_logger().info('Changing mode from STANDUP to STANDBY')
                command_callback(
                    RobotModeCommand.STANDBY, robot_state, robot_command
                )
                return
            elif msg.mode == RobotModeCommand.WALK:
                self.get_logger().info('Changing mode from STANDUP to WALK')
                command_callback(
                    RobotModeCommand.WALK, robot_state, robot_command
                )
                return
        elif robot_mode == RobotModeCommand.WALK:
            if msg.mode == RobotModeCommand.STANDUP:
                self.get_logger().info('Changing mode from WALK to STANDUP')
                command_callback(
                    RobotModeCommand.STANDUP, robot_state, robot_command
                )
                return

        self.get_logger().warn(
            'Invalid mode transition from {} to {}'.format(
                robot_mode, msg.mode
            )
        )


class CanCommunicationNode(BaseNode):
    def __init__(
        self,
        robot_state: RobotState,
        robot_command: RobotCommand,
        peripheral_state: PeripheralState,
    ):
        super().__init__(
            robot_state=robot_state,
            robot_command=robot_command,
            peripheral_state=peripheral_state,
        )

        self.get_logger().info('Init can node')

        self.device = 'can0'
        self.motor_type = 'RobStride02'
        self.n_motor = 12
        self.motors = [
            CanMotorController(
                self.device,
                P.CAN_ID[i],
                motor_type=self.motor_type,
                motor_dir=P.MOTOR_DIR[i],
                external_gear_ratio=P.EXT_GEAR_RATIO[i],
            )
            for i in range(self.n_motor)
        ]

        self.get_logger().info('Setting Initial Offset...')
        for i, motor in enumerate(self.motors):
            motor.set_angle_offset(P.OFFSET_ANGLE[i], deg=False)

        self.get_logger().info('Enabling Motors...')
        for i, motor in enumerate(self.motors):
            pos, vel, cur, tem = motor.enable_motor()
            self.get_logger().info(
                'Enabling Motor {} [Status] Pos: {:.3f}, Vel: {:.3f}, '
                ' Cur: {:.3f}, Temp: {:.3f}'.format(
                    P.JOINT_NAME[i], pos, vel, cur, tem
                )
            )
            with self.robot_state.lock:
                self.robot_state.angle[i] = pos
                self.robot_state.velocity[i] = vel
                self.robot_state.current[i] = cur
                self.robot_state.temperature[i] = tem
        self.get_logger().info('Finish enabling motors!')

        self.get_logger().info('Setting Initial Command...')
        robot_command.robot_mode = (
            RobotModeCommand.TRANSITION_TO_STANDBY
        )
        with robot_state.lock:
            robot_command.angle = robot_state.angle[:]
            robot_command.initial_angle = robot_state.angle[:]
            robot_command.final_angle = P.STANDBY_ANGLE[:]
            robot_command.interpolating_time = (
                P.MODE_TRANSITION_TIME
            )
            robot_command.remaining_time = (
                robot_command.interpolating_time
            )

        with self.robot_command.lock:
            self.robot_command.initialized = True

        self.error_count = [0] * self.n_motor

        self.timer = self.create_timer(1 / P.CAN_HZ, self.timer_callback)

    def timer_callback(self):
        msg = MotorLog()
        msg.header.stamp = Time()
        jointstate_msg = JointState()
        jointstate_msg.header.stamp = Time()

        with self.robot_command.lock:
            ref_angle = self.robot_command.angle[:]
            ref_velocity = self.robot_command.velocity[:]
            ref_kp = self.robot_command.kp[:]
            ref_kd = self.robot_command.kd[:]
            ref_torque = self.robot_command.torque[:]

        with self.robot_state.lock:
            pos_list = self.robot_state.angle[:]
            vel_list = self.robot_state.velocity[:]
            cur_list = self.robot_state.current[:]
            tem_list = self.robot_state.temperature[:]
        for i, motor in enumerate(self.motors):
            try:
                pos, vel, cur, tem = motor.send_rad_command(
                    ref_angle[i],
                    ref_velocity[i],
                    ref_kp[i],
                    ref_kd[i],
                    ref_torque[i],
                )
            except Exception:
                self.error_count[i] += 1
                self.get_logger().warn(
                    '# Can Reciver is Failed for {}, ({})'.format(
                        P.JOINT_NAME[i], self.error_count[i]
                    ),
                )
                # Emergency stop if communication error occurs
                if self.error_count[i] >= 5:
                    self.get_logger().warn('Emergency stop triggered')
                    command_callback(
                        RobotModeCommand.EMERGENCY_STOP,
                        self.robot_state,
                        self.robot_command,
                    )

                continue
            pos_list[i] = pos
            vel_list[i] = vel
            cur_list[i] = cur
            tem_list[i] = tem

        with self.robot_state.lock:
            self.robot_state.angle = pos_list
            self.robot_state.velocity = vel_list
            self.robot_state.current = cur_list
            self.robot_state.temperature = tem_list

        jointstate_msg.name = P.JOINT_NAME
        jointstate_msg.position = pos_list
        jointstate_msg.velocity = vel_list
        jointstate_msg.effort = cur_list
        self.joint_states_pub.publish(jointstate_msg)

        msg.angle = pos_list
        msg.velocity = vel_list
        msg.current = cur_list
        msg.temperature = tem_list

        with self.peripheral_state.lock:
            msg.body_quat = self.peripheral_state.body_quat[:]
            msg.body_gyro = self.peripheral_state.body_gyro[:]
            msg.body_acc = self.peripheral_state.body_acc[:]

        msg.ref_angle = ref_angle
        msg.ref_velocity = ref_velocity
        msg.ref_kp = ref_kp
        msg.ref_kd = ref_kd
        msg.ref_torque = ref_torque

        self.mujina_log_pub.publish(msg)


class SimCommunication(BaseNode):
    def __init__(
        self,
        robot_state: RobotState,
        robot_command: RobotCommand,
        peripheral_state: PeripheralState,
    ):
        super().__init__(
            robot_state=robot_state,
            robot_command=robot_command,
            peripheral_state=peripheral_state,
        )

        self.get_logger().info('Init sim node')

        xml_path = os.path.abspath(
            'src/mujina_ros/mujina_control/models/scene.xml'
        )
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        self.viewer = mujoco.viewer.launch_passive(self.model, self.data)

        mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
        mujoco.mj_step(self.model, self.data)

        self.mujoco_joint_names = [
            self.model.joint(i).name for i in range(self.model.njnt)
        ]
        print("mujoco_joint_names:", self.mujoco_joint_names)
        with self.robot_state.lock:
            for i, name in enumerate(P.JOINT_NAME):
                idx = mujoco.mj_name2id(
                    self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name
                )
                self.robot_state.angle[i] = self.data.qpos[7 + idx]
                self.robot_state.velocity[i] = self.data.qvel[6 + idx]
                self.robot_state.current[i] = 0.0
                self.robot_state.temperature[i] = 25.0

        for i, name in enumerate(P.JOINT_NAME):
            idx = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name
            )
            self.data.ctrl[idx] = P.STANDBY_ANGLE[i]

        with self.robot_command.lock:
            self.robot_command.initialized = True

        self.timer = self.create_timer(1 / P.CAN_HZ, self.timer_callback)

    def timer_callback(self):
        if self.viewer.is_running():
            pass


        msg = MotorLog()

        now = self.get_clock().now()
        builtin_time = Time()
        builtin_time.sec = now.seconds_nanoseconds()[0]  # 現在の秒数
        builtin_time.nanosec = now.seconds_nanoseconds()[1]  # 現在のナノ秒

        jointstate_msg = JointState()
        jointstate_msg.header.stamp = builtin_time

        with self.robot_command.lock:
            ref_angle = self.robot_command.angle[:]
            ref_velocity = self.robot_command.velocity[:]
            ref_kp = self.robot_command.kp[:]
            ref_kd = self.robot_command.kd[:]
            ref_torque = self.robot_command.torque[:]

        mujoco_actuator_names = [
            self.model.actuator(i).name for i in range(self.model.nu)
        ]
        for i, name in enumerate(P.JOINT_NAME):  # mujina
            if name in mujoco_actuator_names:  # mujoco
                idx = mujoco.mj_name2id(
                    self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name
                )  # mujoco
                self.data.ctrl[idx] = ref_angle[i]
                # print("setting ctrl for", name, "to", ref_angle[i])

        mujoco.mj_step(self.model, self.data)

        with self.robot_state.lock:
            for i, name in enumerate(P.JOINT_NAME):
                if name in self.mujoco_joint_names:
                    idx = mujoco.mj_name2id(
                        self.model, mujoco.mjtObj.mjOBJ_JOINT, name
                    )
                    self.robot_state.angle[i] = self.data.qpos[7 + idx -1] # offset pos, quat, basejoint
                    self.robot_state.velocity[i] = self.data.qvel[6 + idx -1]
                    self.robot_state.current[i] = 0.0
                    self.robot_state.temperature[i] = 25.0
        # print(self.data.qpos)
        # print(self.data.qvel)
        with self.robot_state.lock:
            msg.angle = self.robot_state.angle[:]
            msg.velocity = self.robot_state.velocity[:]
            msg.current = self.robot_state.current[:]
            msg.temperature = self.robot_state.temperature[:]

        jointstate_msg.name = P.JOINT_NAME
        jointstate_msg.position = msg.angle
        jointstate_msg.velocity = msg.velocity
        jointstate_msg.effort = msg.current
        self.joint_states_pub.publish(jointstate_msg)

        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now()
        # CAUTION! mujoco and isaacgym's quat ordre is different
        imu_msg.orientation.w = self.data.qpos[3]
        imu_msg.orientation.x = self.data.qpos[4]
        imu_msg.orientation.y = self.data.qpos[5]
        imu_msg.orientation.z = self.data.qpos[6]
        imu_msg.angular_velocity.x = self.data.qvel[3]
        imu_msg.angular_velocity.y = self.data.qvel[4]
        imu_msg.angular_velocity.z = self.data.qvel[5]
        imu_msg.linear_acceleration.x = self.data.qacc[0]
        imu_msg.linear_acceleration.y = self.data.qacc[1]
        imu_msg.linear_acceleration.z = self.data.qacc[2]
        fused_imu_callback(imu_msg, self.peripheral_state)

        with self.peripheral_state.lock:
            msg.body_quat = self.peripheral_state.body_quat[:]
            msg.body_gyro = self.peripheral_state.body_gyro[:]
            msg.body_acc = self.peripheral_state.body_acc[:]

        msg.ref_angle = ref_angle
        msg.ref_velocity = ref_velocity
        msg.ref_kp = ref_kp
        msg.ref_kd = ref_kd
        msg.ref_torque = ref_torque

        self.mujina_log_pub.publish(msg)

        self.viewer.sync()


class MainController(Node):
    def __init__(
        self,
        robot_state: RobotState,
        robot_command: RobotCommand,
        peripheral_state: PeripheralState,
    ):
        super().__init__('mujina_main_controller')
        self.get_logger().info('Init main_controller Node')
        self.timer = self.create_timer(1.0 / P.CONTROL_HZ, self.timer_callback)
        self.robot_state = robot_state
        self.robot_command = robot_command
        self.peripheral_state = peripheral_state
        model_path = os.path.join(
            get_package_share_directory('mujina_control'),
            'models/policy.onnx',
        )
        self.prediction = OnnxPredictor(model_path)

        urdf_fullpath = os.path.join(
            get_package_share_directory('mujina_description'),
            'urdf/mujina.urdf',
        )
        self.joint_params = mujina_utils.get_urdf_joint_params(
            urdf_fullpath, P.JOINT_NAME
        )

        self.is_safe = True
        self.last_actions = [0.0] * 12

    def timer_callback(self):
        #self.get_logger().info('test')
        with self.robot_command.lock:
            robot_mode = self.robot_command.robot_mode

        if robot_mode == RobotModeCommand.STANDBY:
            # reset error
            self.is_safe = True
            self.last_actions = [0.0] * 12

        with self.robot_command.lock:
            self.robot_command.remaining_time -= 1.0 / P.CONTROL_HZ
            self.robot_command.remaining_time = max(
                0, self.robot_command.remaining_time
            )
        if robot_mode in [
            RobotModeCommand.STANDBY,
            RobotModeCommand.STANDUP,
            RobotModeCommand.DEBUG,
            RobotModeCommand.TRANSITION_TO_STANDBY,
            RobotModeCommand.TRANSITION_TO_STANDUP,
        ]:
            with self.robot_command.lock:
                if self.robot_command.remaining_time <= 0:
                    if robot_mode == RobotModeCommand.TRANSITION_TO_STANDBY:
                        self.robot_command.robot_mode = (
                            RobotModeCommand.STANDBY
                        )
                    elif robot_mode == RobotModeCommand.TRANSITION_TO_STANDUP:
                        self.robot_command.robot_mode = (
                            RobotModeCommand.STANDUP
                        )
                else:
                    ratio = (
                        1
                        - self.robot_command.remaining_time
                        / self.robot_command.interpolating_time
                    )
                    self.robot_command.angle = [
                        a + (b - a) * ratio
                        for a, b in zip(
                            self.robot_command.initial_angle,
                            self.robot_command.final_angle,
                        )
                    ]
        elif robot_mode == RobotModeCommand.WALK:
            with self.peripheral_state.lock:
                base_quat = self.peripheral_state.body_quat[:]
                base_ang_vel = self.peripheral_state.body_gyro[:]

                ranges = P.commands.ranges
                coefs = [
                    ranges.lin_vel_x[1],
                    ranges.lin_vel_y[1],
                    ranges.ang_vel_yaw[1],
                    ranges.heading[1],
                ]
                if self.peripheral_state.control_enable:
                    cmd = self.peripheral_state.control_command[:]
                    max_command = 1.0
                    x_vel = cmd[0]
                    y_vel = cmd[1]
                    yaw_vel = cmd[2]
                    commands_ = [x_vel, y_vel, yaw_vel, yaw_vel]
                    commands = [
                        [
                            min(max(-coef, coef * command / max_command), coef)
                            for coef, command in zip(coefs, commands_)
                        ]
                    ]
                else:
                    commands = torch.tensor(
                        [[0.0, 0.0, 0.0, 0.0]],
                        dtype=torch.float,
                        requires_grad=False,
                    )

                self.get_logger().info(
                    'High Level Commands: {}'.format(commands)
                )

                # for safety
                # no sensor data
                if self.peripheral_state.sensor_last_received_time is None:
                    self.is_safe = False
                    self.get_logger().warn('No Connection to Sensor.')
                if (
                    self.peripheral_state.sensor_last_received_time is not None
                ) and (
                    time.time()
                    - self.peripheral_state.sensor_last_received_time
                    > 0.1
                ):
                    self.get_logger().warn('Sensor data is too old.')
                    self.is_safe = False

            # falling down
            try:
                rot_mat = Rotation.from_quat(base_quat).as_matrix()
            except Exception as exc:
                self.get_logger().error(
                    'Exception in fall check: {} quat={}'.format(
                        exc, base_quat
                    )
                )
                return
            if self.is_safe and (rot_mat[2, 2] < 0.6):
                self.is_safe = False
                self.get_logger().warn('Robot is almost fell down.')
                self.get_logger().warn('{}'.format(rot_mat))

            # self.is_safe=True
            if not self.is_safe:
                self.get_logger().info('Robot mode is forced to STANDUP.')
                command_callback(
                    RobotModeCommand.STANDUP,
                    self.robot_state,
                    self.robot_command,
                )
                # print('Robot is not safe. Please reboot the robot.')
                # with self.robot_command.lock:
                #     self.robot_command.kp = [0.0] * 12
                #     self.robot_command.kd = [0.0] * 12
                #     with self.robot_state.lock:
                #         self.robot_command.angle = self.robot_state.angle[:]
                # rate.sleep()
                # continue

            with self.robot_state.lock:
                dof_pos = self.robot_state.angle[:]
                dof_vel = self.robot_state.velocity[:]
            # print(
            #     base_quat,
            #     base_lin_vel,
            #     base_ang_vel,
            #     commands,
            #     dof_pos,
            #     dof_vel,
            #     last_actions,
            # )
            obs = mujina_utils.get_policy_observation(
                base_quat,
                base_ang_vel,
                commands,
                dof_pos,
                dof_vel,
                self.last_actions,
            )
            actions = self.prediction.get_onnx_output(obs)
            scaled_actions = P.control.action_scale * actions

            ref_angle = [
                a + b for a, b in zip(scaled_actions, P.DEFAULT_ANGLE[:])
            ]
            with self.robot_state.lock:
                for i in range(len(ref_angle)):
                    if (
                        self.robot_state.angle[i] < self.joint_params[i][0]
                        or self.robot_state.angle[i] > self.joint_params[i][1]
                    ):
                        ref_angle[i] = max(
                            self.joint_params[i][0] + 0.1,
                            min(ref_angle[i], self.joint_params[i][1] - 0.1),
                        )
                        self.get_logger().warn(
                            '# Joint {} out of range: {:.3f}'.format(
                                P.JOINT_NAME[i], self.robot_state.angle[i]
                            )
                        )
            with self.robot_command.lock:
                self.robot_command.angle = ref_angle

            self.last_actions = actions[:]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--sim', action='store_true', help='do simulation')
    args = parser.parse_args()

    print('Hello mujina!!')
    rclpy.init()
    try:
        # Interface between class
        robot_state = RobotState()
        peripheral_state = PeripheralState()
        robot_command = RobotCommand()

        main_controller = MainController(
            robot_state, robot_command, peripheral_state
        )

        # communication_thread: Union[SimCommunication, CanCommunicationNode]
        if args.sim:
            communication_thread = SimCommunication(
                robot_state, robot_command, peripheral_state
            )
        else:
            communication_thread = CanCommunicationNode(
                robot_state, robot_command, peripheral_state
            )

        executor = SingleThreadedExecutor()
        executor.add_node(communication_thread)
        executor.add_node(main_controller)
        try:
            executor.spin()
        finally:
            executor.shutdown()

    except KeyboardInterrupt:
        sys.exit(1)
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
