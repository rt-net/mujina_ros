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

CAN_ID = [
    10,
    11,
    12,
    7,
    8,
    9,
    4,
    5,
    6,
    1,
    2,
    3,
]

JOINT_NAME = [
    'RL_collar_joint',
    'RL_hip_joint',
    'RL_knee_joint',
    'RR_collar_joint',
    'RR_hip_joint',
    'RR_knee_joint',
    'FL_collar_joint',
    'FL_hip_joint',
    'FL_knee_joint',
    'FR_collar_joint',
    'FR_hip_joint',
    'FR_knee_joint',
]

MOTOR_DIR = [
    1,
    -1,
    -1,
    1,
    1,
    1,
    -1,
    -1,
    -1,
    -1,
    1,
    1,
]

EXT_GEAR_RATIO = [
    1,
    1,
    2,
    1,
    1,
    2,
    1,
    1,
    2,
    1,
    1,
    2,
]


OFFSET_ANGLE = [
    -0.78539565,
    3.14159265,
    0.32794737, # 180-161.21
    -3.14159265*2.+0.78539565,
    -3.14159265,
    -2.81364528,
    3.14159265*2.-0.78539565,
    3.14159265,
    0.32794737, # 180-161.21
    0.78539565,
    -3.14159265,
    -2.81364528,
]

STANDBY_ANGLE = [
    0.2222,
    1.2710,
    -2.6,
    -0.2222,
    1.2710,
    -2.6,
    0.2398,
    1.3063,
    -2.6,
    -0.2398,
    1.3063,
    -2.6,
]

DEFAULT_ANGLE = [
    0.05,
    1.0,
    -1.4,  # RL
    -0.05,
    1.0,
    -1.4,  # RR
    0.05,
    0.8,
    -1.4,  # FL
    -0.05,
    0.8,
    -1.4,  # FR
]

CONTROL_HZ = 50
CAN_HZ = 50

MODE_TRANSITION_TIME = 3.0  # [s] time to transition between modes


class commands:
    heading_command = (
        False  # if true: compute ang vel command from heading error
    )

    class ranges:
        lin_vel_x = [-1.0, 1.0]  # min max [m/s]
        lin_vel_y = [-0.8, 0.8]  # min max [m/s]
        ang_vel_yaw = [-1.0, 1.0]  # min max [rad/s]
        heading = [-3.14, 3.14]


class control:
    stiffness = {'collar': 30.0, 'hip': 30.0, 'knee': 7.5}  # [N*m/rad]
    damping = {'collar': 1.0, 'hip': 1.0, 'knee': 0.25}  # [N*m*s/rad]
    emergency_stop_damping = 100.0  # [N*m*s/rad]

    # action scale: target angle = actionScale * action + defaultAngle
    action_scale = 0.25
    action_clipping = 10.0
    decimation = 4
    # dt = 0.005
