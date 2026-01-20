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

import argparse
import time

from mujina_control.motor_lib.motor_lib import CanMotorController
from mujina_control.mujina_utils.parameters import parameters as P


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--device', '-d', type=str, default='can0', help='can interface name'
    )
    args = parser.parse_args()

    print('# using Socket {} for can communication'.format(args.device))

    motor_type = 'RobStride02'
    n_motor = 12
    motors = [
        CanMotorController(
            args.device,
            P.CAN_ID[i],
            motor_type=motor_type,
            motor_dir=P.MOTOR_DIR[i],
            external_gear_ratio=P.EXT_GEAR_RATIO[i],
        )
        for i in range(n_motor)
    ]

    pos_vec = []
    vel_vec = []
    cur_vec = []
    tem_vec = []
    for i, motor_controller in enumerate(motors):
        motor_controller.set_angle_offset(P.OFFSET_ANGLE[i], deg=False)
        pos, vel, cur, tem = motor_controller.send_rad_command(0.0, 0.0, 0, 0, 0)
        pos_vec.append(pos)
        vel_vec.append(vel)
        cur_vec.append(cur)
        tem_vec.append(tem)

    while True:
        for i, motor_controller in enumerate(motors):

            pos, vel, cur, tem = motor_controller.send_rad_command(
                0, 0, 0, 0, 0
            )
            # print(
            #     'Moving Motor {} Position: {}, Velocity: {}, Torque: {}, Temp: {}'.format(
            #         i, pos, vel, cur, tem
            #     )
            # )
            if i == 2:
                print(
                    'Moving Motor {} Position: {}, Velocity: {}, Torque: {}, Temp: {}'.format(
                        i, pos, vel, cur, tem
                    )
                )
            time.sleep(0.1)



if __name__ == '__main__':
    main()
