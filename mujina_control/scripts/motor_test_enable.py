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


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--device', '-d', type=str, default='can0', help='can interface name'
    )
    parser.add_argument(
        '--ids',
        '-i',
        type=int,
        nargs='+',
        default=None,
        help='motor ids to control',
    )
    args = parser.parse_args()

    print('# using Socket {} for can communication'.format(args.device))
    print('# motor ids: {}'.format(args.ids))
    assert args.ids is not None, 'please input motor ids'

    ids = args.ids
    motors = {}
    for motor_id in ids:
        motor_dir = 1
        motors[motor_id] = CanMotorController(
            args.device, motor_id, motor_dir, 'RobStride02', external_gear_ratio=1.0
        )

    pos_vec = []
    vel_vec = []
    cur_vec = []
    tem_vec = []
    for motor_id, motor_controller in motors.items():
        # _, _, _, _ = motor_controller.set_zero_position()
        pos, vel, cur, tem = motor_controller.enable_motor()
        pos, vel, cur, tem = motor_controller.send_rad_command(0.0, 0.0, 0, 0, 0)
        pos_vec.append(pos)
        vel_vec.append(vel)
        cur_vec.append(cur)
        tem_vec.append(tem)

    # for deg in np.linspace(0.0, 360.0, 36):
    while True:
        for motor_id, motor_controller in motors.items():
            pos, vel, cur, tem = motor_controller.send_deg_command(
                0, 0, 0, 0, 0
            )
            print(
                'Moving Motor {} Position: {}, Velocity: {}, Torque: {}, Temp: {}'.format(
                    motor_id, pos, vel, cur, tem
                )
            )
            time.sleep(0.1)



if __name__ == '__main__':
    main()
