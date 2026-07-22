# The MIT License (MIT)
#
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
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""Forward robot mode messages to the onboard RP2040."""

import rclpy
from rclpy.node import Node
import serial

from mujina_control.interface.robot_mode_command import RobotModeCommand
from mujina_msgs.msg import RobotMode


SERIAL_MODE_MAP = {
    RobotModeCommand.STANDBY: RobotModeCommand.STANDBY,
    RobotModeCommand.STANDUP: RobotModeCommand.STANDUP,
    RobotModeCommand.WALK: RobotModeCommand.WALK,
    RobotModeCommand.CALIBRATING: RobotModeCommand.CALIBRATING,
    RobotModeCommand.EMERGENCY_STOP: RobotModeCommand.EMERGENCY_STOP,
    RobotModeCommand.ERROR: RobotModeCommand.ERROR,
    RobotModeCommand.TRANSITION_TO_STANDBY: RobotModeCommand.STANDBY,
    RobotModeCommand.TRANSITION_TO_STANDUP: RobotModeCommand.STANDUP,
}


def encode_robot_mode(value: str) -> bytes | None:
    """Encode a robot mode for the RP2040."""
    mode = RobotModeCommand(value)
    serial_mode = SERIAL_MODE_MAP.get(mode)
    if serial_mode is None:
        return None
    return f'{serial_mode.value.upper()}\n'.encode('ascii')


class SerialStateBridge(Node):
    """Forward robot mode messages over USB serial."""

    def __init__(self) -> None:
        """Open the serial port and subscribe to robot mode."""
        super().__init__('serial_state_bridge')
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)

        port = self.get_parameter('serial_port').value
        baudrate = self.get_parameter('baudrate').value
        self._serial = serial.serial_for_url(port, baudrate=baudrate)
        self.create_subscription(
            RobotMode,
            'robot_mode',
            self._send_state,
            10,
        )

        self.get_logger().info(f'Connected to {port}')

    def _send_state(self, message: RobotMode) -> None:
        try:
            encoded_mode = encode_robot_mode(message.mode)
        except ValueError:
            self.get_logger().error(
                f'Unknown robot mode received: {message.mode}'
            )
            return

        if encoded_mode is None:
            return

        try:
            self._serial.write(encoded_mode)
        except serial.SerialException as exc:
            self.get_logger().error(f'Failed to send robot mode: {exc}')

    def destroy_node(self) -> None:
        """Close the serial port and destroy the node."""
        self._serial.close()
        super().destroy_node()


def main(args=None) -> None:
    """Run the serial state bridge."""
    rclpy.init(args=args)
    node = SerialStateBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
