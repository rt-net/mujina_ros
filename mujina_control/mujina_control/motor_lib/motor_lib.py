# The MIT License (MIT)
#
# Copyright (c) 2021 DFKI RIC Underactuated Robotics Lab
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

import copy
import math
import socket
import struct
import time

from bitstring import BitArray

# CAN frame packing/unpacking (see `struct can_frame` in <linux/can.h>)
# 8 bytes of data is sent to the motor
can_frame_fmt_send = '=IB3x8s'
can_frame_fmt_send_robstride = '>IB3x8s'
# 6 bytes are received from the motor
can_frame_fmt_recv = '=IB3x8s'
# Total CAN Frame size is 14 Bytes: 8 Bytes overhead + 6 Bytes data
recvBytes = 16

# List of Motors Supported by this Driver
legitimate_motors = [
    'AK80_6_V1',
    'AK80_6_V1p1',
    'AK80_6_V2',
    'AK80_9_V1p1',
    'AK80_9_V2',
    'AK70_10_V1p1',
    'AK10_9_V1p1',
    'AK60_6_V1p1',
    "RobStride02",
]

# Constants for conversion
# Working parameters for AK80-6 V1.0 firmware
AK80_6_V1_PARAMS = {
    'P_MIN': -95.5,
    'P_MAX': 95.5,
    'V_MIN': -45.0,
    'V_MAX': 45.0,
    'KP_MIN': 0.0,
    'KP_MAX': 500,
    'KD_MIN': 0.0,
    'KD_MAX': 5.0,
    'T_MIN': -18.0,
    'T_MAX': 18.0,
    'C_MIN': -20.0,
    'C_MAX': 127.0,
    'AXIS_DIRECTION': 1,
}

# Working parameters for AK80-6 V1.1 firmware
AK80_6_V1p1_PARAMS = {
    'P_MIN': -12.5,
    'P_MAX': 12.5,
    'V_MIN': -22.5,
    'V_MAX': 22.5,
    'KP_MIN': 0.0,
    'KP_MAX': 500,
    'KD_MIN': 0.0,
    'KD_MAX': 5.0,
    'T_MIN': -12.0,
    'T_MAX': 12.0,
    'C_MIN': -20.0,
    'C_MAX': 127.0,
    'AXIS_DIRECTION': 1,
}

# Working parameters for AK80-6 V2.0 firmware
AK80_6_V2_PARAMS = {
    'P_MIN': -12.5,
    'P_MAX': 12.5,
    'V_MIN': -76.0,
    'V_MAX': 76.0,
    'KP_MIN': 0.0,
    'KP_MAX': 500.0,
    'KD_MIN': 0.0,
    'KD_MAX': 5.0,
    'T_MIN': -12.0,
    'T_MAX': 12.0,
    'C_MIN': -20.0,
    'C_MAX': 127.0,
    'AXIS_DIRECTION': 1,
}

# Working parameters for AK80-9 V1.1 firmware
AK80_9_V1p1_PARAMS = {
    'P_MIN': -12.5,
    'P_MAX': 12.5,
    'V_MIN': -22.5,
    'V_MAX': 22.5,
    'KP_MIN': 0.0,
    'KP_MAX': 500,
    'KD_MIN': 0.0,
    'KD_MAX': 5.0,
    'T_MIN': -18.0,
    'T_MAX': 18.0,
    'C_MIN': -20.0,
    'C_MAX': 127.0,
    'AXIS_DIRECTION': 1,
}

# Working parameters for AK80-9 V2.0 firmware
AK80_9_V2_PARAMS = {
    'P_MIN': -12.5,
    'P_MAX': 12.5,
    'V_MIN': -25.64,
    'V_MAX': 25.64,
    'KP_MIN': 0.0,
    'KP_MAX': 500.0,
    'KD_MIN': 0.0,
    'KD_MAX': 5.0,
    'T_MIN': -18.0,
    'T_MAX': 18.0,
    'C_MIN': -20.0,
    'C_MAX': 127.0,
    'AXIS_DIRECTION': 1,
}

#  Working parameters for AK70-10 V1.1 firmware
AK70_10_V1p1_params = {
    'P_MIN': -12.5,
    'P_MAX': 12.5,
    'V_MIN': -50,
    'V_MAX': 50,
    'KP_MIN': 0,
    'KP_MAX': 500,
    'KD_MIN': 0,
    'KD_MAX': 5,
    'T_MIN': -25.0,
    'T_MAX': 25.0,
    'C_MIN': -20.0,
    'C_MAX': 127.0,
    'AXIS_DIRECTION': 1,
}

# Working parameters for AK10-9 V1.1 firmware
AK10_9_V1p1_PARAMS = {
    'P_MIN': -12.5,
    'P_MAX': 12.5,
    'V_MIN': -50.0,
    'V_MAX': 50.0,
    'KP_MIN': 0.0,
    'KP_MAX': 500,
    'KD_MIN': 0.0,
    'KD_MAX': 5.0,
    'T_MIN': -65.0,
    'T_MAX': 65.0,
    'C_MIN': -20.0,
    'C_MAX': 127.0,
    'AXIS_DIRECTION': 1,
}

AK60_6_V1p1_PARAMS = {
    'P_MIN': -12.5,
    'P_MAX': 12.5,
    'V_MIN': -45.0,
    'V_MAX': 45.0,
    'KP_MIN': 0.0,
    'KP_MAX': 500,
    'KD_MIN': 0.0,
    'KD_MAX': 5.0,
    'T_MIN': -15.0,
    'T_MAX': 15.0,
    'C_MIN': -20.0,
    'C_MAX': 127.0,
    'AXIS_DIRECTION': 1,
}

RobStride_PARAMS = {
    'P_MIN': -12.5,
    'P_MAX': 12.5,
    'V_MIN': -44.0,
    'V_MAX': 44.0,
    'KP_MIN': 0.0,
    'KP_MAX': 500,
    'KD_MIN': 0.0,
    'KD_MAX': 5.0,
    'T_MIN': -17.0,
    'T_MAX': 17.0,
    'C_MIN': -20.0,
    'C_MAX': 127.0,
    'AXIS_DIRECTION': 1,
    'EXT_GEAR_RATIO': 1.0,
}

maxRawPosition = 2**16 - 1  # 16-Bits for Raw Position Values
maxRawVelocity = 2**12 - 1  # 12-Bits for Raw Velocity Values
maxRawTorque = 2**12 - 1  # 12-Bits for Raw Torque Values
maxRawKp = 2**12 - 1  # 12-Bits for Raw Kp Values
maxRawKd = 2**12 - 1  # 12-Bits for Raw Kd Values
maxRawCurrent = 2**12 - 1  # 12-Bits for Raw Current Values
dt_sleep = 0.0001  # Time before motor sends a reply
enable_sleep = 0.002  # Time before motor sends a reply
set_zero_sleep = (
    1.5  # Time to wait after setting zero. Motor takes extra time to set zero.
)


def float_to_uint(x, x_min, x_max, numBits):
    span = x_max - x_min
    offset = x_min
    # Attempt to speedup by using pre-computation. Not used currently.
    if numBits == 16:
        bitRange = maxRawPosition
    elif numBits == 12:
        bitRange = maxRawVelocity
    else:
        bitRange = 2**numBits - 1
    return int(((x - offset) * (bitRange)) / span)


def uint_to_float(x_int, x_min, x_max, numBits):
    span = x_max - x_min
    offset = x_min
    if numBits == 16:
        bitRange = maxRawPosition
    elif numBits == 12:
        bitRange = maxRawVelocity
    else:
        bitRange = 2**numBits - 1
    return ((x_int * span) / (bitRange)) + offset


def waitOhneSleep(dt):
    startTime = time.time()
    while time.time() - startTime < dt:
        pass

#####################
# for RobStride
def uint16_to_float(x, x_min, x_max, bits=16):
    span = (1 << bits) - 1
    offset = x_max - x_min
    return (offset * x / span + x_min)

def float_to_uint16(x, x_min, x_max, bits=16):
	span = x_max - x_min
	offset = x_min
	if x > x_max: x = x_max
	elif x < x_min: x = x_min
	return int((x - offset)*(((1 << bits) - 1))/span)

def Byte_to_float(bytedata):
    data_float = struct.unpack("f", bytedata[4:8])[0]
    return data_float

CAN_EFF_FLAG=0x80000000
CAN_RTR_FLAG=0x40000000
CAN_ERR_FLAG=0x20000000
CAN_SFF_MASK=0x000007FF
CAN_EFF_MASK=0x1FFFFFFF
CAN_ERR_MASK=0x1FFFFFFF
Index_List = [0X7005, 0X7006, 0X700A, 0X700B, 0X7010, 0X7011, 0X7014, 0X7016, 0X7017, 0X7018, 0x7019, 0x701A, 0x701B, 0x701C, 0x701D]

##################

class CanMotorController:
    """
    Create a Mini-Cheetah Motor Controller over CAN.

    Uses SocketCAN driver for communication.
    """

    can_socket_declared = False
    motor_socket = None
    angle_range = None  # [rad]
    angle_offset = None  # [rad]
    current_pos = 0  # [rad]
    current_vel = 0  # [rad/s]
    current_cur = 0  # [A]
    current_tem = 20  # [C]

    def __init__(
        self,
        can_socket='can0',
        motor_id=0x01,
        motor_dir=1,
        motor_type='AK80_6_V1p1',
        socket_timeout=0.05,
        external_gear_ratio=1.0,
    ):
        """
        Initialize the class with socket name, motor ID, and socket timeout.

        Sets up the socket communication for rest of the functions.
        """
        self.motorParams = AK80_6_V1p1_PARAMS  # default choice
        print('Using Motor Type: {}'.format(motor_type))
        assert motor_type in legitimate_motors, (
            'Motor Type not in list of accepted motors.'
        )
        if motor_type == 'AK80_6_V1':
            self.motorParams = copy.deepcopy(AK80_6_V1_PARAMS)
        elif motor_type == 'AK80_6_V1p1':
            self.motorParams = copy.deepcopy(AK80_6_V1p1_PARAMS)
        elif motor_type == 'AK80_6_V2':
            self.motorParams = copy.deepcopy(AK80_6_V2_PARAMS)
        elif motor_type == 'AK80_9_V1p1':
            self.motorParams = copy.deepcopy(AK80_9_V1p1_PARAMS)
        elif motor_type == 'AK80_9_V2':
            self.motorParams = copy.deepcopy(AK80_9_V2_PARAMS)
        elif motor_type == 'AK60_6_V1p1':
            self.motorParams = copy.deepcopy(AK60_6_V1p1_PARAMS)
        elif motor_type == 'AK10_9_V1p1':
            self.motorParams = copy.deepcopy(AK10_9_V1p1_PARAMS)
        elif motor_type == 'AK70_10_V1p1':
            self.motorParams = copy.deepcopy(AK70_10_V1p1_params)
        elif motor_type == "RobStride02":
            self.motorParams = copy.deepcopy(RobStride_PARAMS)
            self.master_can_id = 0x00
        
        self.motorParams['AXIS_DIRECTION'] = motor_dir
        self.motorParams['EXT_GEAR_RATIO'] = external_gear_ratio

        self.motor_type = motor_type

        can_socket = (can_socket,)
        self.motor_id = motor_id
        if not CanMotorController.can_socket_declared:
            # create a raw socket and bind it to the given CAN interface
            try:
                CanMotorController.motor_socket = socket.socket(
                    socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW
                )
                CanMotorController.motor_socket.setsockopt(
                    socket.SOL_CAN_RAW, socket.CAN_RAW_LOOPBACK, 0
                )
                CanMotorController.motor_socket.bind(can_socket)
                CanMotorController.motor_socket.settimeout(socket_timeout)
                print('Bound to: ', can_socket)
                CanMotorController.can_socket_declared = True
            except Exception as e:
                print('Unable to Connect to Socket Specified: ', can_socket)
                print('Error:', e)
        elif CanMotorController.can_socket_declared:
            print(
                'Socket already available. Using:  ',
                CanMotorController.motor_socket,
            )
        # Initialize the command BitArrays for performance optimization
        self._p_des_BitArray = BitArray(
            uint=float_to_uint(
                0, self.motorParams['P_MIN'], self.motorParams['P_MAX'], 16
            ),
            length=16,
        )
        self._v_des_BitArray = BitArray(
            uint=float_to_uint(
                0, self.motorParams['V_MIN'], self.motorParams['V_MAX'], 12
            ),
            length=12,
        )
        self._kp_BitArray = BitArray(uint=0, length=12)
        self._kd_BitArray = BitArray(uint=0, length=12)
        self._tau_BitArray = BitArray(uint=0, length=12)
        self._cmd_bytes = BitArray(uint=0, length=64)
        self._recv_bytes = BitArray(uint=0, length=48)

    def set_angle_range(self, low, upper, deg=True):
        """Set the angle range [deg] for the motor."""
        if deg:
            self.angle_range = [low * math.pi / 180, upper * math.pi / 180]
        else:
            self.angle_range = [low, upper]
        assert len(self.angle_range) == 2, 'Invalid Angle Range Specified.'
        assert self.angle_range[0] < self.angle_range[1], (
            'Invalid Angle Range Specified.'
        )

    def set_angle_offset(self, angle_offset, deg=True):
        """Set the angle offset [deg] for the motor."""
        if deg:
            self.angle_offset = angle_offset * math.pi / 180
        else:
            self.angle_offset = angle_offset
        self.current_pos = self.angle_offset

    def _send_can_frame(self, data, cmd=0x00, opt=0x00):
        """Send raw CAN data frame (in bytes) to the motor."""
        can_dlc = len(data)
        if self.motor_type == "RobStride02":
            id_ = cmd << 24 | opt << 8 | self.motor_id | CAN_EFF_FLAG
            can_msg = struct.pack(can_frame_fmt_send, id_, can_dlc, data)
        else:
            can_msg = struct.pack(can_frame_fmt_send, self.motor_id, can_dlc, data)
        try:
            #print(can_msg)
            CanMotorController.motor_socket.send(can_msg)
        except Exception as e:
            print('Unable to Send CAN Frame.')
            print('Error: ', e)

    def _recv_can_frame(self):
        """
        Receive a CAN frame and unpack it.

        Returns can_id, can_dlc (data length), data (in bytes).
        """
        try:
            frame, addr = CanMotorController.motor_socket.recvfrom(recvBytes)
            can_id, can_dlc, data = struct.unpack(can_frame_fmt_recv, frame)

            if can_id & CAN_EFF_FLAG:
                can_id &= CAN_EFF_MASK
            else:
                can_id &= CAN_SFF_MASK

            return can_id, can_dlc, data[:can_dlc]
        except Exception as e:
            print('Unable to Receive CAN Frame.')
            print('Error: ', e)

    def enable_motor(self):
        """Enable the motor."""
        try:
            # Bugfix: To remove the initial kick at motor start.
            # The current working theory is that the motor is not
            # set to zero position when enabled. Hence the
            # last command is executed. So we set zero position
            # and then enable the motor.
            #print(res)
            if self.motor_type == 'RobStride02':
                # robstride has absolute encoder, so no need to set zero position
                cmd = 0x03
                self._send_can_frame(b"\x00\x00\x00\x00\x00\x00\x00\x00", cmd, self.master_can_id)
            else:
                res=self.set_zero_position()
                self._send_can_frame(b'\xff\xff\xff\xff\xff\xff\xff\xfc')
            # 16進数で8バイトのデータを表す.
            waitOhneSleep(enable_sleep)
            can_id, can_dlc, motorStatusData = self._recv_can_frame()
            if self.motor_type == 'RobStride02':
                print('Motor Enabled.')
                return self.decode_robstride_motor_status(can_id,motorStatusData)
            else:
                rawMotorData = self.decode_motor_status(motorStatusData)
                pos, vel, cur, tem = self.convert_raw_to_physical_rad(
                    rawMotorData[0],
                    rawMotorData[1],
                    rawMotorData[2],
                    rawMotorData[3],
                )
                print('Motor Enabled.')
                return pos, vel, cur, tem
        except Exception as e:
            print('Error Enabling Motor!')
            print('Error: ', e)

    def disable_motor(self):
        """Disable the motor."""
        try:
            # Bugfix: To remove the initial kick at motor start.
            # The current working theory is that the motor "remembers" the last command. And this
            # causes an initial kick as the motor controller starts. The fix is then to set the
            # last command to zero so that this does not happen. For the user, the behavior does
            # not change as zero command + disable is same as disable.
            # Do the actual disabling after zero command.
            if self.motor_type == 'RobStride02':
                cmd = 0x04
                self._send_can_frame(b"\x00\x00\x00\x00\x00\x00\x00\x00", cmd, self.master_can_id)
            else:
                _, _, _, _ = self.send_rad_command(0, 0, 0, 0, 0)
                self._send_can_frame(b'\xff\xff\xff\xff\xff\xff\xff\xfd')
            waitOhneSleep(dt_sleep)
            can_id, can_dlc, motorStatusData = self._recv_can_frame()
            if self.motor_type == 'RobStride02':
                print('Motor Disabled.')
                return self.decode_robstride_motor_status(can_id,motorStatusData)
            else:
                rawMotorData = self.decode_motor_status(motorStatusData)
                pos, vel, cur, tem = self.convert_raw_to_physical_rad(
                    rawMotorData[0],
                    rawMotorData[1],
                    rawMotorData[2],
                    rawMotorData[3],
                )
                print('Motor Disabled.')
                return pos, vel, cur, tem
        except Exception as e:
            print('Error Disabling Motor!')
            print('Error: ', e)

    def set_zero_position(self):
        """Set the current position as zero."""
        try:
            if self.motor_type == 'RobStride02':
                cmd =  0x06
                self._send_can_frame(b"\x01\x00\x00\x00\x00\x00\x00\x00", cmd, self.master_can_id)

            else:
                self._send_can_frame(b'\xff\xff\xff\xff\xff\xff\xff\xfe')
            waitOhneSleep(set_zero_sleep)
            can_id, can_dlc, motorStatusData = self._recv_can_frame()
            if self.motor_type == 'RobStride02':
                print('Zero Position set.')
                return self.decode_robstride_motor_status(can_id,motorStatusData)
            else:
                rawMotorData = self.decode_motor_status(motorStatusData)
                pos, vel, cur, tem = self.convert_raw_to_physical_rad(
                    rawMotorData[0],
                    rawMotorData[1],
                    rawMotorData[2],
                    rawMotorData[3],
                )
                print('Zero Position set.')
                return pos, vel, cur, tem
        except Exception as e:
            print('Error Setting Zero Position!')
            print('Error: ', e)

    def decode_motor_status(self, data_frame):
        """
        Decode the motor status reply message into its constituent raw values.

        /// CAN Reply Packet Structure ///
        /// 16 bit position, between -4*pi and 4*pi
        /// 12 bit velocity, between -30 and + 30 rad/s
        /// 12 bit current, between -40 and 40;
        /// CAN Packet is 5 8-bit words
        /// Formatted as follows.  For each quantity, bit 0 is LSB
        /// 0: [position[15-8]]
        /// 1: [position[7-0]]
        /// 2: [velocity[11-4]]
        /// 3: [velocity[3-0], current[11-8]]
        /// 4: [current[7-0]]

        Returns
        -------
        the following raw values as (u)int: motorid, position, velocity, current, temperature

        """
        self._recv_bytes.bytes = data_frame
        dataBitArray = self._recv_bytes.bin

        positionBitArray = dataBitArray[8:24]
        velocityBitArray = dataBitArray[24:36]
        currentBitArray = dataBitArray[36:48]
        temperatureBitArray = dataBitArray[48:56]

        positionRawValue = int(positionBitArray, 2)
        velocityRawValue = int(velocityBitArray, 2)
        currentRawValue = int(currentBitArray, 2)
        temperatureRawValue = int(temperatureBitArray, 2)

        return (
            positionRawValue,
            velocityRawValue,
            currentRawValue,
            temperatureRawValue,
        )

    def decode_robstride_motor_status(self, can_id, data_frame):
        if self.motor_type != "RobStride02" : return 
        if ((can_id & 0x3f000000) >> 24) == 2:
            physicalPositionRad=uint16_to_float((data_frame[0] << 8| data_frame[1]), self.motorParams['P_MIN'], self.motorParams['P_MAX'] )
            physicalVelocityRad=uint16_to_float((data_frame[2] << 8| data_frame[3]), self.motorParams['V_MIN'], self.motorParams['V_MAX'] )
            physicalCurrent=uint16_to_float((data_frame[4] << 8| data_frame[5]), self.motorParams['T_MIN'], self.motorParams['T_MAX'] )
            physicalTemperature=(data_frame[6] << 8| data_frame[7]) * 0.1
            error_code = (can_id & 0x3f0000) >> 16
            pattern = (can_id & 0xc00000) >> 22

            physicalPositionRad = (
                physicalPositionRad * self.motorParams['AXIS_DIRECTION'] / self.motorParams['EXT_GEAR_RATIO']
            )
            physicalVelocityRad = (
                physicalVelocityRad * self.motorParams['AXIS_DIRECTION'] / self.motorParams['EXT_GEAR_RATIO']
            )
            physicalCurrent = physicalCurrent * self.motorParams['AXIS_DIRECTION']

            if self.angle_offset is not None:
                physicalPositionRad = physicalPositionRad + self.angle_offset
            self.current_pos = physicalPositionRad
            self.current_vel = physicalVelocityRad
            self.current_cur = physicalCurrent
            self.current_tem = physicalTemperature

            return (
                physicalPositionRad,
                physicalVelocityRad,
                physicalCurrent,
                physicalTemperature,
            )


        elif ((can_id & 0x3f000000) >> 24) == 17:
            for i in range(12):
                if (data_frame[1] << 8| data_frame[0]) == Index_List[i]:
                    if i == 0:
                        self.run_mode = data_frame(4)
                    elif i == 1:
                        self.iq_ref = Byte_to_float(data_frame)
                    elif i == 2:
                        self.spd_red = Byte_to_float(data_frame)
                    elif i == 3:
                        self.limit_torque = Byte_to_float(data_frame)
                    elif i == 4:
                        self.cur_kp = Byte_to_float(data_frame)
                    elif i == 5:
                        self.cur_ki = Byte_to_float(data_frame)
                    elif i == 6:
                        self.cur_filt_gain = Byte_to_float(data_frame)
                    elif i == 7:
                        self.loc_ref = Byte_to_float(data_frame)
                    elif i == 8:
                        self.limit_spd = Byte_to_float(data_frame)
                    elif i == 9: 
                        self.limit_cur = Byte_to_float(data_frame)
                    elif i == 10: 
                        self.mechPos = Byte_to_float(data_frame)
                    elif i == 11: 
                        self.iqf = Byte_to_float(data_frame)
                    elif i == 12: 
                        self.mechVel = Byte_to_float(data_frame)
                    elif i == 13:
                        self.VBUS = Byte_to_float(data_frame)

        elif (can_id & 0xff) == 0xfe:
           self.motor_id = (can_id & 0xff00) >> 8
        else:
            pass
        


    def convert_raw_to_physical_rad(
        self,
        positionRawValue,
        velocityRawValue,
        currentRawValue,
        temperatureRawValue,
    ):
        """
        Convert the raw values from the motor to physical values.

        /// CAN Reply Packet Structure ///
        /// 16 bit position, between -4*pi and 4*pi
        /// 12 bit velocity, between -30 and + 30 rad/s
        /// 12 bit current, between -40 and 40;
        /// CAN Packet is 5 8-bit words
        /// Formatted as follows.  For each quantity, bit 0 is LSB
        /// 0: [position[15-8]]
        /// 1: [position[7-0]]
        /// 2: [velocity[11-4]]
        /// 3: [velocity[3-0], current[11-8]]
        /// 4: [current[7-0]]

        Returns
        -------
        position (radians), velocity (rad/s), current (amps)

        """
        physicalPositionRad = uint_to_float(
            positionRawValue,
            self.motorParams['P_MIN'],
            self.motorParams['P_MAX'],
            16,
        )
        physicalVelocityRad = uint_to_float(
            velocityRawValue,
            self.motorParams['V_MIN'],
            self.motorParams['V_MAX'],
            12,
        )
        physicalCurrent = uint_to_float(
            currentRawValue,
            self.motorParams['T_MIN'],
            self.motorParams['T_MAX'],
            12,
        )
        physicalTemperature = uint_to_float(
            temperatureRawValue,
            self.motorParams['C_MIN'],
            self.motorParams['C_MAX'],
            8,
        )

        physicalPositionRad = (
            physicalPositionRad * self.motorParams['AXIS_DIRECTION'] / self.motorParams['EXT_GEAR_RATIO']
        )
        physicalVelocityRad = (
            physicalVelocityRad * self.motorParams['AXIS_DIRECTION'] / self.motorParams['EXT_GEAR_RATIO']
        )
        physicalCurrent = physicalCurrent * self.motorParams['AXIS_DIRECTION']

        if self.angle_offset is not None:
            physicalPositionRad = physicalPositionRad + self.angle_offset
        self.current_pos = physicalPositionRad
        self.current_vel = physicalVelocityRad
        self.current_cur = physicalCurrent
        self.current_tem = physicalTemperature

        return (
            physicalPositionRad,
            physicalVelocityRad,
            physicalCurrent,
            physicalTemperature,
        )

    # command q --> range --> offset --> axis
    def convert_physical_rad_to_raw(
        self, p_des_rad, v_des_rad, kp, kd, tau_ff
    ):
        """Convert physical values to raw values for CAN transmission."""
        p_des_rad = p_des_rad * self.motorParams['AXIS_DIRECTION'] * self.motorParams['EXT_GEAR_RATIO']
        v_des_rad = v_des_rad * self.motorParams['AXIS_DIRECTION'] * self.motorParams['EXT_GEAR_RATIO']
        tau_ff = tau_ff * self.motorParams['AXIS_DIRECTION']

        rawPosition = float_to_uint(
            p_des_rad, self.motorParams['P_MIN'], self.motorParams['P_MAX'], 16
        )
        rawVelocity = float_to_uint(
            v_des_rad, self.motorParams['V_MIN'], self.motorParams['V_MAX'], 12
        )
        rawTorque = float_to_uint(
            tau_ff, self.motorParams['T_MIN'], self.motorParams['T_MAX'], 12
        )

        rawKp = (maxRawKp * kp) / self.motorParams['KP_MAX']
        rawKd = (maxRawKd * kd) / self.motorParams['KD_MAX']

        return (
            int(rawPosition),
            int(rawVelocity),
            int(rawKp),
            int(rawKd),
            int(rawTorque),
        )

    def _send_raw_command(self, p_des, v_des, kp, kd, tau_ff):
        """
        Send raw (uint) values to the motor.

        Package and send raw (uint) values of correct length to the motor.

        Returns
        -------
        bytes
            Motor status data.

        """
        self._p_des_BitArray.uint = p_des
        self._v_des_BitArray.uint = v_des
        self._kp_BitArray.uint = kp
        self._kd_BitArray.uint = kd
        self._tau_BitArray.uint = tau_ff
        # print(p_des)
        # print(v_des)
        # print(kp)
        # print(kd)
        # print(tau_ff)
        cmd_BitArray = (
            self._p_des_BitArray.bin
            + self._v_des_BitArray.bin
            + self._kp_BitArray.bin
            + self._kd_BitArray.bin
            + self._tau_BitArray.bin
        )

        self._cmd_bytes.bin = cmd_BitArray

        try:
            self._send_can_frame(self._cmd_bytes.tobytes())
            waitOhneSleep(dt_sleep)
            can_id, can_dlc, data = self._recv_can_frame()
            return data
        except Exception as e:
            print('Error Sending Raw Commands!')
            print('Error: ', e)

    def _send_robstride_move_control(self, angle, speed, kp, kd, torq):
        send_data_ = bytearray(8)
        send_data_[0:2] = struct.pack('>H', float_to_uint16(angle, self.motorParams['P_MIN'], self.motorParams['P_MAX']))
        send_data_[2:4] = struct.pack('>H', float_to_uint16(speed, self.motorParams['V_MIN'], self.motorParams['V_MAX']))
        send_data_[4:6] = struct.pack('>H', float_to_uint16(kp, self.motorParams['KP_MIN'], self.motorParams['KP_MAX']))
        send_data_[6:8] = struct.pack('>H', float_to_uint16(kd, self.motorParams['KD_MIN'], self.motorParams['KD_MAX']))
        try:
            cmd=0x01  # Communication_Type_MotionControl
            opt=float_to_uint16(torq, self.motorParams['T_MIN'], self.motorParams['T_MAX'])
            self._send_can_frame(send_data_, cmd, opt)
            waitOhneSleep(dt_sleep)
            can_id, can_dlc, data = self._recv_can_frame()
            return can_id, data
        except Exception as e:
            print('Error Sending robostride move Commands!')
            print('Error: ', e)
            return None

    def _set_robstride_motor_param(self, idx, value, mode):
        try:
            send_data_ = bytearray(8)
            send_data_[0] = (idx >> 8) & 0xff
            send_data_[1] = (idx & 0xff)
            if mode == 'p':
                send_data_[4:] = struct.pack("f", value)
            elif mode == 'j':
                send_data_[4] = int(value)

            cmd=0x12 # Communication_Type_SetSingleParameter

            self._send_can_frame(send_data_, cmd)
            waitOhneSleep(dt_sleep)
            can_id, can_dlc, data = self._recv_can_frame()
            return can_id, data
        except Exception as e:
            print('Error Setting robstride motor param!')
            print('Error: ', e)

    def send_deg_command(self, p_des_deg, v_des_deg, kp, kd, tau_ff):
        """
        Send command to the motor in physical units.

        Sends data over CAN, reads response, and prints the current status in deg, deg/s, amps.
        If any input is outside limits, it is clipped. Only if torque is outside limits, a log
        message is shown.

        Returns
        -------
        tuple
            (position (deg), velocity (deg/s), current (amps), temperature)

        """
        p_des_rad = math.radians(p_des_deg)
        v_des_rad = math.radians(v_des_deg)

        pos_rad, vel_rad, cur, tem = self.send_rad_command(
            p_des_rad, v_des_rad, kp, kd, tau_ff
        )
        pos = math.degrees(pos_rad)
        vel = math.degrees(vel_rad)
        return pos, vel, cur, tem

    def send_rad_command(self, p_des_rad, v_des_rad, kp, kd, tau_ff):
        """
        Send command to the motor in physical units.

        Sends data over CAN, reads response, and prints the current status in rad, rad/s, amps.
        If any input is outside limits, it is clipped. Only if torque is outside limits, a log
        message is shown.

        Returns
        -------
        tuple
            (position (rad), velocity (rad/s), current (amps), temperature)

        """
        # Check for Torque Limits
        if tau_ff < self.motorParams['T_MIN']:
            print('Torque Commanded lower than the limit. Clipping Torque...')
            print('Commanded Torque: {}'.format(tau_ff))
            print('Torque Limit: {}'.format(self.motorParams['T_MIN']))
            tau_ff = self.motorParams['T_MIN']
        elif tau_ff > self.motorParams['T_MAX']:
            print('Torque Commanded higher than the limit. Clipping Torque...')
            print('Commanded Torque: {}'.format(tau_ff))
            print('Torque Limit: {}'.format(self.motorParams['T_MAX']))
            tau_ff = self.motorParams['T_MAX']

        if self.angle_range is not None:
            p_des_rad = max(
                self.angle_range[0], min(p_des_rad, self.angle_range[1])
            )
        if self.angle_offset is not None:
            p_des_rad = p_des_rad - self.angle_offset

        p_des_rad = min(
            max(self.motorParams['P_MIN'], p_des_rad),
            self.motorParams['P_MAX'],
        )
        v_des_rad = min(
            max(self.motorParams['V_MIN'], v_des_rad),
            self.motorParams['V_MAX'],
        )
        kp = min(
            max(self.motorParams['KP_MIN'], kp), self.motorParams['KP_MAX']
        )
        kd = min(
            max(self.motorParams['KD_MIN'], kd), self.motorParams['KD_MAX']
        )

        if self.motor_type == "RobStride02":
            p_des_rad = p_des_rad * self.motorParams['AXIS_DIRECTION'] * self.motorParams['EXT_GEAR_RATIO']
            v_des_rad = v_des_rad * self.motorParams['AXIS_DIRECTION'] * self.motorParams['EXT_GEAR_RATIO']
            tau_ff = tau_ff * self.motorParams['AXIS_DIRECTION']
            can_id, motorStatusData = self._send_robstride_move_control(
                p_des_rad, v_des_rad, kp, kd, tau_ff
            )
            return self.decode_robstride_motor_status(can_id,motorStatusData)
        else:
            rawPos, rawVel, rawKp, rawKd, rawTauff = (
                self.convert_physical_rad_to_raw(
                    p_des_rad, v_des_rad, kp, kd, tau_ff
                )
            )
            motorStatusData = self._send_raw_command(
                rawPos, rawVel, rawKp, rawKd, rawTauff
            )
            rawMotorData = self.decode_motor_status(motorStatusData)
            pos, vel, cur, tem = self.convert_raw_to_physical_rad(
                rawMotorData[0], rawMotorData[1], rawMotorData[2], rawMotorData[3]
            )

            return pos, vel, cur, tem

    def change_motor_constants(
        self,
        P_MIN_NEW,
        P_MAX_NEW,
        V_MIN_NEW,
        V_MAX_NEW,
        KP_MIN_NEW,
        KP_MAX_NEW,
        KD_MIN_NEW,
        KD_MAX_NEW,
        T_MIN_NEW,
        T_MAX_NEW,
    ):
        """
        Change the global motor constants.

        Default values are for AK80-6 motor from CubeMars. For a different motor,
        the min/max values can be changed here for correct conversion.

        Args:
        ----
        P_MIN_NEW : float
            Minimum position (radians).
        P_MAX_NEW : float
            Maximum position (radians).
        V_MIN_NEW : float
            Minimum velocity (rad/s).
        V_MAX_NEW : float
            Maximum velocity (rad/s).
        KP_MIN_NEW : float
            Minimum position gain.
        KP_MAX_NEW : float
            Maximum position gain.
        KD_MIN_NEW : float
            Minimum velocity gain.
        KD_MAX_NEW : float
            Maximum velocity gain.
        T_MIN_NEW : float
            Minimum torque (Nm).
        T_MAX_NEW : float
            Maximum torque (Nm).

        """
        self.motorParams['P_MIN'] = P_MIN_NEW
        self.motorParams['P_MAX'] = P_MAX_NEW
        self.motorParams['V_MIN'] = V_MIN_NEW
        self.motorParams['V_MAX'] = V_MAX_NEW
        self.motorParams['KP_MIN'] = KP_MIN_NEW
        self.motorParams['KP_MAX'] = KP_MAX_NEW
        self.motorParams['KD_MIN'] = KD_MIN_NEW
        self.motorParams['KD_MAX'] = KD_MAX_NEW
        self.motorParams['T_MIN'] = T_MIN_NEW
        self.motorParams['T_MAX'] = T_MAX_NEW

    def set_mit_control_mode(self):
        if self.motor_type != "RobStride02" : return
        MOVE_CONTROL_MODE = 0
        POS_CONTROL_MODE = 1
        SPEED_CONTROL_MODE = 2
        ELECT_CONTROL_MODE = 3
        SET_ZERO_MODE = 4
        SET_MODE = 'j'
        SET_PARAM = 'p'
        can_id, res=self._set_robstride_motor_param(0x7005, MOVE_CONTROL_MODE, SET_MODE)
        return res
