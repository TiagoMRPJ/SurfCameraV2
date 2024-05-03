#!/usr/bin/env python
# -*- coding:utf-8 -*-

"""
pyHerkuleX - smart HerkuleX servo motors Python package
Copyright (C)  2022  Guenhael LE QUILLIEC

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.

To obtain permission to use this code for commercial purposes,
contact Guenhael LE QUILLIEC (mailto:contact@guenhael.com).

.. Online documentation: https://guenhael.frama.io/pyherkulex/
.. Latest release archive: https://framagit.org/guenhael/pyherkulex/-/archive/master/pyherkulex-master.zip
"""

# Python 3 compatibility
from __future__ import print_function
from functools import reduce

__author__     = 'Guenhael LE QUILLIEC'
__copyright__  = 'Copyright 2020, Guenhael LE QUILLIEC'
__credits__    = ['Guenhael LE QUILLIEC']
__license__    = 'GNU General Public License - 3.0-or-later'
__version__    = '1.1.0'
__maintainer__ = 'Guenhael LE QUILLIEC'
__email__      = 'contact@guenhael.com'
__status__     = 'Production'

import time
import struct
import serial as srl
import platform
import glob

# REQuest and Acknowledgment packet commands
REQ_EEP_WRITE = 0x01 ; ACK_EEP_WRITE = 0x41
REQ_EEP_READ  = 0x02 ; ACK_EEP_READ  = 0x42
REQ_RAM_WRITE = 0x03 ; ACK_RAM_WRITE = 0x43
REQ_RAM_READ  = 0x04 ; ACK_RAM_READ  = 0x44
REQ_I_JOG     = 0x05 ; ACK_I_JOG     = 0x45
REQ_S_JOG     = 0x06 ; ACK_S_JOG     = 0x46
REQ_STAT      = 0x07 ; ACK_STAT      = 0x47
REQ_ROLLBACK  = 0x08 ; ACK_ROLLBACK  = 0x48
REQ_REBOOT    = 0x09 ; ACK_REBOOT    = 0x49

# EEPROM and RAM register addresses
EEP_MODEL_NO1                            = 0
EEP_MODEL_NO2                            = 1
EEP_VERSION1                             = 2
EEP_VERSION2                             = 3
EEP_BAUD_RATE                            = 4
EEP_ID                                   = 6  ; RAM_ID                            = 0
EEP_ACK_POLICY                           = 7  ; RAM_ACK_POLICY                    = 1
EEP_ALARM_LED_POLICY                     = 8  ; RAM_ALARM_LED_POLICY              = 2
EEP_TORQUE_POLICY                        = 9  ; RAM_TORQUE_POLICY                 = 3
EEP_MAX_TEMPERATURE                      = 11 ; RAM_MAX_TEMPERATURE               = 5  ; RANGE_TEMPERATURE                   = [0, 110]
EEP_MIN_VOLTAGE                          = 12 ; RAM_MIN_VOLTAGE                   = 6  ; RANGE_VOLTAGE                       = ([0, 0xFF], [92, 200])
EEP_MAX_VOLTAGE                          = 13 ; RAM_MAX_VOLTAGE                   = 7
EEP_ACCELERATION_RATIO                   = 14 ; RAM_ACCELERATION_RATIO            = 8  ; RANGE_ACCELERATION_RATIO            = [0, 50]
EEP_MAX_ACCELERATION_TIME                = 15 ; RAM_MAX_ACCELERATION_TIME         = 9  ; RANGE_MAX_ACCELERATION_TIME         = [0, 0xFE]
EEP_DEAD_ZONE                            = 16 ; RAM_DEAD_ZONE                     = 10 ; RANGE_DEAD_ZONE                     = [0, 0xFE]
EEP_SATURATOR_OFFSET                     = 17 ; RAM_SATURATOR_OFFSET              = 11 ; RANGE_SATURATOR_OFFSET              = [0, 0xFE]
EEP_SATURATOR_SLOPE                      = 18 ; RAM_SATURATOR_SLOPE               = 12 ; RANGE_SATURATOR_SLOPE               = [0, 0x7FFF]
EEP_PWM_OFFSET                           = 20 ; RAM_PWM_OFFSET                    = 14 ; RANGE_PWM_OFFSET                    = [-127, 127] # -128 in datasheets but actually -127
EEP_MIN_PWM                              = 21 ; RAM_MIN_PWM                       = 15 ; RANGE_MIN_PWM                       = [0, 254]
EEP_MAX_PWM                              = 22 ; RAM_MAX_PWM                       = 16 ; RANGE_MAX_PWM                       = [0, 0x03FF]
EEP_OVERLOAD_PWM_THRESHOLD               = 24 ; RAM_OVERLOAD_PWM_THRESHOLD        = 18 ; RANGE_OVERLOAD_PWM_THRESHOLD        = ([0, 0x7FFE], [0, 0x03FF])
EEP_MIN_POSITION                         = 26 ; RAM_MIN_POSITION                  = 20
EEP_MAX_POSITION                         = 28 ; RAM_MAX_POSITION                  = 22
EEP_POSITION_KP                          = 30 ; RAM_POSITION_KP                   = 24 ; RANGE_POSITION_KP                   = [0, 0x7FFF]
EEP_POSITION_KD                          = 32 ; RAM_POSITION_KD                   = 26 ; RANGE_POSITION_KD                   = [0, 0x7FFF]
EEP_POSITION_KI                          = 34 ; RAM_POSITION_KI                   = 28 ; RANGE_POSITION_KI                   = [0, 0x7FFF]
EEP_POSITION_FEEDFORWARD_1ST_GAIN        = 36 ; RAM_POSITION_FEEDFORWARD_1ST_GAIN = 30 ; RANGE_POSITION_FEEDFORWARD_1ST_GAIN = [0, 0x7FFF]
EEP_POSITION_FEEDFORWARD_2ND_GAIN        = 38 ; RAM_POSITION_FEEDFORWARD_2ND_GAIN = 32 ; RANGE_POSITION_FEEDFORWARD_2ND_GAIN = [0, 0x7FFF]
EEP_VELOCITY_KP                          = 40 ; RAM_VELOCITY_KP                   = 34 ; RANGE_VELOCITY_KP                   = [0, 0x7FFF]
EEP_VELOCITY_KI                          = 42 ; RAM_VELOCITY_KI                   = 36 ; RANGE_VELOCITY_KI                   = [0, 0x7FFF]
EEP_LED_BLINK_PERIOD                     = 44 ; RAM_LED_BLINK_PERIOD              = 38 ; RANGE_LED_BLINK_PERIOD              = [0, 0xFE]
EEP_ADC_FAULT_CHECK_PERIOD               = 45 ; RAM_ADC_FAULT_CHECK_PERIOD        = 39 ; RANGE_ADC_FAULT_CHECK_PERIOD        = [0, 0xFE]
EEP_PACKET_GARBAGE_CHECK_PERIOD          = 46 ; RAM_PACKET_GARBAGE_CHECK_PERIOD   = 40 ; RANGE_PACKET_GARBAGE_CHECK_PERIOD   = [0, 0xFE]
EEP_STOP_DETECTION_PERIOD                = 47 ; RAM_STOP_DETECTION_PERIOD         = 41 ; RANGE_STOP_DETECTION_PERIOD         = [0, 0xFE]
EEP_OVERLOAD_DETECTION_PERIOD            = 48 ; RAM_OVERLOAD_DETECTION_PERIOD     = 42 ; RANGE_OVERLOAD_DETECTION_PERIOD     = [0, 0xFE]
EEP_STOP_THRESHOLD                       = 49 ; RAM_STOP_THRESHOLD                = 43 ; RANGE_STOP_THRESHOLD                = [0, 0xFE]
EEP_INPOSITION_MARGIN                    = 50 ; RAM_INPOSITION_MARGIN             = 44 ; RANGE_INPOSITION_MARGIN             = [0, 0xFE]
EEP_CALIBRATION_DIFFERENCE_LOWER         = 52 ; RAM_CALIBRATION_DIFFERENCE_LOWER  = 46 ; RANGE_CALIBRATION_DIFFERENCE_1      = [-255, 255]
EEP_CALIBRATION_DIFFERENCE_UPPER         = 53 ; RAM_CALIBRATION_DIFFERENCE_UPPER  = 47 ; RANGE_CALIBRATION_DIFFERENCE_2      = [-1495, 1495]
RAM_STATUS_ERROR                         = 48 ; RANGE_STATUS_ERROR                = [0, 0x7F]
RAM_STATUS_DETAIL                        = 49 ; RANGE_STATUS_DETAIL               = [0, 0x7F]
RAM_AUX_1                                = 50 ; RANGE_AUX_1                       = [0, 0x06]
RAM_TORQUE_CONTROL                       = 52
RAM_LED_CONTROL                          = 53
RAM_VOLTAGE                              = 54
RAM_TEMPERATURE                          = 55
RAM_CURRENT_CONTROL_MODE                 = 56
RAM_TICK                                 = 57
RAM_CALIBRATED_POSITION                  = 58
RAM_ABSOLUTE_POSITION                    = 60
RAM_DIFFERENTIAL_POSITION                = 62
RAM_PWM                                  = 64
RAM_ABSOLUTE_2ND_POSITION                = 66
RAM_ABSOLUTE_GOAL_POSITION               = 68
RAM_ABSOLUTE_DESIRED_TRAJECTORY_POSITION = 70
RAM_DESIRED_VELOCITY                     = 72

# Acknolegement policy
ACK_MUTE = 0
ACK_READ = 1
ACK_ALL  = 2

# LED color
LED_OFF    = 0x00
LED_GREEN  = 0x01
LED_BLUE   = 0x02
LED_CYAN   = 0x03
LED_RED    = 0x04
LED_ORANGE = 0x05
LED_VIOLET = 0x06
LED_WHITE  = 0x07

# Servo mode
MODE_FREE    = 0x00
MODE_BRAKE   = 0x40
MODE_CONTROL = 0x60

# Control mode
CONTROL_POSITION_DEGREE = 0x00 # degree
CONTROL_POSITION_STEP   = 0x01 # step
CONTROL_SPEED_DEGREE    = 0x10 # degree/second
CONTROL_SPEED_STEP      = 0x11 # step/second
CONTROL_SPEED_NATIVE    = 0x12

# Max positions
POSITION_MAX_1 = 1023
POSITION_MAX_2 = 2047
POSITION_MAX_3 = 32767

# Resolutions
POSITION_RESOLUTION_1 = (360.-26.7)/1023 # about 0.326 deg./step
POSITION_RESOLUTION_2 = (360.-26.7)/2047 # about 0.163 deg./step
POSITION_RESOLUTION_3 = 360./12962       # about 0.02777 deg./step
SPEED_OUTPUT_RESOLUTION_1 = 29.09  # deg./sec.
SPEED_OUTPUT_RESOLUTION_2 = 3.634  # deg./sec.
SPEED_OUTPUT_RESOLUTION_3 = 0.620  # deg./sec.
GOAL_SPEED_RESOLUTION_1   = 0.199  # deg./sec.
GOAL_SPEED_RESOLUTION_2   = 0.034  # deg./sec.
VOLTAGE_RESOLUTION_1 = 18.889/255 # about 0.074 volts
VOLTAGE_RESOLUTION_2 = 0.1        # volts

# Analog digital converter temperature list (for DRS-0101 and DRS-0201)
ADC_TEMPERATURE = (-80.57,-72.89,-64.26,-58.84,-54.80,-51.55,-48.81,-46.43,
                   -44.32,-42.41,-40.68,-39.08,-37.59,-36.20,-34.89,-33.66,
                   -32.49,-31.37,-30.31,-29.29,-28.31,-27.36,-26.45,-25.57,
                   -24.72,-23.89,-23.09,-22.31,-21.54,-20.80,-20.08,-19.37,
                   -18.68,-18.00,-17.34,-16.69,-16.05,-15.42,-14.81,-14.20,
                   -13.61,-13.02,-12.45,-11.88,-11.32,-10.76,-10.22, -9.68,
                    -9.15, -8.62, -8.10, -7.59, -7.08, -6.58, -6.08, -5.59,
                    -5.10, -4.62, -4.14, -3.66, -3.19, -2.72, -2.26, -1.80,
                    -1.34, -0.89, -0.44,  0.01,  0.46,  0.90,  1.34,  1.78,
                     2.21,  2.64,  3.07,  3.50,  3.93,  4.35,  4.77,  5.19,
                     5.61,  6.03,  6.45,  6.86,  7.27,  7.68,  8.09,  8.50,
                     8.91,  9.32,  9.72, 10.13, 10.53, 10.94, 11.34, 11.74,
                    12.14, 12.55, 12.95, 13.35, 13.75, 14.15, 14.54, 14.94,
                    15.34, 15.74, 16.14, 16.54, 16.94, 17.34, 17.74, 18.13,
                    18.53, 18.93, 19.33, 19.73, 20.13, 20.54, 20.94, 21.34,
                    21.74, 22.15, 22.55, 22.96, 23.36, 23.77, 24.18, 24.59,
                    25.00, 25.41, 25.82, 26.24, 26.65, 27.07, 27.49, 27.91,
                    28.33, 28.75, 29.18, 29.60, 30.03, 30.46, 30.89, 31.32,
                    31.76, 32.20, 32.64, 33.08, 33.52, 33.97, 34.42, 34.87,
                    35.33, 35.78, 36.24, 36.71, 37.17, 37.64, 38.11, 38.59,
                    39.07, 39.55, 40.04, 40.53, 41.02, 41.52, 42.02, 42.52,
                    43.03, 43.55, 44.07, 44.59, 45.12, 45.65, 46.19, 46.74,
                    47.29, 47.84, 48.40, 48.97, 49.54, 50.12, 50.71, 51.30,
                    51.90, 52.51, 53.13, 53.75, 54.38, 55.02, 55.67, 56.33,
                    56.99, 57.67, 58.36, 59.05, 59.76, 60.48, 61.21, 61.96,
                    62.71, 63.48, 64.27, 65.06, 65.88, 66.71, 67.55, 68.41,
                    69.29, 70.19, 71.11, 72.05, 73.01, 74.00, 75.01, 76.04,
                    77.10, 78.19, 79.31, 80.46, 81.65, 82.87, 84.13, 85.44,
                    86.78, 88.17, 89.62, 91.12, 92.67, 94.29, 95.98, 97.75,
                    99.59,101.53,103.57,105.71,107.98,110.38,112.93,115.65,
                   118.57,121.72,125.12,128.83,132.89,137.38,142.40,148.06,
                   154.56,162.13,171.18,182.34,196.72,216.58,247.46,310.08)

# Baud rates
BAUDRATES      = (57600, 115200, 200000, 250000, 400000, 500000, 666666, 1000000)
BAUDRATES_CODE = ( 0x22,   0x10,   0x09,   0x07,   0x04,   0x03,   0x02,    0x01)
MAX_BAUDRATE_1 = 666666
MAX_BAUDRATE_2 = 1000000

# Broadcasting ID
BROADCASTING_ID = 0xFE

#Status messages
MSG_SATUS_ERROR  = ('Exceed input voltage',
                    'Exceed allowed position limit', # Exceed allowed POT limit (potentiometer)
                    'Exceed temperature limit',
                    'Invalid packet',
                    'Overload detected',
                    'Driver fault detected',
                    'EEPROM register distorted',     # EEP REG distorted
                    'Unknown')                       # Reserved
MSG_SATUS_DETAIL = ('Moving',                        # Moving flag
                    'Arrived at goal position',      # Inposition flag
                    'Checksum error',
                    'Unknown command',
                    'Exceed register range',         # Exceed REG range
                    'Garbage detected',
                    'Control mode enabled',          # MOTOR_ON flag
                    'Unknown')                       # Reserved

# Packet header
HEADER = struct.pack('2B',0xFF,0xFF)

# Last opened serial connection
LAST_SERIAL = None

# XOR lambda function
xor = lambda a,b:a^b

def _closest(value, tpl):
    """
    Find closest value index in a sorted tuple
    (We don't use external tool such as Numpy to avoid dependencies)
    """
    value = float(value)
    if value <= tpl[0]:
        return 0
    elif value >= tpl[-1]:
        return len(tpl)-1
    else:
        for i in range(1,len(tpl)):
            if tpl[i] > value:
                if (tpl[i] - value) > (value - tpl[i-1]):
                    return i-1
                else:
                    return i

def _check_serial(s):
    """
    Return the default serial port or open it if no serial instance
    is given as argument.
    """
    global LAST_SERIAL
    if s is None:
        if LAST_SERIAL is None:
            return serial()
        else:
            if not LAST_SERIAL.isOpen():
                LAST_SERIAL.open()
            return LAST_SERIAL
    else:
        LAST_SERIAL = s
        return s

def _check_port(port):
    """
    Return the default port name if no serial port name
    is given as argument.
    """
    if port is None:
        if platform.system() == 'Windows':
            port = 'COM3'
        elif platform.system() == 'Linux':
            ports = glob.glob('/dev/ttyUSB*')
            if ports:
                port = ports[0]
            else:
                ports = glob.glob('/dev/ttyS*')
                if ports:
                    port = ports[0]
        elif platform.system() == 'Darwin':
            from serial.tools.list_ports import comports
            for p in comports():
                if p.serial_number:
                    port = '/dev/tty.usbserial-'+p.serial_number
                    break
    if port is None:
        raise PyHerkuleX_Exception("Cannot find any serial port")
    return port


def serial(port = None, baudrate = 115200, timeout = 1.0):
    """
    Open the serial port where HerkuleX servo communication
    bus is attached.

    The output serial port will be considered as default serial port
    anytime a serial port instance is required but not specified.

    :param str port: Serial port name.
                     If not specified, its considered value will be
                     ``'COM3'`` if using Windows,
                     the first ``'/dev/ttyUSB*'`` or ``'/dev/ttyS*'`` found if using Linux or
                     the first ``'/dev/tty.usbserial-*'`` found if using OSx.

    :param int baudrate: Set baud rate value among the possible
                         values supported by the servo model.

    :param float timeout: Set a read timeout value.

    :return: Connection instance of :obj:`serial.Serial`.
    :rtype:  serial.Serial

    :exception SerialException:
        In case the device can not be found or can not be configured.
    """
    global LAST_SERIAL
    port = _check_port(port)
    LAST_SERIAL = srl.Serial(port, baudrate, timeout = timeout)
    try:
        pass
    except:
        raise PyHerkuleX_Exception("Cannot open serial port")
    return LAST_SERIAL

def find(number = None, port = None, baudrate = BAUDRATES, timeout = 0.05, fast = False, verbose = True):
    """
    Scan all possible ID with each supported baud rate
    and return a dictionary which maps connected servo IDs to tuples
    containing HerkuleX servo model and baudrate.

    :param int number: Number of servos supposed to be connected to serial bus.

    :param str port: Serial port name or tuple of several port names to be successively tested.
                     If not specified, each available port will be checked successively,
                     ``'COM1'`` to ``'COM256'`` if using Windows,
                     any ``'/dev/ttyUSB*'`` and ``'/dev/ttyS*'`` if using Linux or
                     any ``'/dev/tty.usbserial-'`` if using OSx.

    :param int baudrate: Baud rate, or tuple of several baud rate values
                         to be successively tested.

    :param float timeout: Set serial read timeout value.

    :param bool fast: For each baud rate, incremental scanning is
                      performed on all possible IDs separately only if any signal is
                      received to a prior call broadcasted to all servo IDs together.

    :param bool verbose: Print scanning progression.

    :return: Dictionary mapping successfully tested ports
             to dictionaries mapping connected servo IDs
             to tuples containing two items each (servo model, baudrate).
    :rtype:  dict
    """
    ports = port
    if ports is None:
        if platform.system() == 'Windows':
            ports = ['COM%s' % (i + 1) for i in range(256)]
        elif platform.system() == 'Linux':
            ports = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyS*')
        elif platform.system() == 'Darwin':
            from serial.tools.list_ports import comports
            ports = []
            for p in comports():
                if p.serial_number:
                    ports.append('/dev/tty.usbserial-'+p.serial_number)
    elif ports is str:
        ports = [ports]
    if isinstance(baudrate, int):
        baudrate = (baudrate,)

    servos = {}
    for port in ports:
        servos[port]= {}
        try:
            serial = srl.Serial(port, baudrate[0], timeout = timeout)
        except (OSError, srl.SerialException):
            break
        if verbose:
            print("Scanning port "+port)
        _servos = {}
        for br in baudrate:
            serial.baudrate = br
            if fast:
                data = [7, BROADCASTING_ID, REQ_STAT]
                check_sum1 = reduce(xor,data)&0xFE
                data.insert(3, check_sum1)
                check_sum2 = (~check_sum1)&0xFE
                data.insert(4, check_sum2)
                found = False
                if verbose:
                    print(" "*4+"Broadcast fast scanning with serial baudrate %i"%br)
                try:
                    serial.write(HEADER + struct.pack(str(5)+'B',*data))
                    found = serial.read(9)
                except:
                    pass
            else:
                found = True
            if found:
                for servoid in range(0x00, 0xFE):
                    if servoid in servos[port]:
                        continue
                    data = [7, servoid, REQ_STAT]
                    check_sum1 = reduce(xor,data)&0xFE
                    data.insert(3, check_sum1)
                    check_sum2 = (~check_sum1)&0xFE
                    data.insert(4, check_sum2)
                    found = False
                    if verbose:
                        print(" "*4+"Serial baudrate %i"%br+", scanning id %i"%servoid+" (0x%02X"%servoid+")")
                    try:
                        serial.write(HEADER + struct.pack(str(5)+'B',*data))
                        found = serial.read(9)
                    except:
                        pass
                    if found:
                        serial.timeout = 1.0
                        srv = Servo(servoid, serial)
                        servos[port][servoid] = (srv.MODEL ,br)
                        _servos[servoid] = (srv, srv.status)
                        serial.timeout = timeout
                        if verbose:
                            print(" "*25+"Found servo id %i"%servoid+" (0x%02X"%servoid+") model %i"%servos[port][servoid][0])
                        if number == len(servos[port]):
                            break
            if number == len(servos[port]):
                break
        for servoid in servos[port]:
            serial.timeout = 1.0
            serial.baudrate = servos[port][servoid][1]
            srv = _servos[servoid][0]
            # Restore the initial error state as it might have been affected during scanning
            srv._ram_write(RAM_STATUS_ERROR, _servos[servoid][1][0], _servos[servoid][1][1])
        serial.close()

    if verbose:
        found = False
        for port in servos:
            if len(servos[port])>0:
                found = True
                message = '\n'
                if number:
                    if number != len(servos[port]):
                        message = 'Only '
                if len(servos[port])>1:
                    print(message+"%i servos found"%len(servos[port]) + " on port "+port+":")
                else:
                    print(message+"%i servo found"%len(servos[port]) + " on port "+port+":")
                for servoid in servos[port]:
                    print("   id %i"%servoid+" (0x%02X"%servoid+")    model %i"%servos[port][servoid][0]+"    baudrate %i"%servos[port][servoid][1])
        if not found:
            print("No servo found.")

    out = {}
    for port in servos:
        if len(servos[port])>0:
            out[port] = servos[port]

    return out

def independent_control(*instructions):
    """
    Send control instructions to several servos at the same time. Each servo has
    its own goal time to complete its instruction.

    :param tuples \*instructions:

        Each instruction is stored in a tuple containing the following items:

            - *goal time*, the operating time to complete the instruction
            - *servo*, an instance of :class:`pyherkulex.Servo`
            - *goal value*, desired position value or desired speed value, depending on *control mode*, or ``None`` for ignoring this control instruction
            - *control mode* (optional), among the :ref:`position and speed control constants <control-constants>` (default is :obj:`pyherkulex.CONTROL_POSITION_DEGREE`)
            - *led color* (optional), integer among the :ref:`LED color constants <led-constants>` (default is :obj:`pyherkulex.LED_OFF`)
            - *stop* (optional), either ``True`` to stop servo at current position or ``False`` (default) to continue control instructions
            - *velocity overide* - VOR (optional), either ``True`` (default) to enable VOR or ``False`` to disable VOR (if the servo model allows to disable VOR)

    .. note::
        Each servo must have its control mode enabled before sending control instructions.

    :example:
        Sending independent control instructions to two servos with respective IDs 0x01 and 0x02.
        The first servo has 1.5 second to reach a prescribed position set to -10 degree and its LED switched off.
        The second servo has 2.0 second to reach a prescribed speed set to 3.5 degree/second and its LED lighting blue.

        .. code-block:: python

            #!/usr/bin/env python

            import pyherkulex as hx

            srv1 = hx.Servo(0x01)
            srv2 = hx.Servo(0x02)

            srv1.mode = srv2.mode = hx.MODE_CONTROL

            hx.independent_instructions((1.5, srv1, -10.0),
                                        (2.0, srv2, 3.5, hx.SPEED_DEGREE, hx.LED_BLUE))
    """
    len_data = len(instructions)*5
    data = [len_data+7, BROADCASTING_ID, REQ_I_JOG]
    for jog in instructions:
        goaltime = int(round(jog[0]/0.0112))
        jog = jog[1:]
        if jog[1] is None:
            goalvalue = 0
            set0 = 0b100000 # Ignore this jog
        else:
            goalvalue = jog[1] # goal value
            set0 = 0
        if len(jog) > 2:
            if   jog[2] == CONTROL_POSITION_DEGREE:                                            # Adapt goal value (not needed for POSITION_STEP)
                goalvalue = jog[0]._angle_to_position(goalvalue)                               #
            elif jog[2] == CONTROL_SPEED_DEGREE:                                               #
                set0 |= 2                                                                      #
                goalvalue = jog[0]._signed_native_speed(goalvalue/jog[0]._speed_res)           #
            elif jog[2] == CONTROL_SPEED_STEP:                                                 #
                set0 |= 2                                                                      #
                goalvalue = jog[0]._signed_native_speed(goalvalue/jog[0]._absolute_speed_res)  #
            elif jog[2] == CONTROL_SPEED_NATIVE:                                               #
                set0 |= 2                                                                      #
                goalvalue = jog[0]._signed_native_speed(goalvalue)                             #
            if len(jog) > 3:
                set0 |= (jog[3]&0x7)<<2 # led color
                if len(jog) > 4:
                    set0 |= bool(jog[4]) # stop
                    if len(jog) == 6:
                        set0 |= (bool(jog[5])^1)<<6 # VOR
        if len(jog) < 3:
            goalvalue = jog[0]._angle_to_position(goalvalue)                                   # Adapt goal value
        goalvalue = int(round(goalvalue))
        data += [goalvalue&0xFF, goalvalue>>8&0xFF, set0, jog[0]._id, goaltime&0xFF]
    check_sum1 = reduce(xor,data)&0xFE
    data.insert(3, check_sum1)
    check_sum2 = (~check_sum1)&0xFE
    data.insert(4, check_sum2)
    try:
        instructions[0][0].serial.write(HEADER + struct.pack(str(len_data+5)+'B',*data))
    except:
        raise PyHerkuleX_Exception("Independent control failed")

def simultaneous_control(time, *instructions):
    """
    Send control instructions to several servos at the same time. All servos have
    the same operating goal time to complete their instruction.

    :param float time: Operating goal time.

    :param tuples \*instructions:

        Each control instruction is stored in a tuple containing the following items:

            - *servo*, an instance of :class:`pyherkulex.Servo`
            - *goal value*, desired position value or desired speed value, depending on *control mode*, or ``None`` for ignoring this control instruction
            - *control mode* (optional), among the :ref:`position and speed control constants <control-constants>` (default is :obj:`pyherkulex.CONTROL_POSITION_DEGREE`)
            - *led color* (optional), integer among the :ref:`LED color constants <led-constants>` (default is :obj:`pyherkulex.LED_OFF`)
            - *stop* (optional), either ``True`` to stop servo at current position or ``False`` (default) to continue control instructions
            - *velocity override* - VOR (optional), either ``True`` (default) to enable VOR or ``False`` to disable VOR (if the servo model allows to disable VOR)

    .. note::
        Each servo must have its control mode enabled before sending control instructions.

    :example:
        Sending simultaneous instructions to two servos with respective IDs 0x01 and 0x02.
        The first servo has its position set to -10 degree with its LED switched off.
        The second servo has its speed set to 3.5 degree/second with its LED lighting green.
        Both servos have the same operating goal time of 1.5 second.

        .. code-block:: python

            #!/usr/bin/env python

            import pyherkulex as hx

            srv1 = hx.Servo(0x01)
            srv2 = hx.Servo(0x02)

            srv1.mode = srv2.mode = hx.MODE_CONTROL

            hx.simultaneous_control(1.5, (srv1, -10.0),
                                         (srv2, 3.5, hx.SPEED_DEGREE, hx.LED_GREEN))
    """
    len_data = len(instructions)*4
    data = [len_data+8, BROADCASTING_ID, REQ_S_JOG, int(round(time/0.0112))&0xFF]
    for jog in instructions:
        if jog[1] is None:
            goalvalue = 0
            set0 = 0b100000 # Ignore this jog
        else:
            goalvalue = jog[1] # goal value
            set0 = 0
        if len(jog) > 2:
            if   jog[2] == CONTROL_POSITION_DEGREE:                                            # Adapt goal value (not needed for POSITION_STEP)
                goalvalue = jog[0]._angle_to_position(goalvalue)                               #
            elif jog[2] == CONTROL_SPEED_DEGREE:                                               #
                set0 |= 2                                                                      #
                goalvalue = jog[0]._signed_native_speed(goalvalue/jog[0]._speed_res)           #
            elif jog[2] == CONTROL_SPEED_STEP:                                                 #
                set0 |= 2                                                                      #
                goalvalue = jog[0]._signed_native_speed(goalvalue/jog[0]._absolute_speed_res)  #
            elif jog[2] == CONTROL_SPEED_NATIVE:                                               #
                set0 |= 2                                                                      #
                goalvalue = jog[0]._signed_native_speed(goalvalue)                             #
            if len(jog) > 3:
                set0 |= (jog[3]&0x7)<<2 # led color
                if len(jog) > 4:
                    set0 |= bool(jog[4]) # stop
                    if len(jog) == 6:
                        set0 |= (bool(jog[5])^1)<<6 # VOR
        if len(jog) < 3:
            goalvalue = jog[0]._angle_to_position(goalvalue)                                   # Adapt goal value
        goalvalue = int(round(goalvalue))
        data += [goalvalue&0xFF, goalvalue>>8&0xFF, set0, jog[0]._id]
    check_sum1 = reduce(xor,data)&0xFE
    data.insert(3, check_sum1)
    check_sum2 = (~check_sum1)&0xFE
    data.insert(4, check_sum2)
    try:
        instructions[0][0].serial.write(HEADER + struct.pack(str(len_data+6)+'B',*data))
    except:
        raise PyHerkuleX_Exception("Simultaneous control failed")

class PyHerkuleX_Exception(Exception):
    """
    Base class that sets how HerkuleX servo raised exceptions are handled.
    """
    def __init__(self,  message):
        """
        Args:
        message (string): Message to be displayed to user.
        """
        self.message = message
        super(PyHerkuleX_Exception, self).__init__(message)

class Servo(object):
    """
    Represent a connected Smart HerkuleX Actuator.

    Servo class allows to establish serial connection with servos,
    access configuration parameters, get sensor feedbacks and
    send control instructions.

    :param int servoid: The ID of the servo.
        If not provided, the :class:`Servo` instance will broadcast
        any command to all connected servo.

    :param serial.Serial serial: port instance of :obj:`serial.Serial` .
        If not provided, the last used serial port will be used.
        If no serial port were used yet, an auto detection will be attempted.

    :example:
        .. code-block:: python

            #!/usr/bin/env python

            import pyherkulex as hx
            import time

            # Open the serial port (auto detected) with default baud rate 115200
            serial0 = hx.serial()

            # Broadcast reboot command to all servos connected to serial0
            # and light their LED in white
            broadcast_srv = hx.Servo(serial = serial0)
            broadcast_srv.reboot()
            broadcast_srv.led = hx.LED_WHITE

            # Create an instance of servo ID 0x01
            # (considering the last used serial port
            # as no specific port parameter is provided)
            srv1 = hx.Servo(0x01)

            # Check first if there is any error status
            if srv1.status[0]:
                # Print status and raise an exception
                srv1.print_status()
                raise Exception('Please check your HerkuleX servo error status.')

            # Print initial position (in step)
            print('Servo 1 initial position: ', srv1.position)

            # Get initial angular position (in degree)
            angle0 = srv1.angle

            # Enable control mode before sending control instructions
            srv1.mode = hx.MODE_CONTROL

            # Increase angular position of 10 degree
            # to be completed in an operating goal time of 2 second
            # and light LED in blue
            srv1.control_angle(angle0+10.0, 2.0, hx.LED_BLUE)

            # Wait for control instruction to be completed
            time.sleep(2.0)

            # Print current position (in step)
            print('Servo 1 current position: ', srv1.position)

            # Go back to initial position
            # to be reached in an operating goal time of 1.5 second
            # and light LED in green
            srv1.control_angle(angle0, 1.5, hx.LED_GREEN)

            # Wait for control instruction to be completed
            time.sleep(1.5)

            # Switch off LED of servo 1
            srv1.led = hx.LED_OFF
    """

    def __init__(self, servoid = None, serial = None):
        """
        Servo instance initialization.
        """
        self.serial = _check_serial(serial)
        self._alarm_led_policy = {}
        if servoid is None:
            self._id = BROADCASTING_ID
            self._ack_policy = None
            self._model = None
        else:
            # If this ID is not valid, we will know right after, when trying to read its model number
            self._id = int(servoid)
            # Set _ack_policy by calling _init_ack_policy() and _model
            self._init_ack_policy()
            if self._ack_policy == ACK_MUTE:
                self.ack_policy = ACK_READ
                data = self._eep_read(EEP_MODEL_NO1)
                self._model = data[0]*100+data[1]
                self.ack_policy = ACK_MUTE
            else:
                data = self._eep_read(EEP_MODEL_NO1)
                self._model = data[0]*100+data[1]
            if data[0] == 1 or data[0] == 2:
                self._position_max                 = POSITION_MAX_1
                self._position_res                 = POSITION_RESOLUTION_1
                self._angular_speed_output_res     = SPEED_OUTPUT_RESOLUTION_1
                self._angular_goal_speed_res       = GOAL_SPEED_RESOLUTION_1
                self._voltage_res                  = VOLTAGE_RESOLUTION_1
                self._baudrates = [i for i in BAUDRATES if i <= MAX_BAUDRATE_1]
            elif data[0] == 4 or data[0] == 6:
                if data[1] == 1:
                    self._position_max             = POSITION_MAX_2
                    self._position_res             = POSITION_RESOLUTION_2
                    self._angular_speed_output_res = SPEED_OUTPUT_RESOLUTION_2
                    self._angular_goal_speed_res   = GOAL_SPEED_RESOLUTION_1
                else:
                    self._position_max             = POSITION_MAX_3
                    self._position_res             = POSITION_RESOLUTION_3
                    self._angular_speed_output_res = SPEED_OUTPUT_RESOLUTION_3
                    self._angular_goal_speed_res   = GOAL_SPEED_RESOLUTION_2
                self._voltage_res        = VOLTAGE_RESOLUTION_2
                self._baudrates = [i for i in BAUDRATES if i <= MAX_BAUDRATE_2]
            else:
                raise PyHerkuleX_Exception("Unknown model "+str(self._model)+".")
            self._raw_position_res          = POSITION_RESOLUTION_2
            self._angular_speed_input_res   = 0.62
            self._absolute_speed_input_res  = self._angular_speed_input_res/self._position_res
            self._absolute_speed_output_res = self._angular_speed_output_res/self._position_res
            self._absolute_goal_speed_res   = self._angular_goal_speed_res/self._position_res
            self._position_middle           = self._position_max/2+1
            self._magnetic_encoder          = bool(data[1]-1)
            self._model_4or6                = int(data[0] == 4 or data[0] == 6)

    def _write(self, *data):
        """
        Write data to the HerkuleX servo port.
        """
        len0 = len(data)
        data = list(data)
        data.insert(0, self._id)
        data.insert(0, len0+6)
        check_sum1 = reduce(xor,data)&0xFE
        data.insert(3, check_sum1)
        check_sum2 = (~check_sum1)&0xFE
        data.insert(4, check_sum2)
        try:
            self.serial.write(HEADER + struct.pack(str(len0+4)+'B',*data))
        except:
            raise PyHerkuleX_Exception("\nCould not communicate with servo ID: 0x%02X"%self._id+" while sending command:\n"+"0xFF, 0xFF, "+', '.join("0x%02X"%i for i in data))
        if self._ack_policy == ACK_ALL:
            if data[2] not in (REQ_EEP_READ, REQ_RAM_READ, REQ_STAT) and self._id != BROADCASTING_ID:
                len0 = struct.unpack('1B',self.serial.read(3)[2:3])[0]-3
                if struct.unpack('1B',self.serial.read(len0)[1:2])[0] != data[2]+0x40:
                    raise PyHerkuleX_Exception("Wrong acknowledgment with servo ID: 0x%02X"%self._id)

    def _ram_read(self, register, length = 2):
        """
        Read the RAM register from the HerkuleX servo port.
        """
        self._write(REQ_RAM_READ, register, length)
        try:
            return struct.unpack(str(length)+'B',self.serial.read(11+length)[9:9+length])
        except:
            if self._id == BROADCASTING_ID:
                raise PyHerkuleX_Exception("Can not get any reply when using broadcast ID")
            elif self._ack_policy == ACK_MUTE:
                raise PyHerkuleX_Exception("Acknowledgment policy does not allow to communicate with servo ID: 0x%02X"%self._id+". Change acknowledgment policy and try again.")
            else:
                raise PyHerkuleX_Exception("Could not communicate with servo ID: 0x%02X"%self._id)

    def _ram_read_single(self, register):
        """
        Read one single RAM register byte from the HerkuleX servo port.
        """
        self._write(REQ_RAM_READ, register, 1)
        try:
            return struct.unpack('1B',self.serial.read(12)[9:10])[0]
        except:
            if self._id == BROADCASTING_ID:
                raise PyHerkuleX_Exception("Can not get any reply when using broadcast ID")
            elif self._ack_policy == ACK_MUTE:
                raise PyHerkuleX_Exception("Acknowledgment policy does not allow to communicate with servo ID: 0x%02X"%self._id+". Change acknowledgment policy and try again.")
            else:
                raise PyHerkuleX_Exception("Could not communicate with servo ID: 0x%02X"%self._id)

    def _ram_write(self, register, *data):
        """
        Write data to the RAM register from the HerkuleX servo port.
        """
        self._write(REQ_RAM_WRITE, register, len(data), *data)

    def _eep_read(self, register, length = 2):
        """
        Read the EEPROM register from the HerkuleX servo port.
        """
        self._write(REQ_EEP_READ, register, length)
        try:
            return struct.unpack(str(length)+'B',self.serial.read(11+length)[9:9+length])
        except:
            if self._id == BROADCASTING_ID:
                raise PyHerkuleX_Exception("Can not get any reply when using broadcast ID")
            elif self._ack_policy == ACK_MUTE:
                raise PyHerkuleX_Exception("Acknowledgment policy does not allow to communicate with servo ID: 0x%02X"%self._id+". Change acknowledgment policy and try again.")
            else:
                raise PyHerkuleX_Exception("Could not communicate with servo ID: 0x%02X"%self._id)

    def _eep_read_single(self, register):
        """
        Read one single EEPROM register byte from the HerkuleX servo port.
        """
        self._write(REQ_EEP_READ, register, 1)
        try:
            return struct.unpack('1B',self.serial.read(12)[9:10])[0]
        except:
            if self._id == BROADCASTING_ID:
                raise PyHerkuleX_Exception("Can not get any reply when using broadcast ID")
            elif self._ack_policy == ACK_MUTE:
                raise PyHerkuleX_Exception("Acknowledgment policy does not allow to communicate with servo ID: 0x%02X"%self._id+". Change acknowledgment policy and try again.")
            else:
                raise PyHerkuleX_Exception("Could not communicate with servo ID: 0x%02X"%self._id)

    def _eep_write(self, register, *data):
        """
        Write data to the EEPROM register from the HerkuleX servo port.
        """
        self._write(REQ_EEP_WRITE, register, len(data), *data)

#    def _ram2eep(ramregister, eepregister, length = 1):
#        """
#        Copy RAM register data to corresponding EEPROM register from the HerkuleX servo port.
#        """
#        self._write(REQ_EEP_WRITE, eepregister, length, self._ram_read(self, ramregister, length))

    def _check_range(self, value, rng):
        """
        Check if the given value is in the permitted range.
        """
        if value < rng[0]:
            raise PyHerkuleX_Exception("Attempt to set a value ("+str(value)+") bellow the minimal value ("+str(rng[0])+") to servo ID: 0x%02X"%self._id)
        if value > rng[1]:
            raise PyHerkuleX_Exception("Attempt to set a value ("+str(value)+") over the maximal value ("+str(rng[1])+")  to servo ID: 0x%02X"%self._id)

    def _check_range_coef(self, value, rng, coef):
        """
        Check if the given value is in the permitted range multiplied by a given coefficient.
        """
        self._check_range(value, [coef*i for i in rng])

    def _position_to_angle(self, position):
        """
        Convert position to angle.
        """
        return self._position_res*(position - self._position_middle)

    def _angle_to_position(self, angle):
        """
        Convert angle to position.
        """
        return int(round(angle/self._position_res))+self._position_middle

    def _signed_native_speed(self, speed):
        """
        Convert signed speed into native speed value.
        """
        if speed<0:
            return abs(int(round(speed)))&0x3FFF|0x4000
        else:
            return int(round(speed))&0x3FFF
# 1 & 2
    @property
    def MODEL(self):
        """
        HerkuleX servo model value (int) corresponding to the model number
        (e.g. 201 corresponds to DRS-0201)
        """
        return self._model
# 3 & 4
    @property
    def VERSION(self):
        """
        HerkuleX servo firmware version (int)
        """
        data = self._eep_read(EEP_VERSION1)
        return data[0]<<8|data[1]
# Various properties
    @property
    def POSITION_RESOLUTION_DEGREE(self):
        """
        HerkuleX servo angular position resolution in degree/step (float)
        """
        return self._position_res
#
    @property
    def POSITION_MAX(self):
        """
        HerkuleX servo maximum supported operating position in step (int)
        """
        return self._position_max
#
    @property
    def SPEED_INPUT_RESOLUTION_STEP(self):
        """
        HerkuleX servo absolute speed input resolution in step/second (float)
        """
        return self._absolute_speed_input_res
#
    @property
    def SPEED_OUTPUT_RESOLUTION_STEP(self):
        """
        HerkuleX servo absolute speed output resolution in step/second (float)
        """
        return self._absolute_speed_output_res
#
    @property
    def SPEED_INPUT_RESOLUTION_DEGREE(self):
        """
        HerkuleX servo angular speed input resolution in degree/second (float)
        """
        return self._angular_speed_input_res
#
    @property
    def SPEED_OUTPUT_RESOLUTION_DEGREE(self):
        """
        HerkuleX servo angular speed output resolution in degree/second (float)
        """
        return self._angular_speed_output_res
#
    @property
    def GOAL_SPEED_RESOLUTION_STEP(self):
        """
        HerkuleX servo absolute goal speed resolution in step/second (float)
        """
        return self._absolute_goal_speed_res
#
    @property
    def GOAL_SPEED_RESOLUTION_DEGREE(self):
        """
        HerkuleX servo angular goal speed resolution in degree/second (float)
        """
        return self._angular_goal_speed_res
# Various methods
    def reset(self, idskip = True, baudrateskip = True, sleep = 2.0):
        """
        Reset all memory registers to factory default values.

        :param bool idskip: Set to True to skip ID reset.
        :param bool baudrateskip: Set to True to skip baud rate reset.
        :param float sleep: Suspension time for rebooting.
        """
        self._write(REQ_ROLLBACK, int(bool(idskip)), int(bool(baudrateskip)))
        self.reboot(sleep = sleep)
        self._ack_policy = 1
        if not idskip:
            self._id = None
#
    def reboot(self, sleep = 2.0):
        """
        Reboot and copy data from EEPROM register to RAM register.

        :param float sleep: Suspension time for rebooting.
        """
        self._write(REQ_REBOOT)
        time.sleep(sleep)
        if self._id != BROADCASTING_ID:
            # Reset _ack_policy by calling _init_ack_policy()
            self._init_ack_policy()
#
    def clear_status(self):
        """
        Clear error status and detailed status.
        """
        self._ram_write(RAM_STATUS_ERROR, 0, 0)
#
    def print_status(self):
        """
        Print status error and status detail.
        """
        stat0, stat1 = self.status
        print('Status error:')
        if stat0:
            for i in range(8):
                if stat0&2**i:
                    if stat0 == 8:
                        for j in range(2,6):
                            if stat1&2**j:
                                print('    - '+MSG_SATUS_ERROR[i]+': '+MSG_SATUS_DETAIL[j])
                    else:
                        print('    - '+MSG_SATUS_ERROR[i])

        else:
            print('    - None')
        print('Status details:')
        detail = False
        if stat1:
            for i in (0,1,6,7):
                if stat1&0x02**i:
                    detail = True
                    print('    - '+MSG_SATUS_DETAIL[i])
        if not detail:
            print('    - None')
    @property
    def status(self):
        """
        :getter: Get status
        :setter: Manually set status
        :type: tuple of two int values (error status code, detailed status code)
        """
        # Always reply regadless of acknowledgment policy.
        if self._id != BROADCASTING_ID:
            self._write(REQ_STAT)
            try:
                return struct.unpack('2B',self.serial.read(9)[7:])
            except:
                raise PyHerkuleX_Exception("Could not communicate with servo ID: 0x%02X"%self._id)
        else:
            raise PyHerkuleX_Exception("Could not get status when using broadcast ID")
    @status.setter
    def status(self, value):
        self._check_range(value[0], RANGE_STATUS_ERROR)
        self._check_range(value[1], RANGE_STATUS_DETAIL)
        self._ram_write(RAM_STATUS_ERROR, value[0], value[1])
# 43
    @property
    def _status_error(self):
        """
        :getter: Get error status
        :setter: Manually set error status
        :type: int
        """
        return self._ram_read_single(RAM_STATUS_ERROR)
    @_status_error.setter
    def _status_error(self, value):
        self._check_range(value, RANGE_STATUS_ERROR)
        self._ram_write(RAM_STATUS_ERROR, int(value))
# 44
    @property
    def _status_detail(self):
        """
        :getter: Get detailed status
        :setter: Manually set detailed status
        :type: int
        """
        return self._ram_read_single(RAM_STATUS_DETAIL)
    @_status_detail.setter
    def _status_detail(self, value):
        self._check_range(value, RANGE_STATUS_DETAIL)
        self._ram_write(RAM_STATUS_DETAIL, int(value))
# 5
    @property
    def baudrate(self):
        """
        :getter: Get baud rate in bps
        :setter: Set baud rate in bps (save it in EEPROM)
        :type:   int among the values supported by the HerkuleX servo model
        """
        return BAUDRATES[BAUDRATES_CODE.index(self._eep_read_single(EEP_BAUD_RATE))]
    @baudrate.setter
    def baudrate(self, value):
        if value in self._baudrates:
            self._eep_write(EEP_BAUD_RATE, BAUDRATES_CODE[BAUDRATES.index(value)])
        else:
            raise PyHerkuleX_Exception("Not supported baudrate %i for this servo model."%value)
# 7
    @property
    def id(self):
        """
        :getter: Get servo ID
        :setter: Set servo ID
        :type:   int from 0 to 254 (0xFE), with 254 corresponding to broadcast ID.
        """
        return self._id
    @id.setter
    def id(self, value):
        value = int(value)
        # If it is a new ID value
        if value != self._id:
            # Test if the new ID value is permitted
            if value < 0 or value >= BROADCASTING_ID:
                raise PyHerkuleX_Exception("ID value %i is not permitted."%value)
            # If there is no ID so far
            if self._id is None:
                # This is possible only if a prior factory rollback was called
                self.__init__(servoid = value, serial = self.serial)
            else:
                # Set the new ID
                self._ram_write(RAM_ID, value)
                # Update the ID variable
                self._id = value
    @property
    def id_eeprom(self):
        """
        :getter: Get saved ID from EEPROM
        :setter: Save ID in EEPROM
        :type:   int
        """
        return self._eep_read_single(EEP_SERVO_ID)
    @id_eeprom.setter
    def id_eeprom(self, value):
        value = int(value)
        # Test if the ID value is permitted
        if value < 0 or value >= BROADCASTING_ID:
            raise PyHerkuleX_Exception("ID value %i is not permitted."%value)
        self._eep_write(EEP_ID, value)
# 48
    @property
    def led(self):
        """
        :getter: Get LED color
        :setter: Set LED color
        :type:   int

        Supported values are given as :ref:`LED color constants <led-constants>`.
        """
        return self._ram_read_single(RAM_LED_CONTROL)
    @led.setter
    def led(self, value):
        value = int(value)
        if value>>3:
            raise PyHerkuleX_Exception("LED color value %i is not supported."%value)
        self._ram_write(RAM_LED_CONTROL, value)
# 8
    def _init_ack_policy(self):
        """
        Init _ack_policy by reading its value from the RAM.
        """
        # Either _ack_policy is defined or is None, we assume it is ACK_READ
        self._ack_policy = ACK_READ
        try:
            self._ack_policy = self._ram_read_single(RAM_ACK_POLICY)
        except:
            # Change ack policy in case it was ACK_MUTE
            self.ack_policy = ACK_READ
            # Read again and see if it works now, otherwise an error will be raised
            self._ram_read_single(RAM_ACK_POLICY)
            # If no error has been raised, that means it was ACK_MUTE and we reset its initial value
            self.ack_policy = ACK_MUTE
    @property
    def ack_policy(self):
        """
        :getter: Get acknowledgment policy
        :setter: Set acknowledgment policy
        :type:   int

        Three acknowlegement policy values are supported:
          - :ref:`ACK_MUTE <ack-constants>`:
            With this policy, servo will not reply to any request
            except status requests.
          - :ref:`ACK_READ <ack-constants>`:
            With this policy, servo will only reply to read (getter)
            and status requests.
          - :ref:`ACK_ALL <ack-constants>`:
            With this policy, servo will reply to all requests.
        """
        return self._ack_policy
    @ack_policy.setter
    def ack_policy(self, value):
        value = int(value)
        if not value in (ACK_MUTE, ACK_READ, ACK_ALL):
            raise PyHerkuleX_Exception("Attempt to set a wrong value of acknowledgment policy to servo ID: 0x%02X"%self._id)
        # Update ack_policy variable
        self._ack_policy = value
        # Set the new ack_policy
        self._ram_write(RAM_ACK_POLICY, value)
    @property
    def ack_policy_eeprom(self):
        """
        :getter: Get saved acknowledgment policy from EEPROM
        :setter: Save acknowledgment policy in EEPROM
        :type:   int
        """
        return self._eep_read_single(EEP_ACK_POLICY)
    @ack_policy_eeprom.setter
    def ack_policy_eeprom(self, value):
        value = int(value)
        if not value in (ACK_MUTE, ACK_READ, ACK_ALL):
            raise PyHerkuleX_Exception("Attempt to save a wrong value of acknowledgment policy to servo ID: 0x%02X"%self._id)
        self._eep_write(EEP_ACK_POLICY, value)
# 9
    @property
    def voltage_alarm_led_policy(self):
        """
        :getter: Get voltage alarm led policy
        :setter: Set voltage alarm led policy
        :type:   bool
        """
        return bool(self._ram_read_single(RAM_ALARM_LED_POLICY)&0x01)
    @voltage_alarm_led_policy.setter
    def voltage_alarm_led_policy(self, value):
        mask = 1<<0
        self._ram_write(RAM_ALARM_LED_POLICY, (self._ram_read_single(RAM_ALARM_LED_POLICY) & ~mask)|((bool(value)<<0)&mask))
    @property
    def voltage_alarm_led_policy_eeprom(self):
        """
        :getter: Get saved voltage alarm led policy from EEPROM
        :setter: Save voltage alarm led policy in EEPROM
        :type:   bool
        """
        return bool(self._eep_read_single(EEP_ALARM_LED_POLICY)&0x01)
    @voltage_alarm_led_policy_eeprom.setter
    def voltage_alarm_led_policy_eeprom(self, value):
        mask = 1<<0
        self._eep_write(EEP_ALARM_LED_POLICY, (self._eep_read_single(EEP_ALARM_LED_POLICY) & ~mask)|((bool(value)<<0)&mask))
#
    @property
    def position_alarm_led_policy(self):
        """
        :getter: Get position alarm led policy
        :setter: Set position alarm led policy
        :type:   bool
        """
        return bool(self._ram_read_single(RAM_ALARM_LED_POLICY)&0x02)
    @position_alarm_led_policy.setter
    def position_alarm_led_policy(self, value):
        mask = 1<<1
        self._ram_write(RAM_ALARM_LED_POLICY, (self._ram_read_single(RAM_ALARM_LED_POLICY) & ~mask)|((bool(value)<<1)&mask))
    @property
    def position_alarm_led_policy_eeprom(self):
        """
        :getter: Get saved position alarm led policy from EEPROM
        :setter: Save position alarm led policy in EEPROM
        :type:   bool
        """
        return bool(self._eep_read_single(EEP_ALARM_LED_POLICY)&0x02)
    @position_alarm_led_policy_eeprom.setter
    def position_alarm_led_policy_eeprom(self, value):
        mask = 1<<1
        self._eep_write(EEP_ALARM_LED_POLICY, (self._eep_read_single(EEP_ALARM_LED_POLICY) & ~mask)|((bool(value)<<1)&mask))
#
    @property
    def packet_alarm_led_policy(self):
        """
        :getter: Get packet alarm led policy
        :setter: Set packet alarm led policy
        :type:   bool
        """
        return bool(self._ram_read_single(RAM_ALARM_LED_POLICY)&0x04)
    @packet_alarm_led_policy.setter
    def packet_alarm_led_policy(self, value):
        mask = 1<<2
        self._ram_write(RAM_ALARM_LED_POLICY, (self._ram_read_single(RAM_ALARM_LED_POLICY) & ~mask)|((bool(value)<<2)&mask))
    @property
    def packet_alarm_led_policy_eeprom(self):
        """
        :getter: Get saved packet alarm led policy from EEPROM
        :setter: Save packet alarm led policy in EEPROM
        :type:   bool
        """
        return bool(self._eep_read_single(EEP_ALARM_LED_POLICY)&0x04)
    @packet_alarm_led_policy_eeprom.setter
    def packet_alarm_led_policy_eeprom(self, value):
        mask = 1<<2
        self._eep_write(EEP_ALARM_LED_POLICY, (self._eep_read_single(EEP_ALARM_LED_POLICY) & ~mask)|((bool(value)<<2)&mask))
#
    @property
    def overload_alarm_led_policy(self):
        """
        :getter: Get overload alarm led policy
        :setter: Set overload alarm led policy
        :type:   bool
        """
        return bool(self._ram_read_single(RAM_ALARM_LED_POLICY)&0x08)
    @overload_alarm_led_policy.setter
    def overload_alarm_led_policy(self, value):
        mask = 1<<3
        self._ram_write(RAM_ALARM_LED_POLICY, (self._ram_read_single(RAM_ALARM_LED_POLICY) & ~mask)|((bool(value)<<3)&mask))
    @property
    def overload_alarm_led_policy_eeprom(self):
        """
        :getter: Get saved overload alarm led policy from EEPROM
        :setter: Save overload alarm led policy in EEPROM
        :type:   bool
        """
        return bool(self._eep_read_single(EEP_ALARM_LED_POLICY)&0x08)
    @overload_alarm_led_policy_eeprom.setter
    def overload_alarm_led_policy_eeprom(self, value):
        mask = 1<<3
        self._eep_write(EEP_ALARM_LED_POLICY, (self._eep_read_single(EEP_ALARM_LED_POLICY) & ~mask)|((bool(value)<<3)&mask))
#
    @property
    def driver_alarm_led_policy(self):
        """
        :getter: Get driver alarm led policy
        :setter: Set driver alarm led policy
        :type:   bool
        """
        return bool(self._ram_read_single(RAM_ALARM_LED_POLICY)&0x10)
    @driver_alarm_led_policy.setter
    def driver_alarm_led_policy(self, value):
        mask = 1<<4
        self._ram_write(RAM_ALARM_LED_POLICY, (self._ram_read_single(RAM_ALARM_LED_POLICY) & ~mask)|((bool(value)<<4)&mask))
    @property
    def driver_alarm_led_policy_eeprom(self):
        """
        :getter: Get saved driver alarm led policy from EEPROM
        :setter: Save driver alarm led policy in EEPROM
        :type:   bool
        """
        return bool(self._eep_read_single(EEP_ALARM_LED_POLICY)&0x10)
    @driver_alarm_led_policy_eeprom.setter
    def driver_alarm_led_policy_eeprom(self, value):
        mask = 1<<4
        self._eep_write(EEP_ALARM_LED_POLICY, (self._eep_read_single(EEP_ALARM_LED_POLICY) & ~mask)|((bool(value)<<4)&mask))
#
    @property
    def register_alarm_led_policy(self):
        """
        :getter: Get register alarm led policy
        :setter: Set register alarm led policy
        :type:   bool
        """
        return bool(self._ram_read_single(RAM_ALARM_LED_POLICY)&0x20)
    @register_alarm_led_policy.setter
    def register_alarm_led_policy(self, value):
        mask = 1<<5
        self._ram_write(RAM_ALARM_LED_POLICY, (self._ram_read_single(RAM_ALARM_LED_POLICY) & ~mask)|((bool(value)<<5)&mask))
    @property
    def register_alarm_led_policy_eeprom(self):
        """
        :getter: Get saved register alarm led policy from EEPROM
        :setter: Save register alarm led policy in EEPROM
        :type:   bool
        """
        return bool(self._eep_read_single(EEP_ALARM_LED_POLICY)&0x20)
    @register_alarm_led_policy_eeprom.setter
    def register_alarm_led_policy_eeprom(self, value):
        mask = 1<<5
        self._eep_write(EEP_ALARM_LED_POLICY, (self._eep_read_single(EEP_ALARM_LED_POLICY) & ~mask)|((bool(value)<<5)&mask))
# 10
    @property
    def voltage_mode_free_policy(self):
        """
        :getter: Get voltage mode free policy
        :setter: Set voltage mode free policy
        :type:   bool
        """
        return bool(self._ram_read_single(RAM_TORQUE_POLICY)&0x01)
    @voltage_mode_free_policy.setter
    def voltage_mode_free_policy(self, value):
        mask = 1<<0
        self._ram_write(RAM_TORQUE_POLICY, (self._ram_read_single(RAM_TORQUE_POLICY) & ~mask)|((bool(value)<<0)&mask))
    @property
    def voltage_mode_free_policy_eeprom(self):
        """
        :getter: Get saved voltage mode free policy from EEPROM
        :setter: Save voltage mode free policy in EEPROM
        :type:   bool
        """
        return bool(self._eep_read_single(EEP_TORQUE_POLICY)&0x01)
    @voltage_mode_free_policy_eeprom.setter
    def voltage_mode_free_policy_eeprom(self, value):
        mask = 1<<0
        self._eep_write(EEP_TORQUE_POLICY, (self._eep_read_single(EEP_TORQUE_POLICY) & ~mask)|((bool(value)<<0)&mask))
#
    @property
    def position_mode_free_policy(self):
        """
        :getter: Get position mode free policy
        :setter: Set position mode free policy
        :type:   bool
        """
        return bool(self._ram_read_single(RAM_TORQUE_POLICY)&0x02)
    @position_mode_free_policy.setter
    def position_mode_free_policy(self, value):
        mask = 1<<0
        self._ram_write(RAM_TORQUE_POLICY, (self._ram_read_single(RAM_TORQUE_POLICY) & ~mask)|((bool(value)<<0)&mask))
    @property
    def position_mode_free_policy_eeprom(self):
        """
        :getter: Get saved position mode free policy from EEPROM
        :setter: Save position mode free policy in EEPROM
        :type:   bool
        """
        return bool(self._eep_read_single(EEP_TORQUE_POLICY)&0x02)
    @position_mode_free_policy_eeprom.setter
    def position_mode_free_policy_eeprom(self, value):
        mask = 1<<0
        self._eep_write(EEP_TORQUE_POLICY, (self._eep_read_single(EEP_TORQUE_POLICY) & ~mask)|((bool(value)<<0)&mask))
#
    @property
    def packet_mode_free_policy(self):
        """
        :getter: Get packet mode free policy
        :setter: Set packet mode free policy
        :type:   bool
        """
        return bool(self._ram_read_single(RAM_TORQUE_POLICY)&0x04)
    @packet_mode_free_policy.setter
    def packet_mode_free_policy(self, value):
        mask = 1<<1
        self._ram_write(RAM_TORQUE_POLICY, (self._ram_read_single(RAM_TORQUE_POLICY) & ~mask)|((bool(value)<<1)&mask))
    @property
    def packet_mode_free_policy_eeprom(self):
        """
        :getter: Get saved packet mode free policy from EEPROM
        :setter: Save packet mode free policy in EEPROM
        :type:   bool
        """
        return bool(self._eep_read_single(EEP_TORQUE_POLICY)&0x04)
    @packet_mode_free_policy_eeprom.setter
    def packet_mode_free_policy_eeprom(self, value):
        mask = 1<<1
        self._eep_write(EEP_TORQUE_POLICY, (self._eep_read_single(EEP_TORQUE_POLICY) & ~mask)|((bool(value)<<1)&mask))
#
    @property
    def overload_mode_free_policy(self):
        """
        :getter: Get overload mode free policy
        :setter: Set overload mode free policy
        :type:   bool
        """
        return bool(self._ram_read_single(RAM_TORQUE_POLICY)&0x08)
    @overload_mode_free_policy.setter
    def overload_mode_free_policy(self, value):
        mask = 1<<2
        self._ram_write(RAM_TORQUE_POLICY, (self._ram_read_single(RAM_TORQUE_POLICY) & ~mask)|((bool(value)<<2)&mask))
    @property
    def overload_mode_free_policy_eeprom(self):
        """
        :getter: Get saved overload mode free policy from EEPROM
        :setter: Save overload mode free policy in EEPROM
        :type:   bool
        """
        return bool(self._eep_read_single(EEP_TORQUE_POLICY)&0x08)
    @overload_mode_free_policy_eeprom.setter
    def overload_mode_free_policy_eeprom(self, value):
        mask = 1<<3
        self._eep_write(EEP_TORQUE_POLICY, (self._eep_read_single(EEP_TORQUE_POLICY) & ~mask)|((bool(value)<<3)&mask))
#
    @property
    def driver_mode_free_policy(self):
        """
        :getter: Get driver mode free policy
        :setter: Set driver mode free policy
        :type:   bool
        """
        return bool(self._ram_read_single(RAM_TORQUE_POLICY)&0x10)
    @driver_mode_free_policy.setter
    def driver_mode_free_policy(self, value):
        mask = 1<<4
        self._ram_write(RAM_TORQUE_POLICY, (self._ram_read_single(RAM_TORQUE_POLICY) & ~mask)|((bool(value)<<4)&mask))
    @property
    def driver_mode_free_policy_eeprom(self):
        """
        :getter: Get saved driver mode free policy from EEPROM
        :setter: Save driver mode free policy in EEPROM
        :type:   bool
        """
        return bool(self._eep_read_single(EEP_TORQUE_POLICY)&0x10)
    @driver_mode_free_policy_eeprom.setter
    def driver_mode_free_policy_eeprom(self, value):
        mask = 1<<4
        self._eep_write(EEP_TORQUE_POLICY, (self._eep_read_single(EEP_TORQUE_POLICY) & ~mask)|((bool(value)<<4)&mask))
#
    @property
    def register_mode_free_policy(self):
        """
        :getter: Get register mode free policy
        :setter: Set register mode free policy
        :type:   bool
        """
        return bool(self._ram_read_single(RAM_TORQUE_POLICY)&0x20)
    @register_mode_free_policy.setter
    def register_mode_free_policy(self, value):
        mask = 1<<5
        self._ram_write(RAM_TORQUE_POLICY, (self._ram_read_single(RAM_TORQUE_POLICY) & ~mask)|((bool(value)<<5)&mask))
    @property
    def register_mode_free_policy_eeprom(self):
        """
        :getter: Get saved register mode free policy from EEPROM
        :setter: Save register mode free policy in EEPROM
        :type:   bool
        """
        return bool(self._eep_read_single(EEP_TORQUE_POLICY)&0x20)
    @register_mode_free_policy_eeprom.setter
    def register_mode_free_policy_eeprom(self, value):
        mask = 1<<5
        self._eep_write(EEP_TORQUE_POLICY, (self._eep_read_single(EEP_TORQUE_POLICY) & ~mask)|((bool(value)<<5)&mask))
# 12
    @property
    def max_temperature(self):
        """
        :getter: Get maximum allowed temperature (in degree Celsius)
        :setter: Set maximum allowed temperature (in degree Celsius)
        :type:   float
        """
        if self._model_4or6:
            return float(self._ram_read_single(RAM_MAX_TEMPERATURE))
        else:
            return ADC_TEMPERATURE[self._ram_read_single(RAM_MAX_TEMPERATURE)]
    @max_temperature.setter
    def max_temperature(self, value):
        if self._model_4or6:
            self._check_range(value, RANGE_TEMPERATURE)
            self._ram_write(RAM_MAX_TEMPERATURE, int(round(value)))
        else:
            self._check_range(value, (ADC_TEMPERATURE[0], ADC_TEMPERATURE[-1]))
            self._ram_write(RAM_MAX_TEMPERATURE, _closest(value, ADC_TEMPERATURE))
    @property
    def max_temperature_eeprom(self):
        """
        :getter: Get saved maximum allowed temperature (in degree Celsius) from EEPROM
        :setter: Save maximum allowed temperature (in degree Celsius) in EEPROM
        :type:   float
        """
        if self._model_4or6:
            return float(self._eep_read_single(EEP_MAX_TEMPERATURE))
        else:
            return ADC_TEMPERATURE[self._eep_read_single(EEP_MAX_TEMPERATURE)]
    @max_temperature_eeprom.setter
    def max_temperature_eeprom(self, value):
        if self._model_4or6:
            self._check_range(value, RANGE_TEMPERATURE)
            self._eep_write(EEP_MAX_TEMPERATURE, int(round(value)))
        else:
            self._check_range(value, (ADC_TEMPERATURE[0], ADC_TEMPERATURE[-1]))
            self._eep_write(EEP_MAX_TEMPERATURE, _closest(value, ADC_TEMPERATURE))
# 13
    @property
    def min_voltage(self):
        """
        :getter: Get minimum operating voltage (in volt)
        :setter: Set minimum operating voltage (in volt)
        :type:   float
        """
        return self._voltage_res*self._ram_read_single(RAM_MIN_VOLTAGE)
    @min_voltage.setter
    def min_voltage(self, value):
        self._check_range_coef(value, RANGE_VOLTAGE[self._model_4or6], self._voltage_res)
        self._ram_write(RAM_MIN_VOLTAGE, int(round(value/self._voltage_res)))
    @property
    def min_voltage_eeprom(self):
        """
        :getter: Get saved minimum operating voltage (in volt) from EEPROM
        :setter: Save minimum operating voltage (in volt) in EEPROM
        :type:   float
        """
        return self._voltage_res*self._eep_read_single(EEP_MIN_VOLTAGE)
    @min_voltage_eeprom.setter
    def min_voltage_eeprom(self, value = None):
        self._check_range_coef(value, RANGE_VOLTAGE[self._model_4or6], self._voltage_res)
        self._eep_write(EEP_MIN_VOLTAGE, int(round(value/self._voltage_res)))
# 14
    @property
    def max_voltage(self):
        """
        :getter: Get maximum operating voltage (in volt)
        :setter: Set maximum operating voltage (in volt)
        :type:   float
        """
        return self._voltage_res*self._ram_read_single(RAM_MAX_VOLTAGE)
    @max_voltage.setter
    def max_voltage(self, value):
        self._check_range_coef(value, RANGE_VOLTAGE[self._model_4or6], self._voltage_res)
        self._ram_write(RAM_MAX_VOLTAGE, int(round(value/self._voltage_res)))
    @property
    def max_voltage_eeprom(self):
        """
        :getter: Get saved maximum operating voltage (in volt) from EEPROM
        :setter: Save maximum operating voltage (in volt) in EEPROM
        :type:   float
        """
        return self._voltage_res*self._eep_read_single(EEP_MAX_VOLTAGE)
    @max_voltage_eeprom.setter
    def max_voltage_eeprom(self, value):
        self._check_range_coef(value, RANGE_VOLTAGE[self._model_4or6], self._voltage_res)
        self._eep_write(EEP_MAX_VOLTAGE, int(round(value/self._voltage_res)))
# 33
    @property
    def alarm_led_period(self):
        """
        :getter: Get blink period of alarm LED
        :setter: Set blink period of alarm LED
        :type:   float
        """
        return 0.0112*self._ram_read_single(RAM_LED_BLINK_PERIOD)
    @alarm_led_period.setter
    def alarm_led_period(self, value):
        self._check_range_coef(value, RANGE_LED_BLINK_PERIOD, 0.0112)
        self._ram_write(RAM_LED_BLINK_PERIOD, int(round(value/0.0112)))
    @property
    def alarm_led_period_eeprom(self):
        """
        :getter: Get saved blink period of alarm LED from EEPROM
        :setter: Save blink period of alarm LED in EEPROM
        :type:   float
        """
        return 0.0112*self._eep_read_single(EEP_LED_BLINK_PERIOD)
    @alarm_led_period_eeprom.setter
    def alarm_led_period_eeprom(self, value):
        self._check_range_coef(value, RANGE_LED_BLINK_PERIOD, 0.0112)
        self._eep_write(EEP_LED_BLINK_PERIOD, int(round(value/0.0112)))
# 34
    @property
    def fault_check_period(self):
        """
        :getter: Get fault check period for temperature and input voltage
        :setter: Set fault check period for temperature and input voltage
        :type:   float
        """
        return 0.0112*self._ram_read_single(RAM_ADC_FAULT_CHECK_PERIOD)
    @fault_check_period.setter
    def fault_check_period(self, value):
        self._check_range_coef(value, RANGE_ADC_FAULT_CHECK_PERIOD, 0.0112)
        self._ram_write(RAM_ADC_FAULT_CHECK_PERIOD, int(round(value/0.0112)))
    @property
    def fault_check_period_eeprom(self):
        """
        :getter: Get saved fault check period for temperature and input voltage from EEPROM
        :setter: Save fault check period for temperature and input voltage in EEPROM
        :type:   float
        """
        return 0.0112*self._eep_read_single(EEP_ADC_FAULT_CHECK_PERIOD)
    @fault_check_period_eeprom.setter
    def fault_check_period_eeprom(self, value):
        self._check_range_coef(value, RANGE_ADC_FAULT_CHECK_PERIOD, 0.0112)
        self._eep_write(EEP_ADC_FAULT_CHECK_PERIOD, int(round(value/0.0112)))
# 35
    @property
    def packet_garbage_check_period(self):
        """
        :getter: Get packet garbage check period
        :setter: Set packet garbage check period
        :type:   float
        """
        return 0.0112*self._ram_read_single(RAM_PACKET_GARBAGE_CHECK_PERIOD)
    @packet_garbage_check_period.setter
    def packet_garbage_check_period(self, value):
        self._check_range_coef(value, RANGE_PACKET_GARBAGE_CHECK_PERIOD, 0.0112)
        self._ram_write(RAM_PACKET_GARBAGE_CHECK_PERIOD, int(round(value/0.0112)))
    @property
    def packet_garbage_check_period_eeprom(self):
        """
        :getter: Get saved packet garbage check period from EEPROM
        :setter: Save packet garbage check period in EEPROM
        :type:   float
        """
        return 0.0112*self._eep_read_single(EEP_PACKET_GARBAGE_CHECK_PERIOD)
    @packet_garbage_check_period_eeprom.setter
    def packet_garbage_check_period_eeprom(self, value):
        self._check_range_coef(value, RANGE_PACKET_GARBAGE_CHECK_PERIOD, 0.0112)
        self._eep_write(EEP_PACKET_GARBAGE_CHECK_PERIOD, int(round(value/0.0112)))
# 15
    @property
    def acceleration_ratio(self):
        """
        :getter: Get acceleration ratio
        :setter: Set acceleration ratio
        :type:   float
                 from 0 (rectangle speed profile)
                 to 0.5 (triangle speed profile)
        """
        return 0.01*self._ram_read_single(RAM_ACCELERATION_RATIO)
    @acceleration_ratio.setter
    def acceleration_ratio(self, value):
        self._check_range_coef(value, RANGE_ACCELERATION_RATIO, 0.01)
        self._ram_write(RAM_ACCELERATION_RATIO, int(round(value*100)))
    @property
    def acceleration_ratio_eeprom(self):
        """
        :getter: Get saved acceleration ratio from EEPROM
        :setter: Save acceleration_ratio in EEPROM
        :type:   float
                 from 0 (rectangle speed profile)
                 to 0.5 (triangle speed profile)
        """
        return 0.01*self._eep_read_single(EEP_ACCELERATION_RATIO)
    @acceleration_ratio_eeprom.setter
    def acceleration_ratio_eeprom(self, value):
        self._check_range_coef(value, RANGE_ACCELERATION_RATIO, 0.01)
        self._eep_write(EEP_ACCELERATION_RATIO, int(round(value*100)))
# 16
    @property
    def max_acceleration_time(self):
        """
        :getter: Get max acceleration time
        :setter: Set max acceleration time
        :type:   float
        """
        return 0.0112*self._ram_read_single(RAM_MAX_ACCELERATION_TIME)
    @max_acceleration_time.setter
    def max_acceleration_time(self, value):
        self._check_range_coef(value, RANGE_MAX_ACCELERATION_TIME, 0.0112)
        self._ram_write(RAM_MAX_ACCELERATION_TIME, int(round(value/0.0112)))
    @property
    def max_acceleration_time_eeprom(self):
        """
        :getter: Get saved max acceleration time from EEPROM
        :setter: Save max acceleration time in EEPROM
        :type:   float
        """
        return 0.0112*self._eep_read_single(EEP_MAX_ACCELERATION_TIME)
    @max_acceleration_time_eeprom.setter
    def max_acceleration_time_eeprom(self, value):
        self._check_range_coef(value, RANGE_MAX_ACCELERATION_TIME, 0.0112)
        self._eep_write(EEP_MAX_ACCELERATION_TIME, int(round(value/0.0112)))
# 17
    @property
    def dead_zone(self):
        """
        :getter: Get dead zone
        :setter: Set dead zone
        :type:   int

        The dead zone value corresponds to the permitted difference (in step)
        between goal position and actual position.

        If the difference (error) is less than the dead zone value,
        servo assumes it has reached the goal position and stops.
        """
        return self._ram_read_single(RAM_DEAD_ZONE)
    @dead_zone.setter
    def dead_zone(self, value):
        self._check_range(value, RANGE_DEAD_ZONE)
        self._ram_write(RAM_DEAD_ZONE, int(round(value)))
    @property
    def dead_zone_eeprom(self):
        """
        :getter: Get saved dead zone from EEPROM
        :setter: Save dead zone in EEPROM
        :type:   int
        """
        return self._eep_read_single(EEP_DEAD_ZONE)
    @dead_zone_eeprom.setter
    def dead_zone_eeprom(self, value):
        self._check_range(value, RANGE_DEAD_ZONE)
        self._eep_write(EEP_DEAD_ZONE, int(round(value)))
# 18
    @property
    def saturator_offset(self):
        """
        :getter: Get saturator offset
        :setter: Set saturator offset
        :type:   int

        Saturator offset corresponds to the PWM prescribed value
        when the servo reaches the dead zone boundary.
        """
        return self._ram_read_single(RAM_SATURATOR_OFFSET)
    @saturator_offset.setter
    def saturator_offset(self, value):
        self._check_range(value, RANGE_SATURATOR_OFFSET)
        self._ram_write(RAM_SATURATOR_OFFSET, int(round(value)))
    @property
    def saturator_offset_eeprom(self):
        """
        :getter: Get saved saturator offset from EEPROM
        :setter: Save saturator offset in EEPROM
        :type:   int
        """
        return self._eep_read_single(EEP_SATURATOR_OFFSET)
    @saturator_offset_eeprom.setter
    def saturator_offset_eeprom(self, value):
        self._check_range(value, RANGE_SATURATOR_OFFSET)
        self._eep_write(EEP_SATURATOR_OFFSET, int(round(value)))
# 19
    @property
    def saturator_slope(self):
        """
        :getter: Get saturator slop
        :setter: Set saturator slop
        :type:   int

        This value corresponds to the slop of the linear transition
        of the PWM from the saturator offset value
        to the maximum PWM value versus position.
        """
        data = self._ram_read(RAM_SATURATOR_SLOPE)
        return (data[1]<<8|data[0])/256.0
    @saturator_slope.setter
    def saturator_slope(self, value):
        self._check_range_coef(value, RANGE_SATURATOR_SLOPE, 0.00390625)
        value = int(round(value * 256))
        self._ram_write(RAM_SATURATOR_SLOPE, value&0xFF, value>>8)
    @property
    def saturator_slope_eeprom(self):
        """
        :getter: Get saved saturator slop from EEPROM
        :setter: Save saturator slop in EEPROM
        :type:   int
        """
        data = self._eep_read(EEP_SATURATOR_SLOPE)
        return (data[1]<<8|data[0])/256.0
    @saturator_slope_eeprom.setter
    def saturator_slope_eeprom(self, value):
        self._check_range_coef(value, RANGE_SATURATOR_SLOPE, 0.00390625)
        value = int(round(value * 256))
        self._eep_write(EEP_SATURATOR_SLOPE, int(value)&0xFF, int(value)>>8)
# 20
    @property
    def pwm_offset(self):
        """
        :getter: Get PWM offset
        :setter: Set PWM offset
        :type:   int

        Prescribed PWM will be increased by the amount of the offset.
        This offset acts similar to a compensator when constant loads
        are applied on the servo (e.g. from gravity).
        """
        value = self._ram_read_single(RAM_PWM_OFFSET)
        if value > 127:
            value -= 255
        return value
    @pwm_offset.setter
    def pwm_offset(self, value):
        self._check_range(value, RANGE_PWM_OFFSET)
        value = int(round(value))
        if value < 0:
            value += 255
        self._ram_write(RAM_PWM_OFFSET, value)
    @property
    def pwm_offset_eeprom(self):
        """
        :getter: Get saved PWM offset from EEPROM
        :setter: Save PWM offset in EEPROM
        :type:   int
        """
        value = self._ram_read_single(EEP_PWM_OFFSET)
        if value > 127:
            value -= 255
        return value
    @pwm_offset_eeprom.setter
    def pwm_offset_eeprom(self, value):
        self._check_range(value, RANGE_PWM_OFFSET)
        value = int(round(value))
        if value < 0:
            value += 255
        self._eep_write(PWM_OFFSET, value)
# 21
    @property
    def pwm_min(self):
        """
        :getter: Get minimum PWM
        :setter: Set minimum PWM
        :type:   int

        Minimum PWM can be used when there is jerky movement due to
        tight fitting or friction in the servo application system
        but assigning a minimum PWM value that is too large may lead
        to unstable system.
        """
        return self._ram_read_single(RAM_MIN_PWM)
    @pwm_min.setter
    def pwm_min(self, value):
        self._check_range(value, RANGE_MIN_PWM)
        self._ram_write(RAM_MIN_PWM, int(round(value)))
    @property
    def pwm_min_eeprom(self):
        """
        :getter: Get saved minimum PWM from EEPROM
        :setter: Save minimum PWM in EEPROM
        :type:   int
        """
        return self._eep_read_single(EEP_MIN_PWM)
    @pwm_min_eeprom.setter
    def pwm_min_eeprom(self, value):
        self._check_range(value, RANGE_MIN_PWM)
        self._eep_write(EEP_MIN_PWM, int(round(value)))
# 22
    @property
    def pwm_max(self):
        """
        :getter: Get maximum PWM
        :setter: Set maximum PWM
        :type:   int

        Maximum PWM affects the output maximum torque.
        Decreasing maximum PWM can also increase battery life.
        """
        data = self._ram_read(RAM_MAX_PWM)
        return data[1]<<8|data[0]
    @pwm_max.setter
    def pwm_max(self, value):
        self._check_range(value, RANGE_MAX_PWM)
        value = int(round(value))
        self._ram_write(RAM_MAX_PWM, value&0xFF, value>>8)
    @property
    def pwm_max_eeprom(self):
        """
        :getter: Get saved maximum PWM from EEPROM
        :setter: Save maximum PWM in EEPROM
        :type:   int
        """
        data = self._eep_read(EEP_MAX_PWM)
        return data[1]<<8|data[0]
    @pwm_max_eeprom.setter
    def pwm_max_eeprom(self, value):
        self._check_range(value, RANGE_MAX_PWM)
        value = int(round(value))
        self._eep_write(EEP_MAX_PWM, value&0xFF, value>>8)
# 23
    @property
    def pwm_overload_threshold(self):
        """
        :getter: Get PWM overload threshold
        :setter: Set PWM overload threshold
        :type:   int

        Overload activates when external load is greater than
        PWM overload threshold.
        Overload never activates when the PWM overload threshold
        is set to its maximum permitted value.
        """
        data = self._ram_read(RAM_OVERLOAD_PWM_THRESHOLD)
        return data[1]<<8|data[0]
    @pwm_overload_threshold.setter
    def pwm_overload_threshold(self, value):
        self._check_range(value, RANGE_OVERLOAD_PWM_THRESHOLD[self._model_4or6])
        value = int(round(value))
        self._ram_write(RAM_OVERLOAD_PWM_THRESHOLD, value&0xFF, value>>8)
    @property
    def pwm_overload_threshold_eeprom(self):
        """
        :getter: Get saved PWM overload threshold from EEPROM
        :setter: Save PWM overload threshold in EEPROM
        :type:   int
        """
        data = self._eep_read(EEP_OVERLOAD_PWM_THRESHOLD)
        return data[1]<<8|data[0]
    @pwm_overload_threshold_eeprom.setter
    def pwm_overload_threshold_eeprom(self, value):
        self._check_range(value, RANGE_OVERLOAD_PWM_THRESHOLD[self._model_4or6])
        value = int(round(value))
        self._eep_write(EEP_OVERLOAD_PWM_THRESHOLD, value&0xFF, value>>8)
# 24
    @property
    def min_position(self):
        """
        :getter: Get min operating position
        :setter: Set min operating position
        :type:   int
        """
        data = self._ram_read(RAM_MIN_POSITION)
        return data[1]<<8|data[0]
    @min_position.setter
    def min_position(self, value):
        if not self._position_max > value > 0:
            raise PyHerkuleX_Exception("Min position value "+str(value)+" out of bounds (from 0 to "+str(self._position_max)+" for servo model "+str(self._model)+").")
        value = int(round(value))
        self._ram_write(RAM_MIN_POSITION, value&0xFF, value>>8)
    @property
    def min_position_eeprom(self):
        """
        :getter: Get saved min operating position from EEPROM
        :setter: Save min operating position in EEPROM
        :type:   int
        """
        data = self._eep_read(EEP_MIN_POSITION)
        return data[1]<<8|data[0]
    @min_position_eeprom.setter
    def min_position_eeprom(self, value):
        if not self._position_max > value > 0:
            raise PyHerkuleX_Exception("Min position value "+str(value)+" out of bounds (from 0 to "+str(self._position_max)+" for servo model "+str(self._model)+").")
        value = int(round(value))
        self._eep_write(EEP_MIN_POSITION, value&0xFF, value>>8)
# 25
    @property
    def max_position(self):
        """
        :getter: Get max operating position
        :setter: Set max operating position
        :type:   int
        """
        data = self._ram_read(RAM_MAX_POSITION)
        return data[1]<<8|data[0]
    @max_position.setter
    def max_position(self, value):
        if not self._position_max > value > 0:
            raise PyHerkuleX_Exception("Max position value "+str(value)+" out of bounds (from 0 to "+str(self._position_max)+" for servo model "+str(self._model)+").")
        value = int(round(value))
        self._ram_write(RAM_MAX_POSITION, value&0xFF, value>>8)
    @property
    def max_position_eeprom(self):
        """
        :getter: Get saved max operating position from EEPROM
        :setter: Save max operating position in EEPROM
        :type:   int
        """
        data = self._eep_read(EEP_MAX_POSITION)
        return data[1]<<8|data[0]
    @max_position_eeprom.setter
    def max_position_eeprom(self, value):
        if not self._position_max > value > 0:
            raise PyHerkuleX_Exception("Max position value "+str(value)+" out of bounds (from 0 to "+str(self._position_max)+" for servo model "+str(self._model)+").")
        value = int(round(value))
        self._eep_write(EEP_MAX_POSITION, value&0xFF, value>>8)
# 26 to 28
    @property
    def position_gain_p(self):
        """
        :getter: Get proportional gain of the position PID
        :setter: Set proportional gain of the position PID
        :type: float
        """
        data = self._ram_read(RAM_POSITION_KP)
        return (data[1]<<8|data[0])/8
    @position_gain_p.setter
    def position_gain_p(self, value):
        self._check_range_coef(value, RANGE_POSITION_KP, 0.125)
        value = int(round(value * 8))
        self._ram_write(RAM_POSITION_KP, value&0xFF, value>>8)
    @property
    def position_gain_p_eeprom(self):
        """
        :getter: Get saved proportional gain of the position PID from EEPROM
        :setter: Save proportional gain of the position PID in EEPROM
        :type:   float
        """
        data = self._eep_read(EEP_POSITION_KP)
        return (data[1]<<8|data[0])/8
    @position_gain_p_eeprom.setter
    def position_gain_p_eeprom(self, value):
        self._check_range_coef(value, RANGE_POSITION_KP, 0.125)
        value = int(round(value * 8))
        self._eep_write(EEP_POSITION_KP, value&0xFF, value>>8)
#
    @property
    def position_gain_i(self):
        """
        :getter: Get integral gain of the position PID
        :setter: Set integral gain of the position PID
        :type: float
        """
        data = self._ram_read(RAM_POSITION_KI)
        return (data[1]<<8|data[0])/16384
    @position_gain_i.setter
    def position_gain_i(self, value):
        self._check_range_coef(value, RANGE_POSITION_KI, 1.0/16384.0)
        value = int(round(value * 16384))
        self._ram_write(RAM_POSITION_KI, value&0xFF, value>>8)
    @property
    def position_gain_i_eeprom(self):
        """
        :getter: Get saved integral gain of the position PID from EEPROM
        :setter: Save integral gain of the position PID in EEPROM
        :type:   float
        """
        data = self._eep_read(EEP_POSITION_KI)
        return (data[1]<<8|data[0])/16384
    @position_gain_i_eeprom.setter
    def position_gain_i_eeprom(self, value):
        self._check_range_coef(value, RANGE_POSITION_KI, 1.0/16384.0)
        value = int(round(value * 16384))
        self._eep_write(EEP_POSITION_KI, value&0xFF, value>>8)
#
    @property
    def position_gain_d(self):
        """
        :getter: Get derivative gain of the position PID
        :setter: Set derivative gain of the position PID
        :type: float
        """
        data = self._ram_read(RAM_POSITION_KD)
        return (data[1]<<8|data[0])/8
    @position_gain_d.setter
    def position_gain_d(self, value):
        self._check_range_coef(value, RANGE_POSITION_KD, 0.125)
        value = int(round(value * 8))
        self._ram_write(RAM_POSITION_KD, value&0xFF, value>>8)
    @property
    def position_gain_d_eeprom(self):
        """
        :getter: Get saved derivative gain of the position PID from EEPROM
        :setter: Save derivative gain of the position PID in EEPROM
        :type:   float
        """
        data = self._eep_read(EEP_POSITION_KD)
        return (data[1]<<8|data[0])/8
    @position_gain_d_eeprom.setter
    def position_gain_d_eeprom(self, value):
        self._check_range_coef(value, RANGE_POSITION_KD, 0.125)
        value = int(round(value * 8))
        self._eep_write(EEP_POSITION_KD, value&0xFF, value>>8)
# 29
    @property
    def feedforward_gain_1(self):
        """
        :getter: Get position feedforward first gain (Kd)
        :setter: Set position feedforward first gain (Kd)
        :type: float
        """
        data = self._ram_read(RAM_POSITION_FEEDFORWARD_1ST_GAIN)
        return data[1]<<8|data[0]
    @feedforward_gain_1.setter
    def feedforward_gain_1(self, value):
        self._check_range(value, RANGE_POSITION_FEEDFORWARD_1ST_GAIN)
        value = int(round(value))
        self._ram_write(RAM_POSITION_FEEDFORWARD_1ST_GAIN, value&0xFF, value>>8)
    @property
    def feedforward_gain_1_eeprom(self):
        """
        :getter: Get saved position feedforward first gain (Kd) from EEPROM
        :setter: Save position feedforward first gain (Kd) in EEPROM
        :type: float
        """
        data = self._eep_read(EEP_POSITION_FEEDFORWARD_1ST_GAIN)
        return data[1]<<8|data[0]
    @feedforward_gain_1_eeprom.setter
    def feedforward_gain_1_eeprom(self, value):
        self._check_range(value, RANGE_POSITION_FEEDFORWARD_1ST_GAIN)
        value = int(round(value))
        self._eep_write(EEP_POSITION_FEEDFORWARD_1ST_GAIN, value&0xFF, value>>8)
# 30
    @property
    def feedforward_gain_2(self):
        """
        :getter: Get position feedforward second gain (Kdd)
        :setter: Set position feedforward second gain (Kdd)
        :type: float
        """
        data = self._ram_read(RAM_POSITION_FEEDFORWARD_2ND_GAIN)
        return data[1]<<8|data[0]
    @feedforward_gain_2.setter
    def feedforward_gain_2(self, value):
        self._check_range(value, RANGE_POSITION_FEEDFORWARD_2ND_GAIN)
        value = int(round(value))
        self._ram_write(RAM_POSITION_FEEDFORWARD_2ND_GAIN, value&0xFF, value>>8)
    @property
    def feedforward_gain_2_eeprom(self):
        """
        :getter: Get saved position feedforward second gain (Kdd) from EEPROM
        :setter: Save position feedforward second gain (Kdd) in EEPROM
        :type: float
        """
        data = self._eep_read(EEP_POSITION_FEEDFORWARD_2ND_GAIN)
        return data[1]<<8|data[0]
    @feedforward_gain_2_eeprom.setter
    def feedforward_gain_2_eeprom(self, value):
        self._check_range(value, RANGE_POSITION_FEEDFORWARD_2ND_GAIN)
        value = int(round(value))
        self._eep_write(EEP_POSITION_FEEDFORWARD_2ND_GAIN, value&0xFF, value>>8)
# 31
    @property
    def velocity_gain_p(self):
        """
        :getter: Get proportional gain of the velocity PID
        :setter: Set proportional gain of the velocity PID
        :type: float

        .. note::
            This argument is available only with servo models DRS-0402 and DRS-0602.
        """
        if self._magnetic_encoder:
            data = self._ram_read(RAM_VELOCITY_KP)
            return (data[1]<<8|data[0])/64
        else:
            raise PyHerkuleX_Exception("Can't get proportional gain of the velocity PID for this servo model.")
    @velocity_gain_p.setter
    def velocity_gain_p(self, value):
        if self._magnetic_encoder:
            self._check_range_coef(value, RANGE_VELOCITY_KP, 0.015625)
            value = int(round(value * 64))
            self._ram_write(RAM_VELOCITY_KP, value&0xFF, value>>8)
        else:
            raise PyHerkuleX_Exception("Can't set proportional gain of the velocity PID for this servo model.")
    @property
    def velocity_gain_p_eeprom(self):
        """
        :getter: Get saved proportional gain of the velocity PID from EEPROM
        :setter: Save proportional gain of the velocity PID in EEPROM
        :type: float

        .. note::
            This argument is available only with servo models DRS-0402 and DRS-0602.
        """
        if self._magnetic_encoder:
            data = self._eep_read(EEP_VELOCITY_KP)
            return (data[1]<<8|data[0])/64
        else:
            raise PyHerkuleX_Exception("Can't get saved proportional gain of the velocity PID for this servo model.")
    @velocity_gain_p_eeprom.setter
    def velocity_gain_p_eeprom(self, value):
        if self._magnetic_encoder:
            self._check_range_coef(value, RANGE_VELOCITY_KP, 0.015625)
            value = int(round(value * 64))
            self._eep_write(EEP_VELOCITY_KP, value&0xFF, value>>8)
        else:
            raise PyHerkuleX_Exception("Can't save proportional gain of the velocity PID for this servo model.")
# 32
    @property
    def velocity_gain_i(self):
        """
        :getter: Get integral gain of the velocity PID
        :setter: Set integral gain of the velocity PID
        :type: float

        .. note::
            This argument is available only with servo models DRS-0402 and DRS-0602.
        """
        if self._magnetic_encoder:
            data = self._ram_read(RAM_VELOCITY_KI)
            return (data[1]<<8|data[0])/16384
        else:
            raise PyHerkuleX_Exception("Can't get integral gain of the velocity PID for this servo model.")
    @velocity_gain_i.setter
    def velocity_gain_i(self, value):
        if self._magnetic_encoder:
            self._check_range_coef(value, RANGE_VELOCITY_KI, 1.0/16384.0)
            value = int(round(value * 16384))
            self._ram_write(RAM_VELOCITY_KI, value&0xFF, value>>8)
        else:
            raise PyHerkuleX_Exception("Can't set integral gain of the velocity PID for this servo model.")
    @property
    def velocity_gain_i_eeprom(self):
        """
        :getter: Get saved integral gain of the velocity PID from EEPROM
        :setter: Save integral gain of the velocity PID in EEPROM
        :type: float

        .. note::
            This argument is available only with servo models DRS-0402 and DRS-0602.
        """
        if self._magnetic_encoder:
            data = self._eep_read(EEP_VELOCITY_KI)
            return (data[1]<<8|data[0])/16384
        else:
            raise PyHerkuleX_Exception("Can't get saved integral gain of the velocity PID for this servo model.")
    @velocity_gain_i_eeprom.setter
    def velocity_gain_i_eeprom(self, value):
        if self._magnetic_encoder:
            self._check_range_coef(value, RANGE_VELOCITY_KI, 1.0/16384.0)
            value = int(round(value * 16384))
            self._eep_write(EEP_VELOCITY_KI, value&0xFF, value>>8)
        else:
            raise PyHerkuleX_Exception("Can't save integral gain of the velocity PID for this servo model.")
# 36
    @property
    def stop_detection_period(self):
        """
        :getter: Get stop detection period
        :setter: Set stop detection period
        :type:   float

        Servo stop is confirmed if stoppage lasts for detection period time.
        """
        return 0.0112*self._ram_read_single(RAM_STOP_DETECTION_PERIOD)
    @stop_detection_period.setter
    def stop_detection_period(self, value):
        self._check_range_coef(value, RANGE_STOP_DETECTION_PERIOD, 0.0112)
        self._ram_write(RAM_STOP_DETECTION_PERIOD, int(round(value/0.0112)))
    @property
    def stop_detection_period_eeprom(self):
        """
        :getter: Get saved stop detection period from EEPROM
        :setter: Save stop detection period in EEPROM
        :type:   float
        """
        return 0.0112*self._eep_read_single(EEP_STOP_DETECTION_PERIOD)
    @stop_detection_period_eeprom.setter
    def stop_detection_period_eeprom(self, value):
        self._check_range_coef(value, RANGE_STOP_DETECTION_PERIOD, 0.0112)
        self._eep_write(EEP_STOP_DETECTION_PERIOD, int(round(value/0.0112)))
# 37
    @property
    def overload_detection_period(self):
        """
        :getter: Get overload detection period
        :setter: Set overload detection period
        :type:   float
        """
        return 0.0112*self._ram_read_single(RAM_OVERLOAD_DETECTION_PERIOD)
    @overload_detection_period.setter
    def overload_detection_period(self, value):
        self._check_range_coef(value, RANGE_OVERLOAD_DETECTION_PERIOD, 0.0112)
        self._ram_write(RAM_OVERLOAD_DETECTION_PERIOD, int(round(value/0.0112)))
    @property
    def overload_detection_period_eeprom(self):
        """
        :getter: Get saved overload detection period from EEPROM
        :setter: Save overload detection period in EEPROM
        :type:   float
        """
        return 0.0112*self._eep_read_single(EEP_OVERLOAD_DETECTION_PERIOD)
    @overload_detection_period_eeprom.setter
    def overload_detection_period_eeprom(self, value):
        self._check_range_coef(value, RANGE_OVERLOAD_DETECTION_PERIOD, 0.0112)
        self._eep_write(EEP_OVERLOAD_DETECTION_PERIOD, int(round(value/0.0112)))
# 38
    @property
    def stop_threshold(self):
        """
        :getter: Get stop threshold
        :setter: Set stop threshold
        :type:   float

        The servo is seen as not moving (stopped) when the position
        movement of the servo is less than the stop threshold value.
        The servo is determined to be stopped if the stoppage lasts
        longer than the stop detection period.
        """
        return self._ram_read_single(RAM_STOP_THRESHOLD)
    @stop_threshold.setter
    def stop_threshold(self, value):
        self._check_range(value, RANGE_STOP_THRESHOLD)
        self._ram_write(RAM_STOP_THRESHOLD, int(round(value)))
    @property
    def stop_threshold_eeprom(self):
        """
        :getter: Get saved stop threshold from EEPROM
        :setter: Save stop threshold in EEPROM
        :type:   float
        """
        return self._eep_read_single(EEP_STOP_THRESHOLD)
    @stop_threshold_eeprom.setter
    def stop_threshold_eeprom(self, value):
        self._check_range(value, RANGE_STOP_THRESHOLD)
        self._eep_write(EEP_STOP_THRESHOLD, int(round(value)))
# 39
    @property
    def inposition_margin(self):
        """
        :getter: Get inposition margin
        :setter: Set inposition margin
        :type:   float

        Standard value to determine if the prescribed position has been reached.
        """
        return self._ram_read_single(RAM_INPOSITION_MARGIN)
    @inposition_margin.setter
    def inposition_margin(self, value):
        self._check_range(value, RANGE_INPOSITION_MARGIN)
        self._ram_write(RAM_INPOSITION_MARGIN, int(round(value)))
    @property
    def inposition_margin_eeprom(self):
        """
        :getter: Get saved inposition margin from EEPROM
        :setter: Save inposition margin in EEPROM
        :type:   float
        """
        return self._eep_read_single(EEP_INPOSITION_MARGIN)
    @inposition_margin_eeprom.setter
    def inposition_margin_eeprom(self, value):
        self._check_range(value, RANGE_INPOSITION_MARGIN)
        self._eep_write(EEP_INPOSITION_MARGIN, int(round(value)))
# 41 & 42
    @property
    def calibration_difference(self):
        """
        :getter: Get calibration difference
        :setter: Set calibration difference
        :type:   float

        Used to calibrate neutral point of the calibrated position
        (calibrated position = absolute position - calibration difference).
        It is generally used to make adjustments and compensate assembly
        variations.
        """
        if self._model_4or6:
            data = self._ram_read(RAM_CALIBRATION_DIFFERENCE_LOWER)
            return data[1]<<8|data[0]
        else:
            value = self._ram_read_single(RAM_CALIBRATION_DIFFERENCE_UPPER)
            if value>>7:
                return (value&0x7F)-0x7F
            else:
                return value
    @calibration_difference.setter
    def calibration_difference(self, value):
        if self._model_4or6:
            self._check_range(value, RANGE_CALIBRATION_DIFFERENCE_2)
            value = int(round(value))
            if value<0:
                value += 0x10000
                self._ram_write(RAM_CALIBRATION_DIFFERENCE_LOWER, value&0xFF, value>>8)
            else:

                self._ram_write(RAM_CALIBRATION_DIFFERENCE_LOWER, value&0xFF, value>>8)
        else:
            self._check_range(value, RANGE_CALIBRATION_DIFFERENCE_1)
            value = int(round(value))
            if value<0:
                self._ram_write(RAM_CALIBRATION_DIFFERENCE_UPPER, (0x7F-abs(value)&0x7F)|0x80)
            else:
                self._ram_write(RAM_CALIBRATION_DIFFERENCE_UPPER, value&0x7F)
    @property
    def calibration_difference_eeprom(self):
        """
        :getter: Get saved calibration difference from EEPROM
        :setter: Save calibration difference in EEPROM
        :type:   float
        """
        if self._model_4or6:
            data = self._eep_read(EEP_CALIBRATION_DIFFERENCE_LOWER)
            return data[1]<<8|data[0]
        else:
            value = self._eep_read_single(EEP_CALIBRATION_DIFFERENCE_UPPER)
            if value>>7:
                return (value&0x7F)-0x7F
            else:
                return value
    @calibration_difference_eeprom.setter
    def calibration_difference_eeprom(self, value):
        if self._model_4or6:
            self._check_range(value, RANGE_CALIBRATION_DIFFERENCE_2)
            value = int(round(value))
            if value<0:
                value += 0x10000
                self._eep_write(EEP_CALIBRATION_DIFFERENCE_LOWER, value&0xFF, value>>8)
            else:

                self._eep_write(EEP_CALIBRATION_DIFFERENCE_LOWER, value&0xFF, value>>8)
        else:
            self._check_range(value, RANGE_CALIBRATION_DIFFERENCE_1)
            value = int(round(value))
            if value<0:
                self._eep_write(EEP_CALIBRATION_DIFFERENCE_UPPER, (0x7F-abs(value)&0x7F)|0x80)
            else:
                self._eep_write(EEP_CALIBRATION_DIFFERENCE_UPPER, value&0x7F)
# 47
    @property
    def mode(self):
        """
        :getter: Get servo control mode
        :setter: Set servo control mode
        :type: int

        Three control mode values are supported:
          - :ref:`MODE_FREE <mode-constants>`:
            In this mode, the servo shaft is freely movable.
            Position control and speed control will not work.
            Set MODE_CONTROL before that.
          - :ref:`MODE_BRAKE <mode-constants>`:
            In this mode, servo motion is prevented.
            Position control and speed control will not work.
            Set MODE_CONTROL before that.
          - :ref:`MODE_CONTROL <mode-constants>`:
            In this mode, control instructions are permitted.
        """
        return self._ram_read_single(RAM_TORQUE_CONTROL)
    @mode.setter
    def mode(self, value):
        value = int(value)
        if value in (MODE_FREE, MODE_BRAKE, MODE_CONTROL):
            self._ram_write(RAM_TORQUE_CONTROL, value)
        else:
            raise PyHerkuleX_Exception("Servo control mode not supported by servo ID: 0x%02X"%self._id)

# 45
    def set_absolute_position_origin(self, option = 'middle'):
        """
        Set the absolute position origin from the current position.

        :param string option: Only 5 different options are permitted:

          - ``'min'``: current position becomes the min absolute position (current absolute position becomes 0)
          - ``'half_forward'``: current position plus 180° becomes the middle absolute position (current absolute position becomes 9903)
          - ``'middle'`` (default): current position becomes the middle absolute position (current absolute position becomes 16398)
          - ``'half_backward'``: current position minus 180° becomes the middle absolute position (current absolute position becomes 22865)
          - ``'max'``: current position becomes the max absolute position (current absolute position becomes 32767)

        .. note::
            This action is permitted only with servo models DRS-0402 and DRS-0602.
        """
        if self._magnetic_encoder:
            self._ram_write(RAM_AUX_1, {'min':2, 'half_forward':3, 'middle':4, 'half_backward':5, 'max':6}[option])
        else:
            raise PyHerkuleX_Exception("Can't set absolute position origin for this servo model.")
    def reset_absolute_position_origin(self):
        """
        Reset the absolute position origin to its initial state.

        .. note::
            This action is permitted only with servo models DRS-0402 and DRS-0602.
        """
        if self._magnetic_encoder:
            self._ram_write(RAM_AUX_1, 1)
        else:
            raise PyHerkuleX_Exception("Can't reset absolute position origin for this servo model.")
# 51
    @property
    def speed_mode(self):
        """
        :getter: Get speed control mode state
        :type:   bool

        Servo.mode value is MODE_CONTROL and continuous rotation is in progress if speed mode is ``True``.
        """
        return bool(self._ram_read_single(RAM_CURRENT_CONTROL_MODE))
# 52
    @property
    def operating_time(self):
        """
        :getter: Get current operating time (max 2.856 second)
        :type:   float
        """
        return 0.0112*self._ram_read_single(RAM_TICK)
# 53
    def control_stop(self, led = LED_OFF):
        """
        Stop servo in its current position.

        :param int led: LED color among the supported values given as :ref:`LED color constants <led-constants>`.

        .. note::
            Control mode must be enabled before sending any control instruction to the servo.
        """
        self._write(REQ_I_JOG, 0, 0, (led&0x7)<<2|1, self._id, 0)

    @property
    def position(self):
        """
        :getter: Get current (calibrated) position
        :type:   int
        """
        data = self._ram_read(RAM_CALIBRATED_POSITION)
        if self._magnetic_encoder:
            return data[1]<<8|data[0]
        else:
            return (data[1]&0x07)<<8|data[0]
    @position.setter
    def position(self, value):
        raise PyHerkuleX_Exception("Can't set position attribute. Use Servo.control_position() method instead.")
    def control_position(self, position, time, led = LED_OFF, velocity_override = True):
        """
        Set new (calibrated) position.

        :param int position: Desired position.

        :param float time: Time taken to move from present
                               position to desired position.

        :param int led: LED color among the supported values given as :ref:`LED color constants <led-constants>`.

        :param bool velocity_override: VOR is either ``True`` to enable it or ``False`` to disable it (if the servo model allows to disable VOR)

        .. note::
            Control mode must be enabled before sending any control instruction to the servo.
        """
        position = int(round(position))
        self._write(REQ_I_JOG, position&0xFF, position>>8&0xFF, (bool(velocity_override)^1)<<6|(led&0x7)<<2, self._id, int(round(time/0.0112))&0xFF)
#
    def control_position_increment(self, increment, time, led = LED_OFF, velocity_override = True):
        """
        Increment current (calibrated) position.

        :param int increment: Desired increment to be added to the
                              current position.

        :param int time: Time taken to complete the increment
                         from current position.

        :param int led: LED color among the supported values given as :ref:`LED color constants <led-constants>`.

        :param bool velocity_override: either ``True`` to enable it or ``False`` to disable VOR (if the servo model allows it)

        .. note::
            Control mode must be enabled before sending any control instruction to the servo.
        """
        position = int(round(increment+self.position))
        self._write(REQ_I_JOG, position&0xFF, position>>8&0xFF, (bool(velocity_override)^1)<<6|(led&0x7)<<2, self._id, int(round(time/0.0112))&0xFF)
    @property
    def angle(self):
        """
        :getter: Get current angular (calibrated) position in degree
        :type:   float
        """
        return self._position_to_angle(self.position)
    @angle.setter
    def angle(self, value):
        raise PyHerkuleX_Exception("Can't set angle attribute. Use control_angle() method instead.")
    def control_angle(self, angle, time, led = LED_OFF, velocity_override = True):
        """
        Sets new angular (calibrated) position.

        :param float angle: Desired angular position in degree.

        :param float time: Time taken to move from present
                           position to the desired position.

        :param int led: LED color among the supported values given as :ref:`LED color constants <led-constants>`.

        :param bool velocity_override: VOR is either ``True`` to enable it or ``False`` to disable it (if the servo model allows to disable VOR)

        .. note::
            Control mode must be enabled before sending any control instruction to the servo.
        """
        self.control_position(self._angle_to_position(angle), time, led, velocity_override)
#
    def control_angle_increment(self, increment, time, led = LED_OFF, velocity_override = True):
        """
        Increment current angular position.

        :param int increment: Desired angular increment to be added to
                              the current angular position in degree.

        :param int time: Time taken to complete the angular increment
                         from current position.

        :param int led: LED color among the supported values given as :ref:`LED color constants <led-constants>`.

        :param bool velocity_override: either ``True`` to enable it or ``False`` to disable VOR (if the servo model allows it)

        .. note::
            Control mode must be enabled before sending any control instruction to the servo.
        """
        self.control_angle(self.angle+increment, time, led, velocity_override)
# 55
    @property
    def native_speed(self):
        """
        :getter: Get current native speed
        :type:   int
        """
        data = self._ram_read(RAM_DIFFERENTIAL_POSITION)
        if data[1]&0x80:
            return -((data[1]<<8|data[0])^0xFFFF)
        else:
            return (data[1]<<8|data[0])
    @native_speed.setter
    def native_speed(self, value):
        raise PyHerkuleX_Exception("Can't set native_speed attribute. Use control_native_speed() method instead.")
    def control_native_speed(self, speed, time, led = LED_OFF, velocity_override = True):
        """
        Set native speed.

        :param int speed: Desired speed in native unit.

        :param int time: Time taken to reach the desired speed
                         from the current speed.

        :param int led: LED color among the supported values given as :ref:`LED color constants <led-constants>`.

        :param bool velocity_override: VOR is either ``True`` to enable it or ``False`` to disable it (if the servo model allows to disable VOR)

        .. note::
            Control mode must be enabled before sending any control instruction to the servo.
        """
        speed = self._signed_native_speed(speed)
        self._write(REQ_I_JOG, speed&0xFF, speed>>8&0xFF, 2|(bool(velocity_override)^1)<<6|(led&0x7)<<2, self._id, int(round(time/0.0112))&0xFF)
#
    @property
    def speed(self):
        """
        :getter: Get current speed in step/sec
        :type:   float
        """
        return self._absolute_speed_output_res*self.native_speed
    @speed.setter
    def speed(self, value):
        raise PyHerkuleX_Exception("Can't set speed attribute. Use control_speed() method instead.")
    def control_speed(self, speed, time, led = LED_OFF, velocity_override = True):
        """
        Set native speed.

        :param float speed: Desired speed in step/second.

        :param float time: Time taken to reach the desired speed
                           from the current speed.

        :param int led: LED color among the supported values given as :ref:`LED color constants <led-constants>`.

        :param bool velocity_override: VOR is either ``True`` to enable it or ``False`` to disable it (if the servo model allows to disable VOR)

        .. note::
            Control mode must be enabled before sending any control instruction to the servo.
        """
        self.control_native_speed(speed/self._absolute_speed_input_res, time, led, velocity_override)
#
    @property
    def angular_speed(self):
        """
        :getter: Get current angular speed in degree/second
        :type:   int
        """
        return self._angular_speed_output_res*self.native_speed
    @angular_speed.setter
    def angular_speed(self, value):
        raise PyHerkuleX_Exception("Can't set angular_speed attribute. Use control_angular_speed() method instead.")
    def control_angular_speed(self, speed, time, led = LED_OFF, velocity_override = True):
        """
        Set angular speed.

        :param float speed: Desired speed in degree/second.

        :param float time: Time taken to reach the desired speed
                           from the current speed.

        :param int led: LED color among the supported values given as :ref:`LED color constants <led-constants>`.

        :param bool velocity_override: VOR is either ``True`` to enable it or ``False`` to disable it (if the servo model allows to disable VOR)

        .. note::
            Control mode must be enabled before sending any control instruction to the servo.
        """
        self.control_native_speed(speed/self._angular_speed_input_res, time, led, velocity_override)
# 54
    @property
    def absolute_position(self):
        """
        :getter: Get current absolute position
        :type:   int
        """
        data = self._ram_read(RAM_ABSOLUTE_POSITION)
        return data[1]<<8|data[0]
#
    @property
    def absolute_angle(self):
        """
        :getter: Get current absolute angle
        :type:   float
        """
        data = self._ram_read(RAM_ABSOLUTE_POSITION)
        return self._position_res*(data[1]<<8|data[0])
# 57
    @property
    def absolute_raw_position(self):
        """
        :getter: Get current absolute raw position
                 (from potentiometer)
        :type:   int

        .. note::
            This argument is available only with servo models DRS-0402 and DRS-0602.
        """
        if self._magnetic_encoder:
            data = self._ram_read(RAM_ABSOLUTE_2ND_POSITION)
            return data[1]<<8|data[0]
        else:
            raise PyHerkuleX_Exception("Can't get absolute raw position for this servo model.")
    @property
    def absolute_raw_angle(self):
        """
        :getter: Get current absolute raw angle in degree
                 (from potentiometer)
        :type:   float

        .. note::
            This argument is available only with servo models DRS-0402 and DRS-0602.
        """
        if self._magnetic_encoder:
            data = self._ram_read(RAM_ABSOLUTE_2ND_POSITION)
            return self._raw_position_res*(data[1]<<8|data[0])
        else:
            raise PyHerkuleX_Exception("Can't get absolute raw angle for this servo model.")
# 58
    @property
    def absolute_goal_position(self):
        """
        :getter: Get current absolute goal position
        :type:   int
        """
        data = self._ram_read(RAM_ABSOLUTE_GOAL_POSITION)
        return data[1]<<8|data[0]
#
    @property
    def absolute_goal_angle(self):
        """
        :getter: Get current absolute goal angle
        :type:   float
        """
        data = self._ram_read(RAM_ABSOLUTE_GOAL_POSITION)
        return self._position_res*(data[1]<<8|data[0])
# 59
    @property
    def absolute_intermediate_goal_position(self):
        """
        :getter: Get current absolute intermediate goal position
        :type:   int
        """
        data = self._ram_read(RAM_ABSOLUTE_DESIRED_TRAJECTORY_POSITION)
        return data[1]<<8|data[0]
#
    @property
    def absolute_intermediate_goal_angle(self):
        """
        :getter: Get current absolute intermediate goal angle
        :type:   float
        """
        data = self._ram_read(RAM_ABSOLUTE_DESIRED_TRAJECTORY_POSITION)
        return self._position_res*(data[1]<<8|data[0])
# 60
    @property
    def intermediate_native_goal_speed(self):
        """
        :getter: Get current intermediate native goal speed
        :type:   int
        """
        data = self._ram_read(RAM_DESIRED_VELOCITY)
        return data[1]<<8|data[0]
    @property
    def intermediate_absolute_goal_speed(self):
        """
        :getter: Get current intermediate absolute goal speed in step/second
        :type:   float
        """
        data = self._ram_read(RAM_DESIRED_VELOCITY)
        return self._absolute_goal_speed_res*(data[1]<<8|data[0])
#
    @property
    def intermediate_angular_goal_speed(self):
        """
        :getter: Get current intermediate angular goal speed in degree/second
        :type:   float
        """
        data = self._ram_read(RAM_DESIRED_VELOCITY)
        return self._angular_goal_speed_res*(data[1]<<8|data[0])
# 56
    @property
    def pwm(self):
        """
        :getter: Get current pulse width modulation value.
        :type:   int from -1023 to 1023

        .. tip::
            PWM can be considered as a torque like quantity.
        """
        data = self._ram_read(RAM_PWM)
        if data[1]&0x80:
            return -((data[1]<<8|data[0])^0xFFFF)
        else:
            return data[1]<<8|data[0]
# 49
    @property
    def voltage(self):
        """
        :getter: Get voltage
        :type:   float
        """
        return self._voltage_res*self._ram_read_single(RAM_VOLTAGE)
# 50
    @property
    def temperature(self):
        """
        :getter: Get temperature in degree Celsius
        :type:   float
        """
        if self._model_4or6:
            return float(self._ram_read_single(RAM_TEMPERATURE))
        else:
            return ADC_TEMPERATURE[self._ram_read_single(RAM_TEMPERATURE)]
