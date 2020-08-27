"""
.. module::constants
   :platform: Windows, Unix
   :synopsis: libdhd constants

.. moduleauthor:: Drason Chow <drasonchow@gmail.com>
"""

from enum import IntEnum

MAX_DOF: int = 8
MAX_BUTTONS: int = 16
TIMEGUARD: int = 1

VELOCITY_WINDOWING: int = 0
VELOCITY_WINDOW: int = 20

MAX_STATUS: int = 16

MOTOR_SATURATED: int = 2


class DeviceType(IntEnum):
    NONE = 0
    OMEGA3 = 33
    OMEGA33 = 34
    OMEGA33_LEFT = 36
    OMEGA331 = 35
    OMEGA331_LEFT = 37
    FALCON = 60
    CONTROLLER = 81
    CONTROLLER_HR = 82
    CUSTOM = 91
    SIGMA331 = 104
    SIGMA331_LEFT = 105
    DELTA3 = 63


class ErrorNum(IntEnum):
    NO_ERROR = 0
    ERROR = 1
    COM = 2
    DHC_BUSY = 3
    NO_DRIVER_FOUND = 4
    NO_DEVICE_FOUND = 5
    NOT_AVAILABLE = 6
    TIMEOUT = 7
    GEOMETRY = 8
    EXPERT_MODE_DISABLED = 9
    NOT_IMPLEMENTED = 10
    OUT_OF_MEMORY = 11
    DEVICE_NOT_READY = 12
    FILE_NOT_FOUND = 13
    CONFIGURATION = 14
    NULL_ARGUMENT = 15
    REDUNDANT_FAIL = 16
    NOT_ENABLED = 17
    DEVICE_IN_USE = 18


class DeltaMotorID(IntEnum):
    MOTOR0 = 0
    MOTOR1 = 1
    MOTOR2 = 2


class DeltaEncID(IntEnum):
    ENC0 = 0
    ENC1 = 1
    ENC2 = 2


class WristMotorID(IntEnum):
    MOTOR0 = 3
    MOTOR1 = 4
    MOTOR2 = 5


class WristEncID(IntEnum):
    ENC0 = 3
    ENC1 = 4
    ENC2 = 5


class NovintButtonID(IntEnum):
    CENTER = 0
    LEFT = 1
    UP = 2
    RIGHT = 3


class StatusIndex(IntEnum):
    POWER = 0
    CONNECTED = 1
    STARTED = 2
    RESET = 3
    IDLE = 4
    FORCE = 5
    BRAKE = 6
    TORQUE = 7
    WRIST_DETECTED = 8
    ERROR = 9
    GRAVITY = 10
    TIMEGUARD = 11
    WRIST_INIT = 12
    REDUNDANCY = 13
    FORCEOFFCAUSE = 14


class State(IntEnum):
    OFF = 0
    ON = 1


class ComMode(IntEnum):
    SYNC = 0
    ASYNC = 1
    VIRTUAL = 3
    NETWORK = 4


class ThreadPriority(IntEnum):
    DEFAULT = 0
    HIGH = 1
    LOW = 2
