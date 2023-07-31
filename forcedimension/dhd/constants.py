from enum import IntEnum
from typing import Final, Literal, Tuple

MAX_DOF: Final[int] = 8   #: Maximum number of DOF a device can have
MAX_BUTTONS: Final[int] = 16  #: Maximum number of buttons a device can have

VELOCITY_WINDOWING: Final[int] = 0  #: Velocity Windowing Mode
DEFAULT_VELOCITY_WINDOW: Final[int] = 20   #: Default velocity window size

MAX_STATUS: Final[int] = 16  #: Maximum number of elements in a status tuple

#: Return value used when the TimeGuard feature prevented an unnecessary
#: communication with the device.
TIMEGUARD: Final[int] = 1

#: Return value used when at least one of the motors cannot deliver the
#: requested torque. Motor groups are scaled in order to preserve force and
#: torque direction over magnitude.
MOTOR_SATURATED: Final[int] = 2

class VelocityEstimatorMode(IntEnum):
    """
    Supported velocity estimator modes. Currently only WINDOWING mode is
    available.
    """
    #: The default velocity estimator mode. In this mode, the velocity is
    #: estimated by comparing the current position with the position a given
    #: time interval ago. This time interval (or "window") can be adjusted
    #: using dhdConfigLinearVelocity, and should be modified to best suit the
    #: dynamic behavior of the device for a given application. The windowing
    #: estimator mode is the least resource intensive.
    WINDOWING = 0
    """
    The default velocity estimator mode. In this mode, the velocity is
    estimated by comparing the current position with the position a given
    time interval ago.
    """


class DeviceType(IntEnum):
    """
    Supported device type IDs as an enumeration.
    """
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
    """
    Error numbers reported by DHD as an enumeration.
    """
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

#: Array index for motors/encoders of the WRIST structure.
#: i.e. DELTA motor/encoder 0 has index 0
DELTA_IDX: Final[Tuple[Literal[0], Literal[1], Literal[2]]] = (0, 1, 2)

#: Array index for motors/encoders of the WRIST structure.
#: i.e. WRIST motor/encoder 0 has index 3
WRIST_IDX: Final[Tuple[Literal[3], Literal[4], Literal[5]]] = (3, 4, 5)

class NovintButtonID(IntEnum):
    """
    Enumeration mapping button type to button ID for the Novint Falcon.
    """
    CENTER = 0
    LEFT = 1
    UP = 2
    RIGHT = 3


class StatusIndex(IntEnum):
    """
    Index of statuses in the DHD status vector as an enumeration.
    """
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


class ForceOffCause(IntEnum):
    """
    The event that caused forces to be disabled on the device (the last time
    forces were turned off).

    Info
    ----
    Not all devices suppport all the force-disabling mechanisms listed above.
    """
    FORCES_NOT_OFF = 0
    BUTTON = 1
    VELOCITY = 2
    WATCHDOG = 3
    SOFTWARE = 4
    USBDISCN = 5
    DEADMAN = 6


class ComMode(IntEnum):
    """
    Communication mode with the device. USB operations using
    :data:`forcedimension.dhd.constants.ComMode.SYNC` and
    :data:`forcedimension.dhd.constants.ComMode.ASYNC`.
    Other operation modes are reported for virtual devices
    (:data:`forcedimension.dhd.constants.ComMode.VIRTUAL`) and devices that are
    connected over the network
    (:data:`forcedimension.dhd.constants.ComMode.NETWORK`).
    """
    #: The synchronous USB mode performs USB read and write operations in
    #: sequence, allowing for a theoretical haptic refresh rate of 4 kHz.
    #: Please note that Other factors also influence USB performance, including
    #: the choice of operating system, machine load and program optimisation.
    SYNC = 0

    #: The asynchronous USB mode is the default. The asynchronous USB mode
    #: allows the operating system to parallelise the read and write operations
    #: on the USB port. This parallel operation improves refresh rate stability
    #: by reducing communication jitter. Other factors also influence USB
    #: performance, including the choice of operating system, machine load and
    #: program optimisation.
    ASYNC = 1

    #: This mode is reported when connected to a virtual device.
    VIRTUAL = 3

    #: This mode is reported when connected to a haptic device using the Force
    #: Dimension network connection mode.
    NETWORK = 4


class ThreadPriority(IntEnum):
    DEFAULT = 0
    HIGH = 1
    LOW = 2
