"""
.. module::expert
   :platform: Windows, Unix
   :synopsis: libdhd "Expert SDK" Python bindings

.. moduleauthor:: Drason Chow <drasonchow@gmail.com>
"""


from typing import Tuple, List, Union, Optional, cast

from ctypes import (
    c_int, c_uint, c_bool, c_byte, c_ubyte, c_ushort, c_char_p, c_double
)

from ctypes import byref, POINTER

from forcedimension.dhd.bindings import _libdhd

from forcedimension.dhd.bindings.constants import (  # NOQA
    DeviceType, ComMode, StatusIndex, Error, ThreadPriority,
    DeltaMotorID, DeltaEncID, WristMotorID, WristEncID,
    State, MAX_BUTTONS, MAX_DOF, TIMEGUARD,
    VELOCITY_WINDOW, VELOCITY_WINDOWING,
    MAX_STATUS, MOTOR_SATURATED
)


_libdhd.dhdEnableExpertMode.argtypes = []
_libdhd.dhdEnableExpertMode.restype = c_int
def enableExpertMode() -> int: # NOQA
    """
    Enable expert mode.

    :rtype: int
    :returns: 0 on success, -1 otherwise.
    """
    return _libdhd.dhdEnableExpertMode()


_libdhd.dhdDisableExpertMode.argtypes = []
_libdhd.dhdDisableExpertMode.restype = c_int
def disableExpertMode() -> int: # NOQA
    """
    Enable expert mode.

    :rtype: int
    :returns: 0 on success, -1 otherwise.
    """
    return _libdhd.dhdDisableExpertMode()


# TODO add page for generic controller
# TODO add a page for multiple devices
_libdhd.dhdPreset.argtypes = [c_int * MAX_DOF, c_ubyte, c_byte]
_libdhd.dhdPreset.restype = c_int
DofTuple = Tuple[int, int, int, int, int, int, int, int]
def preset(val: DofTuple, mask: int, ID: int = -1) -> int: # NOQA
    """
    Set selected encoder offsets to a given value. Intended for use with the
    generic controller when no RESET button is available.

    :param DofTuple val: motor values array
    :param int mask: bitwise mask of which encoder should be set.
    :param int ID: [default=-1] device ID (see multiple devices section
    for details)

    :raises ValueError: if val is not implicitly convertible to C int[9]
    :raises ValueError: if mask is not implicitly convertible to C uchar
    :raises ValueError: if ID is not implicitly convertible to C char

    :rtype: int
    :returns: 0 on success, -1 otherwise
    """
    return _libdhd.dhdPreset(val, mask, ID)


# TODO add a page about TimeGuard
_libdhd.dhdSetTimeGuard.argtypes = [c_int, c_byte]
_libdhd.dhdSetTimeGuard.restype = c_int
def setTimeGuard(min_period: int, ID: int = -1) -> int: # NOQA
    """
    Enable/disable the TimeGuard feature with an arbitrary minimum period.

    :param int min_period: minimum refresh period in [us]. A value of 0
    disables the TimeGuard feature, while a value of -1 resets the default
    recommended value.

    :param int ID: [default=-1] device ID (see multiple devices section
    for details)

    :raises ValueError: if min_period is not implicitly convertible to C int.
    :raises ValueError: if ID is not implicitly convertible to C char

    :rtype: int
    :returns: 0 on success, -1 otherwise.
    """

    return _libdhd.dhdSetTimeGuard(min_period, ID)


# TODO Make a page for velocity threshold
_libdhd.dhdSetVelocityThreshold.argtypes = [c_uint, c_byte]
_libdhd.dhdSetVelocityThreshold.restype = c_int
def setVelocityThreshold(thresh: int, ID: int = -1) -> int: # NOQA
    """
    Adjust the velocity threshold of the device. Velocity threshold is a safety
    feature that prevents the device from accelerating to high velocities
    without control. If the velocity of one of the device axis passes the
    threshold, the device enters BRAKE mode.

    Warning: since the range of threshold values is device dependent, it is
    recommended NOT to modify factory settings.

    :param int thresh: an arbitrary value of velocity threshold
    :param int ID: [default=-1] device ID (see multiple devices section
    for details)

    :raises ValueError: if thresh  is not implicitly convertible to C uint.
    :raises ValueError: if ID is not implicitly convertible to C char

    :rtype: int
    :returns: 0 on success, -1 otherwise.
    """

    return _libdhd.dhdSetVelocityThreshold(thresh, ID)


_libdhd.dhdGetVelocityThreshold.argtypes = [POINTER(c_uint), c_byte]
_libdhd.dhdGetVelocityThreshold.restype = c_int
def getVelocityThreshold(ID: int = -1) -> Tuple[int, int]: # NOQA
    """
    Get the velocity threshold of the device. Velocity threshold is a safety
    feature that prevents the device from accelerating to high velocities
    without control. If the velocity of one of the device axis passes the
    threshold, the device enters BRAKE mode.

    Warning: Velocity thresholds are device dependent.

    :param int ID: [default=-1] device ID (see multiple devices section
    for details)

    :raises ValueError: if ID is not implicitly convertible to C char

    :rtype: int
    :returns: Tuple of (thresh, err) where err 0 on success, -1 otherwise and
    thresh is value of the velocity threshold.
    """

    thresh = c_uint()
    return (thresh.value, _libdhd.dhdGetVelocityThreshold(byref(thresh), ID))


_libdhd.dhdUpdateEncoders.argtypes = [c_byte]
_libdhd.dhdUpdateEncoders.restype = c_int
def updateEncoders(ID: int = -1) -> int: # NOQA
    """
    Force an update of the internal encoder values in the state vector. This
    call retrieves the encoder readings from the device and places them into
    the state vector. No kinematic model is called.

    :param int ID: [default=-1] device ID (see multiple devices section
    for details)

    :raises ValueError: if ID is not implicitly convertible to C char

    :rtype: int
    :returns: 0 on success, -1 otherwise.
    """
    return _libdhd.dhdUpdateEncoders(ID)


_libdhd.dhdGetDeltaEncoders.argtypes = [
    POINTER(c_int),
    POINTER(c_int),
    POINTER(c_int),
    c_byte
]
_libdhd.dhdGetDeltaEncoders.restypes = c_int
def getDeltaEncoders(ID: int = -1) -> Tuple[List[int], int]: # NOQA
    """
    Read all encoders values of the DELTA structure

    :param int ID: [default=-1] device ID (see multiple devices section
    for details)

    :raises ValueError: if ID is not implicitly convertible to C char

    :rtype: Tuple[List[int], int]
    :returns: tuple of ([enc0, enc1, enc2], err) where err is 0 or
    dhd.bindings.constants.TIMEGUARD on success, -1 otherwise and
    enc0, enc1, and enc2 are the axis 0, axis 1, and axis 2 encoder readings,
    respectively.
    """

    enc0 = c_int()
    enc1 = c_int()
    enc2 = c_int()

    return ([enc0.value, enc1.value, enc2.value],
            _libdhd.dhdGetDeltaEncoders(
                byref(enc0),
                byref(enc1),
                byref(enc2),
                ID
        )
    )


_libdhd.dhdGetWristEncoders.argtypes = [
    POINTER(c_int),
    POINTER(c_int),
    POINTER(c_int),
    c_byte
]
_libdhd.dhdGetWristEncoders.restypes = c_int
def getWristEncoders(ID: int = -1) -> Tuple[List[int], int]: # NOQA
    """
    Read all encoders values of the wrist structure.

    This feature only applies to the following devices:
        dhd.bindings.constants.DeviceType.OMEGA33
        dhd.bindings.constants.DeviceType.OMEGA33_LEFT
        dhd.bindings.constants.DeviceType.OMEGA331
        dhd.bindings.constants.DeviceType.OMEGA331_LEFT
        dhd.bindings.constants.DeviceType.SIGMA331
        dhd.bindings.constants.DeviceType.SIGMA331_LEFT

    :param int ID: [default=-1] device ID (see multiple devices section
    for details)

    :raises ValueError: if ID is not implicitly convertible to C char

    :rtype: Tuple[List[int], int]
    :returns: tuple of ([enc0, enc1, enc2], err) where err is 0 or
    dhd.bindings.constants.TIMEGUARD on success, -1 otherwise and
    enc0, enc1, and enc2 are the axis 0, axis 1, and axis 2 encoder readings,
    respectively.
    """

    enc0 = c_int()
    enc1 = c_int()
    enc2 = c_int()

    return ([enc0.value, enc1.value, enc2.value],
            _libdhd.dhdGetWristEncoders(
                byref(enc0),
                byref(enc1),
                byref(enc2),
                ID
        )
    )


_libdhd.dhdGetGripperEncoder.argtypes = [POINTER(c_int), c_byte]
_libdhd.dhdGetGripperEncoder.argtypes = c_int
def getGripperEncoder(ID: int = -1) -> Tuple[int, int]: # NOQA
    """
    Read the encoder value of the force gripper.

    This feature only applies to the following devices:
        dhd.bindings.constants.DeviceType.OMEGA331
        dhd.bindings.constants.DeviceType.OMEGA331_LEFT
        dhd.bindings.constants.DeviceType.SIGMA331
        dhd.bindings.constants.DeviceType.SIGMA331_LEFT

    :param int ID: [default=-1] device ID (see multiple devices section
    for details)

    :raises ValueError: if ID is not implicitly convertible to C char

    :rtype: Tuple[int, int]
    :returns: tuple of (enc, err) where err is 0 or
    dhd.bindings.constants.TIMEGUARD on success, -1 otherwise and enc is the
    gripper encoder reading.
    """

    enc = c_int()

    return (enc.value, _libdhd.dhdGetGripperEncoder(byref(enc), ID))


_libdhd.dhdGetEncoder.argtypes = [c_int, c_byte]
_libdhd.dhdGetEncoder.restype = c_int
def getEncoder(index: int, ID: int = -1) -> int: # NOQA
    """
    Read a single encoder value from the haptic device

    :param int index: the motor index number as defined by
    dhd.bindings.constants.MAX_DOF
    :param int ID: [default=-1] device ID (see multiple devices section
    for details)

    :raises ValueError: if index is not implicitly convertible to C int
    :raises ValueError: if ID is not implicitly convertible to C char

    :rtype: int
    :returns: the (positive) encoder reading on success, -1 otherwise
    """

    return _libdhd.dhdGetEncoder(index, ID)


_libdhd.dhdSetMotor.argtypes = [c_int, c_ushort, c_byte]
_libdhd.dhdSetMotor.restype = c_int
def setMotor(index: int, output: int, ID: int = -1) -> int: # NOQA
    """
    Program a command to a single motor channel.

    :param int index: the motor index number as defined by
    dhd.bindings.constants.MAX_DOF

    :param int output: the motor DAC value

    :param int ID: [default=-1] device ID (see multiple devices section
    for details)

    :raises ValueError: if index is not implicitly convertible to C int
    :raises ValueError: if output is not implicitly convertible to C ushort
    :raises ValueError: if ID is not implicitly convertible to C char

    :rtype: int
    :returns: 0 on success, -1 otherwise.
    """

    return _libdhd.dhdSetMotor(index, output, ID)



