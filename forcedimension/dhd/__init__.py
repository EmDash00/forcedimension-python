from typing import Tuple, Union, Optional

import ctypes

from ctypes import (
    c_int, c_uint, c_bool, c_byte, c_ubyte, c_ushort, c_char_p, c_double
)

from ctypes import byref, POINTER
import forcedimension.runtime as runtime

# It's better to import everything like this,
# so linters can have an easier time.
from forcedimension.dhd.constants import (  # NOQA
    DeviceType, ComMode, StatusIndex, Error, ThreadPriority,
    DeltaMotorID, DeltaEncID, WristMotorID, WristEncID,
    State, MAX_BUTTONS, MAX_DOF, TIMEGUARD,
    VELOCITY_WINDOW, VELOCITY_WINDOWING,
    MAX_STATUS, MOTOR_SATURATED
)

"""
.. module::dhd
   :platform: Windows, Unix
   :synopsis: libdhd Python bindings

.. moduleauthor:: Drason Chow <drasonchow@gmail.com>
"""

# Load the runtime from the backend
_libdhd = runtime.load("libdhd")


_libdhd.dhdEnableSimulator.argtypes = [c_bool]
_libdhd.dhdEnableSimulator.restype = None


def enableSimulator(on: bool) -> None:  # NOQA
    """
    Enable device simulator support.
    This enables network access on the loopback interface.

    :param on: True to enable, False to disable
    """
    _libdhd.dhdEnableSimulator(on)


_libdhd.dhdGetDeviceCount.argtypes = []
_libdhd.dhdGetDeviceCount.restype = c_int


def getDeviceCount() -> int:  # NOQA
    """
    Return the number of compatible Force Dimension devices connected to
    the system. This encompasses all devices connected locally,
    including devices already locked by other applications.

    Devices are given a unique identifier, as explained in the
    multiple devices section.

    :rtype: int
    :returns: the number of connected devices on success, -1 otherwise
    """

    return _libdhd.dhdGetDeviceCount()


_libdhd.dhdGetAvailableCount.argtypes = []
_libdhd.dhdGetAvailableCount.restype = c_int


def getAvailableCount() -> int:  # NOQA
    """
    Return the number of available Force Dimension devices connected to the
    system. This encompasses all devices connected locally, but excludes
    devices already locked by other applications.

    Devices are given a unique identifier, as explained in the
    multiple devices section.

    :rtype: int
    :returns: the number of devices available on success, -1 otherwise
    """
    return _libdhd.dhdGetAvailableCount()


_libdhd.dhdSetDevice.argtypes = [c_byte]
_libdhd.dhdSetDevice.restype = c_int


def setDevice(ID: int = -1) -> int:  # NOQA
    """
    Select the default device that will receive the SDK commands.
    The SDK supports multiple devices.

    This routine allows the programmer to decide which device the SDK
    dhd_single_device_call single-device calls will address. Any subsequent
    SDK call that does not specifically mention the device ID in its
    parameter list will be sent to that device.

    :param int: ID [default=-1]

    :raises ValueError: if ID is not convertible to a C char type

    :rtype: int
    :returns: 0 on success, -1 otherwise

    """

    return _libdhd.setDevice(ID)


_libdhd.dhdGetSerialNumber.argtypes = [POINTER(c_ushort), c_byte]
_libdhd.dhdGetSerialNumber.restype = c_int


def getSerialNumber(ID: int = -1) -> Tuple[int, int]:  # NOQA
    """
    Return the device serial number.

    :param int ID: [default=-1] device ID (see multiple devices
    section for details)

    :raises ValueError: if ID is not convertible to a C char

    :rtype: Tuple[int, int]
    :returns: tuple of (serial number, err). err is 0 on success, -1 otherwise.
    """
    sn = c_ushort()
    err = _libdhd.dhdGetSerialNumber(byref(sn), ID)

    return (sn.value, err)


_libdhd.dhdOpen.argtypes = []
_libdhd.dhdOpen.restype = c_int


def open() -> int:  # NOQA
    """
    Open a connection to the first available device connected to the
    system. The order in which devices are opened persists until devices
    are added or removed.

    If this call is successful, the default device ID is set to the newly
    opened device. See the multiple device section for more information on
    using multiple devices on the same computer.

    See also dhd.openID()

    :rtype: int
    :returns: The device ID on success, -1 otherwise.
    """
    return _libdhd.dhdOpen()


_libdhd.dhdOpenType.argtypes = [c_int]
_libdhd.dhdOpenType.restype = c_int


def openType(device_type: int) -> int:  # NOQA
    """
    Open a connection to the first device of a given type connected to the
    system. The order in which devices are opened persists until devices
    are added or removed.

    If this call is successful, the default device ID is set to the newly
    opened device. See the multiple device section for more information on
    using multiple devices on the same computer.

    See also dhd.openID()

    :param int device_type: requested system Device Type type

    :rtype: int
    :returns: The device ID on success, -1 otherwise.
    """

    return _libdhd.dhdOpenType(device_type)


_libdhd.dhdOpenSerial.argtypes = [c_int]
_libdhd.dhdOpenSerial.restype = c_int


def openSerial(serial: int) -> int:  # NOQA
    """
    Open a connection to the device with a given serial number (available on
    recent models only).

    If this call is successful the default device ID is set to the newly opened
    device. See the multiple device section for more information on using
    multiple devices on the same computer.

    See also dhd.openID()

    :param int serial: requested system serial number.

    :rtype: int
    :returns: The device ID on success, -1 otherwise.

    """

    return _libdhd.dhdOpenSerial(serial)


_libdhd.dhdOpenID.argtypes = [c_int]
_libdhd.dhdOpenID.restype = c_int


def openID(index: int) -> int:  # NOQA
    """
    Open a connection to one particular device connected to the system. The
    order in which devices are opened persists until devices are added or
    removed. If the device at the specified index is already opened, its device
    ID is returned.

    If this call is successful, the default device ID is set to the newly
    opened device. See the multiple device section for more information on
    using multiple devices on the same computer.

    See also dhd.open()

    :param in index: the device enumeration index, as assigned by the
    underlying operating system (must be between 0 and the number of devices
    connected to the system)

    :raises ValueError: if index is not convertible to a C int.

    :returns: The device ID on success, -1 otherwise
    :rtype: int
    """

    return _libdhd.dhdOpenID(index)


_libdhd.dhdClose.argtypes = [c_byte]
_libdhd.dhdClose.restype = c_int


def close(ID: int = -1) -> int:  # NOQA
    """
    Close the connection to a particular device.

    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if ID is not implicitly convertible to C char.
    :rtype: int
    :returns: 0 on success, -1 otherwise.
    """
    return _libdhd.dhdClose(ID)


_libdhd.dhdStop.argtypes = [c_byte]
_libdhd.dhdStop.restype = c_int


def stop(ID: int = -1) -> int:  # NOQA
    """
    Stop the device. This routine disables the force on the haptic device and
    puts it into BRAKE mode.

    :param ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if ID is not implicitly convertible to C int.

    :rtype: int
    :returns: 0 on success, -1 otherwise.
    """
    return _libdhd.dhdStop(ID)


_libdhd.dhdGetComMode.argtypes = [c_byte]
_libdhd.dhdGetComMode.restype = c_int
def getComMode(ID: int = -1) -> int:  # NOQA
    """
    Retrive the COM operation mode on compatible devices.

    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if ID is not implicitly convertible to C char.

    :rtype: int
    :returns: the current COM operation mode on success, -1 otherwise.
    """

    return _libdhd.dhdGetComMode(ID)


_libdhd.dhdEnableForce.argtypes = [c_bool, c_byte]
_libdhd.dhdEnableForce.restype = c_int
def enableForce(val: bool, ID: int = -1) -> int: # NOQA
    """
    Enable the force mode in the device controller.

    :param bool val: True to enable force, False to disable it.
    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if val is not implicitly convertible to C bool
    :raises ValueError: if ID is not implicitly convertible to C int

    :rtype: int
    :returns: 0 on success, -1 otherwise.
    """

    return _libdhd.dhdEnableForce(val, c_byte)


_libdhd.dhdEnableGripperForce.argtypes = [c_bool, c_byte]
_libdhd.dhdEnableGripperForce.restype = c_int
def enableGripperForce(enable: bool, ID: int = -1) -> int: # NOQA
    """
    Enable the force mode in the device controller.

    :param bool enable: True to enable force, False to disable it.
    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if val is not implicitly convertible to C bool.
    :raises ValueError: if ID is not implicitly convertible to C int

    :rtype: int
    :returns: 0 on success, -1 otherwise.
    """

    return _libdhd.dhdEnableGripperForce(enable, ID)


_libdhd.dhdGetSystemType.argtypes = [c_byte]
_libdhd.dhdGetSystemType.restype = c_int
def getSystemType(ID: int = -1) -> int:  # NOQA
    """
    Retrive the COM operation mode on compatible devices.

    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if ID is not implicitly convertible to C char.

    :rtype: int
    :returns: The device type on success, -1 otherwise.
    """

    return _libdhd.dhdGetSystemType(ID)


_libdhd.dhdGetSystemName.argtypes = [c_byte]
_libdhd.dhdGetSystemName.restype = c_char_p
def getSystemName(ID: int = -1) -> Union[str, None]:  # NOQA
    """
    Retrive the COM operation mode on compatible devices.

    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if ID is not implicitly convertible to C char.

    :rtype: int
    :returns: The device type as a string on success, None otherwise.
    """

    ret = _libdhd.dhdGetSystemName(ID)
    if (ret is not None):
        return ret.decode("utf-8")  # python using decode bytes as unicode str
    else:
        return None


_libdhd.dhdGetVersion.argtypes = [POINTER(c_double), c_byte]
_libdhd.dhdGetVersion.restype = c_int
def getVersion(ID: int = -1) -> Tuple[float, int]: # NOQA
    """
    Return the device controller version. As this SDK can be used to control
    all of Force Dimension haptic products, this can help programmers ensure
    that their application is running on the appropriate version of the haptic
    controller.

    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if ID is not convertible to a C char type

    :rtype: Tuple[float, int]
    :returns: Tuple of (version, err). err is 0 on success, -1 otherwise.
    """

    ver = c_double()

    return (ver.value, _libdhd.dhdGetVersion(byref(ver), ID))


_libdhd.dhdGetSDKVersion.argtypes = [
    POINTER(c_int),
    POINTER(c_int),
    POINTER(c_int),
    POINTER(c_int)
]
_libdhd.dhdGetSDKVersion.restype = None
def getSDKVersion() -> Tuple[int, int, int, int]:  # NOQA
    """
    Get the version of the ForceDimensionSDK. Versions of the ForceDimensionSDK
    are reported as major.minor.release-revision by ForceDimension.

    :returns: A tuple in the form (major, minor, release, revision)
    :rtype: Tuple[int, int, int, int]
    """
    major = c_int()
    minor = c_int()
    release = c_int()
    revision = c_int()

    _libdhd.dhdGetSDKVersion(
        byref(major),
        byref(minor),
        byref(release),
        byref(revision)
    )

    return (major.value, minor.value, release.value, revision.value)


_libdhd.dhdGetStatus.argtypes = [c_int * MAX_STATUS, c_byte]
_libdhd.dhdGetStatus.restype = c_int
StatusTuple = Tuple[
    int, int, int, int,
    int, int, int, int,
    int, int, int, int,
    int, int, int, int
]
def getStatus(ID: int = -1) -> Tuple[StatusTuple, int]: # NOQA
    """
    Returns the status list of the haptic device. The status is described in
    the status section.

    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if ID is not convertible to a C char type

    :rtype: Tuple[StatusTuple, int]
    :returns: Tuple of (status_lst, err). err is 0 on success, -1 otherwise.
    """

    status_vec = (c_int * MAX_STATUS)()
    err = _libdhd.dhdGetStatus(status_vec, ID)

    return (tuple(status_vec), err)


_libdhd.dhdGetDeviceAngleRad.argtypes = [POINTER(c_double), c_byte]
_libdhd.dhdGetDeviceAngleRad.restype = c_int
def getDeviceAngleRad(ID: int = -1) -> Tuple[float, int]: # NOQA
    """
    Get the device base plate angle around the Y axis in radians.

    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if ID is not convertible to a C char type

    :rtype: Tuple[float, int]
    :returns: Tuple of (angle_rad, err). err is 0 on success, -1 otherwise.
    """

    angle_rad = c_double()
    return (angle_rad.value,
            _libdhd.dhdGetDeviceAngleRad(byref(angle_rad), ID))


_libdhd.dhdGetDeviceAngleDeg.argtypes = [POINTER(c_double), c_byte]
_libdhd.dhdGetDeviceAngleDeg.restype = c_int
def getDeviceAngleDeg(ID: int = -1) -> Tuple[float, int]: # NOQA
    """
    Get the device base plate angle around the Y axis in degrees.

    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if ID is not convertible to a C char type

    :rtype: Tuple[float, int]
    :returns: Tuple of (angle_deg, err). err is 0 on success, -1 otherwise.
    """

    angle_deg = c_double()
    return (angle_deg.value,
            _libdhd.dhdGetDeviceAngleDeg(byref(angle_deg), ID))


_libdhd.dhdGetEffectorMass.argtypes = [POINTER(c_double), c_byte]
_libdhd.dhdGetEffectorMass.restype = c_int
def getEffectorMass(ID: int = -1) -> Tuple[float, int]: # NOQA
    """
    Get the mass of the end-effector currently defined for a device.
    The gripper mass is used in the gravity compensation feature.

    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if ID is not implicitly convertible to a C char type

    :rtype: Tuple[float, int]
    :returns: Tuple of (mass, err). err is 0 on success, -1 otherwise.
    """

    mass = c_double()
    return (mass.value, _libdhd.dhdGetEffectorMass(byref(mass), ID))


_libdhd.dhdGetButton.argtypes = [c_int, c_byte]
_libdhd.dhdGetButton.restype = c_int
def getButton(index: int, ID: int = -1) -> Union[int, State]: # NOQA
    """
    Return the status of the button located on the end-effector

    :param int index: button index, 0 for the gripper button (up to
    DHD_MAX_BUTTONS)

    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if ID is not implicitly convertible to a C char type

    :rtype: Union[int, State]
    :returns: State.ON if button is pressed, State.OFF otherwise,
    and -1 on error.
    """

    state = _libdhd.dhdGetButton(index, ID)

    return (State(state) if state != -1 else -1)


_libdhd.dhdGetButtonMask.argtypes = [c_byte]
_libdhd.dhdGetButtonMask.restype = c_uint
def getButtonMask(ID: int = -1) -> int: # NOQA
    """

    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if ID is not implicitly convertible to a C char type

    :rtype: int
    :returns: 32-bit long bitmask. Each bit is set to 1 if the button is
    pressed, 0 otherwise.
    """

    return _libdhd.dhdGetButtonMask(ID)


_libdhd.dhdSetOutput.argtypes = [c_uint, c_byte]
_libdhd.dhdSetOutput.restype = c_int
def setOutput(output: int, ID: int = -1) -> int: # NOQA
    """
    Set the user programmable output bits on devices that support it.

    This feature only applies to the following devices:
        DeviceType.DELTA3
        DeviceType.SIGMA331
        DeviceType.SIGMA331_LEFT

    :param int output: a bitwise mask that toggles the programmable output bits

    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if ID is not implicitly convertible to a C char type
    :raises ValueError: if output is not implicitly convertible to C uint type

    :rtype: int
    :returns: 0 on success, -1 otherwise
    """

    return _libdhd.dhdSetOutput(output, ID)


_libdhd.dhdIsLeftHanded.argtypes = [c_byte]
_libdhd.dhdIsLeftHanded.restype = c_bool
def isLeftHanded(ID: int = -1) -> bool: # NOQA
    """
    Returns True if the device is configured for left-handed use,
    False otherwise.

    This feature only applies to the following devices:
        DeviceType.OMEGA33
        DeviceType.OMEGA33_LEFT
        DeviceType.OMEGA331
        DeviceType.OMEGA331_LEFT
        DeviceType.SIGMA331
        DeviceType.SIGMA331_LEFT

    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if ID is not implicitly convertible to a C char type

    :rtype: bool
    :returns: True if the device is configured for left-handed use, False
    otherwise
    """

    return _libdhd.dhdIsLeftHanded(ID)


_libdhd.dhdHasBase.argtypes = [c_byte]
_libdhd.dhdHasBase.restype = c_bool
def hasBase(ID: int = -1) -> bool: # NOQA
    """
    Returns True if the device has a base, False otherwise.

    This feature only applies to the following devices:
        DeviceType.OMEGA33
        DeviceType.OMEGA33_LEFT
        DeviceType.OMEGA331
        DeviceType.OMEGA331_LEFT
        DeviceType.SIGMA331
        DeviceType.SIGMA331_LEFT
        DeviceType.DELTA3
        DeviceType.FALCON

    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if ID is not implicitly convertible to a C char type

    :rtype: bool
    :returns: Returns True if the device has a base, False otherwise.
    """

    return _libdhd.dhdHasBase(ID)


_libdhd.dhdHasWrist.argtypes = [c_byte]
_libdhd.dhdHasWrist.restype = c_bool
def hasWrist(ID: int = -1) -> bool: # NOQA
    """
    Returns True if the device has a wrist, False otherwise.

    This feature only applies to the following devices:
        DeviceType.OMEGA33
        DeviceType.OMEGA33_LEFT
        DeviceType.OMEGA331
        DeviceType.OMEGA331_LEFT
        DeviceType.SIGMA331
        DeviceType.SIGMA331_LEFT

    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if ID is not implicitly convertible to a C char type

    :rtype: bool
    :returns: Returns True if the device has a wrist, False otherwise.
    """

    return _libdhd.dhdHasWrist(ID)


_libdhd.dhdHasActiveWrist.argtypes = [c_byte]
_libdhd.dhdHasActiveWrist.restype = c_bool
def hasActiveWrist(ID: int = -1) -> bool: # NOQA
    """
    Returns True if the device has an active wrist, False otherwise.

    This feature only applies to the following devices:
        DeviceType.OMEGA33
        DeviceType.OMEGA33_LEFT
        DeviceType.OMEGA331
        DeviceType.OMEGA331_LEFT
        DeviceType.SIGMA331
        DeviceType.SIGMA331_LEFT

    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if ID is not implicitly convertible to a C char type

    :rtype: bool
    :returns: Returns True if the device has an active wrist, False otherwise.
    """

    return _libdhd.dhdHasActiveWrist(ID)


_libdhd.dhdHasGripper.argtypes = [c_byte]
_libdhd.dhdHasGripper.restype = c_bool
def hasGripper(ID: int = -1) -> bool: # NOQA
    """
    Returns True if the device has a gripper, False otherwise.

    This feature only applies to the following devices:
        DeviceType.OMEGA331
        DeviceType.OMEGA331_LEFT
        DeviceType.SIGMA331
        DeviceType.SIGMA331_LEFT

    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if ID is not implicitly convertible to a C char type

    :rtype: bool
    :returns: Returns True if the device has a gripper, False otherwise.
    """

    return _libdhd.dhdHasGripper(ID)


_libdhd.dhdHasActiveGripper.argtypes = [c_byte]
_libdhd.dhdHasActiveGripper.restype = c_bool
def hasActiveGripper(ID: int = -1) -> bool: # NOQA
    """
    Returns True if the device has an active wrist, False otherwise.

    This feature only applies to the following devices:
        DeviceType.OMEGA331
        DeviceType.OMEGA331_LEFT
        DeviceType.SIGMA331
        DeviceType.SIGMA331_LEFT

    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if ID is not implicitly convertible to a C char type

    :rtype: bool
    :returns: Returns True if the device has an active gripper,
    False otherwise.
    """

    return _libdhd.dhdHasActiveGripper(ID)


_libdhd.dhdReset.argtypes = [c_byte]
_libdhd.dhdReset.restype = c_int
def reset(ID: int = -1) -> int: # NOQA
    """
    Puts the device in RESET mode.

    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if ID is not implicitly convertible to a C char type

    :rtype: int
    :returns: Returns 0 on success, -1 otherwise.
    """

    return _libdhd.dhdReset(ID)


# _libdhd.dhdWaitForReset.argtypes = [c_int, c_byte]
_libdhd.dhdWaitForReset.restype = c_int
def waitForReset(timeout: Optional[int] = None, ID: int = -1) -> int: # NOQA
    """
    Puts the device in RESET mode and waits for the user to calibrate the
    device. Optionally, a timeout can be defined after which the call returns
    even if calibration has not occured.

    If the timeout is reached, the call returns an error (-1) and dhdErrno is
    set to Error.TIMEOUT.

    :param Optional[int] timeout: Maximum time to wait for calibration in [ms]

    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if timeout is not implicitly convertible to C int type
    if set

    :raises ValueError: if ID is not implicitly convertible to a C char type

    :rtype: int
    :returns: Returns 0 on success, -1 otherwise.
    """

    if timeout is not None:
        return _libdhd.dhdWaitForReset(timeout, ID)
    else:
        return _libdhd.dhdWaitForReset(ID)


_libdhd.dhdSetStandardGravity.argtypes = [c_double, c_byte]
_libdhd.dhdSetStandardGravity.restype = c_int
def setStandardGravity(g: float, ID: int = -1) -> int: # NOQA
    """
    Set the standard gravity constant used in gravity compensation. By default,
    the constant is set to 9.81 m/s^2

    :param float g: standard gravity constant [m/s^2]

    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if g is not implicitly convertible to C double type.
    :raises ValueError: if ID is not implicitly convertible to a C char type

    :rtype: int
    :returns: Returns 0 on success, -1 otherwise.
    """

    return _libdhd.dhdSetStandardGravity(g, ID)


_libdhd.dhdSetGravityCompensation.argtypes = [c_bool, c_byte]
_libdhd.dhdSetGravityCompensation.restype = c_int
def setGravityCompensation(enable: bool, ID: int = -1) -> int: # NOQA
    """
    Enable/disable gravity compensation.

    :param bool enable: True to turn on gravity compensation, False to turn off
    gravity compensation.

    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if val is not implicitly convertible to C bool type.
    :raises ValueError: if ID is not implicitly convertible to a C char type

    :rtype: int
    :returns: Returns 0 on success, -1 otherwise.
    """

    return _libdhd.dhdSetStandardGravity(enable, ID)


_libdhd.dhdSetBrakes.argtypes = [c_bool, c_byte]
_libdhd.dhdSetBrakes.restype = c_int
def setBrakes(enable: bool, ID: int = -1) -> int: # NOQA
    """
    Enable/disable device electromagnetic brakes.

    :param bool enable: True to turn on gravity compensation, False to turn off
    gravity compensation.

    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if val is not implicitly convertible to C bool type.
    :raises ValueError: if ID is not implicitly convertible to a C char type

    :rtype: int
    :returns: Returns 0 on success, -1 otherwise.
    """

    return _libdhd.dhdSetBrakes(enable, ID)


_libdhd.dhdSetDeviceAngleRad.argtypes = [c_double, c_byte]
_libdhd.dhdSetDeviceAngleRad.restype = c_int
def setDeviceAngleRad(angle: float, ID: int = -1) -> int: # NOQA
    """
    Set the device base plate angle around the (inverted) Y axis. Please refer
    to your device user manual for more information on your device coordinate
    system. An angle value of 0 corresponds to the device "upright" position,
    with its base plate perpendicular to axis X. An angle value of pi/2
    corresponds to the device base plate resting horizontally.

    See also dhd.setDeviceAngleDeg()

    :param float angle: device base plate angle [rad]
    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if angle is not implicitly convertible to C double type
    :raises ValueError: if ID is not implicitly convertible to a C char type

    :rtype: int
    :returns: Returns 0 on success, -1 otherwise.
    """

    return _libdhd.dhdSetDeviceAngleRad(angle, ID)


_libdhd.dhdSetDeviceAngleRad.argtypes = [c_double, c_byte]
_libdhd.dhdSetDeviceAngleRad.restype = c_int
def setDeviceAngleDeg(angle: float, ID: int = -1) -> int: # NOQA
    """
    Set the device base plate angle around the (inverted) Y axis. Please refer
    to your device user manual for more information on your device coordinate
    system. An angle value of 0 corresponds to the device "upright" position,
    with its base plate perpendicular to axis X. An angle value of 90
    corresponds to the device base plate resting horizontally.

    See also dhd.setDeviceAngleRad()

    :param float angle: device base plate angle [deg]
    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if angle is not implicitly convertible to C double type
    :raises ValueError: if ID is not implicitly convertible to a C char type

    :rtype: int
    :returns: Returns 0 on success, -1 otherwise.
    """

    return _libdhd.dhdSetDeviceAngleDeg(angle, ID)


_libdhd.dhdSetEffectorMass.argtypes = [c_double, c_byte]
_libdhd.dhdSetEffectorMass.restype = c_int
def setEffectorMass(mass: float, ID: int = -1) -> int: # NOQA
    """
    Define the mass of the end-effector. This function is required to provide
    accurate gravity compensation when custom-made or modified end-effectors
    are used.

    See also dhd.getEffectorMass()

    :param float mass: the actual end-effector mass in [kg]
    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if angle is not implicitly convertible to C double type
    :raises ValueError: if ID is not implicitly convertible to a C char type

    :rtype: int
    :returns: Returns 0 on success, -1 otherwise.
    """

    return _libdhd.dhdSetEffectorMass(mass, ID)


_libdhd.dhdGetPosition.argtypes = [
    POINTER(c_double),
    POINTER(c_double),
    POINTER(c_double),
    POINTER(c_double),
    c_byte
]
_libdhd.dhdGetPosition.restype = c_int
def getPosition(ID: int = -1) -> Tuple[list(float), int]:  # NOQA
    """
    Retrieve the position of the end-effector in Cartesian coordinates. Please
    refer to your device user manual for more information on your device
    coordinate system.

    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if ID is not implicitly convertible to a C char type

    :returns: A tuple in the form ([px, py, pz], err) where err is either 0
    or dhd.constants.TIMEGUARD on success, -1 otherwise.

    :rtype: Tuple[list(float), int]
    """

    px = c_double()
    py = c_double()
    pz = c_double()

    return ([px.value, py.value, pz.value],
            _libdhd.dhdGetPosition(byref(px), byref(py), byref(pz), ID))


_libdhd.dhdGetForce.argtypes = [
    POINTER(c_double),
    POINTER(c_double),
    POINTER(c_double),
    POINTER(c_double),
    c_byte
]
_libdhd.dhdGetForce.restype = c_int
def getForce(ID: int = -1) -> Tuple[list(float), int]:  # NOQA
    """
    Retrieve the force vector applied to the end-effector in Cartesian
    coordinates. Please refer to your device user manual for more information
    on your device coordinate system.

    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if ID is not implicitly convertible to a C char type

    :returns: A tuple in the form ([fx, fy, fz], err) where err is either 0,
    -1 otherwise.

    :rtype: Tuple[list(float), int]
    """

    fx = c_double()
    fy = c_double()
    fz = c_double()

    return ([fx.value, fy.value, fz.value],
            _libdhd.dhdGetPosition(byref(fx), byref(fy), byref(fz), ID))


_libdhd.dhdSetForce.argtypes = [c_double, c_double, c_double, c_double, c_byte]
_libdhd.dhdSetForce.restype = c_int
def setForce(f: Tuple[float, float, float], ID: int = -1) -> int:  # NOQA
    """
    Set the desired force vector in Cartesian coordinates to be applied
    to the end-effector of the device.

    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if ID is not implicitly convertible to a C char type
    :raises ValueError: if any members of f are not implicitly convertible to a
    C double type.

    :returns: 0 or dhd.MOTOR_SATURATED on success -1 otherwise.

    :rtype: Tuple[list(float), int]
    """

    return _libdhd.dhdSetForce(f[0], f[1], f[2], ID)


_libdhd.dhdGetOrientationRad.argtypes = [c_byte]
_libdhd.dhdGetOrientationRad.restype = c_int
def getOrientationRad(ID: int = -1) -> Tuple[list(float), int]:  # NOQA
    """
    For devices with a wrist structure, retrieve individual angle of each
    joint, starting with the one located nearest to the wrist base plate. For
    the DHD_DEVICE_OMEGA33 and DHD_DEVICE_OMEGA33_LEFT devices, angles are
    computed with respect to their internal reference frame, which is rotated
    pi/4 radians around the Y axis. Please refer to your device user manual for
    more information on your device coordinate system.

    This feature only applies to the following devices:
        dhd.DeviceType.OMEGA33
        dhd.DeviceType.OMEGA33_LEFT
        dhd.DeviceType.OMEGA331
        dhd.DeviceType.OMEGA331_LEFT
        dhd.DeviceType.SIGMA331
        dhd.DeviceType.SIGMA331_LEFT


    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if ID is not implicitly convertible to a C char type

    :returns: Tuple of ([oa, ob, og], err) where err is 0 or dhd.TIMEGUARD on
    success, -1 otherwise. oa, ob, and og refer to the device orientation
    around the first, second, and third wrist joints, respectively, in [rad]

    :rtype: Tuple[list(float), int]
    """

    oa = c_double()
    ob = c_double()
    og = c_double()

    return ([oa.value, ob.value, og.value],
            _libdhd.dhdGetOrientationRad(byref(oa), byref(ob), byref(og), ID))


_libdhd.dhdGetOrientationDeg.argtypes = [c_byte]
_libdhd.dhdGetOrientationDeg.restype = c_int
def getOrientationDeg(ID: int = -1) -> Tuple[list(float), int]:  # NOQA
    """
    For devices with a wrist structure, retrieve individual angle of each
    joint, starting with the one located nearest to the wrist base plate. For
    the DHD_DEVICE_OMEGA33 and DHD_DEVICE_OMEGA33_LEFT devices, angles are
    computed with respect to their internal reference frame, which is rotated
    45 degrees around the Y axis. Please refer to your device user manual for
    more information on your device coordinate system.

    This feature only applies to the following devices:
        dhd.DeviceType.OMEGA33
        dhd.DeviceType.OMEGA33_LEFT
        dhd.DeviceType.OMEGA331
        dhd.DeviceType.OMEGA331_LEFT
        dhd.DeviceType.SIGMA331
        dhd.DeviceType.SIGMA331_LEFT

    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if ID is not implicitly convertible to a C char type

    :returns: Tuple of ([oa, ob, og], err) where err is 0 or dhd.TIMEGUARD on
    success, -1 otherwise. oa, ob, and og refer to the device orientation
    around the first, second, and third wrist joints, respectively, in [rdeg]

    :rtype: Tuple[list(float), int]
    """

    oa = c_double()
    ob = c_double()
    og = c_double()

    return ([oa.value, ob.value, og.value],
            _libdhd.dhdGetOrientationDeg(byref(oa), byref(ob), byref(og), ID))


