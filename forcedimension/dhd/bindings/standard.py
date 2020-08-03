"""
.. module::standard
   :platform: Windows, Unix
   :synopsis: libdhd Standard SDK Python bindings

.. moduleauthor:: Drason Chow <drasonchow@gmail.com>
"""

from typing import NamedTuple, Tuple, List, Union, Optional

from ctypes import c_int, c_uint, c_bool, c_byte, c_ushort, c_char_p, c_double
from ctypes import byref, POINTER

from forcedimension.dhd.bindings.constants import DeviceType, State, MAX_STATUS
from forcedimension.dhd.bindings import _libdhd


class VersionTuple(NamedTuple):
    major: int
    minor: int
    release: int
    revision: int


class StatusTuple(NamedTuple):
    power: int
    connected: int
    started: int
    reset: int
    idle: int
    force: int
    brake: int
    torque: int
    wrist_detected: int
    error: int
    gravity: int
    timeguard: int
    wrist_init: int
    redundancy: int
    force_off_cause: int


_libdhd.dhdEnableSimulator.argtypes = [c_bool]
_libdhd.dhdEnableSimulator.restype = None
def enableSimulator(enable: bool) -> None:  # NOQA
    """
    Enable device simulator support.
    This enables network access on the loopback interface.

    :param : True to enable, False to disable
    """
    _libdhd.dhdEnableSimulator(enable)


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

    return (sn.value, _libdhd.dhdGetSerialNumber(byref(sn), ID))


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
def enableForce(enable: bool, ID: int = -1) -> int: # NOQA
    """
    Enable the force mode in the device controller.

    :param bool enable: True to enable force, False to disable it.
    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if val is not implicitly convertible to C bool
    :raises ValueError: if ID is not implicitly convertible to C int

    :rtype: int
    :returns: 0 on success, -1 otherwise.
    """

    return _libdhd.dhdEnableForce(enable, c_byte)


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
def getSDKVersion() -> VersionTuple:  # NOQA
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

    return VersionTuple(
        major.value,
        minor.value,
        release.value,
        revision.value
    )


_libdhd.dhdGetStatus.argtypes = [c_int * MAX_STATUS, c_byte]
_libdhd.dhdGetStatus.restype = c_int
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

    return (StatusTuple(*tuple(status_vec)),
            _libdhd.dhdGetStatus(status_vec, ID))


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

    :param int index: button index, 0 for the gripper button up to
    dhd.bindings.constants.MAX_BUTTONS

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
        dhd.bindings.constants.DeviceType.OMEGA33
        dhd.bindings.constants.DeviceType.OMEGA33_LEFT
        dhd.bindings.constants.DeviceType.OMEGA331
        dhd.bindings.constants.DeviceType.OMEGA331_LEFT
        dhd.bindings.constants.DeviceType.SIGMA331
        dhd.bindings.constants.DeviceType.SIGMA331_LEFT

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
        dhd.bindings.constants.DeviceType.OMEGA33
        dhd.bindings.constants.DeviceType.OMEGA33_LEFT
        dhd.bindings.constants.DeviceType.OMEGA331
        dhd.bindings.constants.DeviceType.OMEGA331_LEFT
        dhd.bindings.constants.DeviceType.SIGMA331
        dhd.bindings.constants.DeviceType.SIGMA331_LEFT
        dhd.bindings.constants.DeviceType.DELTA3
        dhd.bindings.constants.DeviceType.FALCON

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
        dhd.bindings.constants.DeviceType.OMEGA33
        dhd.bindings.constants.DeviceType.OMEGA33_LEFT
        dhd.bindings.constants.DeviceType.OMEGA331
        dhd.bindings.constants.DeviceType.OMEGA331_LEFT
        dhd.bindings.constants.DeviceType.SIGMA331
        dhd.bindings.constants.DeviceType.SIGMA331_LEFT

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
        dhd.bindings.constants.DeviceType.OMEGA33
        dhd.bindings.constants.DeviceType.OMEGA33_LEFT
        dhd.bindings.constants.DeviceType.OMEGA331
        dhd.bindings.constants.DeviceType.OMEGA331_LEFT
        dhd.bindings.constants.DeviceType.SIGMA331
        dhd.bindings.constants.DeviceType.SIGMA331_LEFT

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
        dhd.bindings.constants.DeviceType.OMEGA331
        dhd.bindings.constants.DeviceType.OMEGA331_LEFT
        dhd.bindings.constants.DeviceType.SIGMA331
        dhd.bindings.constants.DeviceType.SIGMA331_LEFT

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
        dhd.bindings.constants.DeviceType.OMEGA331
        dhd.bindings.constants.DeviceType.OMEGA331_LEFT
        dhd.bindings.constants.DeviceType.SIGMA331
        dhd.bindings.constants.DeviceType.SIGMA331_LEFT

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
    set to dhd.bindings.constants.Error.TIMEOUT

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
    c_byte
]
_libdhd.dhdGetPosition.restype = c_int
def getPosition(ID: int = -1) -> Tuple[List[float], int]:  # NOQA
    """
    Retrieve the position of the end-effector in Cartesian coordinates. Please
    refer to your device user manual for more information on your device
    coordinate system.

    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if ID is not implicitly convertible to a C char type

    :returns: A tuple in the form ([px, py, pz], err) where err is either 0
    or dhd.bindings.constants.TIMEGUARD on success, -1 otherwise and
    px, py, pz are the position of the end-effector about the X, Y, and Z axes,
    respectively in [m]

    :rtype: Tuple[List[float], int]
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
    c_byte
]
_libdhd.dhdGetForce.restype = c_int
def getForce(ID: int = -1) -> Tuple[List[float], int]:  # NOQA
    """
    Retrieve the force vector applied to the end-effector in Cartesian
    coordinates. Please refer to your device user manual for more information
    on your device coordinate system.

    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if ID is not implicitly convertible to a C char type

    :returns: A tuple in the form ([fx, fy, fz], err) where err is either 0,
    -1 otherwise.

    :rtype: Tuple[List[float], int]
    """

    fx = c_double()
    fy = c_double()
    fz = c_double()

    return ([fx.value, fy.value, fz.value],
            _libdhd.dhdGetPosition(byref(fx), byref(fy), byref(fz), ID))


_libdhd.dhdSetForce.argtypes = [c_double, c_double, c_double, c_byte]
_libdhd.dhdSetForce.restype = c_int
def setForce(f: Tuple[float, float, float], ID: int = -1) -> int:  # NOQA
    """
    Set the desired force vector in Cartesian coordinates to be applied
    to the end-effector of the device.

    :param Tuple[float, float, float] f: Translation force vector (fx, fy, fz)
    in [N].
    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if ID is not implicitly convertible to a C char type
    :raises ValueError: if any members of f are not implicitly convertible to a
    C double type.

    :returns: 0 or dhd.MOTOR_SATURATED on success -1 otherwise.

    :rtype: Tuple[List[float], int]
    """

    return _libdhd.dhdSetForce(f[0], f[1], f[2], ID)


_libdhd.dhdGetOrientationRad.argtypes = [c_byte]
_libdhd.dhdGetOrientationRad.restype = c_int
def getOrientationRad(ID: int = -1) -> Tuple[List[float], int]:  # NOQA
    """
    For devices with a wrist structure, retrieve individual angle of each
    joint, starting with the one located nearest to the wrist base plate.

    Note:
        dhd.bindings.constants.DeviceType.OMEGA33
        dhd.bindings.constants.DeviceType.OMEGA33_LEFT

    have angles that are instead computed with respect to their internal
    reference frame, which is rotated pi/4 radians around the Y axis.
    Please refer to your device user manual for more information on your
    device coordinate system.

    This feature only applies to the following devices:
        dhd.bindings.constants.DeviceType.OMEGA33
        dhd.bindings.constants.DeviceType.OMEGA33_LEFT
        dhd.bindings.constants.DeviceType.OMEGA331
        dhd.bindings.constants.DeviceType.OMEGA331_LEFT
        dhd.bindings.constants.DeviceType.SIGMA331
        dhd.bindings.constants.DeviceType.SIGMA331_LEFT


    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if ID is not implicitly convertible to a C char type

    :returns: Tuple of ([oa, ob, og], err) where err is 0 or
    dhd.bindings.constants.TIMEGUARD on
    success, -1 otherwise. oa, ob, and og refer to the device orientation
    around the first, second, and third wrist joints, respectively, in [rad]

    :rtype: Tuple[List[float], int]
    """

    oa = c_double()
    ob = c_double()
    og = c_double()

    return ([oa.value, ob.value, og.value],
            _libdhd.dhdGetOrientationRad(byref(oa), byref(ob), byref(og), ID))


_libdhd.dhdGetOrientationDeg.argtypes = [c_byte]
_libdhd.dhdGetOrientationDeg.restype = c_int
def getOrientationDeg(ID: int = -1) -> Tuple[List[float], int]:  # NOQA
    """
    For devices with a wrist structure, retrieve individual angle of each
    joint, starting with the one located nearest to the wrist base plate.

    Note:
        dhd.bindings.constants.DeviceType.OMEGA33
        dhd.bindings.constants.DeviceType.OMEGA33_LEFT

    have angles that are instead computed with respect to their internal
    reference frame, which is rotated 45 degrees around the Y axis.
    Please refer to your device user manual for more information on your
    device coordinate system.

    This feature only applies to the following devices:
        dhd.bindings.constants.DeviceType.OMEGA33
        dhd.bindings.constants.DeviceType.OMEGA33_LEFT
        dhd.bindings.constants.DeviceType.OMEGA331
        dhd.bindings.constants.DeviceType.OMEGA331_LEFT
        dhd.bindings.constants.DeviceType.SIGMA331
        dhd.bindings.constants.DeviceType.SIGMA331_LEFT

    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if ID is not implicitly convertible to a C char type

    :returns: Tuple of ([oa, ob, og], err) where err is 0 or
    dhd.bindings.constants.TIMEGUARD on success, -1 otherwise. [oa, ob, og]
    refer to the device orientation around the first, second, and third wrist
    joints, respectively, in [deg]

    :rtype: Tuple[List[float], int]
    """

    oa = c_double()
    ob = c_double()
    og = c_double()

    return ([oa.value, ob.value, og.value],
            _libdhd.dhdGetOrientationDeg(byref(oa), byref(ob), byref(og), ID))


_libdhd.dhdGetPositionAndOrientationRad.argtypes = [
    POINTER(c_double),
    POINTER(c_double),
    POINTER(c_double),
    POINTER(c_double),
    POINTER(c_double),
    POINTER(c_double),
    c_byte
]
_libdhd.dhdGetPositionAndOrientationRad.restype = c_int
def getPositionAndOrientationRad(ID: int = -1) -> Tuple[List[float], # NOQA
                                                        List[float],
                                                        int]:
    """
    For devices with a wrist structure, retrieve individual angle of each
    joint, starting with the one located nearest to the wrist base plate.

    Note:
        dhd.bindings.constants.DeviceType.OMEGA33
        dhd.bindings.constants.DeviceType.OMEGA33_LEFT

    have angles that are instead computed with respect to their internal
    reference frame, which is rotated pi/4 radians around the Y axis.
    Please refer to your device user manual for more information on your
    device coordinate system.

    This feature only applies to the following devices:
        dhd.bindings.constants.DeviceType.OMEGA33
        dhd.bindings.constants.DeviceType.OMEGA33_LEFT
        dhd.bindings.constants.DeviceType.OMEGA331
        dhd.bindings.constants.DeviceType.OMEGA331_LEFT
        dhd.bindings.constants.DeviceType.SIGMA331
        dhd.bindings.constants.DeviceType.SIGMA331_LEFT

    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if ID is not implicitly convertible to a C char type

    :returns: Tuple of ([px, py, pz], [oa, ob, og], err) where err is 0 or
    dhd.bindings.constants.TIMEGUARD on success, -1 otherwise, [oa, ob, og]
    refer to the device orientation around the first, second, and third wrist
    joints, respectively, in [rad], and [px, py, pz] is the position about the
    X, Y, and Z axes, respectively in [m].

    :rtype: Tuple[List[float], List[float], int]
    """

    px = c_double()
    py = c_double()
    pz = c_double()

    oa = c_double()
    ob = c_double()
    og = c_double()

    return ([px.value, py.value, pz.value], [oa.value, ob.value, og.value],
            _libdhd.dhdGetPositionAndOrientationRad(byref(px),
                                                    byref(py),
                                                    byref(pz),
                                                    byref(oa),
                                                    byref(ob),
                                                    byref(og),
                                                    ID))


_libdhd.dhdGetPositionAndOrientationDeg.argtypes = [
    POINTER(c_double),
    POINTER(c_double),
    POINTER(c_double),
    POINTER(c_double),
    POINTER(c_double),
    POINTER(c_double),
    c_byte
]
_libdhd.dhdGetPositionAndOrientationDeg.restype = c_int
def getPositionAndOrientationDeg(ID: int = -1) -> Tuple[List[float], # NOQA
                                                        List[float],
                                                        int]:
    """
    For devices with a wrist structure, retrieve individual angle of each
    joint, starting with the one located nearest to the wrist base plate.

    Note:
        dhd.bindings.constants.DeviceType.OMEGA33
        dhd.bindings.constants.DeviceType.OMEGA33_LEFT

    have angles that are instead computed with respect to their internal
    reference frame, which is rotated 45 degrees around the Y axis.
    Please refer to your device user manual for more information on your
    device coordinate system.

    This feature only applies to the following devices:
        dhd.bindings.constants.DeviceType.OMEGA33
        dhd.bindings.constants.DeviceType.OMEGA33_LEFT
        dhd.bindings.constants.DeviceType.OMEGA331
        dhd.bindings.constants.DeviceType.OMEGA331_LEFT
        dhd.bindings.constants.DeviceType.SIGMA331
        dhd.bindings.constants.DeviceType.SIGMA331_LEFT

    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if ID is not implicitly convertible to a C char type

    :returns: Tuple of ([px, py, pz], [oa, ob, og], err) where err is 0 or
    dhd.bindings.constants.TIMEGUARD on success, -1 otherwise, [oa, ob, og]
    refer to the device orientation around the first, second, and third wrist
    joints, respectively, in [deg], and [px, py, pz] is the position about the
    X, Y, and Z axes, respectively in [m].

    :rtype: Tuple[List[float], List[float], int]
    """

    px = c_double()
    py = c_double()
    pz = c_double()

    oa = c_double()
    ob = c_double()
    og = c_double()

    return ([px.value, py.value, pz.value], [oa.value, ob.value, og.value],
            _libdhd.dhdGetPositionAndOrientationDeg(byref(px),
                                                    byref(py),
                                                    byref(pz),
                                                    byref(oa),
                                                    byref(ob),
                                                    byref(og),
                                                    ID))


_libdhd.dhdGetPositionAndOrientationFrame.argtypes = [
    POINTER(c_double),
    POINTER(c_double),
    POINTER(c_double),
    (c_double * 3) * 3,
    c_byte
]
_libdhd.dhdGetPositionAndOrientationFrame.restype = c_int
def getPositionAndOrientationFrame(ID: int = -1) -> Tuple[List[float], # NOQA
                                                          List[List[float]],
                                                          int]:
    """
    Retrieve the position and orientation matrix of the end-effector in
    Cartesian coordinates. Please refer to your device user manual for more
    information on your device coordinate system.

    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if ID is not implicitly convertible to a C char type

    :returns: Tuple of ([px, py, pz], frame, err) where err is 0 or
    dhd.bindings.constants.TIMEGUARD on success, -1 otherwise.
    [px, py, pz] is the position about the X, Y, and Z axes, respectively in
    [m] andmatrix is a 3x3 rotation matrix that describes the orientation
    of your device.

    :rtype: Tuple[List[float], List[List[float]], int]
    """

    px = c_double()
    py = c_double()
    pz = c_double()

    matrix = ((c_double * 3) * 3)()

    return (
                [px.value, py.value, pz.value],
                [list(row) for row in matrix],
                _libdhd.dhdGetPositionAndOrientationFrame(
                    byref(px),
                    byref(py),
                    byref(pz),
                    matrix,
                    ID
                )
            )


_libdhd.dhdGetForceAndTorque.argtypes = [
    POINTER(c_double),
    POINTER(c_double),
    POINTER(c_double),
    POINTER(c_double),
    POINTER(c_double),
    POINTER(c_double),
    c_byte
]
_libdhd.dhdGetForceAndTorque.restype = c_int
def getForceAndTorque(ID: int = -1) -> Tuple[List[float], # NOQA
                                             List[float],
                                             int]:
    """
    Retrieve the force and torque vectors applied to the device end-effector.

    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if ID is not implicitly convertible to a C char type

    :returns: Tuple of ([fx, fy, fz], [tx, ty, tz], err) where err is 0, on
    success, -1 otherwise.

    :rtype: Tuple[List[float], List[float], int]
    """

    fx = c_double()
    fy = c_double()
    fz = c_double()

    tx = c_double()
    ty = c_double()
    tz = c_double()

    return ([fx.value, fy.value, fz.value], [tx.value, ty.value, tz.value],
            _libdhd.dhdGetForceAndTorque(byref(fx), byref(fy), byref(fz),
                                         byref(tx), byref(ty), byref(tz),
                                         ID))


_libdhd.dhdSetForceAndTorque.argtypes = [
    c_double,
    c_double,
    c_double,
    c_double,
    c_double,
    c_double,
    c_byte
]
_libdhd.dhdSetForceAndTorque.restype = c_int
def setForceAndTorque(f: Tuple[float, float, float], # NOQA
                      t: Tuple[float, float, float],
                      ID: int = -1) -> int:
    """
    Set the desired force and torque vectors to be applied to the device
    end-effector.

    :param Tuple[float, float, float] f: Translational force vector
    (fx, fy, fz) in [N]

    :param Tuple[float, float, float] t: Torque vector (tx, ty, tz) in [Nm]

    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if ID is not implicitly convertible to a C char type
    :raises ValueError: if a member of f is not implicitly to a C double type
    :raises ValueError: if a member of t is not implicitly to a C double type

    :returns: 0 or dhd.MOTOR_SATURATED on success, -1 otherwise
    :rtype: int
    """

    return _libdhd.dhdSetForceAndTorque(f[0], f[1], f[2], t[0], t[1], t[2], ID)


_libdhd.dhdGetOrientationFrame.argtypes = [
    (c_double * 3) * 3,
    c_byte
]
_libdhd.dhdGetOrientationFrame.restype = c_int
def getOrientationFrame(ID: int = -1) -> Tuple[List[List[float]], int]: # NOQA
    """
    Retrieve the rotation matrix of the wrist structure. The identity matrix
    is returned for devices that do not support orientations.

    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if ID is not implicitly convertible to a C char type

    :returns: Tuple of (frame, err) where err is 0 or
    dhd.bindings.constants.TIMEGUARD on success, -1 otherwise. and frame is a
    3x3 rotation matrix that describes the device's orientation. If the device
    doesn't support orientations, the identity matrix is returned.

    :rtype: Tuple[List[List[float]], int]
    """

    matrix = ((c_double * 3) * 3)()

    return (
                [list(row) for row in matrix],
                _libdhd.dhdGetPositionAndOrientationFrame(matrix, ID)
           )


_libdhd.dhdGetGripperAngleDeg.argtypes = [POINTER(c_double), c_byte]
_libdhd.dhdGetGripperAngleDeg.restype = c_int
def getGripperAngleDeg(ID: int = -1) -> Tuple[float, int]: # NOQA
    """
    Get the gripper opening angle in degrees.

    This feature only applies to the following devices:
        dhd.bindings.constants.DeviceType.OMEGA331
        dhd.bindings.constants.DeviceType.OMEGA331_LEFT
        dhd.bindings.constants.DeviceType.SIGMA331
        dhd.bindings.constants.DeviceType.SIGMA331_LEFT

    See also dhd.getGripperAngleRad()

    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if ID is not implicitly convertible to a C char type

    :returns: Tuple of (angle, err) where err is 0 or
    dhd.bindings.constants.TIMEGUARD on success, -1 otherwise.
    angle is in [deg]

    :rtype: Tuple[float, int]
    """

    angle = c_double()
    return (angle.value, _libdhd.dhdGetGripperAngleDeg(byref(angle), ID))


_libdhd.dhdGetGripperAngleRad.argtypes = [POINTER(c_double), c_byte]
_libdhd.dhdGetGripperAngleRad.restype = c_int
def getGripperAngleRad(ID: int = -1) -> Tuple[float, int]: # NOQA
    """
    Get the gripper opening angle in degrees.

    This feature only applies to the following devices:
        dhd.bindings.constants.DeviceType.OMEGA331
        dhd.bindings.constants.DeviceType.OMEGA331_LEFT
        dhd.bindings.constants.DeviceType.SIGMA331
        dhd.bindings.constants.DeviceType.SIGMA331_LEFT

   See also dhd.getGripperAngleRad()

    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if ID is not implicitly convertible to a C char type

    :returns: Tuple of (angle, err) where err is 0 or
    dhd.bindings.constants.TIMEGUARD on success, -1 otherwise.
    angle is in [rad]

    :rtype: Tuple[float, int]
    """

    angle = c_double()
    return (angle.value, _libdhd.dhdGetGripperAngleRad(byref(angle), ID))


_libdhd.dhdGetGripperGap.argtypes = [POINTER(c_double), c_byte]
_libdhd.dhdGetGripperGap.restype = c_int
def getGripperGap(ID: int = -1) -> Tuple[float, int]: # NOQA
    """
    Get the gripper opening distance in meters.

    This feature only applies to the following devices:
        dhd.bindings.constants.DeviceType.OMEGA331
        dhd.bindings.constants.DeviceType.OMEGA331_LEFT
        dhd.bindings.constants.DeviceType.SIGMA331
        dhd.bindings.constants.DeviceType.SIGMA331_LEFT

    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if ID is not implicitly convertible to a C char type

    :returns: Tuple of (gap, err) where err is 0 or
    dhd.bindings.constants.TIMEGUARD on success, -1 otherwise and gap is in [m]

    :rtype: Tuple[float, int]
    """

    gap = c_double()
    return (gap.value, _libdhd.dhdGetGripperGap(byref(gap), ID))


_libdhd.dhdGetGripperThumbPos.argtypes = [
    POINTER(c_double),
    POINTER(c_double),
    POINTER(c_double),
    c_byte
]
_libdhd.dhdGetGripperThumbPos.restype = c_int
def getGripperThumbPos(ID: int = -1) -> Tuple[List[float], int]: # NOQA
    """
    Read the position in Cartesian coordinates of thumb rest location of the
    force gripper structure if present.

    This feature only applies to the following devices:
        dhd.bindings.constants.DeviceType.OMEGA331
        dhd.bindings.constants.DeviceType.OMEGA331_LEFT
        dhd.bindings.constants.DeviceType.SIGMA331
        dhd.bindings.constants.DeviceType.SIGMA331_LEFT

    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if ID is not implicitly convertible to a C char type

    :returns: Tuple of ([px, py, pz], err) where err is 0 or
    dhd.bindings.constants.TIMEGUARD on success, -1 otherwise and [px, py, pz]
    are the position about the X, Y, and Z axes, respectively in [m]

    :rtype: Tuple[float, int]
    """

    px = c_double()
    py = c_double()
    pz = c_double()

    return ([px.value, py.value, pz.value],
            _libdhd.dhdGetThumbPos(byref(px), byref(py), byref(pz), ID))


_libdhd.dhdGetGripperFingerPos.argtypes = [
    POINTER(c_double),
    POINTER(c_double),
    POINTER(c_double),
    c_byte
]
_libdhd.dhdGetGripperFingerPos.restype = c_int
def getGripperFingerPos(ID: int = -1) -> Tuple[List[float], int]: # NOQA
    """
    Read the position in Cartesian coordinates of forefinger rest location of
    the force gripper structure if present.

    This feature only applies to the following devices:
        dhd.bindings.constants.DeviceType.OMEGA331
        dhd.bindings.constants.DeviceType.OMEGA331_LEFT
        dhd.bindings.constants.DeviceType.SIGMA331
        dhd.bindings.constants.DeviceType.SIGMA331_LEFT

    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if ID is not implicitly convertible to a C char type

    :returns: Tuple of ([px, py, pz], err) where err is 0 or
    dhd.bindings.constants.TIMEGUARD on success, -1 otherwise and
    [px, py, pz] are the position in the X, Y, and Z axes, respectively in [m]

    :rtype: Tuple[float, int]
    """

    px = c_double()
    py = c_double()
    pz = c_double()

    return ([px.value, py.value, pz.value],
            _libdhd.dhdGetGripperFingerPos(byref(px),
                                           byref(py),
                                           byref(pz),
                                           ID))


_libdhd.dhdGetComFreq.argtypes = [c_byte]
_libdhd.dhdGetComFreq.restype = c_double
def getComFreq(ID: int = -1) -> float: # NOQA
    """
    Read the position in Cartesian coordinates of forefinger rest location of
    the force gripper structure if present.

    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if ID is not implicitly convertible to a C char type

    :returns: the refresh rate in [kHz], 0.0 otherwise
    :rtype: float
    """

    return _libdhd.dhdGetComFreq(ID)


_libdhd.dhdSetAndGripperForce.argtypes = [
    c_double,
    c_double,
    c_double,
    c_double,
    c_byte
]
_libdhd.dhdSetForce.restype = c_int
def setForceAndGripperForce(f: Tuple[float, float, float],  # NOQA
                            fg: float,
                            ID: int = -1) -> int:
    """
    Set the desired force vector in Cartesian coordinates to be applied
    to the end-effector and force gripper of the device.

    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :param Tuple[float, float, float] f: Translational force vector
    (fx, fy, fz) in [N]

    :param float fg: Grasping force in [N]

    :raises ValueError: if ID is not implicitly convertible to a C char type
    :raises ValueError: if any members of f are not implicitly convertible to a
    C double type.

    :returns: 0 or dhd.MOTOR_SATURATED on success -1 otherwise.

    :rtype: int
    """

    return _libdhd.dhdSetForceAndGripperForce(f[0], f[1], f[2], fg, ID)


_libdhd.dhdSetForceAndTorqueAndGripperForce.argtypes = [
    c_double,
    c_double,
    c_double,
    c_double,
    c_double,
    c_double,
    c_double,
    c_byte
]
_libdhd.dhdSetForceAndTorqueAndGripperForce.restype = c_int
def setForceAndTorqueAndGripperForce(f: Tuple[float, float, float], # NOQA
                                     t: Tuple[float, float, float],
                                     fg: float,
                                     ID: int = -1) -> int:
    """
    Set the desired force and torque vectors to be applied to the device
    end-effector and gripper.

    :param Tuple[float, float, float] f: Translational force vector
    (fx, fy, fz) in [N]

    :param Tuple[float, float, float] t: Torque vector (tx, ty, tz) in [Nm]

    :param float fg: Grasping force in [N]

    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if ID is not implicitly convertible to a C char type
    :raises ValueError: if a member of f is not implicitly to a C double type
    :raises ValueError: if a member of t is not implicitly to a C double type

    :returns: 0 or dhd.MOTOR_SATURATED on success, -1 otherwise
    :rtype: int
    """

    return _libdhd.dhdSetForceAndTorqueAndGripperForce(f[0],
                                                       f[1],
                                                       f[2],
                                                       t[0],
                                                       t[1],
                                                       t[2],
                                                       fg,
                                                       ID)


_libdhd.dhdConfigLinearVelocity.argtypes = [c_int, c_int, c_byte]
_libdhd.dhdConfigLinearVelocity.restype = c_int
def configLinearVelocity(ms: int, mode: int, # NOQA
                                     ID: int = -1) -> int:
    """
    Configure the internal linear velocity computation estimator.
    This only applies to the device base.

    :param int ms: [default=dhd.VELOCITY_WINDOW] time interval used to
    compute velocity in [ms]

    :param int mode: [default=dhd.VELOCITY_WINDOWING] velocity estimator mode
    (see velocity estimator modes section for details)

    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if ID is not implicitly convertible to a C char type
    :raises ValueError: if ms is not implicitly convertible to a C int type
    :raises ValueError: if mode is not implicitly convertible to a C int type


    :returns: 0 on success, -1 otherwise
    :rtype: int
    """

    return _libdhd.dhdConfigLinearVelocity(ms, mode, ID)


_libdhd.dhdGetLinearVelocity.argtypes = [
    POINTER(c_double),
    POINTER(c_double),
    POINTER(c_double),
    c_byte
]
_libdhd.dhdGetLinearVelocity.restype = c_int
def getLinearVelocity(ID: int = -1) -> Tuple[List[float], int]:  # NOQA
    """
    Retrieve the estimated instanteous linear velocity in [m/s]. Velocity
    computation can be figured by calling dhd.bindings.configAngularVelocity().
    By default dhd.bindings.VELOCITY_WINDOW and dhd.bindings.VELOCITY_WINDOWING
    are used. See velocity estimator for details.

    Please note that the velocity estimator requires at least 2 position
    updates during the time interval defined in
    dhd.bindings.configLinearVelocity() in order to be able to compute the
    estimate. Otherwise, e.g. if there are no calls to
    dhd.bindings.getPosition(), dhd.bindings.getLinearVelocity(), or
    dhd.bindings.getLinearVelocity() will set an error
    (dhd.bindings.Error.TIMEOUT).


    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if ID is not implicitly convertible to a C char type

    :returns: A tuple in the form ([wx, wy, wz], err) where err is either 0
    or on success, -1 otherwise and wx, wy, wz are in [rad/s]

    :rtype: Tuple[List[float], int]
    """

    vx = c_double()
    vy = c_double()
    vz = c_double()

    return ([vx.value, vy.value, vz.value],
            _libdhd.dhdGetLinearVelocityRad(byref(vx),
                                            byref(vy),
                                            byref(vz),
                                            ID))


_libdhd.dhdConfigAngularVelocity.argtypes = [c_int, c_int, c_byte]
_libdhd.dhdConfigAngularVelocity.restype = c_int
def configAngularVelocity(ms: int, mode: int, # NOQA
                          ID: int = -1) -> int:
    """
    Configure the internal velocity computation estimator. This only applies to
    the device wrist.

    :param int ms: [default=dhd.bindings.VELOCITY_WINDOW] time interval used to
    compute velocity in [ms]

    :param int mode: [default=dhd.bindings.VELOCITY_WINDOWING] velocity
    estimator mode (see velocity estimator modes section for details)

    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if ID is not implicitly convertible to a C char type
    :raises ValueError: if ms is not implicitly convertible to a C int type
    :raises ValueError: if mode is not implicitly convertible to a C int type


    :returns: 0 on success, -1 otherwise
    :rtype: int
    """

    return _libdhd.dhdConfigAngularVelocity(ms, mode, ID)


_libdhd.dhdGetAngularVelocityRad.argtypes = [
    POINTER(c_double),
    POINTER(c_double),
    POINTER(c_double),
    c_byte
]
_libdhd.dhdGetAngularVelocityRad.restype = c_int
def getAngularVelocityRad(ID: int = -1) -> Tuple[List[float], int]:  # NOQA
    """
    Retrieve the estimated instanteous angular velocity in [rad/s]. Velocity
    computation can be figured by calling:
        dhd.bindings.standard.configAngularVelocity()

    By default:
        dhd.bindings.constants.VELOCITY_WINDOW
        dhd.bindings.constants.VELOCITY_WINDOWING
    are used. See velocity estimator for details.

    Please note that the velocity estimator requires at least 2 position
    updates during the time interval defined in:
        dhd.bindings.standard.configAngularVelocity()

    in order to be able to compute the estimate. Otherwise, e.g. if there are
    no calls to:
        dhd.bindings.standard.getPosition()
        dhd.bindings.standard.getAngularVelocityRad()
        dhd.bindings.standard.getAngularVelocityDeg()
    within the time interval window, this function will set an error with:
        dhd.bindings.constants.Error.TIMEOUT

    See also dhd.bindings.standard.getAngularVelocityDeg().
    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if ID is not implicitly convertible to a C char type

    :returns: A tuple in the form ([wx, wy, wz], err) where err is either 0
    or on success, -1 otherwise and wx, wy, wz are in [rad/s]

    :rtype: Tuple[List[float], int]
    """

    wx = c_double()
    wy = c_double()
    wz = c_double()

    return ([wx.value, wy.value, wz.value],
            _libdhd.dhdGetAngularVelocityRad(byref(wx),
                                             byref(wy),
                                             byref(wz),
                                             ID))


_libdhd.dhdGetAngularVelocityDeg.argtypes = [
    POINTER(c_double),
    POINTER(c_double),
    POINTER(c_double),
    c_byte
]
_libdhd.dhdGetAngularVelocityDeg.restype = c_int
def getAngularVelocityDeg(ID: int = -1) -> Tuple[List[float], int]:  # NOQA
    """
    Retrieve the estimated instanteous angular velocity in [deg/s]. Velocity
    computation can be figured by calling:
        dhd.bindings.standard.configAngularVelocity()

    By default:
        dhd.bindings.constants.VELOCITY_WINDOW
        dhd.bindings.constants.VELOCITY_WINDOWING
    are used. See velocity estimator for details.

    Please note that the velocity estimator requires at least 2 position
    updates during the time interval defined in:
        dhd.bindings.standard.configAngularVelocity()

    in order to be able to compute the estimate. Otherwise, e.g. if there are
    no calls to:
        dhd.bindings.standard.getPosition()
        dhd.bindings.standard.getAngularVelocityRad()
        dhd.bindings.standard.getAngularVelocityDeg()
    within the time interval window, this function will set an error with:
        dhd.bindings.constants.Error.TIMEOUT

    See also dhd.bindings.standard.getAngularVelocityRad().

    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if ID is not implicitly convertible to a C char type

    :returns: A tuple in the form ([wx, wy, wz], err) where err is either 0
    or on success, -1 otherwise and wx, wy, wz are in [deg/s]

    :rtype: Tuple[List[float], int]
    """
    wx = c_double()
    wy = c_double()
    wz = c_double()

    return ([wx.value, wy.value, wz.value],
            _libdhd.dhdGetAngularVelocityRad(byref(wx),
                                             byref(wy),
                                             byref(wz),
                                             ID))


_libdhd.dhdConfigGripperVelocity.argtypes = [c_int, c_int, c_byte]
_libdhd.dhdConfigGripperVelocity.restype = c_int
def configGripperVelocity(ms: int, mode: int, # NOQA
                          ID: int = -1) -> int:
    """
    Configure the internal velocity computation estimator. This only applies to
    the device gripper. See velocity estimator for details.

    See also:
        dhd.bindings.getGripperLinearVelocity()
        dhd.bindings.getGripperAngularVelocityRad()
        dhd.bindings.getGripperAngularVelocityDeg()

    :param int ms: [default=dhd.bindings.VELOCITY_WINDOW] time interval used to
    compute velocity in [ms]

    :param int mode: [default=dhd.bindings.VELOCITY_WINDOWING] velocity
    estimator mode (see velocity estimator modes section for details)

    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if ID is not implicitly convertible to a C char type
    :raises ValueError: if ms is not implicitly convertible to a C int type
    :raises ValueError: if mode is not implicitly convertible to a C int type


    :returns: 0 on success, -1 otherwise
    :rtype: int
    """

    return _libdhd.dhdConfigAngularVelocity(ms, mode, ID)


_libdhd.dhdGetGripperLinearVelocity.argtypes = [POINTER(c_double), c_byte]
_libdhd.dhdGetGripperLinearVelocity.restype = c_int
def getGripperLinearVelocity(ID: int = -1) -> Tuple[float, int]: # NOQA
    """
    Retrieve the estimated linear velocity of the gripper in in [m/s].
    Velocity computation can be figured by calling:
        dhd.bindings.standard.configGripperVelocity()

    By default:
        dhd.bindings.constants.VELOCITY_WINDOW
        dhd.bindings.constants.VELOCITY_WINDOWING
    are used. See velocity estimator for details.

    Please note that the velocity estimator requires at least 2 position
    updates during the time interval defined in:
        dhd.bindings.standard.configGripperVelocity()
    in order to be able to compute the estimate. Otherwise, e.g. if there are
    no calls to:
        dhd.bindings.standard.getPosition()
        dhd.bindings.standard.getGripperLinearVelocity()
    within the time interval window, this function will set an error with:
        dhd.bindings.constants.Error.TIMEOUT

    See also:
        dhd.bindings.configGripperVelocity()
        dhd.bindings.getGripperAngularVelocityRad()
        dhd.bindings.getGripperAngularVelocityDeg()

    :param int ms: [default=dhd.bindings.VELOCITY_WINDOW] time interval used to
    compute velocity in [ms]

    :param int mode: [default=dhd.bindings.VELOCITY_WINDOWING] velocity
    estimator mode (see velocity estimator modes section for details)

    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if ID is not implicitly convertible to a C char type

    :returns: tuple of (vg, err) where err is 0 on success, -1 otherwise and
    vg is the gripper estimated instanteous linear velocity in [m/s]

    :rtype: Tuple[float, int]
    """

    vg = c_double()
    return (vg.value, _libdhd.dhdGetGripperLinearVelocity(byref(vg), ID))


_libdhd.dhdGetGripperAngularVelocityRad.argtypes = [POINTER(c_double), c_byte]
_libdhd.dhdGetGripperAngularVelocityRad.restype = c_int
def getGripperAngularVelocityRad(ID: int = -1) -> Tuple[float, int]: # NOQA
    """
    Retrieve the estimated angular velocity of the gripper in in [rad/s].
    Velocity computation can be figured by calling:
        dhd.bindings.standard.configGripperVelocity()

    By default:
        dhd.bindings.constants.VELOCITY_WINDOW
        dhd.bindings.constants.VELOCITY_WINDOWING
    are used. See velocity estimator for details.

    Please note that the velocity estimator requires at least 2 position
    updates during the time interval defined in:
        dhd.bindings.standard.configGripperVelocity
    in order to be able to compute the estimate. Otherwise, e.g. if there are
    no calls to:
        dhd.bindings.standard.getPosition()
        dhd.bindings.standard.getGripperLinearVelocity()
    within the time interval window, this function will set an error with:
        dhd.bindings.constants.Error.TIMEOUT

    See also:
        dhd.bindings.configGripperVelocity()
        dhd.bindings.getGripperLinearVelocity()
        dhd.bindings.getGripperAngularVelocityDeg()

    :param int ms: [default=dhd.bindings.VELOCITY_WINDOW] time interval used to
    compute velocity in [ms]

    :param int mode: [default=dhd.bindings.VELOCITY_WINDOWING] velocity
    estimator mode (see velocity estimator modes section for details)

    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if ID is not implicitly convertible to a C char type

    :returns: tuple of (wg, err) where err is 0 on success, -1 otherwise and
    wg is the gripper estimated instanteous linear velocity in [rad/s]

    :rtype: Tuple[float, int]
    """

    wg = c_double()
    return (wg.value, _libdhd.dhdGetGripperAngularVelocityRad(byref(wg), ID))


_libdhd.dhdGetGripperAngularVelocityDeg.argtypes = [POINTER(c_double), c_byte]
_libdhd.dhdGetGripperAngularVelocityDeg.restype = c_int
def getGripperAngularVelocityDeg(ID: int = -1) -> Tuple[float, int]: # NOQA
    """
    Retrieve the estimated angular velocity of the gripper in in [rad/s].
    Velocity computation can be figured by calling:
        dhd.bindings.standard.configGripperVelocity()

    By default:
        dhd.bindings.constants.VELOCITY_WINDOW
        dhd.bindings.constants.VELOCITY_WINDOWING
    are used. See velocity estimator for details.

    Please note that the velocity estimator requires at least 2 position
    updates during the time interval defined in:
        dhd.bindings.standard.configGripperVelocity
    in order to be able to compute the estimate. Otherwise, e.g. if there are
    no calls to:
        dhd.bindings.standard.getPosition()
        dhd.bindings.standard.getGripperLinearVelocity()
    within the time interval window, this function will set an error with:
        dhd.bindings.constants.Error.TIMEOUT

    See also:
        dhd.bindings.configGripperVelocity()
        dhd.bindings.getGripperLinearVelocity()
        dhd.bindings.getGripperAngularVelocityRad()

    :param int ms: [default=dhd.bindings.VELOCITY_WINDOW] time interval used to
    compute velocity in [ms]

    :param int mode: [default=dhd.bindings.VELOCITY_WINDOWING] velocity
    estimator mode (see velocity estimator modes section for details)

    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if ID is not implicitly convertible to a C char type

    :returns: tuple of (wg, err) where err is 0 on success, -1 otherwise and
    wg is the gripper estimated instanteous linear velocity in [deg/s]

    :rtype: Tuple[float, int]
    """

    wg = c_double()
    return (wg.value, _libdhd.dhdGetGripperAngularelocityDeg(byref(wg), ID))


_libdhd.dhdEmulateButton.argtypes = [c_bool, c_byte]
_libdhd.dhdEmulateButton.restype = c_int
def emulateButton(enable: bool, ID: int = -1) -> int: # NOQA
    """
    Enable the button behavior emulation in devices that feature a gripper.

    This feature only applies to the following devices:
        dhd.bindings.constants.DeviceType.OMEGA331
        dhd.bindings.constants.DeviceType.OMEGA331_LEFT
        dhd.bindings.constants.DeviceType.SIGMA331
        dhd.bindings.constants.DeviceType.SIGMA331_LEFT

    For omega.7 devices with firmware versions 2.x, forces need to be enabled
    for the button emulation to report the emulated button status.

    :param enable: True to enable, False to disable
    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if ID is not implicitly convertible to a C char type

    :rtype: int
    :returns: 0 on success, -1 otherwise.
    """

    return _libdhd.dhdEmulateButton(enable, ID)


_libdhd.dhdGetBaseAngleXRad.argtypes = [POINTER(c_double), c_byte]
_libdhd.dhdGetBaseAngleXRad.argtypes = c_int
def getBaseAngleXRad(ID: int = -1) -> Tuple[float, int]: # NOQA
    """
    Get the device base plate angle around the X axis.

    See also:
        dhd.bindings.getBaseAngleXDeg()

    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if ID is not implicitly convertible to a C char type

    :rtype: Tuple[float, int]
    :returns: tuple of (angle, err) where err is 0 on success, -1 otherwise
    and angle is the device angle in [rad]
    """

    angle = c_double()
    return (angle.value, _libdhd.dhdGetBaseAngleXRad(byref(angle), ID))


_libdhd.dhdGetBaseAngleXDeg.argtypes = [POINTER(c_double), c_byte]
_libdhd.dhdGetBaseAngleXDeg.argtypes = c_int
def getBaseAngleXDeg(ID: int = -1) -> Tuple[float, int]: # NOQA
    """
    Get the device base plate angle around the X axis.

    See also:
        dhd.bindings.getBaseAngleXRad()

    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if ID is not implicitly convertible to a C char type

    :rtype: Tuple[float, int]
    :returns: tuple of (angle, err) where err is 0 on success, -1 otherwise
    and angle is the device angle in [deg]
    """

    angle = c_double()
    return (angle.value, _libdhd.dhdGetBaseAngleXDeg(byref(angle), ID))


_libdhd.dhdSetBaseAngleXRad.argtypes = [c_double, c_byte]
_libdhd.dhdSetBaseAngleXRad.restype = c_int
def setBaseAngleXRad(angle: float, ID: int = -1) -> int: # NOQA
    """
    Set the device base plate angle around the X axis. Please refer to your
    device user manual for more information on your device coordinate system.

    See also:
        dhd.bindings.setBaseAngleXDeg()

    :param float angle: device base plate angle around the X axis in [rad]
    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if ID is not implicitly convertible to a C char type

    :rtype: int
    :returns: 0 on success, -1 otherwise
    """

    return _libdhd.dhdSetBaseAngleXRad(angle, ID)


_libdhd.dhdSetBaseAngleXDeg.argtypes = [c_double, c_byte]
_libdhd.dhdSetBaseAngleXDeg.restype = c_int
def setBaseAngleXDeg(angle: float, ID: int = -1) -> int: # NOQA
    """
    Set the device base plate angle around the X axis. Please refer to your
    device user manual for more information on your device coordinate system.

    See also:
        dhd.bindings.setBaseAngleXRad()

    :param float angle: device base plate angle around the X axis in [deg]
    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if ID is not implicitly convertible to a C char type

    :rtype: int
    :returns: 0 on success, -1 otherwise
    """

    return _libdhd.dhdSetBaseAngleXDeg(angle, ID)


_libdhd.dhdGetBaseAngleXRad.argtypes = [POINTER(c_double), c_byte]
_libdhd.dhdGetBaseAngleXRad.argtypes = c_int
def getBaseAngleZRad(ID: int = -1) -> Tuple[float, int]: # NOQA
    """
    Get the device base plate angle around the Z axis.

    See also:
        dhd.bindings.getBaseAngleZDeg()

    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if ID is not implicitly convertible to a C char type

    :rtype: Tuple[float, int]
    :returns: tuple of (angle, err) where err is 0 on success, -1 otherwise
    and angle is the device angle in [rad]
    """

    angle = c_double()
    return (angle.value, _libdhd.dhdGetBaseAngleZRad(byref(angle), ID))


_libdhd.dhdGetBaseAngleZDeg.argtypes = [POINTER(c_double), c_byte]
_libdhd.dhdGetBaseAngleZDeg.argtypes = c_int
def getBaseAngleZDeg(ID: int = -1) -> Tuple[float, int]: # NOQA
    """
    Get the device base plate angle around the Z axis.

    See also:
        dhd.bindings.getBaseAngleZRad()

    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if ID is not implicitly convertible to a C char type

    :rtype: Tuple[float, int]
    :returns: tuple of (angle, err) where err is 0 on success, -1 otherwise
    and angle is the device angle in [deg]
    """

    angle = c_double()
    return (angle.value, _libdhd.dhdGetBaseAngleZDeg(byref(angle), ID))


_libdhd.dhdSetBaseAngleZRad.argtypes = [c_double, c_byte]
_libdhd.dhdSetBaseAngleZRad.restype = c_int
def setBaseAngleZRad(angle: float, ID: int = -1) -> int: # NOQA
    """
    Set the device base plate angle around the Z axis. Please refer to your
    device user manual for more information on your device coordinate system.

    See also:
        dhd.bindings.setBaseAngleZDeg()

    :param float angle: device base plate angle around the Z axis in [rad]
    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if ID is not implicitly convertible to a C char type

    :rtype: int
    :returns: 0 on success, -1 otherwise
    """

    return _libdhd.dhdSetBaseAngleZRad(angle, ID)


_libdhd.dhdSetBaseAngleZDeg.argtypes = [c_double, c_byte]
_libdhd.dhdSetBaseAngleZDeg.restype = c_int
def setBaseAngleZDeg(angle: float, ID: int = -1) -> int: # NOQA
    """
    Set the device base plate angle around the Z axis. Please refer to your
    device user manual for more information on your device coordinate system.

    See also:
        dhd.bindings.setBaseAngleZRad()

    :param float angle: device base plate angle around the Z axis in [deg]
    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if angle is not implicitly convertible to a C double
    type
    :raises ValueError: if ID is not implicitly convertible to a C char type

    :rtype: int
    :returns: 0 on success, -1 otherwise
    """

    return _libdhd.dhdSetBaseAngleZDeg(angle, ID)


_libdhd.dhdSetVibration.argtypes = [c_double, c_double, c_int, c_byte]
_libdhd.dhdSetVibration.restype = c_int
def setVibration(f: float, A: float, # NOQA
                 device_type: DeviceType, ID: int = -1) -> int:
    """
    Apply a vibration to the end-effector. The vibration is added to the force
    requested by dhd.bindings.setForce()

    :param float f: vibration frequency in [Hz]
    :param float A: vibration amplitude in [m]
    :param DeviceType device_type: the type of device
    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if f is not implicitly convertible to a C double type
    :raises ValueError: if A is not implicitly convertible to a C double type
    :raises ValueError: if device_type is not implicitly convertible to a C int
    type
    :raises ValueError: if ID is not implicitly convertible to a C char type

    :rtype: int
    :returns: 0 on success, -1 otherwise
    """

    return _libdhd.dhdSetVibration(f, A, device_type, ID)


_libdhd.dhdSetMaxForce.argtypes = [c_double, c_byte]
_libdhd.dhdSetMaxForce.restype = c_int
def setMaxForce(limit: float, ID: int = -1) -> int: # NOQA
    """
    Define a limit (in N) to the magnitude of the force that can be applied to
    the haptic device. The limit applies to all dhd.bindings.setForce() and
    related calls, and ensures the force applied to the device end-effector
    remains below the requested value. If a negative limit is set, there is no
    max force and the full range of force can be applied.

    Note that the force limit enforced only applies to forces set
    programatically by dhd.bindings.setForce(). Setting DAC values directly
    will bypass this limit.

    See also:
        dhd.bindings.getMaxForce()
        dhd.bindings.setMaxTorque()
        dhd.bindings.setMaxGripperForce()
        dhd.bindings.expert.setMaxPower()
        dhd.bindings.expert.setMaxUsablePower()

    :param float limit: max magnitude of force that can be applied in [N]
    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if limit is not implicitly convertible to a C double
    type
    :raises ValueError: if ID is not implicitly convertible to a C char type

    :rtype: int
    :returns: 0 on success, -1 otherwise
    """

    return _libdhd.dhdSetMaxForce(limit, ID)


_libdhd.dhdSetMaxTorque.argtypes = [c_double, c_byte]
_libdhd.dhdSetMaxTorque.restype = c_int
def setMaxTorque(limit: float, ID: int = -1) -> int: # NOQA
    """
    Define a limit (in Nm) to the magnitude of the torque that can be applied
    to the haptic device. The limit applies to all dhd.bindings.setForce() and
    related calls, and ensures the force applied to the device end-effector
    remains below the requested value. If a negative limit is set, there is no
    max force and the full range of force can be applied.

    Note that the force limit enforced only applies to torques set
    programatically by dhd.bindings.setTorque(). Setting DAC values directly
    will bypass this limit.

    See also:
        dhd.bindings.getMaxTorque()
        dhd.bindings.setMaxForce()
        dhd.bindings.setMaxGripperForce()
        dhd.bindings.expert.setMaxPower()
        dhd.bindings.expert.setMaxUsablePower()

    :param float limit: max magnitude of torque that can be applied in [Nm]
    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if limit is not implicitly convertible to a C double
    type
    :raises ValueError: if ID is not implicitly convertible to a C char type

    :rtype: int
    :returns: 0 on success, -1 otherwise
    """

    return _libdhd.dhdSetMaxTorque(limit, ID)


_libdhd.dhdSetMaxGripperForce.argtypes = [c_double, c_byte]
_libdhd.dhdSetMaxGripperForce.restype = c_int
def setMaxGripperForce(limit: float, ID: int = -1) -> int: # NOQA
    """
    Define a limit (in N) to the magnitude of the force that can be applied to
    the haptic device gripper. The limit applies to all
    dhd.bindings.setForceAndTorqueAndGripperForce() and related calls, and
    ensures the force applied to the device gripper remains below the
    requested value. If a negative limit is set, there is no max force and the
    full range of force can be applied.

    Note that the force limit enforced only applies to forces set
    programatically by dhd.bindings.setForceAndTorqueAndGripperForce. Setting
    DAC values directly will bypass this limit.

    See also:
        dhd.bindings.getMaxGripperForce()
        dhd.bindings.getMaxForce()
        dhd.bindings.setMaxTorque()
        dhd.bindings.expert.setMaxPower()
        dhd.bindings.expert.setMaxUsablePower()

    :param float limit: max magnitude of force that can be applied in [N]
    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if limit is not implicitly convertible to a C double
    type
    :raises ValueError: if ID is not implicitly convertible to a C char type

    :rtype: int
    :returns: 0 on success, -1 otherwise
    """

    return _libdhd.dhdSetMaxGripperForce(limit, ID)


_libdhd.dhdGetMaxForce.argtypes = [c_byte]
_libdhd.dhdGetMaxForce.restype = c_double
def getMaxForce(ID: int = -1) -> float: # NOQA
    """
    Retrieve the current limit (in N) to the force magnitude that can be
    appliedby the haptic device. If the return value if negative, the limit is
    disabled and the full range of force available can be applied.

    See also:
        dhd.bindings.setMaxForce()
        dhd.bindings.getMaxTorque()
        dhd.bindings.getMaxGripperForce()
        dhd.bindings.expert.getMaxPower()
        dhd.bindings.expert.getMaxUsablePower()

    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if ID is not implicitly convertible to a C char type

    :rtype: float
    :returns: The current force limit (in N) if set, -1.0 if no limit is
    enforced
    """

    return _libdhd.dhdGetMaxForce(ID)


_libdhd.dhdGetMaxForce.argtypes = [c_byte]
_libdhd.dhdGetMaxForce.restype = c_double
def getMaxTorque(ID: int = -1) -> float: # NOQA
    """
    Retrieve the current limit (in Nm) to the force magnitude that can be
    appliedby the haptic device. If the return value if negative, the limit is
    disabled and the full range of force available can be applied.

    See also:
        dhd.bindings.setMaxTorque()
        dhd.bindings.getMaxForce()
        dhd.bindings.getMaxGripperForce()
        dhd.bindings.expert.getMaxPower()
        dhd.bindings.expert.getMaxUsablePower()

    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if ID is not implicitly convertible to a C char type

    :rtype: float
    :returns: The current torque limit (in Nm) if set, -1.0 if no limit is
    enforced
    """

    return _libdhd.dhdGetMaxTorque(ID)


_libdhd.dhdGetMaxGripperForce.argtypes = [c_byte]
_libdhd.dhdGetMaxGripperForce.restype = c_double
def getMaxGripperForce(ID: int = -1) -> float: # NOQA
    """
    Retrieve the current limit (in N) to the force magnitude that can be
    appliedby the haptic device gripper. If the return value if negative, the
    limit is disabled and the full range of force available can be applied.

    See also:
        dhd.bindings.setMaxGripperForce()
        dhd.bindings.getMaxForce()
        dhd.bindings.getMaxTorque()
        dhd.bindings.expert.getMaxPower()
        dhd.bindings.expert.getMaxUsablePower()

    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if ID is not implicitly convertible to a C char type

    :rtype: float
    :returns: The current force limit (in N) if set, -1.0 if no limit is
    enforced
    """

    return _libdhd.dhdGetMaxGripperForce(ID)
