from ctypes import (
    c_bool, c_byte, c_char_p, c_double, c_int, c_uint, c_ushort
)
from typing import Optional, Tuple, Union

import forcedimension.runtime as _runtime
from forcedimension.containers import VersionTuple
from forcedimension.typing import c_double_ptr, c_int_ptr, c_ushort_ptr
from forcedimension.dhd.adaptors import (
    DHDError,
    DHDErrorCom,
    DHDErrorConfiguration,
    DHDErrorDeviceInUse,
    DHDErrorDeviceNotReady,
    DHDErrorDHCBusy,
    DHDErrorExpertModeDisabled,
    DHDErrorFeatureNotAvailable,
    DHDErrorFeatureNotEnabled,
    DHDErrorGeometry,
    DHDErrorNoDeviceFound,
    DHDErrorNoDriverFound,
    DHDErrorRedundantFail,
    DHDErrorTimeout, DHDFeatureError,
    DHDIOError,
    Status,
    errno_to_exception
)
from forcedimension.dhd.constants import (
    MAX_BUTTONS,
    MAX_DOF,
    MAX_STATUS,
    MOTOR_SATURATED,
    TIMEGUARD,
    DEFAULT_VELOCITY_WINDOW,
    VelocityEstimatorMode,
    ComMode,
    DELTA_IDX,
    DeviceType,
    ErrorNum,
    StatusIndex,
    ThreadPriority,
    WRIST_IDX
)
from forcedimension.typing import (
    FloatVectorLike, SupportsPtr, SupportsPtrs3
)

from . import expert, os_independent

# DRD contains DHD and you should use the DRD versions for compatibility with
# DRD
_libdhd = _runtime.load("libdrd")

if _libdhd is None:
    raise ImportError("There were problems loading libdhd.")

_libdhd.dhdEnableSimulator.argtypes = [c_bool]
_libdhd.dhdEnableSimulator.restype = None


def enableSimulator(enable: bool) -> None:
    """
    Enable device simulator support. This enables network access on the
    loopback interface.

    :param enable:
        ``True`` to enable, ``False`` to disable
    """
    _libdhd.dhdEnableSimulator(enable)


_libdhd.dhdGetDeviceCount.argtypes = []
_libdhd.dhdGetDeviceCount.restype = c_int


def getDeviceCount() -> int:
    """
    Return the number of compatible Force Dimension devices connected to
    the system. This encompasses all devices connected locally,
    including devices already locked by other applications.

    Devices are given a unique identifier, as explained in the
    multiple devices section.

    :returns:
        The number of connected devices on success, -1 otherwise.
    """

    return _libdhd.dhdGetDeviceCount()


_libdhd.dhdGetAvailableCount.argtypes = []
_libdhd.dhdGetAvailableCount.restype = c_int


def getAvailableCount() -> int:
    """
    Return the number of available Force Dimension devices connected to the
    system. This encompasses all devices connected locally, but excludes
    devices already locked by other applications.

    Devices are given a unique identifier, as explained in the multiple devices
    section.

    :returns:
        The number of devices available on success, -1 otherwise.
    """
    return _libdhd.dhdGetAvailableCount()


_libdhd.dhdSetDevice.argtypes = [c_byte]
_libdhd.dhdSetDevice.restype = c_int


def setDevice(ID: int = -1) -> int:
    """
    Select the default device that will receive the SDK commands.
    The SDK supports multiple devices.

    This routine allows the programmer to decide which device the SDK
    dhd_single_device_call single-device calls will address. Any subsequent
    SDK call that does not specifically mention the device ID in its
    parameter list will be sent to that device.

    :param int ID:
        Device ID (see multiple devices section for details).

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to C char.

    :returns:
        0 on success, -1 otherwise.
    """

    return _libdhd.setDevice(ID)


_libdhd.dhdGetDeviceID.argtypes = []
_libdhd.dhdGetDeviceID.restype = c_int


def getDeviceID() -> int:
    """
    Get the ID of the current default device.

    :returns:
        The ID of the current default device
    """

    return _libdhd.dhdGetDeviceID()


_libdhd.dhdGetSerialNumber.argtypes = [c_ushort_ptr, c_byte]
_libdhd.dhdGetSerialNumber.restype = c_int


def getSerialNumber(ID: int = -1) -> Tuple[int, int]:
    """
    Return the device serial number.

    :param int ID:
        Device ID (see multiple devices section for details).

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to C char.

    :returns:
        A tuple in the form ``(serial number, err)``. ``serial`` is the device
        serial number. ``err`` is 0 on success, -1 otherwise.
    """
    sn = c_ushort()

    return (sn.value, _libdhd.dhdGetSerialNumber(sn, ID))


_libdhd.dhdOpen.argtypes = []
_libdhd.dhdOpen.restype = c_int


def open() -> int:
    """
    Open a connection to the first available device connected to the
    system. The order in which devices are opened persists until devices
    are added or removed.

    If this call is successful, the default device ID is set to the newly
    opened device. See the multiple device section for more information on
    using multiple devices on the same computer.

    See Also
    --------
    :func:`forcedimension.dhd.openID()`


    :returns:
        The device ID on success, -1 otherwise.
    """
    return _libdhd.dhdOpen()


_libdhd.dhdOpenType.argtypes = [c_int]
_libdhd.dhdOpenType.restype = c_int


def openType(device_type: DeviceType) -> int:
    """
    Open a connection to the first device of a given type connected to the
    system. The order in which devices are opened persists until devices
    are added or removed.

    If this call is successful, the default device ID is set to the newly
    opened device. See the multiple device section for more information on
    using multiple devices on the same computer.

    See Also
    --------
    :func:`forcedimension.dhd.openID()`
    :class:`forcedimension.dhd.adaptors.DeviceType`


    :param int device_type:
        Requested DeviceType to open.

    :returns:
        The device ID on success, -1 otherwise.
    """

    return _libdhd.dhdOpenType(device_type)


_libdhd.dhdOpenSerial.argtypes = [c_int]
_libdhd.dhdOpenSerial.restype = c_int


def openSerial(serial: int) -> int:
    """
    Open a connection to the device with a given serial number (available on
    recent models only).

    If this call is successful the default device ID is set to the newly opened
    device. See the multiple device section for more information on using
    multiple devices on the same computer.

    See Also
    --------
    :func:`forcedimension.dhd.openID()`


    :param int serial:
        Requested system serial number.

    :returns:
        The device ID on success, -1 otherwise.
    """

    return _libdhd.dhdOpenSerial(serial)


_libdhd.dhdOpenID.argtypes = [c_int]
_libdhd.dhdOpenID.restype = c_int


def openID(index: int) -> int:
    """
    Open a connection to one particular device connected to the system. The
    order in which devices are opened persists until devices are added or
    removed. If the device at the specified index is already opened, its device
    ID is returned.

    If this call is successful, the default device ID is set to the newly
    opened device. See the multiple device section for more information on
    using multiple devices on the same computer.

    See Also
    --------
    :func:`forcedimension.dhd.open()`

    :param int index:
        The device enumeration index, as assigned by the
        underlying operating system (must be between 0 and the number of
        devices connected to the system).

    :raises ArgumentError:
        If ``index`` is not convertible to C int.

    :returns:
        The device ID on success, -1 otherwise.
    """

    return _libdhd.dhdOpenID(index)


_libdhd.dhdClose.argtypes = [c_byte]
_libdhd.dhdClose.restype = c_int


def close(ID: int = -1) -> int:
    """
    Close the connection to a particular device.

    :param int ID:
        Device ID (see multiple devices section for details).

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to C char.

    :returns:
        0 on success, -1 otherwise.
    """
    return _libdhd.dhdClose(ID)


_libdhd.dhdStop.argtypes = [c_byte]
_libdhd.dhdStop.restype = c_int


def stop(ID: int = -1) -> int:
    """
    Stop the device. This routine disables the force on the haptic device and
    puts it into BRAKE mode.

    :param int ID:
        Device ID (see multiple devices section for details).

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to C char.

    :returns:
        0 on success, -1 otherwise.
    """
    return _libdhd.dhdStop(ID)


_libdhd.dhdGetComMode.argtypes = [c_byte]
_libdhd.dhdGetComMode.restype = c_int


def getComMode(ID: int = -1) -> ComMode:
    """
    Retrive the COM operation mode on compatible devices.

    :param int ID:
        Device ID (see multiple devices section for details).

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to C char.

    :returns:
        The current COM operation mode on success, -1 otherwise.
    """

    return ComMode(_libdhd.dhdGetComMode(ID))


_libdhd.dhdEnableForce.argtypes = [c_bool, c_byte]
_libdhd.dhdEnableForce.restype = c_int


def enableForce(enable: bool, ID: int = -1) -> int:
    """
    Enable the force mode in the device controller.

    :param bool enable:
        ``True`` to enable force, ``False`` to disable it.

    :param int ID:
        Device ID (see multiple devices section for details).

    :raises ArgumentError:
        If ``val`` is not implicitly convertible to C bool.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to C char.

    :returns:
        0 on success, -1 otherwise.
    """

    return _libdhd.dhdEnableForce(enable, ID)


_libdhd.dhdEnableGripperForce.argtypes = [c_bool, c_byte]
_libdhd.dhdEnableGripperForce.restype = c_int


def enableGripperForce(enable: bool, ID: int = -1) -> int:
    """
    Enable the force mode in the device controller.

    :param bool enable:
        ``True`` to enable force, ``False`` to disable it.

    :param int ID:
        Device ID (see multiple devices section for details).

    :raises ArgumentError:
        If ``val`` is not implicitly convertible to C bool.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to C char.

    :returns:
        0 on success, -1 otherwise.
    """

    return _libdhd.dhdEnableGripperForce(enable, ID)


_libdhd.dhdGetSystemType.argtypes = [c_byte]
_libdhd.dhdGetSystemType.restype = c_int


def getSystemType(ID: int = -1) -> DeviceType:
    """
    Return the haptic device type. As this SDK can be used to control all of
    Force Dimension haptic products, this can help programmers ensure that
    their application is running on the appropriate target haptic device.

    :param int ID:
        Device ID (see multiple devices section for details).

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to C char.

    :returns:
        The device type on success, -1 otherwise.
    """

    return DeviceType(_libdhd.dhdGetSystemType(ID))


_libdhd.dhdGetSystemName.argtypes = [c_byte]
_libdhd.dhdGetSystemName.restype = c_char_p


def getSystemName(ID: int = -1) -> Union[str, None]:
    """
    Return the haptic device type. As this SDK can be used to control all of
    Force Dimension haptic products, this can help programmers ensure that
    their application is running on the appropriate target haptic device.

    :param int ID:
        Device ID (see multiple devices section for details).

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to C char.

    :returns:
        The device type string on success, None otherwise.
    """

    ret = _libdhd.dhdGetSystemName(ID)
    if (ret is not None):
        return ret.decode("utf-8")  # python using decode bytes as unicode str
    else:
        return None


_libdhd.dhdGetVersion.argtypes = [c_double_ptr, c_byte]
_libdhd.dhdGetVersion.restype = c_int


def getVersion(ID: int = -1) -> Tuple[float, int]:
    """
    Return the device controller version. As this SDK can be used to control
    all of Force Dimension haptic products, this can help programmers ensure
    that their application is running on the appropriate version of the haptic
    controller.

    :param int ID:
        Device ID (see multiple devices section for details).

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to C char.

    :returns:
        A tuple in the form ``(version, err)``. ``version`` is the device
        controller version. ``err`` is 0 on success, -1 otherwise.
    """

    ver = c_double()

    return (ver.value, _libdhd.dhdGetVersion(ver, ID))


_libdhd.dhdGetSDKVersion.argtypes = [
    c_int_ptr,
    c_int_ptr,
    c_int_ptr,
    c_int_ptr
]
_libdhd.dhdGetSDKVersion.restype = None


def getSDKVersion() -> VersionTuple:
    """
    Get the version of the ForceDimensionSDK in the form (major, minor,
    release, revision). Versions of the ForceDimensionSDK
    are reported as major.minor.release-revision by ForceDimension.

    See Also
    --------
    :class:`forcedimension.dhd.adaptors.VersionTuple`


    :returns:
        A ``VersionTuple`` that represents the version.
    """
    major = c_int()
    minor = c_int()
    release = c_int()
    revision = c_int()

    _libdhd.dhdGetSDKVersion(major, minor, release, revision)

    return VersionTuple(
        major.value,
        minor.value,
        release.value,
        revision.value
    )


_libdhd.dhdGetStatus.argtypes = [c_int_ptr, c_byte]
_libdhd.dhdGetStatus.restype = c_int


def getStatus(out: Status, ID: int = -1) -> int:
    """
    Get a tuple representing the status of the haptic device.
    The status is described in the status section.

    :param int ID:
        Device ID (see multiple devices section for details).

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to C char.

    :returns:
        A tuple in the form ``(status, err)``. ``status`` is a ``StatusTuple``
        containing status information. ``err`` is 0 on success, -1
        otherwise.
    """

    return _libdhd.dhdGetStatus(out.ptr, ID)


_libdhd.dhdGetDeviceAngleRad.argtypes = [c_double_ptr, c_byte]
_libdhd.dhdGetDeviceAngleRad.restype = c_int


def getDeviceAngleRad(ID: int = -1) -> Tuple[float, int]:
    """
    Get the device base plate angle around the Y axis in radians.

    :param int ID:
        Device ID (see multiple devices section for details).

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to C char.

    :returns:
        A tuple in the form ``(angle_rad, err)``. ``angle_rad`` is the device
        baseplate angle (in [rad]) around the Y axis. ``err`` is 0 on success,
        and -1 otherwise.
    """

    angle_rad = c_double()
    return (
        angle_rad.value,
        _libdhd.dhdGetDeviceAngleRad(angle_rad, ID)
    )


_libdhd.dhdGetDeviceAngleDeg.argtypes = [c_double_ptr, c_byte]
_libdhd.dhdGetDeviceAngleDeg.restype = c_int


def getDeviceAngleDeg(ID: int = -1) -> Tuple[float, int]:
    """
    Get the device base plate angle around the Y axis in degrees.

    :param int ID:
        Device ID (see multiple devices section for details).

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to C char.

    :returns:
        A tuple in the form ``(angle_deg, err)``. ``angle_deg`` is the device
        baseplate angle (in [deg]) around the Y axis. ``err`` is 0 on success,
        and -1 otherwise.
    """

    angle_deg = c_double()
    return (
        angle_deg.value,
        _libdhd.dhdGetDeviceAngleDeg(angle_deg, ID)
    )


_libdhd.dhdGetEffectorMass.argtypes = [c_double_ptr, c_byte]
_libdhd.dhdGetEffectorMass.restype = c_int


def getEffectorMass(ID: int = -1) -> Tuple[float, int]:
    """
    Get the mass (in [kg]) of the end-effector currently defined for a device.
    The gripper mass is used in the gravity compensation feature.

    :param int ID:
        Device ID (see multiple devices section for details).

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to C char.

    :returns:
        A tuple in the form ``(mass, err)``. ``mass`` is the currently defined
        mass for the end-effector. ``err`` is 0 on success, -1 otherwise.
    """

    mass = c_double()
    return (mass.value, _libdhd.dhdGetEffectorMass(mass, ID))


_libdhd.dhdGetButton.argtypes = [c_int, c_byte]
_libdhd.dhdGetButton.restype = c_int


def getButton(index: int, ID: int = -1) -> Tuple[bool, int]:
    """
    Return the status of the button located on the end-effector

    :param int index:
        button index, 0 for the gripper button up to
        :data:`forcedimension.dhd.constants.MAX_BUTTONS`

    :param int ID:
        Device ID (see multiple devices section for details).

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to C char.

    :returns:
        A tuple in the form ``(state, err)`` where state is True if the button
        is pressed, and False otherwise. ``err`` is 0 on success, -1 otherwise.
    """

    state = _libdhd.dhdGetButton(index, ID)

    if (state == -1):
        return False, -1

    return bool(state), 0



_libdhd.dhdGetButtonMask.argtypes = [c_byte]
_libdhd.dhdGetButtonMask.restype = c_uint


def getButtonMask(ID: int = -1) -> int:
    """
    Return the 32-bit binary mask of the device buttons.

    :param int ID:
        Device ID (see multiple devices section for details).

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to C char.

    :returns:
        32-bit long bitmask. Each bit is set to 1 if the button is
        pressed, 0 otherwise.
    """

    return _libdhd.dhdGetButtonMask(ID)


_libdhd.dhdSetOutput.argtypes = [c_uint, c_byte]
_libdhd.dhdSetOutput.restype = c_int


def setOutput(output: int, ID: int = -1) -> int:
    """
    Set the user programmable output bits on devices that support it.

    This feature only applies to the following devices:
    :data:`forcedimension.dhd.adaptors.DeviceType.DELTA3`
    :data:`forcedimension.dhd.adaptors.DeviceType.SIGMA331`
    :data:`forcedimension.dhd.adaptors.DeviceType.SIGMA331_LEFT`

    :param int output:
        A bitwise mask that toggles the programmable output bits.

    :param int ID:
        Device ID (see multiple devices section for details).

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to C char.

    :raises ArgumentError:
        If ``output`` is not implicitly convertible to C uint.

    :returns:
        0 on success, -1 otherwise.
    """

    return _libdhd.dhdSetOutput(output, ID)


_libdhd.dhdIsLeftHanded.argtypes = [c_byte]
_libdhd.dhdIsLeftHanded.restype = c_bool


def isLeftHanded(ID: int = -1) -> bool:
    """
    ``True`` if the device is configured for left-handed use,
    ``False`` otherwise.

    This feature only applies to the following devices
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA3`
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA33_LEFT`
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA331`
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA331_LEFT`
        :data:`forcedimension.dhd.constants.DeviceType.SIGMA331`
        :data:`forcedimension.dhd.constants.DeviceType.SIGMA331_LEFT`

    :param int ID:
         Device ID (see multiple devices section for details).

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to C char.

    :returns:
        ``True`` if the device is configured for left-handed use, and ``False``
        otherwise.
    """

    return _libdhd.dhdIsLeftHanded(ID)


_libdhd.dhdHasBase.argtypes = [c_byte]
_libdhd.dhdHasBase.restype = c_bool


def hasBase(ID: int = -1) -> bool:
    """
     ``True`` if the device has a base, ``False`` otherwise.

    This feature only applies to the following devices
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA33`
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA33_LEFT`
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA331`
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA331_LEFT`
        :data:`forcedimension.dhd.constants.DeviceType.SIGMA331`
        :data:`forcedimension.dhd.constants.DeviceType.SIGMA331_LEFT`
        :data:`forcedimension.dhd.constants.DeviceType.DELTA3`
        :data:`forcedimension.dhd.constants.DeviceType.FALCON`

    :param int ID:
         Device ID (see multiple devices section for details).

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to C char.

    :returns:
        ``True`` if the device has a base, and ``False`` otherwise.
    """

    return _libdhd.dhdHasBase(ID)


_libdhd.dhdHasWrist.argtypes = [c_byte]
_libdhd.dhdHasWrist.restype = c_bool


def hasWrist(ID: int = -1) -> bool:
    """
     ``True`` if the device has a wrist, ``False`` otherwise.

    This feature only applies to the following devices
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA33`
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA33_LEFT`
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA331`
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA331_LEFT`
        :data:`forcedimension.dhd.constants.DeviceType.SIGMA331`
        :data:`forcedimension.dhd.constants.DeviceType.SIGMA331_LEFT`

    :param int ID:
         Device ID (see multiple devices section for details).

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to C char.

    :returns:
        ``True`` if the device has a wrist, and ``False`` otherwise.
    """

    return _libdhd.dhdHasWrist(ID)


_libdhd.dhdHasActiveWrist.argtypes = [c_byte]
_libdhd.dhdHasActiveWrist.restype = c_bool


def hasActiveWrist(ID: int = -1) -> bool:
    """
     ``True`` if the device has an active wrist, ``False`` otherwise.

    This feature only applies to the following devices
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA33`
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA33_LEFT`
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA331`
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA331_LEFT`
        :data:`forcedimension.dhd.constants.DeviceType.SIGMA331`
        :data:`forcedimension.dhd.constants.DeviceType.SIGMA331_LEFT`

    :param int ID:
         Device ID (see multiple devices section for details).

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to C char.

    :returns:
        ``True`` if the device has an active wrist, and ``False`` otherwise.
    """

    return _libdhd.dhdHasActiveWrist(ID)


_libdhd.dhdHasGripper.argtypes = [c_byte]
_libdhd.dhdHasGripper.restype = c_bool


def hasGripper(ID: int = -1) -> bool:
    """
    Checks if the device has a gripper.

    This feature only applies to the following devices:
    :data:`forcedimension.dhd.constants.DeviceType.OMEGA331`
    :data:`forcedimension.dhd.constants.DeviceType.OMEGA331_LEFT`
    :data:`forcedimension.dhd.constants.DeviceType.SIGMA331`
    :data:`forcedimension.dhd.constants.DeviceType.SIGMA331_LEFT`

    :param int ID:
         Device ID (see multiple devices section for details).

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to C char.

    :returns:
        ``True`` if the device has a gripper, and ``False`` otherwise.
    """

    return _libdhd.dhdHasGripper(ID)


_libdhd.dhdHasActiveGripper.argtypes = [c_byte]
_libdhd.dhdHasActiveGripper.restype = c_bool


def hasActiveGripper(ID: int = -1) -> bool:
    """
    Checks if the specified device has an active gripper.

    This feature only applies to the following devices
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA331`
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA331_LEFT`
        :data:`forcedimension.dhd.constants.DeviceType.SIGMA331`
        :data:`forcedimension.dhd.constants.DeviceType.SIGMA331_LEFT`

    :param int ID:
         Device ID (see multiple devices section for details).

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to C char.

    :returns:
        ``True`` if the device has an active gripper, and ``False`` otherwise.
    """

    return _libdhd.dhdHasActiveGripper(ID)


_libdhd.dhdReset.argtypes = [c_byte]
_libdhd.dhdReset.restype = c_int


def reset(ID: int = -1) -> int:
    """
    Puts the device in RESET mode.

    :param int ID:
         Device ID (see multiple devices section for details).

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to C char.

    :returns:
         0 on success, -1 otherwise.
    """

    return _libdhd.dhdReset(ID)


_libdhd.dhdWaitForReset.argtypes = [c_int, c_byte]
_libdhd.dhdWaitForReset.restype = c_int


def waitForReset(timeout: Optional[int] = None, ID: int = -1) -> int:
    """
    Puts the device in RESET mode and waits for the user to calibrate the
    device. Optionally, a timeout can be defined after which the call returns
    even if calibration has not occured.

    If the timeout is reached, the call returns an error (-1) and dhdErrno is
    set to :data:`forcedimension.dhd.constants.ErrorNum.TIMEOUT`.

    :param Optional[int] timeout:
        Maximum time to wait for calibration (in [ms]).

    :param int ID:
         Device ID (see multiple devices section for details).

    :raises ArgumentError:
        If not implicitly convertible to C int.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to C char.

    :returns:
        0 on success, -1 otherwise.
    """

    if timeout is not None:
        return _libdhd.dhdWaitForReset(timeout, ID)
    else:
        return _libdhd.dhdWaitForReset(ID)


_libdhd.dhdSetStandardGravity.argtypes = [c_double, c_byte]
_libdhd.dhdSetStandardGravity.restype = c_int


def setStandardGravity(g: float, ID: int = -1) -> int:
    """
    Set the standard gravity constant used in gravity compensation. By default,
    the constant is set to 9.81 m/s^2.

    :param float g:
        standard gravity constant in [m/s^2].

    :param int ID:
         Device ID (see multiple devices section for details).

    :raises ArgumentError:
        If ``g`` is not implicitly convertible to C double.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to C char.

    :returns:
        0 on success, -1 otherwise.
    """

    return _libdhd.dhdSetStandardGravity(g, ID)


_libdhd.dhdSetGravityCompensation.argtypes = [c_bool, c_byte]
_libdhd.dhdSetGravityCompensation.restype = c_int


def setGravityCompensation(enable: bool, ID: int = -1) -> int:
    """
    Enable/disable gravity compensation.

    :param bool enable:
        ``True`` to turn on gravity compensation, ``False`` to turn off
        gravity compensation.

    :param int ID:
         Device ID (see multiple devices section for details).

    :raises ArgumentError:
        If ``val`` is not implicitly convertible to C bool.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to C char.

    :returns:
        0 on success, -1 otherwise.
    """

    return _libdhd.dhdSetStandardGravity(enable, ID)


_libdhd.dhdSetBrakes.argtypes = [c_bool, c_byte]
_libdhd.dhdSetBrakes.restype = c_int


def setBrakes(enable: bool, ID: int = -1) -> int:
    """
    Enable/disable [device electromagnetic brakes].

    :param bool enable:
        ``True`` to turn on the device electromagnetic brakes, ``False`` to
        turn off the device electromagnetic brakes.

    :param int ID:
         Device ID (see multiple devices section for details).

    :raises ArgumentError:
        If ``val`` is not implicitly convertible to C bool.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to C char.

    :returns:
        0 on success, -1 otherwise.
    """

    return _libdhd.dhdSetBrakes(enable, ID)


_libdhd.dhdSetDeviceAngleRad.argtypes = [c_double, c_byte]
_libdhd.dhdSetDeviceAngleRad.restype = c_int


def setDeviceAngleRad(angle: float, ID: int = -1) -> int:
    """
    Set the device base plate angle around the (inverted) Y axis. Please refer
    to your device user manual for more information on your device coordinate
    system. An angle value of 0 refers to the device "upright" position,
    with its base plate perpendicular to axis X. An angle value of π/2
    refers to the device base plate resting horizontally.

    See Also
    --------
    :func:`forcedimension.dhd.setDeviceAngleDeg()`


    :param float angle:
        device base plate angle [rad]

    :param int ID:
         Device ID (see multiple devices section for details).

    :raises ArgumentError:
        If ``angle`` is not implicitly convertible to C double.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to C char.

    :returns:
         0 on success, -1 otherwise.
    """

    return _libdhd.dhdSetDeviceAngleRad(angle, ID)


_libdhd.dhdSetDeviceAngleRad.argtypes = [c_double, c_byte]
_libdhd.dhdSetDeviceAngleRad.restype = c_int


def setDeviceAngleDeg(angle: float, ID: int = -1) -> int:
    """
    Set the device base plate angle around the (inverted) Y axis. Please refer
    to your device user manual for more information on your device coordinate
    system. An angle value of 0 refers to the device "upright" position,
    with its base plate perpendicular to axis X. An angle value of 90
    refers to the device base plate resting horizontally.

    See Also
    --------
    :func:`forcedimension.dhd.setDeviceAngleRad()`


    :param float angle:
        device base plate angle [deg]

    :param int ID:
         Device ID (see multiple devices section for details).

    :raises ArgumentError:
        If ``angle`` is not implicitly convertible to C double.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to C char.

    :returns:
         0 on success, -1 otherwise.
    """

    return _libdhd.dhdSetDeviceAngleDeg(angle, ID)


_libdhd.dhdSetEffectorMass.argtypes = [c_double, c_byte]
_libdhd.dhdSetEffectorMass.restype = c_int


def setEffectorMass(mass: float, ID: int = -1) -> int:
    """
    Define the mass of the end-effector. This function is required to provide
    accurate gravity compensation when custom-made or modified end-effectors
    are used.

    See Also
    --------
    :func:`forcedimension.dhd.getEffectorMass()`


    :param float mass:
        The actual end-effector mass in [kg].

    :param int ID:
         Device ID (see multiple devices section for details).

    :raises ArgumentError:
        If ``angle`` is not implicitly convertible to C double.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to C char.

    :returns:
         0 on success, -1 otherwise.
    """

    return _libdhd.dhdSetEffectorMass(mass, ID)


_libdhd.dhdGetPosition.argtypes = [
    c_double_ptr,
    c_double_ptr,
    c_double_ptr,
    c_byte
]
_libdhd.dhdGetPosition.restype = c_int


def getPosition(out: SupportsPtrs3[c_double], ID: int = -1) -> int:
    """
    Retrieve the position of the end-effector about the x, y, and z axes.
    Please refer to your device user manual for more information on your device
    coordinate system.

    Info
    ----
    Unlike :func:`forcedimension.dhd.getPosition()`, this
    function does not copy the result into an intermediate buffer and instead
    copies the result directly into the return buffer.


    :param SupportsPtrs3[c_double] out:
        An output buffer to store the position of the end-effector (in [m]).

    :param int ID:
         Device ID (see multiple devices section for details).

    :raises AttributeError:
        If ``out.ptrs`` is not a valid attribute of ``out``

    :raises TypeError:
        If ``out.ptrs`` is not iterable.

    :raises ArgumentError:
        If ``out.ptrs`` does not expand into a tuple of 3
        ``Pointer[c_double]`` types.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to C char.

    :returns:
        0 or :data:`forcedimension.dhd.TIMEGUARD` on success,
        -1 otherwise.
    """

    return _libdhd.dhdGetPosition(*out.ptrs, ID)


_libdhd.dhdGetForce.argtypes = [
    c_double_ptr,
    c_double_ptr,
    c_double_ptr,
    c_byte
]
_libdhd.dhdGetForce.restype = c_int


def getForce(out: SupportsPtrs3[c_double], ID: int = -1) -> int:
    """
    Retrieve the force vector applied to the end-effector (in [N])
    about the x, y, and z axes Please refer to your device user manual for more
    information on your device coordinate system.

    Info
    ----
    Unlike :func:`forcedimension.dhd.getForce()`, this
    function does not copy the result into an intermediate buffer and instead
    copies the result directly into the return buffer.


    :param SupportsPtrs3[c_double] out:
        An output buffer to store the applied forces on the end-effector
        (in [N]).

    :param int ID:
         Device ID (see multiple devices section for details).

    :raises AttributeError:
        If ``out.ptrs`` is not a valid attribute of ``out``

    :raises TypeError:
        If ``out`` does not support item assignment either
        because its immutable or not subscriptable.

    :raises TypeError:
        If ``out.ptrs`` is not iterable.

    :raises ArgumentError:
        If ``out.ptrs`` does not expand into a tuple of 3
        ``Pointer[c_double]`` types.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to C char.

    :returns:
        0 on success, -1 otherwise.
    """

    return _libdhd.dhdGetForce(*out.ptrs, ID)


_libdhd.dhdSetForce.argtypes = [c_double, c_double, c_double, c_byte]
_libdhd.dhdSetForce.restype = c_int


def setForce(f: FloatVectorLike, ID: int = -1) -> int:
    """
    Set the desired force (in [N]) about the x, y, and z axes to be applied
    to the end-effector of the device.

    :param VectorLike f:
        Translation force vector (fx, fy, fz) in [N].

    :param int ID:
         Device ID (see multiple devices section for details).

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to C char.

    :raises ArgumentError:
        If any elements of ``f`` are not implicitly convertible to C double.

    :raises IndexError:
        If ``len(f) < 3)``.

    :raises TypeError:
        If ``f`` is not subscriptable.

    :returns:
        0 or :data:`forcedimension.dhd.constants.MOTOR_SATURATED` on
        success, -1 otherwise.
    """

    return _libdhd.dhdSetForce(f[0], f[1], f[2], ID)


_libdhd.dhdGetOrientationRad.argtypes = [
    c_double_ptr,
    c_double_ptr,
    c_double_ptr,
    c_byte
]
_libdhd.dhdGetOrientationRad.restype = c_int


def getOrientationRad(
    out: SupportsPtrs3[c_double], ID: int = -1
) -> int:
    """
    For devices with a wrist structure, retrieve individual angle (in [rad])
    of each joint, starting with the one located nearest to the wrist base
    plate.

    This feature only applies to the following devices
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA33`
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA33_LEFT`
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA331`
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA331_LEFT`
        :data:`forcedimension.dhd.constants.DeviceType.SIGMA331`
        :data:`forcedimension.dhd.constants.DeviceType.SIGMA331_LEFT`


    Info
    ----
    :data:`forcedimension.dhd.constants.DeviceType.OMEGA33` and
    :data:`forcedimension.dhd.constants.DeviceType.OMEGA33_LEFT`
    have angles that are instead computed with respect to their internal
    reference frame, which is rotated π/4 radians around the Y axis.
    Please refer to your device user manual for more information on your
    device coordinate system.


    Info
    ----
    Unlike :func:`forcedimension.dhd.getOrientationRad()`, this
    function does not copy the result into an intermediate buffer and instead
    copies the result directly into the return buffer.


    :param SupportsPtrs3[c_double] out:
        An output buffer to store the joint angles (in [rad]).

    :param int ID:
         Device ID (see multiple devices section for details).

    :raises AttributeError:
        If ``out.ptrs`` is not a valid attribute of ``out``

    :raises TypeError:
        If ``out`` does not support item assignment either
        because its immutable or not subscriptable.

    :raises TypeError:
        If ``out.ptrs`` is not iterable.

    :raises ArgumentError:
        If ``out.ptrs`` does not expand into a tuple of 3
        ``Pointer[c_double]`` types.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to C char.

    :returns:
        0 or :data:`forcedimension.dhd.constants.TIMEGUARD` on success,
        -1 otherwise.
    """

    return _libdhd.dhdGetOrientationRad(*out.ptrs, ID)


_libdhd.dhdGetOrientationDeg.argtypes = [
    c_double_ptr,
    c_double_ptr,
    c_double_ptr,
    c_byte
]
_libdhd.dhdGetOrientationDeg.restype = c_int


def getOrientationDeg(
    out: SupportsPtrs3[c_double], ID: int = -1
) -> int:
    """
    For devices with a wrist structure, retrieve individual angle (in [deg])
    of each joint, starting with the one located nearest to the wrist base
    plate.

    This feature only applies to the following devices
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA33`
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA33_LEFT`
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA331`
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA331_LEFT`
        :data:`forcedimension.dhd.constants.DeviceType.SIGMA331`
        :data:`forcedimension.dhd.constants.DeviceType.SIGMA331_LEFT`


    Info
    ----
    :data:`forcedimension.dhd.constants.DeviceType.OMEGA33` and
    :data:`forcedimension.dhd.constants.DeviceType.OMEGA33_LEFT`
    have angles that are instead computed with respect to their internal
    reference frame, which is rotated π/4 radians around the Y axis.
    Please refer to your device user manual for more information on your
    device coordinate system.


    Info
    ----
    Unlike :func:`forcedimension.dhd.getOrientationDeg()`, this
    function does not copy the result into an intermediate buffer and instead
    copies the result directly into the return buffer.


    :param SupportsPtrs3[c_double] out:
        An output buffer to store the joint angles (in [deg]).

    :param int ID:
         Device ID (see multiple devices section for details).

    :raises AttributeError:
        If ``out.ptrs`` is not a valid attribute of ``out``

    :raises TypeError:
        If ``out`` does not support item assignment either
        because its immutable or not subscriptable.

    :raises TypeError:
        If ``out.ptrs`` is not iterable.

    :raises ArgumentError:
        If ``out.ptrs`` does not expand into a tuple of 3
        ``Pointer[c_double]`` types.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to C char.

    :returns:
        0 or :data:`forcedimension.dhd.constants.TIMEGUARD` on success,
        -1 otherwise.
    """

    return _libdhd.dhdGetOrientationDeg(*out.ptrs, ID)


_libdhd.dhdGetPositionAndOrientationRad.argtypes = [
    c_double_ptr,
    c_double_ptr,
    c_double_ptr,
    c_double_ptr,
    c_double_ptr,
    c_double_ptr,
    c_byte
]
_libdhd.dhdGetPositionAndOrientationRad.restype = c_int


def getPositionAndOrientationRad(
    p_out: SupportsPtrs3[c_double],
    o_out: SupportsPtrs3[c_double],
    ID: int = -1
) -> int:
    """
    Retrieve the position (in [m]) and
    for devices with a wrist structure, retrieve individual angle
    of each joint in [rad], starting with the one located nearest to the wrist
    base plate.

    Note:

    :data:`forcedimension.dhd.constants.DeviceType.OMEGA33`
    :data:`forcedimension.dhd.constants.DeviceType.OMEGA33_LEFT`

    have angles that are instead computed with respect to their internal
    reference frame, which is rotated π/4 radians around the Y axis.
    Please refer to your device user manual for more information on your
    device coordinate system.

    This feature only applies to the following devices
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA33`
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA33_LEFT`
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA331`
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA331_LEFT`
        :data:`forcedimension.dhd.constants.DeviceType.SIGMA331`
        :data:`forcedimension.dhd.constants.DeviceType.SIGMA331_LEFT`

    Info
    ----
    Unlike :func:`forcedimension.dhd.getPositionAndOrientationDeg()`, this
    function does not copy the result into an intermediate buffer and instead
    copies the result directly into the return buffer.


    :param int ID:
         Device ID (see multiple devices section for details).

    :param SupportsPtrs3[c_double] p_out:
        An output buffer to store the position (in [m]).

    :param SupportsPtrs3[c_double] o_out:
        An output buffer to store the joint angles (in [rad]).

    :raises AttributeError:
        If ``p_out.ptrs`` is not a valid attribute of ``out``

    :raises TypeError:
        If ``p_out.ptrs`` is not iterable.

    :raises ArgumentError:
        If ``p_out.ptrs`` does not expand into a tuple of 3
        ``Pointer[c_double]`` types.

    :raises AttributeError:
        If ``o_out.ptrs`` is not a valid attribute of ``out``

    :raises TypeError:
        If ``o_out`` does not support item assignment either
        because its immutable or not subscriptable.

    :raises TypeError:
        If ``o_out.ptrs`` is not iterable.

    :raises ArgumentError:
        If ``o_out.ptrs`` does not expand into a tuple of 3
        ``Pointer[c_double]`` types.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to C char.

    :returns:
        0 or :data:`forcedimension.dhd.constants.TIMEGUARD` on success,
        -1 otherwise.
    """

    return _libdhd.dhdGetPositionAndOrientationRad(
        *p_out.ptrs, *o_out.ptrs, ID
    )


_libdhd.dhdGetPositionAndOrientationDeg.argtypes = [
    c_double_ptr,
    c_double_ptr,
    c_double_ptr,
    c_double_ptr,
    c_double_ptr,
    c_double_ptr,
    c_byte
]
_libdhd.dhdGetPositionAndOrientationDeg.restype = c_int


def getPositionAndOrientationDeg(
    p_out: SupportsPtrs3[c_double],
    o_out: SupportsPtrs3[c_double],
    ID: int = -1
) -> int:
    """
    Retrieve the position (in [m]) and
    for devices with a wrist structure, retrieve individual angle
    of each joint in [deg], starting with the one located nearest to the wrist
    base plate.

    Note:

    :data:`forcedimension.dhd.constants.DeviceType.OMEGA33`
    :data:`forcedimension.dhd.constants.DeviceType.OMEGA33_LEFT`

    have angles that are instead computed with respect to their internal
    reference frame, which is rotated π/4 radians around the Y axis.
    Please refer to your device user manual for more information on your
    device coordinate system.

    This feature only applies to the following devices
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA33`
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA33_LEFT`
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA331`
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA331_LEFT`
        :data:`forcedimension.dhd.constants.DeviceType.SIGMA331`
        :data:`forcedimension.dhd.constants.DeviceType.SIGMA331_LEFT`

    Info
    ----
    Unlike :func:`forcedimension.dhd.getPositionAndOrientationDeg()`, this
    function does not copy the result into an intermediate buffer and instead
    copies the result directly into the return buffer.


    :param int ID:
         Device ID (see multiple devices section for details).

    :param SupportsPtrs3[c_double] p_out:
        An output buffer to store the position (in [m]).

    :param SupportsPtrs3[c_double] o_out:
        An output buffer to store the joint angles (in [rad]).

    :raises AttributeError:
        If ``p_out.ptrs`` is not a valid attribute of ``out``

    :raises TypeError:
        If ``p_out.ptrs`` is not iterable.

    :raises ArgumentError:
        If ``p_out.ptrs`` does not expand into a tuple of 3
        ``Pointer[c_double]`` types.

    :raises AttributeError:
        If ``o_out.ptrs`` is not a valid attribute of ``out``

    :raises TypeError:
        If ``o_out`` does not support item assignment either
        because its immutable or not subscriptable.

    :raises TypeError:
        If ``o_out.ptrs`` is not iterable.

    :raises ArgumentError:
        If ``o_out.ptrs`` does not expand into a tuple of 3
        ``Pointer[c_double]`` types.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to C char.

    :returns:
        0 or :data:`forcedimension.dhd.constants.TIMEGUARD` on success,
        -1 otherwise.
    """

    return _libdhd.dhdGetPositionAndOrientationDeg(
        *p_out.ptrs, *o_out.ptrs, ID
    )


_libdhd.dhdGetPositionAndOrientationFrame.argtypes = [
    c_double_ptr,
    c_double_ptr,
    c_double_ptr,
    c_double_ptr,
    c_byte
]
_libdhd.dhdGetPositionAndOrientationFrame.restype = c_int


def getPositionAndOrientationFrame(
    p_out: SupportsPtrs3[c_double],
    matrix_out: SupportsPtr[c_double],
    ID: int = -1
) -> int:
    """
    Retrieve the position (in [m]) and orientation matrix of the end-effector
    about the x, y, and z axes. Please refer to your device user manual for
    more information on your device coordinate system.

    Info
    ----
    Unlike :func:`forcedimension.dhd.getPositionAndOrientationFrame()`, this
    function does not copy the result into an intermediate buffer and instead
    copies the result directly into the return buffer.

    :param int ID:
         Device ID (see multiple devices section for details).

    :raises AttributeError:
        If ``p_out.ptrs`` is not a valid attribute of ``p_out``

    :raises TypeError:
        If ``p_out.ptrs`` is not iterable.

    :raises ArgumentError:
        If ``p_out.ptrs`` does not expand into a tuple of 3
        ``Pointer[c_double]`` types.

    :raises AttributeError:
        If ``matrix_out.ptr`` is not a valid attribute of ``matrix_out``

    :raises ArgumentError:
        If ``p_out.ptr`` is not a ``Pointer[c_double]`` type.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to C char.

    :returns:
        0 or :data:`forcedimension.dhd.constants.TIMEGUARD` on success,
        -1 otherwise.
    """

    return _libdhd.dhdGetPositionAndOrientationFrame(
        *p_out.ptrs, matrix_out.ptr, ID
    )


_libdhd.dhdGetForceAndTorque.argtypes = [
    c_double_ptr,
    c_double_ptr,
    c_double_ptr,
    c_double_ptr,
    c_double_ptr,
    c_double_ptr,
    c_byte
]
_libdhd.dhdGetForceAndTorque.restype = c_int


def getForceAndTorque(
    f_out: SupportsPtrs3[c_double],
    t_out: SupportsPtrs3[c_double],
    ID: int = -1
) -> int:
    """
    Retrieve the forces (in [N]) and torques (in [Nm]) applied to the device
    end-effector about the x, y, and z axes. Please refer to your device user
    manual for more information on your device coordinate system.

    Info
    ----
    Unlike :func:`forcedimension.dhd.getForceAndTorque()`, this
    function does not copy the result into an intermediate buffer and instead
    copies the result directly into the return buffers.


    :param int ID:
         Device ID (see multiple devices section for details).

    :param SupportsPtrs3[c_double] f_out:
        An output buffer to store the applied forces on the end-effector
        (in [N]).

    :param SupportsPtrs3[c_double] t_out:
        An output buffer to store the applied torques on the end-effector
        (in [N]).

    :raises AttributeError:
        If ``f_out.ptrs`` is not a valid attribute of ``f_out``

    :raises TypeError:
        If ``f_out.ptrs`` is not iterable.

    :raises ArgumentError:
        If ``f_out.ptrs`` does not expand into a tuple of 3
        ``Pointer[c_double]`` types.

    :raises AttributeError:
        If ``t_out.ptrs`` is not a valid attribute of ``t_out``

    :raises TypeError:
        If ``t_out.ptrs`` is not iterable.

    :raises ArgumentError:
        If ``t_out.ptrs`` does not expand into a tuple of 3
        ``Pointer[c_double]`` types.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to C char.

    :returns:
        0, on success, -1 otherwise.
    """

    return _libdhd.dhdGetForceAndTorque(*f_out.ptrs, *t_out.ptrs, ID)


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


def setForceAndTorque(
    f: FloatVectorLike,
    t: FloatVectorLike,
    ID: int = -1
) -> int:
    """
    Set the desired force and torque vectors to be applied to the device
    end-effector.

    :param VectorLike f:
        Translational force vector ``(fx, fy, fz)`` where ``fx``, ``fy``, and
        ``fz`` are the translation force (in [N]) on the end-effector about the
        X, Y, and Z axes, respectively.

    :param VectorLike t:
        Torque vector ``(tx, ty, tz)`` where ``tx``, ``ty``, and ``tz``
        are the torque (in [Nm]) on the end-effector about the X, Y, and Z
        axes, respectively.

    :param int ID:
         Device ID (see multiple devices section for details).

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to C char.

    :raises ArgumentError:
        If any elements of ``f`` is not implicitly convertible to a C double.

    :raises IndexError:
        If ``len(f) < 3``.

    :raises TypeError:
        If ``f`` is not subscriptable.

    :raises ArgumentError:
        If any elements of ``t`` is not implicitly convertible to a C double.

    :raises IndexError:
        If ``len(t) < 3``.

    :raises TypeError:
        If ``t`` is not subscriptable.

   :raises ArgumentError:
        If ``gripper_force`` is not implicitly convertible to a C double.

    :raises IndexError:
        If ``len(f) < 3``.

    :raises IndexError:
        If ``len(t) < 3``.

    :returns:
        0 or :data:`forcedimension.dhd.constants.MOTOR_SATURATED` on success,
        and -1 otherwise.
    """

    return _libdhd.dhdSetForceAndTorque(f[0], f[1], f[2], t[0], t[1], t[2], ID)


_libdhd.dhdGetOrientationFrame.argtypes = [
    c_double_ptr,
    c_byte
]
_libdhd.dhdGetOrientationFrame.restype = c_int


def getOrientationFrame(out: SupportsPtr[c_double], ID: int = -1) -> int:
    """
    Retrieve the rotation matrix of the wrist structure. The identity matrix
    is returned for devices that do not support orientations.

    Info
    ----
    Unlike :func:`forcedimension.dhd.getOrientationFrame()`, this
    function does not copy the result into an intermediate buffer and instead
    copies the result directly into the return buffer.


    :param int ID:
         Device ID (see multiple devices section for details).

    :param SupportsPtr[c_double] out:
        An output buffer to store the orientation frame.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to C char.

    :raises AttributeError:
        If ``out.ptr`` is not a valid attribute of ``p_out``

    :raises ArgumentError:
        If ``out.ptr`` is not a ``Pointer[c_double]`` type.

    :returns:
        0 or :data:`forcedimension.dhd.constants.TIMEGUARD` on success,
        -1 otherwise.
    """

    return _libdhd.dhdGetOrientationFrame(out.ptr, ID)


_libdhd.dhdGetGripperAngleDeg.argtypes = [c_double_ptr, c_byte]
_libdhd.dhdGetGripperAngleDeg.restype = c_int


def getGripperAngleDeg(ID: int = -1) -> Tuple[float, int]:
    """
    Get the gripper opening angle in degrees.

    This feature only applies to the following devices
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA331`
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA331_LEFT`
        :data:`forcedimension.dhd.constants.DeviceType.SIGMA331`
        :data:`forcedimension.dhd.constants.DeviceType.SIGMA331_LEFT`

    See Also
    --------
    :func:`forcedimension.dhd.getGripperAngleRad()`


    :param int ID:
         Device ID (see multiple devices section for details).

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to C char.

    :returns:
        0 or :data:`forcedimension.dhd.constants.TIMEGUARD` on success,
        -1 otherwise.
    """

    angle = c_double()
    err = _libdhd.dhdGetGripperAngleDeg(angle, ID)
    return (angle.value, err)


_libdhd.dhdGetGripperAngleRad.argtypes = [c_double_ptr, c_byte]
_libdhd.dhdGetGripperAngleRad.restype = c_int


def getGripperAngleRad(ID: int = -1) -> Tuple[float, int]:
    """
    Get the gripper opening angle in degrees.

    This feature only applies to the following devices
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA331`
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA331_LEFT`
        :data:`forcedimension.dhd.constants.DeviceType.SIGMA331`
        :data:`forcedimension.dhd.constants.DeviceType.SIGMA331_LEFT`

    See Also
    --------
    :data:`forcdimension.dhd.getGripperAngleDeg()`


    :param int ID:
         Device ID (see multiple devices section for details).

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to C char.

    :returns:
        A tuple in the form ``(angle, err)``. ``angle`` is the griper angle
        (in [rad]).
        ``err`` is 0 or :data:`forcedimension.dhd.constants.TIMEGUARD`
        on success, -1 otherwise.
    """

    angle = c_double()
    return (angle.value, _libdhd.dhdGetGripperAngleRad(angle, ID))


_libdhd.dhdGetGripperGap.argtypes = [c_double_ptr, c_byte]
_libdhd.dhdGetGripperGap.restype = c_int


def getGripperGap(ID: int = -1) -> Tuple[float, int]:
    """
    Get the gripper opening distance in meters.

    This feature only applies to the following devices
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA331`
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA331_LEFT`
        :data:`forcedimension.dhd.constants.DeviceType.SIGMA331`
        :data:`forcedimension.dhd.constants.DeviceType.SIGMA331_LEFT`

    :param int ID:
         Device ID (see multiple devices section for details).

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to C char.

    :returns:
        A tuple in the form ``(gap, err)``. ``gap`` is the gripper opening
        distance (in [m]). ``err`` is 0 or
        :data:`forcedimension.dhd.constants.TIMEGUARD` on success,
        -1 otherwise.

    """

    gap = c_double()
    err = _libdhd.dhdGetGripperGap(gap, ID)
    return (gap.value, err)


_libdhd.dhdGetGripperThumbPos.argtypes = [
    c_double_ptr,
    c_double_ptr,
    c_double_ptr,
    c_byte
]
_libdhd.dhdGetGripperThumbPos.restype = c_int


def getGripperThumbPos(
    out: SupportsPtrs3[c_double], ID: int = -1
) -> int:
    """
    Read the position (in [m]) of thumb rest location about the x, y and z axes
    of the force gripper structure if present.

    This feature only applies to the following devices
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA331`
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA331_LEFT`
        :data:`forcedimension.dhd.constants.DeviceType.SIGMA331`
        :data:`forcedimension.dhd.constants.DeviceType.SIGMA331_LEFT`

    Info
    ----
    Unlike :func:`forcedimension.dhd.getGripperThumbPos()`, this
    function does not copy the result into an intermediate buffer and instead
    copies the result directly into the return buffer.


    :param int ID:
         Device ID (see multiple devices section for details).

    :param SupportsPtrs3[c_double] out:
        An output buffer to store the grippper thumb position (in [m]).

    :raises AttributeError:
        If ``out.ptrs`` is not a valid attribute of ``out``

    :raises TypeError:
        If ``out.ptrs`` is not iterable.

    :raises ArgumentError:
        If ``out.ptrs`` does not expand into a tuple of 3
        ``Pointer[c_double]`` types.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to C char.

    :returns:
        0 or :data:`forcedimension.dhd.constants.TIMEGUARD` on success,
        -1 otherwise.

    """

    return _libdhd.dhdGetGripperFingerPos(*out.ptrs, ID)


_libdhd.dhdGetGripperFingerPos.argtypes = [
    c_double_ptr,
    c_double_ptr,
    c_double_ptr,
    c_byte
]
_libdhd.dhdGetGripperFingerPos.restype = c_int


def getGripperFingerPos(
    out: SupportsPtrs3[c_double], ID: int = -1
) -> int:
    """
    Read the position (in [m]) of forefinger rest location about the x, y, and
    z axes of the force gripper structure if present.

    This feature only applies to the following devices
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA331`
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA331_LEFT`
        :data:`forcedimension.dhd.constants.DeviceType.SIGMA331`
        :data:`forcedimension.dhd.constants.DeviceType.SIGMA331_LEFT`

    Info
    ----
    Unlike :func:`forcedimension.dhd.getGripperFingerPos()`, this
    function does not copy the result into an intermediate buffer and instead
    copies the result directly into the return buffer.


    :param int ID:
         Device ID (see multiple devices section for details).

    :param SupportsPtrs3[c_double] out:
        An output buffer to store the gripper finger position (in [m]).

    :raises AttributeError:
        If ``out.ptrs`` is not a valid attribute of ``out``

    :raises TypeError:
        If ``out.ptrs`` is not iterable.

    :raises ArgumentError:
        If ``out.ptrs`` does not expand into a tuple of 3
        ``Pointer[c_double]`` types.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to C char.

    :returns:
        0 or :data:`forcedimension.dhd.constants.TIMEGUARD` on success,
        -1 otherwise.

    """

    return _libdhd.dhdGetGripperFingerPos(*out.ptrs, ID)


_libdhd.dhdGetComFreq.argtypes = [c_byte]
_libdhd.dhdGetComFreq.restype = c_double


def getComFreq(ID: int = -1) -> float:
    """
    Return the communication refresh rate between the computer and the device.
    Refresh rate computation is based on function calls that apply a force on
    the device (e.g. :func:`forcedimension.dhd.setForce()`).

    :param int ID:
         Device ID (see multiple devices section for details).

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to C char.

    :returns:
        The refresh rate in [kHz], ``0.0`` otherwise.
    """

    return _libdhd.dhdGetComFreq(ID)


_libdhd.dhdSetForceAndGripperForce.argtypes = [
    c_double,
    c_double,
    c_double,
    c_double,
    c_byte
]
_libdhd.dhdSetForceAndGripperForce.restype = c_int


def setForceAndGripperForce(f: FloatVectorLike,
                            fg: float,
                            ID: int = -1) -> int:
    """
    Set the desired force to be applied to the end-effector about the x, y,
    and z axes as well as the force applied by force gripper of the device.

    :param int ID:
         Device ID (see multiple devices section for details).

    :param VectorLike f:
        Translational force vector ``(fx, fy, fz)`` where ``fx``, ``fy``, and
        ``fz`` are the translation force (in [N]) about the X, Y, and Z axes,
        respectively.

    :param float fg:
        Grasping force of the gripper in [N].

    :raises ArgumentError:
        If any elements of ``f`` is not implicitly convertible to a C double.

    :raises IndexError:
        If ``len(f) < 3``.

    :raises TypeError:
        If ``f`` is not subscriptable.

   :raises ArgumentError:
        If ``gripper_force`` is not implicitly convertible to a C double.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to C char.

    :raises IndexError:
        If ``len(f) < 3``g

    :returns:
        0 or :data:`forcedimension.dhd.constants.MOTOR_SATURATED` on
        success, -1 otherwise.
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


def setForceAndTorqueAndGripperForce(f: FloatVectorLike,
                                     t: FloatVectorLike,
                                     fg: float,
                                     ID: int = -1) -> int:
    """
    Set the desired force and torque vectors to be applied to the device
    end-effector and gripper.

    :param VectorLike f:
        Translational force vector ``(fx, fy, fz)`` where ``fx``, ``fy``, and
        ``fz`` are the translation force (in [N]) on the end-effector about the
        X, Y, and Z axes, respectively.

    :param VectorLike t:
        Torque vector ``(tx, ty, tz)`` where ``tx``, ``ty``, and ``tz``
        are the torque (in [Nm]) on the end-effector about the X, Y, and Z
        axes, respectively.

    :param float fg:
        Grasping force of the gripper in [N].

    :param int ID:
         Device ID (see multiple devices section for details).

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to C char.

    :raises ArgumentError:
        If any elements of ``f`` is not implicitly convertible to a C double.

    :raises IndexError:
        If ``len(f) < 3``.

    :raises TypeError:
        If ``f`` is not subscriptable.

    :raises ArgumentError:
        If any elements of ``t`` is not implicitly convertible to a C double.

    :raises IndexError:
        If ``len(t) < 3``.

    :raises TypeError:
        If ``t`` is not subscriptable.

   :raises ArgumentError:
        If ``gripper_force`` is not implicitly convertible to a C double.

    :raises IndexError:
        If ``len(f) < 3``.

    :raises IndexError:
        If ``len(t) < 3``.

    :returns:
        0 or
        :data:`forcedimension.dhd.constants.MOTOR_SATURATED` on success, and
        -1 otherwise.

    """

    return _libdhd.dhdSetForceAndTorqueAndGripperForce(
        f[0],
        f[1],
        f[2],
        t[0],
        t[1],
        t[2],
        fg,
        ID
    )


_libdhd.dhdGetForceAndTorqueAndGripperForce.argtypes = [
    c_double_ptr,
    c_double_ptr,
    c_double_ptr,
    c_double_ptr,
    c_double_ptr,
    c_double_ptr,
    c_double_ptr,
    c_byte
]
_libdhd.dhdGetForceAndTorqueAndGripperForce.restype = c_int


def getForceAndTorqueAndGripperForce(
    f_out: SupportsPtrs3[c_double],
    t_out: SupportsPtrs3[c_double],
    fg_out: c_double,
    ID: int = -1
) -> int:
    """
    Retrieve the forces (in [N]) and torques (in [Nm]) applied to the device
    end-effector as well as the gripper force (in [N]).
    Forces and torques are about the x, y, and z axes.

    Info
    ----
    Unlike :func:`forcedimension.dhd.getPosition()`, this
    function does not copy the result into an intermediate buffer and instead
    copies the result directly into the return buffer.


    :param int ID:
         Device ID (see multiple devices section for details).

    :param SupportsPtrs3[c_double] f_out:
        An output buffer to store the applied forces on the end-effector
        (in [N]).

    :param SupportsPtrs3[c_double] t_out:
        An output buffer to store the applied torques on the end-effector
        (in [Nm]).

    :raises AttributeError:
        If ``f_out.ptrs`` is not a valid attribute of ``f_out``

    :raises TypeError:
        If ``f_out.ptrs`` is not iterable.

    :raises ArgumentError:
        If ``f_out.ptrs`` does not expand into a tuple of 3
        ``Pointer[c_double]`` types.

    :raises AttributeError:
        If ``t_out.ptrs`` is not a valid attribute of ``t_out``

    :raises TypeError:
        If ``t_out.ptrs`` is not iterable.

    :raises ArgumentError:
        If ``t_out.ptrs`` does not expand into a tuple of 3
        ``Pointer[c_double]`` types.

    :raises AttributeError:
        If ``fg_out.ptr`` is not a valid attribute of ``fg_out``

    :raises ArgumentError:
        If ``fg_out.ptr`` is not a ``Pointer[c_double]``.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to C char.

    :returns:
       0, on success, -1 otherwise.
    """

    err = _libdhd.dhdGetForceAndTorqueAndGripperForce(
        *f_out.ptrs, *t_out.ptrs, fg_out, ID
    )
    return err


def configLinearVelocity(
    ms: int = DEFAULT_VELOCITY_WINDOW,
    mode: int = VelocityEstimatorMode.WINDOWING,
    ID: int = -1
) -> int:
    """
    Configure the internal linear velocity computation estimator.
    This only applies to the device base.

    :param int ms:
        Time interval used to compute velocity in [ms].

    :param int mode:
        Velocity estimator mode (see [velocity estimator] modes section for
        details).

    :param int ID:
         Device ID (see multiple devices section for details).

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to C char.

    :raises ArgumentError:
        If ``ms`` is not implicitly convertible to C int.

    :raises ArgumentError:
        If ``mode`` is not implicitly convertible to C int.

    :returns:
        0 on success, -1 otherwise.
    """

    return _libdhd.dhdConfigLinearVelocity(ms, mode, ID)


_libdhd.dhdGetLinearVelocity.argtypes = [
    c_double_ptr,
    c_double_ptr,
    c_double_ptr,
    c_byte
]
_libdhd.dhdGetLinearVelocity.restype = c_int


def getLinearVelocity(out: SupportsPtrs3[c_double], ID: int = -1) -> int:
    """
    Retrieve the estimated instanteous linear velocity (in [m/s]).

    By default :data:`forcedimension.dhd.constants.VELOCITY_WINDOW` and
    :data:`forcedimension.dhd.constants.VELOCITY_WINDOWING`
    are used. See velocity estimator for details.

    Info
    ----
    The velocity estimator requires at least 2 position
    updates during the time interval defined in
    :func:`forcedimension.dhd.configLinearVelocity()` in order to be able to
    compute the estimate. Otherwise, e.g. if there are no calls to
    :func:`forcedimension.dhd.getPosition(), dhd.libdhd.getLinearVelocity()`,
    or :func:`forcedimension.dhd.getLinearVelocity()` will return an error
    (:data:`forcedimension.dhd.constants.ErrorNum.TIMEOUT`).


    Info
    ----
    Unlike :func:`forcedimension.dhd.getLinearVelocity()`, this
    function does not copy the result into an intermediate buffer and instead
    copies the result directly into the return buffer.


    See Also
    --------
    :func:`forcedimension.dhd.configLinearVelocity()`


    :param int ID:
         Device ID (see multiple devices section for details).

    :param SupportsPtr[c_double] out:
        An output buffer to store the linear velocity (in [m/s]).

    :raises AttributeError:
        If ``out.ptrs`` is not a valid attribute of ``out``

    :raises TypeError:
        If ``out.ptrs`` is not iterable.

    :raises ArgumentError:
        If ``out.ptrs`` does not expand into a tuple of 3
        ``Pointer[c_double]`` types.


   :raises ArgumentError:
        If ``ID`` is not implicitly convertible to C char.

    :returns:
        0 on success, -1 otherwise.
    """

    return _libdhd.dhdGetLinearVelocity(*out.ptrs, ID)


_libdhd.dhdConfigAngularVelocity.argtypes = [c_int, c_int, c_byte]
_libdhd.dhdConfigAngularVelocity.restype = c_int


def configAngularVelocity(
    ms: int = DEFAULT_VELOCITY_WINDOW,
    mode: int = VelocityEstimatorMode.WINDOWING,
    ID: int = -1

) -> int:
    """
    Configure the internal angular velocity computation estimator.
    This only applies to the device wrist.

    :param int ms:
        time interval used to compute velocity (in [ms]).

    :param int mode:
        velocity estimator mode (see [velocity estimator] modes section for
        details).

    :param int ID:
         Device ID (see multiple devices section for details).

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to C char.

    :raises ArgumentError:
        If ``ms`` is not implicitly convertible to C int.

    :raises ArgumentError:
        If ``mode`` is not implicitly convertible to C int.

    :returns:
        0 on success, -1 otherwise.
    """

    return _libdhd.dhdConfigAngularVelocity(ms, mode, ID)


_libdhd.dhdGetAngularVelocityRad.argtypes = [
    c_double_ptr,
    c_double_ptr,
    c_double_ptr,
    c_byte
]
_libdhd.dhdGetAngularVelocityRad.restype = c_int


def getAngularVelocityRad(
    out: SupportsPtrs3[c_double], ID: int = -1
) -> int:
    """
    Retrieve the estimated angular velocity (in [rad/s]).

    Info
    ----
    The velocity estimator requires at least 2 position
    updates during the time interval defined in
    :func:`forcedimension.dhd.configLinearVelocity()` in order to be able to
    compute the estimate. Otherwise, e.g. if there are no calls to
    :func:`forcedimension.dhd.getPosition(), dhd.libdhd.getLinearVelocity()`,
    or :func:`forcedimension.dhd.getLinearVelocity()` will return an error
    (:data:`forcedimension.dhd.constants.ErrorNum.TIMEOUT`).

    See Also
    --------
    :func:`forcedimension.dhd.configAngularVelocity()`
    :func:`forcedimension.dhd.getAngularVelocityDeg()`


    Info
    ----
    Unlike :func:`forcedimension.dhd.getAngularVelocityRad()`, this
    function does not copy the result into an intermediate buffer and instead
    copies the result directly into the return buffer.


    :param int ID:
         Device ID (see multiple devices section for details).

    :param out:
        An output buffer to store the angular velocity (in [rad/s]).

    :raises AttributeError:
        If ``out.ptrs`` is not a valid attribute of ``out``

   :raises TypeError:
        If ``out.ptrs`` is not iterable.

    :raises ArgumentError:
        If ``out.ptrs`` does not expand into a tuple of 3
        ``Pointer[c_double]`` types.


    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to C char.

    :returns:
        0 on success, -1 otherwise.
    """

    return _libdhd.dhdGetAngularVelocityRad(*out.ptrs, ID)


_libdhd.dhdGetAngularVelocityDeg.argtypes = [
    c_double_ptr,
    c_double_ptr,
    c_double_ptr,
    c_byte
]
_libdhd.dhdGetAngularVelocityDeg.restype = c_int


def getAngularVelocityDeg(
    out: SupportsPtrs3[c_double], ID: int = -1
) -> int:
    """
    Retrieve the estimated angular velocity (in [deg/s]).

    Info
    ----
    The velocity estimator requires at least 2 position
    updates during the time interval defined in
    :func:`forcedimension.dhd.configLinearVelocity()` in order to be able to
    compute the estimate. Otherwise, e.g. if there are no calls to
    :func:`forcedimension.dhd.getPosition(), dhd.libdhd.getLinearVelocity()`,
    or :func:`forcedimension.dhd.getLinearVelocity()` will return an error
    (:data:`forcedimension.dhd.constants.ErrorNum.TIMEOUT`).

    See Also
    --------
    :func:`forcedimension.dhd.configAngularVelocity()`
    :func:`forcedimension.dhd.getAngularVelocityDeg()`


    Info
    ----
    Unlike :func:`forcedimension.dhd.getAngularVelocityDeg()`, this
    function does not copy the result into an intermediate buffer and instead
    copies the result directly into the return buffer.


    :param int ID:
         Device ID (see multiple devices section for details).

    :param out:
        An output buffer to store the angular velocity (in [deg/s]).

    :raises AttributeError:
        If ``out.ptrs`` is not a valid attribute of ``out``

   :raises TypeError:
        If ``out.ptrs`` is not iterable.

    :raises ArgumentError:
        If ``out.ptrs`` does not expand into a tuple of 3
        ``Pointer[c_double]`` types.


    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to C char.

    :returns:
        0 on success, -1 otherwise.
    """

    return _libdhd.dhdGetAngularVelocityDeg(*out.ptrs, ID)


_libdhd.dhdConfigGripperVelocity.argtypes = [c_int, c_int, c_byte]
_libdhd.dhdConfigGripperVelocity.restype = c_int


def configGripperVelocity(
    ms: int,
    mode: int,
    ID: int = -1
) -> int:
    """
    Configure the internal linear velocity computation estimator.
    This only applies to the device gripper.

    See Also
    --------
    :func:`forcedimension.dhd.getGripperLinearVelocity()`
    :func:`forcedimension.dhd.getGripperAngularVelocityRad()`
    :func:`forcedimension.dhd.getGripperAngularVelocityDeg()`


    :param int ms:
        time interval used to compute velocity (in [ms]).

    :param int mode:
        velocity estimator mode (see [velocity estimator] modes section for
        details).

    :param int ID:
         Device ID (see multiple devices section for details).

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to C char.

    :raises ArgumentError:
        If ``ms`` is not implicitly convertible to C int.

    :raises ArgumentError:
        If ``mode`` is not implicitly convertible to C int.

    :returns:
        0 on success, -1 otherwise.
    """

    return _libdhd.dhdConfigAngularVelocity(ms, mode, ID)


_libdhd.dhdGetGripperLinearVelocity.argtypes = [c_double_ptr, c_byte]
_libdhd.dhdGetGripperLinearVelocity.restype = c_int


def getGripperLinearVelocity(out: c_double, ID: int = -1) -> int:
    """
    Retrieve the estimated linear velocity of the gripper (in [m/s]).

    Please note that the velocity estimator requires at least 2 position
    updates during the time interval defined in
    :func:`forcedimension.dhd.configLinearVelocity()` in order to be able to
    compute the estimate. Otherwise, e.g. if there are no calls to
    :func:`forcedimension.dhd.getPosition(), dhd.libdhd.getLinearVelocity()`,
    or :func:`forcedimension.dhd.getLinearVelocity()` will return an error
    (:data:`forcedimension.dhd.constants.ErrorNum.TIMEOUT`).

    See Also
    --------
    :data:`forcedimension.dhd.configGripperVelocity()`
    :data:`forcedimension.dhd.getGripperAngularVelocityRad()`
    :func:`forcedimension.dhd.getGripperAngularVelocityDeg()`


    :param c_double out:
        Output buffer to store the gripper linear velocity (in [m/s]).

    :param int ID:
         Device ID (see multiple devices section for details).

    :param SupportsPtr[c_double] out:
        An output buffer to store the gripper linear velocity (in [m/s]).

    :raises TypeError:
        If ``out`` does not support item assignment either
        because its immutable or not subscriptable.

    :raises IndexError:
        If ``len(out) < 3``.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to C char.

    :returns:
        0 on success, -1 otherwise.
    """

    return _libdhd.dhdGetGripperLinearVelocity(out, ID)


_libdhd.dhdGetGripperAngularVelocityRad.argtypes = [c_double_ptr, c_byte]
_libdhd.dhdGetGripperAngularVelocityRad.restype = c_int


def getGripperAngularVelocityRad(out: c_double, ID: int = -1) -> int:
    """
    Retrieve the estimated angular velocity of the gripper (in [rad/s]).

    Info
    ----
    The velocity estimator requires at least 2 position
    updates during the time interval defined in
    :func:`forcedimension.dhd.configGripperVelocity()` in order to be able to
    compute the estimate. Otherwise, e.g. if there are no calls to
    :func:`forcedimension.dhd.getPosition()`,
    :func:`forcedimension.dhd.getGripperAngularVelocityRad()`,
    or :func:`forcedimension.dhd.getGripperAngularVelocityRad()` will return an
    error (:data:`forcedimension.dhd.constants.ErrorNum.TIMEOUT`).

    See Also
    --------
    :func:`forcedimension.dhd.configGripperVelocity()`
    :func:`forcedimension.dhd.getGripperLinearVelocity()`
    :func:`forcedimension.dhd.getGripperAngularVelocityDeg()`


    :param int ID:
         Device ID (see multiple devices section for details).

    :param c_double out:
        An output buffer to store the gripper angular velocity (in [rad/s])

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to C char.

    :returns:
        0 on success, -1 otherwise.
    """

    return _libdhd.dhdGetGripperAngularVelocityRad(out, ID)


_libdhd.dhdGetGripperAngularVelocityDeg.argtypes = [c_double_ptr, c_byte]
_libdhd.dhdGetGripperAngularVelocityDeg.restype = c_int


def getGripperAngularVelocityDeg(out: c_double, ID: int = -1) -> int:
    """
    Retrieve the estimated angular velocity of the gripper (in [rad/s]).
    Velocity computation can be figured by calling:

    :func:`forcedimension.dhd.configGripperVelocity()`

    By default:
    :func:`forcedimension.dhd.constants.VELOCITY_WINDOW` and
    :func:`forcedimension.dhd.constant.VELOCITY_WINDOWING`
    are used. See velocity estimator for details.

    Please note that the velocity estimator requires at least 2 position
    updates during the time interval defined in:
    :func:`forcedimension.dhd.configGripperVelocity()`

    in order to be able to compute the estimate. Otherwise, e.g. if there are
    no calls to:
    :func:`forcedimension.dhd.getPosition()`
    :func:`forcedimension.dhd.getGripperLinearVelocity()`
    within the time interval window, this function will set an error with:
    :data:`forcedimension.dhd.constants.ErrorNum.TIMEOUT`

    See Also
    --------
    :func:`forcedimension.dhd.configGripperVelocity()`
    :func:`forcedimension.dhd.getGripperLinearVelocity()`
    :func:`forcedimension.dhd.getGripperAngularVelocityRad()`


    :param int ID:
         Device ID (see multiple devices section for details).

    :param c_double out:
        An output buffer to store the gripper angular velocity (in [deg/s]).

    :returns:
        0 on success, and -1 otherwise.
    """

    return _libdhd.dhdGetGripperAngularelocityDeg(out, ID)


_libdhd.dhdEmulateButton.argtypes = [c_bool, c_byte]
_libdhd.dhdEmulateButton.restype = c_int


def emulateButton(enable: bool, ID: int = -1) -> int:
    """
    Enable the button behavior emulation in devices that feature a gripper.

    This feature only applies to the following devices:
    :data:`forcedimension.dhd.constants.DeviceType.OMEGA331`
    :data:`forcedimension.dhd.constants.DeviceType.OMEGA331_LEFT`
    :data:`forcedimension.dhd.constants.DeviceType.SIGMA331`
    :data:`forcedimension.dhd.constants.DeviceType.SIGMA331_LEFT`

    For omega.7 devices with firmware versions 2.x, forces need to be enabled
    for the button emulation to report the emulated button status.

    :param enable:
        ``True`` to enable, ``False`` to disable.

    :param int ID:
         Device ID (see multiple devices section for details).

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to C char.

    :returns:
        0 on success, -1 otherwise.
    """

    return _libdhd.dhdEmulateButton(enable, ID)


_libdhd.dhdGetBaseAngleXRad.argtypes = [c_double_ptr, c_byte]
_libdhd.dhdGetBaseAngleXRad.restype = c_int


def getBaseAngleXRad(ID: int = -1) -> Tuple[float, int]:
    """
    Get the device base plate angle around the X axis (in [rad]).

    See Also
    --------
    :func:`forcedimension.dhd.getBaseAngleXDeg()`


    :param int ID:
         Device ID (see multiple devices section for details).

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to C char.

    :returns:
        A tuple in the form ``(x_angle, err)``. ``x_angle`` is the device angle
        (in [rad]). ``err`` is 0 on success, -1 otherwise.
    """

    angle = c_double()
    err = _libdhd.dhdGetBaseAngleXRad(angle, ID)
    return (angle.value, err)


_libdhd.dhdGetBaseAngleXDeg.argtypes = [c_double_ptr, c_byte]
_libdhd.dhdGetBaseAngleXDeg.restype = c_int


def getBaseAngleXDeg(ID: int = -1) -> Tuple[float, int]:
    """
    Get the device base plate angle around the X axis (in [deg]).

    See Also
    --------
    :func:`forcedimension.dhd.getBaseAngleXRad()`


    :param int ID:
         Device ID (see multiple devices section for details).

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to C char.

    :returns:
        A tuple in the form ``(err, x_angle)``. ``x_angle`` is the device angle
        (in [deg]) about the x axis. err`` is 0 on success, -1 otherwise.
    """

    angle = c_double()
    err = _libdhd.dhdGetBaseAngleXDeg(angle, ID)
    return (angle.value, err)


_libdhd.dhdSetBaseAngleXRad.argtypes = [c_double, c_byte]
_libdhd.dhdSetBaseAngleXRad.restype = c_int


def setBaseAngleXRad(angle: float, ID: int = -1) -> int:
    """
    Set the device base plate angle around the X axis (in [rad]).
    Please refer to your device user manual for more information on your device
    coordinate system.

    See Also
    --------
    :func:`forcedimension.dhd.setBaseAngleXDeg()`


    :param float angle:
        device base plate angle around the X axis (in [rad]).

    :param int ID:
         Device ID (see multiple devices section for details).

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to C char.

    :returns:
        0 on success, -1 otherwise.
    """

    return _libdhd.dhdSetBaseAngleXRad(angle, ID)


_libdhd.dhdSetBaseAngleXDeg.argtypes = [c_double, c_byte]
_libdhd.dhdSetBaseAngleXDeg.restype = c_int


def setBaseAngleXDeg(angle: float, ID: int = -1) -> int:
    """
    Set the device base plate angle around the X axis (in [deg]).
    Please refer to your device user manual for more information on your device
    coordinate system.

    See Also
    --------
    :func:`forcedimension.dhd.setBaseAngleXRad()`


    :param float angle:
        device base plate angle around the X axis (in [deg]).

    :param int ID:
         Device ID (see multiple devices section for details).

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to C char.

    :returns:
        0 on success, -1 otherwise.
    """

    return _libdhd.dhdSetBaseAngleXDeg(angle, ID)


_libdhd.dhdGetBaseAngleXRad.argtypes = [c_double_ptr, c_byte]
_libdhd.dhdGetBaseAngleXRad.restype = c_int


def getBaseAngleZRad(ID: int = -1) -> Tuple[float, int]:
    """
    Get the device base plate angle around the Z axis (in [rad]).

    See Also
    --------
    :func:`forcedimension.dhd.getBaseAngleZDeg()`


    :param int ID:
         Device ID (see multiple devices section for details).

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to C char.

    :returns:
        A tuple in the form ``(z_angle, err)``. ``z_angle`` is the device angle
        (in [rad]) about the Z axis. ``err`` is 0 on success, -1 otherwise.
    """

    angle = c_double()
    err = _libdhd.dhdGetBaseAngleZRad(angle, ID)
    return (angle.value, err)


_libdhd.dhdGetBaseAngleZDeg.argtypes = [c_double_ptr, c_byte]
_libdhd.dhdGetBaseAngleZDeg.restype = c_int


def getBaseAngleZDeg(ID: int = -1) -> Tuple[int, float]:
    """
    Get the device base plate angle around the Z axis (in [deg]).

    See Also
    --------
    :func:`forcedimension.dhd.getBaseAngleZRad()`


    :param int ID:
         Device ID (see multiple devices section for details).

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to C char.

    :returns:
        A tuple in the form ``(err, z_angle)``. ``z_angle`` is the device
        angle (in [deg]) about the Z axis.
        ``err`` is 0 on success, -1 otherwise.
    """

    angle = c_double()
    err = _libdhd.dhdGetBaseAngleZDeg(angle, ID)
    return (err, angle.value)


_libdhd.dhdSetBaseAngleZRad.argtypes = [c_double, c_byte]
_libdhd.dhdSetBaseAngleZRad.restype = c_int


def setBaseAngleZRad(angle: float, ID: int = -1) -> int:
    """
    Set the device base plate angle around the Z axis (in [rad]).
    Please refer to your device user manual for more information on your device
    coordinate system.

    See Also
    --------
    :func:`forcedimension.dhd.setBaseAngleZDeg()`


    :param float angle:
        device base plate angle around the Z axis (in [rad]).

    :param int ID:
         Device ID (see multiple devices section for details).

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to C char.

    :returns:
        0 on success, -1 otherwise.
    """

    return _libdhd.dhdSetBaseAngleZRad(angle, ID)


_libdhd.dhdSetBaseAngleZDeg.argtypes = [c_double, c_byte]
_libdhd.dhdSetBaseAngleZDeg.restype = c_int


def setBaseAngleZDeg(angle: float, ID: int = -1) -> int:
    """
    Set the device base plate angle around the Z axis (in [deg]).
    Please refer to your device user manual for more information on your device
    coordinate system.

    See Also
    --------
    :func:`forcedimension.dhd.setBaseAngleZRad()`


    :param float angle:
        device base plate angle around the Z axis (in [deg])

    :param int ID:
         Device ID (see multiple devices section for details).

    :raises ArgumentError:
        If ``angle`` is not implicitly convertible to C double

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to C char.

    :returns:
        0 on success, -1 otherwise.
    """

    return _libdhd.dhdSetBaseAngleZDeg(angle, ID)


_libdhd.dhdSetVibration.argtypes = [c_double, c_double, c_int, c_byte]
_libdhd.dhdSetVibration.restype = c_int


def setVibration(
    f: float, A: float, device_type: DeviceType, ID: int = -1
) -> int:
    """
    Apply a vibration to the end-effector. The vibration is added to the force
    requested by :func:`forcedimension.dhd.setForce()`.

    See Also
    --------
    :class:`forcedimension.dhd.adaptors.DeviceType`


    :param float f:
        Vibration frequency (in [Hz]).

    :param float A:
        Vibration amplitude (in [m]).

    :param DeviceType device_type:
        The type of device.

    :param int ID:
         Device ID (see multiple devices section for details).

    :raises ArgumentError:
        If ``f`` is not implicitly convertible to C double.

    :raises ArgumentError:
        If ``A`` is not implicitly convertible to C double.

    :raises ArgumentError:
        If ``device_type`` is not implicitly convertible to C int.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to C char.

    :returns:
        0 on success, -1 otherwise.
    """

    return _libdhd.dhdSetVibration(f, A, device_type, ID)


_libdhd.dhdSetMaxForce.argtypes = [c_double, c_byte]
_libdhd.dhdSetMaxForce.restype = c_int


def setMaxForce(limit: float, ID: int = -1) -> int:
    """
    Define a limit (in [N]) to the magnitude of the force that can be applied
    to the haptic device. The limit applies to all
    :func:`forcedimension.dhd.setForce()` and related calls, and ensures the
    force applied to the device end-effector remains below the requested value.
    If a negative limit is set, there is no max force and the full range of
    force can be applied.

    Note that the force limit enforced only applies to forces set
    programatically by dhd.libdhd.setForce(). Setting DAC values directly
    will bypass this limit.

    See Also
    --------
    :func:`forcedimension.dhd.getMaxForce()`
    :func:`forcedimension.dhd.setMaxTorque()`
    :func:`forcedimension.dhd.setMaxGripperForce()`
    :func:`forcedimension.dhd.expert.setMaxPower()`
    :func:`forcedimension.dhd.expert.setMaxUsablePower()`


    :param float limit:
        max magnitude of force that can be applied in [N].

    :param int ID:
         Device ID (see multiple devices section for details).

    :raises ArgumentError:
        If ``limit`` is not implicitly convertible to C double.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to C char.

    :returns:
        0 on success, -1 otherwise.
    """

    return _libdhd.dhdSetMaxForce(limit, ID)


_libdhd.dhdSetMaxTorque.argtypes = [c_double, c_byte]
_libdhd.dhdSetMaxTorque.restype = c_int


def setMaxTorque(limit: float, ID: int = -1) -> int:
    """
    Define a limit (in [Nm]) to the magnitude of the torque that can be applied
    to the haptic device. The limit applies to all
    :func:`forcedimension.dhd.libdhd.setTorque()` and
    related calls, and ensures the torque applied to the device end-effector
    remains below the requested value. If a negative limit is set, there is no
    max force and the full range of force can be applied.

    Note that the force limit enforced only applies to torques set
    programatically by :func:`forcedimension.dhd.setTorque()`.
    Setting DAC values directly will bypass this limit.

    See Also
    --------
    :func:`forcedimension.dhd.getMaxTorque()`
    :func:`forcedimension.dhd.setMaxForce()`
    :func:`forcedimension.dhd.setMaxGripperForce()`
    :func:`forcedimension.dhd.expert.setMaxPower()`
    :func:`forcedimension.dhd.expert.setMaxUsablePower()`


    :param float limit:
        max magnitude of torque that can be applied in [Nm].

    :param int ID:
         Device ID (see multiple devices section for details).

    :raises ArgumentError:
        If ``limit`` is not implicitly convertible to C double.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to C char.

    :returns:
        0 on success, -1 otherwise.
    """

    return _libdhd.dhdSetMaxTorque(limit, ID)


_libdhd.dhdSetMaxGripperForce.argtypes = [c_double, c_byte]
_libdhd.dhdSetMaxGripperForce.restype = c_int


def setMaxGripperForce(limit: float, ID: int = -1) -> int:
    """
    Define a limit (in [N]) to the magnitude of the force that can be applied
    to the haptic device gripper. The limit applies to all
    :func:`forcedimension.dhd.setForceAndTorqueAndGripperForce()` and related
    calls, and ensures the force applied to the device gripper remains below
    the requested value. If a negative limit is set, there is no max force and
    the full range of force can be applied.

    Note that the force limit enforced only applies to forces set
    programatically by dhd.libdhd.setForceAndTorqueAndGripperForce. Setting
    DAC values directly will bypass this limit.

    See Also
    --------
    :func:`forcedimension.dhd.getMaxGripperForce()`
    :func:`forcedimension.dhd.getMaxForce()`
    :func:`forcedimension.dhd.setMaxTorque()`
    :func:`forcedimension.dhd.expert.setMaxPower()`
    :func:`forcedimension.dhd.expert.setMaxUsablePower()`


    :param float limit:
        Max magnitude of force that can be applied in [N].

    :param int ID:
         Device ID (see multiple devices section for details).

    :raises ArgumentError:
        If ``limit`` is not implicitly convertible to C double.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to C char.

    :returns:
        0 on success, -1 otherwise.
    """

    return _libdhd.dhdSetMaxGripperForce(limit, ID)


_libdhd.dhdGetMaxForce.argtypes = [c_byte]
_libdhd.dhdGetMaxForce.restype = c_double


def getMaxForce(ID: int = -1) -> float:
    """
    Retrieve the current limit (in [N]) to the force magnitude that can be
    appliedby the haptic device. If the return value if negative, the limit is
    disabled and the full range of force available can be applied.

    See Also
    --------
    :func:`forcedimension.dhd.setMaxForce()`
    :func:`forcedimension.dhd.getMaxTorque()`
    :func:`forcedimension.dhd.getMaxGripperForce()`
    :func:`forcedimension.dhd.expert.etMaxPower()`
    :func:`forcedimension.dhd.expert.etMaxUsablePower()`


    :param int ID:
         Device ID (see multiple devices section for details).

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to C char.

    :returns:
        The current force limit (in N) if set, and ``-1.0`` if no limit is
        enforced.
    """

    return _libdhd.dhdGetMaxForce(ID)


_libdhd.dhdGetMaxForce.argtypes = [c_byte]
_libdhd.dhdGetMaxForce.restype = c_double


def getMaxTorque(ID: int = -1) -> float:
    """
    Retrieve the current limit (in [Nm]) to the force magnitude that can be
    appliedby the haptic device. If the return value if negative, the limit is
    disabled and the full range of force available can be applied.

    See Also
    --------
    :func:`forcedimension.dhd.setMaxTorque()`
    :func:`forcedimension.dhd.getMaxForce()`
    :func:`forcedimension.dhd.getMaxGripperForce()`
    :func:`forcedimension.dhd.expert.etMaxPower()`
    :func:`forcedimension.dhd.expert.etMaxUsablePower()`


    :param int ID:
         Device ID (see multiple devices section for details).

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to C char.

    :returns:
        The current torque limit (in [Nm]) if set, and ``-1.0`` if no limit is
        enforced.
    """

    return _libdhd.dhdGetMaxTorque(ID)


_libdhd.dhdGetMaxGripperForce.argtypes = [c_byte]
_libdhd.dhdGetMaxGripperForce.restype = c_double


def getMaxGripperForce(ID: int = -1) -> float:
    """
    Retrieve the current limit (in [N]) to the force magnitude that can be
    applied by the haptic device gripper. If the return value if negative, the
    limit is disabled and the full range of force available can be applied.

    See Also
    --------
    :func:`forcedimension.dhd.setMaxGripperForce()`
    :func:`forcedimension.dhd.getMaxForce()`
    :func:`forcedimension.dhd.getMaxTorque()`
    :func:`forcedimension.dhd.expert.etMaxPower()`
    :func:`forcedimension.dhd.expert.etMaxUsablePower()`


    :param int ID:
         Device ID (see multiple devices section for details).

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to C char.

    :returns:
        The current force limit (in [N]) if set, and
        ``-1.0`` if no limit is enforced.
    """

    return _libdhd.dhdGetMaxGripperForce(ID)


_libdhd.dhdErrorGetLast.argtypes = []
_libdhd.dhdErrorGetLast.restype = c_int


def errorGetLast() -> ErrorNum:
    """
     the last error code encountered in the running thread.

    See error management for details.

    :returns:
        :class:`forcedimension.dhd.adaptors.ErrorNum` enum elements refering
        to the last error encountered in the current thread.
    """
    return ErrorNum(_libdhd.dhdErrorGetLast())


_libdhd.dhdErrorGetLastStr.argtypes = []
_libdhd.dhdErrorGetLastStr.restype = c_char_p


def errorGetLastStr() -> str:
    """
    Get a brief string describing the last error encountered in the running
    thread.

    See error management for details.

    :returns:
        A string describing the last error encountered in the running thread.
    """
    return _libdhd.dhdErrorGetLastStr().decode('utf-8')


_libdhd.dhdErrorGetStr.argtypes = [c_int]
_libdhd.dhdErrorGetStr.restype = c_char_p


def errorGetStr(error: ErrorNum) -> str:
    """
    Get a brief string describing a given error code.

    See error management for details.

    :param ErrorNum error:
        the error code to describe.

    :returns:
        A string describing error.
    """
    return _libdhd.dhdErrorGetStr(error).decode('utf-8')
