from typing import Tuple, Union

import ctypes

from ctypes import (
    c_int, c_uint, c_bool, c_byte, c_ubyte, c_ushort, c_char_p, c_double
)

from ctypes import byref, POINTER
import forcedimension.runtime as runtime

# It's better to import everything like this,
# so linters can have an easier time.
from forcedimension.dhd.constants import (  # NOQA
    Device, ComMode, StatusIndex, Error, ThreadPriority,
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


def setDevice(ID=-1: int) -> int:  # NOQA
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


def getSerialNumber(ID=-1: int) -> Tuple[int, int]:  # NOQA
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


def close(ID=-1: int) -> int:  # NOQA
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


def stop(ID=-1: int) -> int:  # NOQA
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
def getComMode(ID=-1: int) -> int:  # NOQA
    """
    Retrive the COM operation mode on compatible devices.

    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if ID is not implicitly convertible to C char.

    :rtype: int
    :returns: the current COM operation mode on success, -1 otherwise.
    """

    return _libdhd.dhdGetComMode(ID)


_libdhd.dhdEnableForce.argtypes = [c_ubyte, c_byte]
_libdhd.dhdEnableForce.restype = c_int
def enableForce(val: bool, ID=-1: int) -> int: # NOQA
    """
    Enable the force mode in the device controller.

    :param bool val: True to enable force, False to disable it.
    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if ID is not implicitly convertible to C int
    :raises ValueError: if val is not implicitly convertible to C unsigned
    char.

    :rtype: int
    :returns: 0 on success, -1 otherwise.
    """

    return _libdhd.dhdEnableForce(val, c_byte)


_libdhd.dhdEnableGripperForce.argtypes = [c_ubyte, c_byte]
_libdhd.dhdEnableGripperForce.restype = c_int
def enableGripperForce(val: bool, ID=-1: int) -> int: # NOQA
    """
    Enable the force mode in the device controller.

    :param bool val: True to enable force, False to disable it.
    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if ID is not implicitly convertible to C int
    :raises ValueError: if val is not implicitly convertible to C unsigned
    char.

    :rtype: int
    :returns: 0 on success, -1 otherwise.
    """

    return _libdhd.dhdEnableGripperForce(val, ID)


_libdhd.dhdGetSystemType.argtypes = [c_byte]
_libdhd.dhdGetSystemType.restype = c_int
def getSystemType(ID=-1: int) -> int:  # NOQA
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
def getSystemName(ID=-1: int) -> Union[str, None]:  # NOQA
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
def getVersion(ID=-1: int) -> Tuple[float, int]: # NOQA
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
def getStatus(ID=-1: int) -> Tuple[list(int), int]: # NOQA
    """
    Returns the status list of the haptic device. The status is described in
    the status section.

    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if ID is not convertible to a C char type

    :rtype: Tuple[list(int), int]
    :returns: Tuple of (status_lst, err). err is 0 on success, -1 otherwise.
    """

    status_vec = (c_int * MAX_STATUS)()
    err = _libdhd.dhdGetStatus(status_vec, ID)

    return (list(status_vec), err)


_libdhd.dhdGetAngleRad.argtypes = [POINTER(c_double), c_byte]
_libdhd.dhdGetAngleRad.restype = c_int
def getAngleRad(ID=-1: int) -> Tuple[float, int]: # NOQA
    """
    Get the device base plate angle around the Y axis in radians.

    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if ID is not convertible to a C char type

    :rtype: Tuple[float, int]
    :returns: Tuple of (angle_rad, err). err is 0 on success, -1 otherwise.
    """

    angle_rad = c_double()
    return (angle_rad.value, _libdhd.dhdGetAngleRad(byref(angle_rad), ID))


_libdhd.dhdGetAngleDeg.argtypes = [POINTER(c_double), c_byte]
_libdhd.dhdGetAngleDeg.restype = c_int
def getAngleDeg(ID=-1: int) -> Tuple[float, int]: # NOQA
    """
    Get the device base plate angle around the Y axis in degrees.

    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if ID is not convertible to a C char type

    :rtype: Tuple[float, int]
    :returns: Tuple of (angle_deg, err). err is 0 on success, -1 otherwise.
    """

    angle_deg = c_double()
    return (angle_deg.value, _libdhd.dhdGetAngleDeg(byref(angle_deg), ID))


_libdhd.dhdGetEffectorMass.argtypes = [POINTER(c_double), c_byte]
_libdhd.dhdGetEffectorMass.restype = c_int
def getEffectorMass(ID=-1: int) -> Tuple[float, int]: # NOQA
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
def getButton(index: int, ID=-1: int) -> Union[int, State]: # NOQA
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
def getButtonMask(ID=-1: int) -> int: # NOQA
    """

    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if ID is not implicitly convertible to a C char type

    :rtype: int
    :returns: 32-bit long bitmask. Each bit is set to 1 if the button is
    pressed, 0 otherwise.
    """

    return _libdhd.dhdGetButtonMask(ID)
