import forcedimension.runtime as runtime  # type: ignore
from forcedimension.dhd.adaptors import CartesianTuple  # type: ignore

from ctypes import c_int, c_bool, c_byte, c_double

# Load the runtime from the backend
_libdrd = runtime.load("libdrd")

if _libdrd is None:
    raise ImportError("There were problems loading libdrd.")

_libdrd.drdOpen.argtypes = []
_libdrd.drdOpen.restype = c_int
def open() -> int:  # NOQA
    """
    Open a connection to the first available device connected to the
    system. The order in which devices are opened persists until devices
    are added or removed.

    If this call is successful, the default device ID is set to the newly
    opened device. See the multiple device section for more information on
    using multiple devices on the same computer.

    See also dhd.libdrd.openID()

    :rtype: int
    :returns: The device ID on success, -1 otherwise.
    """
    return _libdrd.drdOpen()


_libdrd.drdOpenID.argtypes = [c_int]
_libdrd.drdOpenID.restype = c_int
def openID(ID: int) -> int:  # NOQA
    """
    Open a connection to one particular device connected to the system. The
    order in which devices are opened persists until devices are added or
    removed. If the device at the specified index is already opened, its device
    ID is returned.

    If this call is successful, the default device ID is set to the newly
    opened device. See the multiple device section for more information on
    using multiple devices on the same computer.

    See also dhd.libdrd.open()

    :param in index: the device enumeration index, as assigned by the
    underlying operating system (must be between 0 and the number of devices
    connected to the system)

    :raises ValueError: if index is not convertible to a C int.

    :returns: The device ID on success, -1 otherwise
    :rtype: int
    """

    return _libdrd.drdOpenID(ID)


_libdrd.drdClose.argtypes = [c_byte]
_libdrd.drdClose.restype = c_int
def close(ID: int = -1) -> int:  # NOQA
    """
    Close the connection to a particular device.

    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises ValueError: if ID is not implicitly convertible to C char.
    :rtype: int
    :returns: 0 on success, -1 otherwise.
    """
    return _libdrd.drdClose(ID)


_libdrd.drdSetDevice.argtypes = [c_byte]
_libdrd.drdSetDevice.restype = None
def enableSimulator(ID: int = -1) -> int:  # NOQA
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

    return _libdrd.drdEnableSimulator(ID)


_libdrd.drdIsInitialized.argtypes = [c_byte]
_libdrd.drdIsInitialized.restype = c_bool
def isInitialized(ID: int = -1) -> bool:  # NOQA
    """
    Checks the initialization status of a particular robot. The initialization
    status reflects the status of the controller RESET LED.
    The robot can be (re)initialized by calling
    :func:`forcedimension.drd.libdrd.autoInit()`.

    :param int: ID [default=-1]

    :raises ValueError: if ID is not convertible to a C char type

    :rtype: bool
    :returns: True if the robot is initialized, False otherwise
    """

    return _libdrd.drdIsInitialized(ID)


_libdrd.drdIsMoving.argtypes = [c_byte]
_libdrd.drdIsMoving.restype = c_int
def isMoving(ID: int = -1) -> bool:  # NOQA
    """
    Checks whether the particular robot is moving (following a call to
    :func:`forcedimension.drd.libdrd.moveToPos()`,
    :func:`forcedimension.drd.libdrd.moveToEnc()`,
    :func:`forcedimension.drd.libdrd.trackPos()` or
    :func:`forcedimension.drd.libdrd.trackEnc()`),
    as opposed to holding the target position after successfully reaching it.

    :param int: ID [default=-1]

    :raises ValueError: if ID is not convertible to a C char type

    :rtype: bool
    :returns: True if the robot is moving, False otherwise
    """

    return _libdrd.drdIsMoving(ID)


_libdrd.drdIsRunning.argtypes = [c_byte]
_libdrd.drdIsRunning.restype = c_bool
def isRunning(ID: int = -1) -> bool:  # NOQA
    """
    Checks the state of the robotic control thread for a particular device.

    :param int: ID [default=-1]

    :raises ValueError: if ID is not convertible to a C char type

    :rtype: bool
    :returns: True if the control thread is running, False otherwise
    """

    return _libdrd.drdIsRunning(ID)


_libdrd.drdIsSupported.argtypes = [c_byte]
_libdrd.drdIsSupported.restype = c_int
def isSupported(ID: int = -1) -> int:  # NOQA
    """
    Determine if the device is supported out-of-the-box by the DRD.
    The regulation gains of supported devices are configured internally so
    that such devices are ready to use. Unsupported devices can still be
    operated with the DRD, but their regulation gains must first be configured
    using the drdSetEncPGain(), drdSetEncIGain() and drdSetEncDGain()
    functions.

    :param int: ID [default=-1]

    :raises ValueError: if ID is not convertible to a C char type

    :rtype: int
    :returns: 0 on success, -1 otherwise
    """

    return _libdrd.drdIsSupported(ID)


_libdrd.drdAutoInit.argtypes = [c_byte]
_libdrd.drdAutoInit.restype = c_int
def autoInit(ID: int = -1) -> int:  # NOQA
    """
    Performs automatic initialization of that particular robot by robotically
    moving to a known position and reseting encoder counters to their correct
    values.

    :param int: ID [default=-1]

    :raises ValueError: if ID is not convertible to a C char type

    :rtype: int
    :returns: 0 on success, -1 otherwise
    """

    return _libdrd.drdAutoInit(ID)


_libdrd.drdCheckInit.argtypes = [c_byte]
_libdrd.drdCheckInit.restype = c_int
def checkInit(ID: int = -1) -> int:  # NOQA
    """
    Check the validity of that particular robot initialization by robotically
    sweeping all endstops and comparing their joint space position to expected
    values (stored in each device internal memory). If the robot is not yet
    initialized, this function will first perform the same initialization
    routine as drdAutoInit() before running the endstop check.

    :param int: ID [default=-1]

    :raises ValueError: if ID is not convertible to a C char type

    :rtype: int
    :returns: 0 on success, -1 otherwise
    """

    return _libdrd.drdCheckInit(ID)


_libdrd.drdStart.argtypes = [c_byte]
_libdrd.drdStart.restype = c_int
def start(ID: int = -1) -> int:  # NOQA
    """
    Start the robotic control loop for the given robot. The robot must be
    initialized (either manually or with
    :func:`forcedimension.drd.libdrd.autoInit()`) before
    :func:`forcedimension.drd.libdrd.drdStart()` can be called successfully.

    :param int: ID [default=-1]

    :raises ValueError: if ID is not convertible to a C char type

    :rtype: int
    :returns: 0 on success, -1 otherwise
    """

    return _libdrd.drdStart(ID)


_libdrd.drdStop.argtypes = [c_byte]
_libdrd.drdStop.restype = c_int
def stop(ID: int = -1) -> int:  # NOQA
    """
    Stop the robotic control loop for the given robot.

    :param int: ID [default=-1]

    :raises ValueError: if ID is not convertible to a C char type

    :rtype: int
    :returns: 0 on success, -1 otherwise
    """

    return _libdrd.drdStop(ID)


_libdrd.drdMoveToPos.argtypes = [c_double, c_double, c_double, c_bool, c_byte]
_libdrd.drdMoveToPos.restype = c_int
def moveToPos(pos: CartesianTuple, block: bool, ID: int = -1):  # NOQA
    """
    Send the robot end-effector to a desired Cartesian position. The motion
    follows a straight line, with smooth acceleration/deceleration. The
    acceleration and velocity profiles can be controlled by adjusting the
    trajectory generation parameters.

    :param int: ID [default=-1]
    :param CartesianTuple pos:
        A tuple of x, y, and z coordinates to move the end effector to.

    :raises ValueError: if ID is not convertible to a C char type

    :rtype: int
    :returns: 0 on success, -1 otherwise

    """
    return _libdrd.drdMoveToPos(pos[0], pos[1], pos[2], block, ID)


_libdrd.drdHold.argtypes = [c_byte]
_libdrd.drdHold.restype = c_int
def hold(ID: int = -1) -> int:  # NOQA
    """
    Immediately make the robot hold its current position. All motion commands
    are abandoned. Determine if the device is supported out-of-the-box by
    the DRD.

    :param int: ID [default=-1]

    :raises ValueError: if ID is not convertible to a C char type

    :rtype: int
    :returns: 0 on success, -1 otherwise
    """

    return _libdrd.drdHold(ID)
