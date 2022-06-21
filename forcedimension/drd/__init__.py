from ctypes import c_bool, c_byte, c_double, c_int, POINTER, byref

from typing import Sequence, MutableSequence, Tuple, List
from typing import Optional, cast
from forcedimension.dhd.adaptors import CartesianTuple
import forcedimension.runtime as runtime

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

    :param int ID: the device enumeration index, as assigned by the
    underlying operating system (must be between 0 and the number of devices
    connected to the system)

    :raises ValueError: if index is not convertible to a C int.

    :returns: The device ID on success, -1 otherwise
    :rtype: int
    """

    return _libdrd.drdOpenID(ID)


_libdrd.drdSetDevice.argtypes = [c_byte]
_libdrd.drdSetDevice.restype = c_int
def setDevice(ID: int) -> int:
    """
    Select the default device that will receive the API commands. The API
    supports multiple devices. This routine allows the programmer to decide
    which device the API dhd_single_device_call single-device calls will
    address. Any subsequent API call that does not specifically mention the
    device ID in its parameter list will be sent to that device.

    :param int ID:
        device ID (see multiple devices section for details), defaults to -1

    :raises ValueError: if ID is not implicitly convertible to C char.
    :rtype: int
    :returns: 0 on success, -1 otherwise.
    """
    return _libdrd.drdSetDevice(ID)


_libdrd.drdGetDeviceID.argtypes = []
_libdrd.drdGetDeviceID.restype = c_int
def getDeviceID() -> int:
    """
    :rtype: int
    :returns: The device ID on success, -1 otherwise.
    """
    return _libdrd.drdGetDeviceID()


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


_libdrd.drdIsSupported.argtypes = [c_byte]
_libdrd.drdIsSupported.restype = c_bool
def isSupported(ID: int) -> bool:
    """
    Determine if the device is supported out-of-the-boa by the DRD. The
    regulation gains of supported devices are configured internally so that
    such devices are ready to use. Unsupported devices can still be operated
    with the DRD, but their regulation gains must first be configured using the
    drdSetEncPGain(), drdSetEncIGain() and drdSetEncDGain() functions.

    :param int ID:
        device ID (see multiple devices section for details), defaults to -1

    :raises ValueError: if ID is not implicitly convertible to C char.
    :rtype: bool
    :returns: True if the device is supported, False otherwise

    """
    return _libdrd.drdIsSupported(ID)


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


_libdrd.drdIsMoving.argtypes = [c_byte]
_libdrd.drdIsMoving.restype = c_int
def isFiltering(ID: int = -1) -> bool:  # NOQA
    """
    Checks whether the particular robot control thread is applying a motion
    filter while tracking a target using
    :func:`forcedimension.drd.libdrd.trackPos()` or
    :func:`forcedimension.drd.libdrd.trackEnc()`),

    :param int ID: device ID, defaults to -1

    :raises ValueError: if ID is not convertible to a C char type

    :rtype: bool
    :returns: True if the motion filter is enabled, False otherwise.
    """

    return _libdrd.drdIsMoving(ID)


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


_libdrd.drdGetPositionAndOrientation.argtypes = [
    POINTER(c_double),
    POINTER(c_double),
    POINTER(c_double),
    POINTER(c_double),
    POINTER(c_double),
    POINTER(c_double),
    POINTER(c_double),
    (c_double * 3) * 3,
    c_byte
]
_libdrd.drdGetPositionAndOrientation.restype = c_int
def getPositionAndOrientationFrame( # NOQA
        ID: int = -1,
        p_out: Optional[MutableSequence[float]] = None,
        o_out: Optional[MutableSequence[float]] = None,
        matrix_out: Optional[MutableSequence[MutableSequence[float]]] = None
    ) -> Tuple[
            MutableSequence[float],
            MutableSequence[float],
            float,
            MutableSequence[MutableSequence[float]],
            int
    ]:
    """
    Retrieve the position (in Cartesian coordinates), angle of each joint
    (if applicable), gripper position, and orientation matrix of the
    end-effector. Please refer to your device user manual for more
    information on your device coordinate system.

    :raises TypeError: if matrix_out is specified and does not support item
    assignment, either because it is not subscriptable or because it is not
    mutable.

    :raises IndexError: if matrix_out is specified any dimension is less than
    length 3.

    :param int ID: [default=-1] device ID (see multiple devices section for
    details)

    :raises TypeError: if p_out is specified and does not support item
    assignment either because its immutable or not subscriptable.

    :raises IndexError: if p_out is specified and len(p_out) < 3

    :raises TypeError: if o_out is specified and does not support item
    assignment either because its immutable or not subscriptable.

    :raises IndexError: if o_out is specified and len(o_out) < 3

    :raises ValueError: if ID is not implicitly convertible to a C char type

    :returns:
        Tuple of ([px, py, pz], [oa, ob, og], pg, matrix, err) where err is 0
        or dhd.libdhd.TIMEGUARD on success, -1 otherwise. [px, py, pz] is the
        position about the X, Y, and Z axes, respectively in [m] and matrix is
        a 3x3 rotation matrix that describes the orientation of your device.

    :rtype: Tuple[
        MutableSequence[float],
        MutableSequence[float],
        float
        MutableSequence[MutableSequence[float]],
        int
    ]
    """

    px = c_double()
    py = c_double()
    pz = c_double()

    oa = c_double()
    ob = c_double()
    og = c_double()

    pg = c_double()

    matrix = ((c_double * 3) * 3)()

    err = _libdrd.drdGetPositionAndOrientationFrame(
                byref(px), byref(py), byref(pz),
                byref(oa), byref(ob), byref(og),
                byref(pg),
                byref(matrix),
                ID
          )

    if p_out is None:
        p_out = [px.value, py.value, pz.value]
    else:
        p_out[0] = px.value
        p_out[1] = py.value
        p_out[2] = pz.value

    if o_out is None:
        o_out = [oa.value, ob.value, og.value]
    else:
        o_out[0] = oa.value
        o_out[1] = ob.value
        o_out[2] = og.value

    if matrix_out is None:
        matrix_out = [list(row) for row in matrix]
    else:
        for i in range(3):
            for j in range(3):
                matrix_out[i][j] = matrix[i][j]

    return (p_out, o_out, pg, matrix_out, err)


_libdrd.drdGetVelocity.argtypes = [
    POINTER(c_double),
    POINTER(c_double),
    POINTER(c_double),
    POINTER(c_double),
    POINTER(c_double),
    POINTER(c_double),
    c_byte
]
_libdrd.drdGetVelocity.restype = c_int
def getLinearVelocity( # NOQA
        ID: int = -1,
        v_out: Optional[MutableSequence[float]] = None,
        w_out: Optional[MutableSequence[float]] = None,
    ) -> Tuple[MutableSequence[float], MutableSequence[float], int]:
    """
 	Retrieve the linear velocity of the end-effector position in Cartesian
    coordinates as well as the angular velocity about the X, Y, and Z axes.
    Please refer to your device user manual for more information on your
    device coordinate system.

    :param int ID:
        device ID (see multiple devices section for details), defaults to -1

    :param Optional[MutableSequence[float]] v_out:
        OPTIONAL mutable sequence to use instead of generating a new List. If
        this is specified, the mutable sequence provided will be updated with
        the new values and the return will be a reference to the same mutable
        sequence passed in.

    :param Optional[MutableSequence[float]] w_out:
        OPTIONAL mutable sequence to use instead of generating a new List.
        If this is specified, the mutable sequence provided will be updated
        with the new values and the return will be a reference to the same
        mutable sequence passed in.


    :raises TypeError:
        if v_out is specified and does not support item assignment either
        because its immutable or not support item assignment

    :raises IndexError: if v_out is specified and len(out) < 3

    :raises TypeError:
        if w_out is specified and does not support item assignment either
        because its immutable or does not support item assignment

    :raises IndexError: if w_out is specified and len(out) < 3

    :raises ValueError: if ID is not implicitly convertible to a C char type

    :returns:
        A tuple in the form ([vx, vy, vz], [wx, wy, wz], err) where
        err is either 0 on success and -1 otherwise. The units of
        wx, wy, wz are and [vx, vy, vz] are in [rad/s] and [m/s], respectively.

    :rtype: Tuple[MutableSequence[float], MutableSequence[float], int]
    """

    vx = c_double()
    vy = c_double()
    vz = c_double()

    wx = c_double()
    wy = c_double()
    wz = c_double()

    err = _libdrd.drdGetVelocity(
        byref(vx),
        byref(vy),
        byref(vz),
        byref(wx),
        byref(wy),
        byref(wz),
        ID
    )

    if v_out is None:
        v_out = [vx, vy, vz]
    else:
        v_out[0] = vx.value
        v_out[1] = vy.value
        v_out[2] = vz.value

    if w_out is None:
        w_out = [wx, wy, wz]
    else:
        w_out[0] = wx.value
        w_out[1] = wy.value
        w_out[2] = wz.value

        return (v_out, w_out, err)


_libdrd.drdGetCtrlFreq.argtypes = [c_byte]
_libdrd.drdGetCtrlFreq.restype = c_int
def getComFreq(ID: int = -1) -> int:  # NOQA
    """
    This function returns the average refresh rate of the control loop (in kHz)
    since the function was last called.

    :param int ID:
        device ID (see multiple devices section for details), defaults to -1

    :raises ValueError: if ID is not convertible to a C char type

    :rtype: int
    :returns: 0 on success, -1 otherwise
    """

    return _libdrd.drdGetCtrlFreq(ID)


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


_libdrd.drdRegulatePos.argtypes = [c_bool, c_byte]
_libdrd.drdRegulatePos.restype = c_int
def regulatePos(enable: bool, ID: int = -1) -> int:  # NOQA
    """
    Enable/disable robotic regulation of the device delta base, which provides
    translations. If regulation is disabled, the base can move freely and
    will display any force set using drdSetForceAndTorqueAndGripperForce().
    If it is enabled, base position is locked and can be controlled by calling
    all robotic functions (e.g. drdMoveToPos()). By default, delta base
    regulation is enabled.

    :param bool enable: True to enable base regulation, False to disable it.

    :param int ID:
        device ID (see multiple devices section for details), defaults to -1

    :raises ValueError: if val is not implicitly convertible to C bool
    :raises ValueError: if ID is not implicitly convertible to C int

    :rtype: int
    :returns: 0 on success, -1 otherwise.
    """

    return _libdrd.drdRegulatePos(enable, ID)


_libdrd.drdRegulateRot.argtypes = [c_bool, c_byte]
_libdrd.drdRegulateRot.restype = c_int
def regulateRot(enable: bool, ID: int = -1) -> int: # NOQA
    """
    Enable/disable robotic regulation of the device wrist. If regulation is
    disabled, the wrist can move freely and will display any torque set using
    drdSetForceAndTorqueAndGripperForce(). If it is enabled, wrist orientation
    is locked and can be controlled by calling all robotic functions
    (e.g. drdMoveTo()). By default, wrist regulation is enabled.

    :param bool enable: True to enable wrist regulation, False to disable it.

    :param int ID:
        device ID (see multiple devices section for details), defaults to -1

    :raises ValueError: if val is not implicitly convertible to C bool
    :raises ValueError: if ID is not implicitly convertible to C int

    :rtype: int
    :returns: 0 on success, -1 otherwise.
    """

    return _libdrd.drdRegulateRot(enable, ID)


_libdrd.drdRegulateGrip.argtypes = [c_bool, c_byte]
_libdrd.drdRegulateGrip.restype = c_int
def regulateGrip(enable: bool, ID: int = -1) -> int: # NOQA
    """
    Enable/disable robotic regulation of the device gripper. If regulation is
    disabled, the gripper can move freely and will display any force set using
    drdSetForceAndTorqueAndGripperForce(). If it is enabled, gripper
    orientation is locked and can be controlled by calling all robotic
    functions (e.g. drdMoveTo()). By default, gripper regulation is enabled.

    :param bool enable: True to enable gripper regulation, False to disable it.

    :param int ID:
        device ID (see multiple devices section for details), defaults to -1

    :raises ValueError: if val is not implicitly convertible to C bool
    :raises ValueError: if ID is not implicitly convertible to C int

    :rtype: int
    :returns: 0 on success, -1 otherwise.
    """

    return _libdrd.drdRegulateGrip(enable, ID)


_libdrd.drdEnableFilter.argtypes = [c_bool, c_byte]
_libdrd.drdEnableFilter.restype = c_int
def enableFilter(enabled: bool, ID: int = -1) -> int:
    """
    Enable or disable motion filtering for subsequent calls to
    :func:`forcedimension.drd.libdrd.trackPos()` and
    :func:`forcedimension.drd.libdrd.trackEnc()`

    :param bool enabled: True to enable motion filtering, False to disable it

    :param int ID:
        Device ID (see multiple devices section for details), defaults to -1.

    :raises ValueError: if enabled is not implicitly convertible to C bool
    :raises ValueError: if ID is not implicitly convertible to C int
    """
    return _libdrd.drdEnableFilter(enabled, ID)


_libdrd.drdMoveToPos.argtypes = [c_double, c_double, c_double, c_bool, c_byte]
_libdrd.drdMoveToPos.restype = c_int
def moveToPos(pos: CartesianTuple, block: bool, ID: int = -1):  # NOQA
    """
    Send the robot end-effector to a desired Cartesian position. The motion
    follows a straight line, with smooth acceleration/deceleration. The
    acceleration and velocity profiles can be controlled by adjusting the
    trajectory generation parameters.

    :param int: ID [default=-1]

    :param bool block: if true, the call blocks until the destination is
    reached. If false, the call returns immediately.

    :param CartesianTuple pos:
        A tuple of x, y, and z coordinates to move the end effector to.

    :raises ValueError: if ID is not convertible to a C char type

    :rtype: int
    :returns: 0 on success, -1 otherwise

    """
    return _libdrd.drdMoveToPos(pos[0], pos[1], pos[2], block, ID)


_libdrd.drdMoveToRot.argtypes = [c_double, c_double, c_double, c_bool, c_byte]
_libdrd.drdMoveToRot.restype = c_int
def moveToRot(o: CartesianTuple, block: bool, ID: int = -1):  # NOQA
    """
    Send the robot end-effector to a desired Cartesian rotation. The motion
    follows a straight curve, with smooth acceleration/deceleration. The
    acceleration and velocity profiles can be controlled by adjusting the
    trajectory generation parameters.

    :param int ID:
        device ID (see multiple devices section for details), defaults to -1

    :param bool block: if true, the call blocks until the destination is
    reached. If false, the call returns immediately.

    :param CartesianTuple o:
        A tuple of angles about the first, second, and third joints in [rad]

    :raises ValueError: if ID is not convertible to a C char type

    :rtype: int
    :returns: 0 on success, -1 otherwise

    """
    return _libdrd.drdMoveToRot(o[0], o[1], o[2], block, ID)


_libdrd.drdMoveToGrip.argtypes = [c_double, c_bool, c_byte]
_libdrd.drdMoveToGrip.restype = c_int
def moveToGrip(pg: float, block: bool, ID: int = -1):  # NOQA
    """
    Send the robot gripper to a desired opening distance. The motion is
    executed with smooth acceleration/deceleration. The acceleration and
    velocity profiles can be controlled by adjusting the trajectory
    generation parameters.

    :param int ID:
        device ID (see multiple devices section for details), defaults to -1

    :param bool block: if true, the call blocks until the destination is
    reached. If false, the call returns immediately.

    :param float pg: target gripper opening distance in [m]

    :raises ValueError: if ID is not convertible to a C char type

    :rtype: int
    :returns: 0 on success, -1 otherwise

    """
    return _libdrd.drdMoveToGrip(pg, block, ID)


_libdrd.drdMoveTo.argtypes = [c_double * 7, c_bool, c_byte]
_libdrd.drdMoveTo.restype = c_int
def moveTo(p: Sequence[float], block: bool, ID: int = -1):  # NOQA
    """
    Send the robot end-effector to a desired Cartesian 7-DOF configuration.
    The motion uses smooth acceleration/deceleration. The acceleration and
    velocity profiles can be controlled by adjusting the trajectory generation
    parameters.

    :param float p: target positions/orientations in [m] / [rad], respectively

    :param int ID:
        device ID (see multiple devices section for details), defaults to -1

    :raises ValueError: if ID is not convertible to a C char type

    :rtype: int
    :returns: 0 on success, -1 otherwise
    """

    return _libdrd.drdMoveTo((c_double * 7)(*p), block, ID)


_libdrd.drdMoveToEnc.argtypes = [c_int, c_int, c_int, c_bool, c_byte]
_libdrd.drdMoveToEnc.restype = c_int
def moveToEnc(enc: Sequence[int], block: bool, ID: int = -1) -> int:
    """
    Send the robot end-effector to a desired encoder position. The motion
    follows a straight line in the encoder space, with smooth
    acceleration/deceleration. The acceleration and velocity profiles can be
    controlled by adjusting the trajectory generation parameters.

    :param int enc: a sequence of (enc0, enc1, enc2) where enc0, enc1, and
        enc2 are the target encoder position on axis 0, 1, and 2.

    :param bool block:
        If true, the call blocks until the destination is reached. If false,
        the call returns immediately.

    :param int ID:
        device ID (see multiple devices section for details), defaults to -1

    :raises ValueError:
        if any of enc0, enc1, or enc2 are not implicitly convertible to
        C int

    :raises ValueError: if ID is not convertible to a C char type

    :rtype: int
    :returns: 0 on success, -1 otherwise
    """

    return _libdrd.drdMoveToEnc(enc[0], enc[1], enc[2], block, ID)


_libdrd.drdMoveToAllEnc = [c_int * 7, c_bool, c_byte]
_libdrd.drdMoveToAllEnc = c_int
def moveToAllEnc(enc: Sequence[int], block: bool, ID: int = -1):
    """
    Send the robot end-effector to a desired encoder position. The motion
    follows a straight line in the encoder space, with smooth
    acceleration/deceleration. The acceleration and velocity profiles can be
    controlled by adjusting the trajectory generation parameters.


    :param int enc: target encoder positions

    :param bool block:
        If true, the call blocks until the destination is reached. If false,
        the call returns immediately.

    :param int ID:
        device ID (see multiple devices section for details), defaults to -1

    :raises ValueError:
        if any of elements of enc are not implicitly convertible to C int

    :raises ValueError: if ID is not convertible to a C char type

    :rtype: int
    :returns: 0 on success, -1 otherwise
    """
    return _libdrd.drdMoveToAllEnc((c_int * 7)(*enc), block, ID)


_libdrd.drdLock.argtypes = [c_byte]
_libdrd.drdLock.restype = c_int
def hold(ID: int = -1) -> int:  # NOQA
    """
    Depending on the value of the mask parameter, either:

    - move the device to its park position and engage the locks, or
    - removes the locks

    This function only applies to devices equipped with mechanical locks, and
    will return an error when called on other devices.

    :param int ID:
        device ID (see multiple devices section for details), defaults to -1

    :raises ValueError: if ID is not convertible to a C char type

    :rtype: int
    :returns: 0 on success, -1 otherwise
    """

    return _libdrd.drdLock(ID)


_libdrd.drdStop.argtypes = [c_byte]
_libdrd.drdStop.restype = c_int
def stop(ID: int = -1) -> int:  # NOQA
    """
    Stop the robotic control loop for the given robot.

    :param int ID:
        device ID (see multiple devices section for details), defaults to -1

    :raises ValueError: if ID is not convertible to a C char type

    :rtype: int
    :returns: 0 on success, -1 otherwise
    """

    return _libdrd.drdStop(ID)


_libdrd.drdGetPriorities.argtypes = [c_byte]
_libdrd.drdGetPriorities.restype = c_int
def getPriorities(ID: int = -1) -> Tuple[int, int, int]:
    """
    This function makes it possible to retrieve the priority of the control
    thread and the calling thread. Thread priority is system dependent, as
    described in thread priorities.

    :param int ID:
        device ID (see multiple devices section for details), defaults to -1

    :raises ValueError: if ID is not convertible to a C char type

    :rtype: Tuple[int, int, int]
    :returns: Tuple of (err, prio, ctrlprio) where err is 0 on success, -1
        otherwise. prio is the calling thread's priority level and ctrlprio is
        the control thread priority level. These values are system independent.
    """
    prio = c_int()
    ctrlprio = c_int()

    ret: int = _libdrd.drdGetPriorities(byref(prio), byref(ctrlprio), ID)

    return (ret, prio.value, ctrlprio.value)


_libdrd.drdSetPriorities.argtypes = [c_int, c_int, c_byte]
_libdrd.drdSetPriorities.restype = c_int
def setPriorities(prio: int, ctrlprio: int, ID: int = -1) -> int:
    """
    This function makes it possible to adjust the priority of the control
    thread and the calling thread. Thread priority is system dependent, as
    described in thread priorities.


    Note
    ----
    Please keep in mind that administrator/superuser access is required on many
        platforms in order to increase thread priority. The first argument
        (prio) is always applied to the calling thread, even when the call
        returns an error.

    :param int prio: calling thread priority level (value is system dependent)

    :param int ctrlprio: control thread priority level
        (value is system dependent)

    :param int ID:
        device ID (see multiple devices section for details), defaults to -1

    :raises ValueError: if prio is not convertible to a C int type
    :raises ValueError: if ctrlprio is not convertible to a C int type
    :raises ValueError: if ID is not convertible to a C char type

    :rtype: int
    :returns: 0 on success, -1 otherwise.
    """

    return _libdrd.drdSetPriorities(prio, ctrlprio, ID)


_libdrd.drdSetEncPGain.argtypes = [c_double, c_byte]
_libdrd.drdSetEncPGain.restype = c_int
def getEncPGain(gain: float, ID: int = -1) -> int:
    """
    Set the P term of the PID controller that regulates the base joint
    positions. In practice, this affects the stiffness of the regulation.

    :param float gain: P parameter of the PID regulator
    :param int ID:
        device ID (see multiple devices section for details), defaults to -1

    :raises ValueError: if gain is not convertible to a C double type
    :raises ValueError: if ID is not convertible to a C char type

    :rtype: int
    :returns: 0 on success, -1 otherwise.
    """

    return _libdrd.drdSetEncPGain(gain, ID)


_libdrd.drdGetEncPGain.argtypes = [c_byte]
_libdrd.drdGetEncPGain.restype = c_double
def setEncPGain(ID: int = -1) -> int:
    """
    Retrieve the P term of the PID controller that regulates the base joint
    positions.

    :param int ID:
        device ID (see multiple devices section for details), defaults to -1

    :raises ValueError: if ID is not convertible to a C char type

    :rtype: int
    :returns: The P term of the PID controller that regulates the base joint
        positions.
    """

    return _libdrd.drdGetEncPGain(ID)


_libdrd.drdSetEncIGain.argtypes = [c_double, c_byte]
_libdrd.drdSetEncIGain.restype = c_int
def setEncIGain(gain: float, ID: int = -1) -> int:
    """
    Set the I term of the PID controller that regulates the base joint
    positions. In practice, this affects the precision of the regulation.

    :param float gain: I parameter of the PID regulator
    :param int ID:
        device ID (see multiple devices section for details), defaults to -1

    :raises ValueError: if gain is not convertible to a C double type
    :raises ValueError: if ID is not convertible to a C char type

    :rtype: int
    :returns: 0 on success, -1 otherwise.
    """

    return _libdrd.drdSetEncIGain(gain, ID)


_libdrd.drdGetEncIGain.argtypes = [c_byte]
_libdrd.drdGetEncIGain.restype = c_double
def getEncIGain(ID: int = -1) -> int:
    """
    Retrieve the P term of the PID controller that regulates the base joint
    positions.

    :param int ID:
        device ID (see multiple devices section for details), defaults to -1

    :raises ValueError: if ID is not convertible to a C char type

    :rtype: int
    :returns: The I term of the PID controller that regulates the base joint
        positions.
    """

    return _libdrd.drdGetEncIGain(ID)


_libdrd.drdSetEncDGain.argtypes = [c_double, c_byte]
_libdrd.drdSetEncDGain.restype = c_int
def setEncDGain(gain: float, ID: int = -1) -> int:
    """
    Set the D term of the PID controller that regulates the base joint
    positions. In practice, this affects the velocity of the regulation.

    :param float gain: D parameter of the PID regulator
    :param int ID:
        device ID (see multiple devices section for details), defaults to -1

    :raises ValueError: if gain is not convertible to a C double type
    :raises ValueError: if ID is not convertible to a C char type

    :rtype: int
    :returns: 0 on success, -1 otherwise.
    """

    return _libdrd.drdSetEncDGain(gain, ID)


_libdrd.drdGetEncDGain.argtypes = [c_int]
_libdrd.drdGetEncDGain.restype = c_double
def getEncDGain(ID: int = -1) -> int:
    """
    Retrieve the D term of the PID controller that regulates the base joint
    positions.

    :param int ID:
        device ID (see multiple devices section for details), defaults to -1

    :raises ValueError: if ID is not convertible to a C char type

    :rtype: int
    :returns: The D term of the PID controller that regulates the base joint
        positions.
    """

    return _libdrd.drdGetEncDGain(ID)


_libdrd.drdSetMotRatioMax.argtypes = [c_double, c_byte]
_libdrd.drdSetMotRatioMax.restype = c_int
def setMotRatioMax(scale: float, ID: int = -1) -> int:
    """
    Set the maximum joint torque applied to all regulated joints expressed as
    a fraction of the maximum torque available for each joint.

    In practice, this limits the maximum regulation torque (in joint space),
    making it potentially safer to operate in environments where humans or
    delicate obstacles are present.

    :param float scale:
        the joint torque scaling factor (must be between 0.0 and 1.0)

    :param int ID:
        device ID (see multiple devices section for details), defaults to -1

    :raises ValueError: if scale is not convertible to a C double type
    :raises ValueError: if ID is not convertible to a C char type

    :rtype: int
    :returns: 0 on success, -1 otherwise.
    """

    return _libdrd.drdSetMotRatioMax(scale, ID)


_libdrd.drdGetMotRatioMax.argtypes = [c_byte]
_libdrd.drdGetMotRatioMax.restype = c_double
def setMotRatioMax(scale: float, ID: int = -1) -> float:
    """
    Retrieve the maximum joint torque applied to all regulated joints expressed
    as a fraction of the maximum torque available for each joint.

    :param int ID:
        device ID (see multiple devices section for details), defaults to -1

    :raises ValueError: if ID is not convertible to a C char type

    :rtype: float
    :returns: The maximum joint torque ratio (between 0.0 and 1.0)
    """

    return _libdrd.drdGetMotRatioMax(ID)


_libdrd.drdSetEncMoveParam.argtypes = [c_double, c_double, c_double, c_byte]
_libdrd.drdSetEncMoveParam.restype = c_int
def setEncoderMoveParam(
    vmax: float, amax: float, jerk: float, ID: int = -1
) -> int:
    """
    Set encoder positioning trajectory generation parameters.

    :param float amax: max acceleration [m/s^2]
    :param float vmax: max velocity [m/s]
    :param float jerk: jerk [m/s^3]

    :rtype: int
    :returns: 0 on success, -1 otherwise
    """

    return _libdrd.drdSetEncMoveParam(amax, vmax, jerk, ID)
