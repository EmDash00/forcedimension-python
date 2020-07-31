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

    :rtype: Tuple[int, int]
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

    :rtype: Tuple[int, int]
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


_libdhd.dhdSetDeltaMotor.argtypes = [c_ushort, c_ushort, c_ushort, c_byte]
_libdhd.dhdSetDeltaMotor.restype = c_int
def setDeltaMotor(output: Tuple[int, int, int], ID: int = -1) -> int: # NOQA
    """
    Set desired motor commands to the amplifier channels commanding the DELTA
    motors.

    :param Tuple[int, int, int] output: Tuple of (motor0, motor1, motor2) where
    motor0, motor1, and motor2 are the axis 0, 1, and 2 DELTA motor commands,
    respectively.

    :param int ID: [default=-1] device ID (see multiple devices section
    for details)

    :raises ValueError: if any member of output is not implicitly convertible
    to C ushort.

    :raises ValueError: if ID is not implicitly convertible to C char

    :rtype: int
    :returns: 0 on success, -1 otherwise.

    """
    return _libdhd.dhdSetDeltaMotor(output[0], output[1], output[2], ID)


_libdhd.dhdSetWristMotor.argtypes = [c_ushort, c_ushort, c_ushort, c_byte]
_libdhd.dhdSetWristMotor.restype = c_int
def setWristMotor(output: Tuple[int, int, int], ID: int = -1) -> int: # NOQA
    """
    Set desired motor commands to the amplifier channels commanding the wrist
    motors.

    :param Tuple[int, int, int] output: Tuple of (motor0, motor1, motor2) where
    motor0, motor1, and motor2 are the axis 0, 1, and 2 wrist motor commands,
    respectively.

    :param int ID: [default=-1] device ID (see multiple devices section
    for details)

    :raises ValueError: if any member of output is not implicitly convertible
    to C ushort.

    :raises ValueError: if ID is not implicitly convertible to C char

    :rtype: int
    :returns: 0 on success, -1 otherwise.

    """
    return _libdhd.dhdSetWristMotor(output[0], output[1], output[2], ID)


_libdhd.dhdSetWristMotor.argtypes = [c_ushort, c_byte]
_libdhd.dhdSetWristMotor.restype = c_int
def setGripperMotor(output: int, ID: int = -1) -> int: # NOQA
    """
    Set desired motor commands to the amplifier channels commanding the force
    gripper.

    :param int output: gripper motor command

    :param int ID: [default=-1] device ID (see multiple devices section
    for details)

    :raises ValueError: if output is not implicitly convertible to C ushort.

    :raises ValueError: if ID is not implicitly convertible to C char

    :rtype: int
    :returns: 0 on success, -1 otherwise.
    """
    return _libdhd.dhdSetWristMotor(output, ID)


_libdhd.dhdDeltaEncoderToPosition.argtypes = [
    c_int,
    c_int,
    c_int,
    POINTER(c_double),
    POINTER(c_double),
    POINTER(c_double),
    c_byte
]
_libdhd.dhdDeltaEncoderToPosition.restype = c_int
def deltaEncoderToPosition(enc: Tuple[int, int, int], # NOQA
                           ID: int = -1) -> Tuple[List[float], int]:
    """
    This routine computes and returns the position of the end-effector for a
    given set of encoder readings.

    :param Tuple[int, int, int] enc: tuple of (enc0, enc1, enc2) where enc0,
    enc1, and enc2 coresspond to encoder readings on axis 0, 1, and 2,
    respectively.

    :param int ID: [default=-1] device ID (see multiple devices section
    for details)

    :raises ValueError: if any member of enc is not implicitly convertible
    to C int.

    :raises ValueError: if ID is not implicitly convertible to C char

    :rtype: int
    :returns: tuple of ([px, py, pz], err) where err is 0 on success,
    -1 otherwise and [px, py, pz] is a list of floats corresponding to the
    DELTA end-effector position on the X, Y, and Z axes, respectively in [m].
    """

    px = c_double()
    py = c_double()
    pz = c_double()

    return ([px.value, py.value, pz.value],
            _libdhd.dhdDeltaEncoderToPosition(
                    enc[0],
                    enc[1],
                    enc[2],
                    byref(px),
                    byref(py),
                    byref(pz)
                )
            )


_libdhd.dhdDeltaPositionToEncoder.argtypes = [
    c_int,
    c_int,
    c_int,
    POINTER(c_double),
    POINTER(c_double),
    POINTER(c_double),
    c_byte
]
_libdhd.dhdDeltaPositionToEncoder.restype = c_int
def deltaPositionToEncoder(pos: Tuple[float, float, float], # NOQA
                           ID: int = -1) -> Tuple[List[int], int]:
    """
    This routine computes and returns the encoder values of the end-effector
    for a given Cartesian end-effector position.

    :param Tuple[float, float, float] pos: tuple of (px, py, pz) where px, py,
    and pz correspond to the end-effector position on the X, Y, and Z axes,
    respectively in [m].

    :param int ID: [default=-1] device ID (see multiple devices section
    for details)

    :raises ValueError: if any member of pos is not implicitly convertible
    to C double

    :raises ValueError: if ID is not implicitly convertible to C char

    :rtype: Tuple[List[int], int]
    :returns: tuple of ([enc0, enc1, enc2], err) where err is 0 on success,
    -1 otherwise and [enc0, enc1, enc2] is a list of floats corresponding to
    the DELTA end-effector encoder readings on axis 0, 1, and 2, respectively.
    """

    enc0 = c_int()
    enc1 = c_int()
    enc2 = c_int()

    return ([enc0.value, enc1.value, enc2.value],
            _libdhd.dhdDeltaEncoderToPosition(
                    pos[0],
                    pos[1],
                    pos[2],
                    byref(enc0),
                    byref(enc1),
                    byref(enc2)
                )
            )


_libdhd.dhdDeltaMotorToForce.argtypes = [
    c_ushort,
    c_ushort,
    c_ushort,
    c_int,
    c_int,
    c_int,
    POINTER(c_double),
    POINTER(c_double),
    POINTER(c_double),
    c_byte
]
_libdhd.dhdDeltaMotorToForce.restype = c_int
def deltaMotorToForce(output: Tuple[int, int, int], # NOQA
                      enc: Tuple[int, int, int],
                      ID: int = -1) -> Tuple[List[float], int]:
    """
    This routine computes and returns the force applied to the end-effector for
    a given set of motor commands at a given position (defined by encoder
    readings)

    :param Tuple[int, int, int] output: Tuple of (motor0, motor1, motor2) where
    motor0, motor1, and motor2 are the axis 0, 1, and 2 DELTA motor commands,
    respectively.

    :param Tuple[int, int, int] enc: tuple of (enc0, enc1, enc2) where enc0,
    enc1, and enc2 coresspond to encoder readings on axis 0, 1, and 2,
    respectively.

    :param int ID: [default=-1] device ID (see multiple devices section
    for details)

    :raises ValueError: if any member of output is not implicitly convertible
    to C ushort.

    :raises ValueError: if any member of enc is not implicitly convertible to
    C int.

    :raises ValueError: if ID is not implicitly convertible to C char

    :rtype: Tuple[List[float], int]
    :returns: tuple of ([fx, fy, fz], err) where err is 0 on success, -1
    otherwise and [fx, fy, fz] correspond to the translational force on the
    DELTA end-effector on the X, Y and Z axes, respectively in [N].
    """

    fx = c_double()
    fy = c_double()
    fz = c_double()

    return ([fx.value, fy.value, fz.value],
            _libdhd.dhdDeltaMotorToForce(
                output[0],
                output[1],
                output[2],
                enc[0],
                enc[1],
                enc[2],
                byref(fx),
                byref(fy),
                byref(fz)
        )
    )


_libdhd.dhdDeltaForceToMotor.argtypes = [
    c_double,
    c_double,
    c_double,
    c_int,
    c_int,
    c_int,
    POINTER(c_ushort),
    POINTER(c_ushort),
    POINTER(c_ushort),
    c_byte
]
_libdhd.dhdDeltaForceToMotor.restype = c_int
def deltaForceToMotor(f: Tuple[float, float, float], # NOQA
                      enc: Tuple[int, int, int],
                      ID: int = -1) -> Tuple[List[int], int]:
    """
    This routine computes and returns the motor commands necessary to obtain a
    given force on the end-effector at a given position (defined by encoder
    readings).

    :param Tuple[float, float, float] f: Tuple of (fx, fy, fz) where fx, fy,
    and fz are the force on the DELTA end-effector on the X, Y, and Z axes,
    respectively in [N]

    :param Tuple[int, int, int] enc: tuple of (enc0, enc1, enc2) where enc0,
    enc1, and enc2 coresspond to encoder readings on axis 0, 1, and 2,
    respectively.

    :param int ID: [default=-1] device ID (see multiple devices section
    for details)

    :raises ValueError: if any member of f is not implicitly convertible to C
    double

    :raises ValueError: if any member of output is not implicitly convertible
    to C ushort.

    :raises ValueError: if ID is not implicitly convertible to C char

    :rtype: Tuple[List[int], int]
    :returns: tuple of ([output0, output1, output2], err) where err is 0 or
    dhd.bindings.constants.MOTOR_SATURATED on success, -1 otherwise and
    [output0, output1, output2] correspond to the DELTA end-effector motor
    commands on axes 0, 1, and 2, respectively.
    """

    output0 = c_ushort()
    output1 = c_ushort()
    output2 = c_ushort()

    return ([output0.value, output1.value, output2.value],
            _libdhd.dhdDeltaMotorToForce(
                f[0],
                f[1],
                f[2],
                enc[0],
                enc[1],
                enc[2],
                byref(output0),
                byref(output1),
                byref(output2)
        )
    )


_libdhd.dhdWristEncoderToOrientation.argtypes = [
    c_int,
    c_int,
    c_int,
    POINTER(c_double),
    POINTER(c_double),
    POINTER(c_double),
    c_byte
]
_libdhd.dhdWristEncoderToOrientation.restype = c_int
def wristEncoderToOrientation(enc: Tuple[int, int, int], # NOQA
                              ID: int = -1) -> Tuple[List[float], int]:
    """
    For devices with a wrist structure, compute the individual angle of each
    joint, starting with the one located nearest to the wrist base plate. For
    the dhd.bindings.constants.DeviceType.OMEGA33 and the
    dhd.bindings.constants.DeviceType.OMEGA33_LEFT devices, angles are computed
    with respect to their internal reference frame, which is rotated 45 degrees
    or pi/4 radians about the Y axis. Please refer to your device user manual
    for more information on your device coordinate system.

    This feature only applies to the following devices:
        dhd.bindings.constants.DeviceType.OMEGA33
        dhd.bindings.constants.DeviceType.OMEGA33_LEFT
        dhd.bindings.constants.DeviceType.OMEGA331
        dhd.bindings.constants.DeviceType.OMEGA331_LEFT
        dhd.bindings.constants.DeviceType.SIGMA331
        dhd.bindings.constants.DeviceType.SIGMA331_LEFT

    :param Tuple[int, int, int] enc: tuple of (enc0, enc1, enc2) where enc0,
    enc1, and enc2 coresspond to wrist encoder readings on the first, second,
    and third joint, respectively

    :param int ID: [default=-1] device ID (see multiple devices section
    for details)

    :raises ValueError: if any member of enc is not implicitly convertible
    to C int.

    :raises ValueError: if ID is not implicitly convertible to C char

    :rtype: Tuple[List[float], int]
    :returns: tuple of ([oa, ob, og], err) where err is 0 on success,
    -1 otherwise and [oa, ob, og] is a list of floats corresponding to the
    wrist end-effector's orientation about the first, second, and third wrist
    joint, respectively in [rad].
    """

    px = c_double()
    py = c_double()
    pz = c_double()

    return ([px.value, py.value, pz.value],
            _libdhd.dhdDeltaEncoderToPosition(
                    enc[0],
                    enc[1],
                    enc[2],
                    byref(px),
                    byref(py),
                    byref(pz)
                )
            )


_libdhd.dhdWristOrientationToEncoder.argtypes = [
    c_double,
    c_double,
    c_double,
    POINTER(c_int),
    POINTER(c_int),
    POINTER(c_int),
    c_byte
]
_libdhd.dhdWristOrientationToEncoder.restype = c_int
def wristOrientationToEncoder(orientation: Tuple[float, float, float], # NOQA
                              ID: int = -1) -> Tuple[List[int], int]:
    """
    For devices with a wrist structure, compute the encoder values from the
    individual angle of each joint, starting witht he one located nearest to
    the wrist plate base. For the dhd.bindings.DeviceType.OMEGA33 and
    dhd.bindings.DeviceType.OMEGA33_LEFT devices, angles must be expressed
    withrespect to their internal reference frame, which is rotated 45 degrees
    or pi/4 radians about the Y axis. Please refer to your device user manual
    for more information on your device coordinate system.

    This feature only applies to the following devices:
        dhd.bindings.constants.DeviceType.OMEGA33
        dhd.bindings.constants.DeviceType.OMEGA33_LEFT
        dhd.bindings.constants.DeviceType.OMEGA331
        dhd.bindings.constants.DeviceType.OMEGA331_LEFT
        dhd.bindings.constants.DeviceType.SIGMA331
        dhd.bindings.constants.DeviceType.SIGMA331_LEFT

    :param Tuple[float, float, float] orientation: tuple of (oa, ob, og) where
    (oa, ob, og) coresspond to wrist end effector orientation around the X, Y,
    and Z axes, respectively in [rad].

    :param int ID: [default=-1] device ID (see multiple devices section
    for details)

    :raises ValueError: if any member of enc is not implicitly convertible
    to C int.

    :raises ValueError: if ID is not implicitly convertible to C char

    :rtype: Tuple[List[int], int]
    :returns: tuple of ([enc0, enc1, enc2], err) where err is 0 on success,
    -1 otherwise and [enc0, enc1, enc2] is a list of ints corresponding to
    the wrist end-effector orientation around the X, Y, and Z axes,
    respectively.
    """

    enc0 = c_int()
    enc1 = c_int()
    enc2 = c_int()

    return ([enc0.value, enc1.value, enc2.value],
            _libdhd.dhdDeltaEncoderToPosition(
                    orientation[0],
                    orientation[1],
                    orientation[2],
                    byref(enc0),
                    byref(enc1),
                    byref(enc2)
                )
            )


_libdhd.dhdWristMotorToTorque.argtypes = [
    c_ushort,
    c_ushort,
    c_ushort,
    c_int,
    c_int,
    c_int,
    POINTER(c_double),
    POINTER(c_double),
    POINTER(c_double),
    c_byte
]
_libdhd.dhdWristMotorToTorque.restype = c_int
def wristMotorToTorque(output: Tuple[int, int, int], # NOQA
                       enc: Tuple[int, int, int],
                       ID: int = -1) -> Tuple[List[float], int]:
    """
    This routine computes and returns the torque applied to the wrist
    end-effector for a given set of motor commands at a given orientation
    (defined by encoder readings)

    :param Tuple[int, int, int] output: Tuple of (motor0, motor1, motor2) where
    motor0, motor1, and motor2 are the axis 0, 1, and 2 DELTA motor commands,
    respectively.

    :param Tuple[int, int, int] enc: tuple of (enc0, enc1, enc2) where enc0,
    enc1, and enc2 coresspond to encoder readings on axis 0, 1, and 2,
    respectively.

    :param int ID: [default=-1] device ID (see multiple devices section
    for details)

    :raises ValueError: if any member of output is not implicitly convertible
    to C ushort.

    :raises ValueError: if any member of enc is not implicitly convertible to
    C int.

    :raises ValueError: if ID is not implicitly convertible to C char

    :rtype: Tuple[List[float], int]
    :returns: tuple of ([tx, ty, tz], err) where err is 0 on success, -1
    otherwise and [tx, ty, tz] correspond to the torque on the wrist
    end-effector around the X, Y and Z axes, respectively in [Nm].
    """

    tx = c_double()
    ty = c_double()
    tz = c_double()

    return ([tx.value, ty.value, tz.value],
            _libdhd.dhdDeltaMotorToForce(
                output[0],
                output[1],
                output[2],
                enc[0],
                enc[1],
                enc[2],
                byref(tx),
                byref(ty),
                byref(tz)
        )
    )


_libdhd.dhdWristTorqueToMotor.argtypes = [
    c_double,
    c_double,
    c_double,
    c_int,
    c_int,
    c_int,
    POINTER(c_ushort),
    POINTER(c_ushort),
    POINTER(c_ushort),
    c_byte
]
_libdhd.dhdWristTorqueToMotor.restype = c_int
def wristTorqueToMotor(t: Tuple[float, float, float], # NOQA
                       enc: Tuple[int, int, int],
                       ID: int = -1) -> Tuple[List[int], int]:
    """
    This routine computes and returns the wrist motor commands necessary to
    obtain a given torque on the wrist end-effector at a given orientation
    (defined by encoder readings)

    :param Tuple[int, int, int] output: Tuple of (motor0, motor1, motor2) where
    motor0, motor1, and motor2 are the axis 0, 1, and 2 DELTA motor commands,
    respectively.

    :param Tuple[int, int, int] enc: tuple of (enc0, enc1, enc2) where enc0,
    enc1, and enc2 coresspond to encoder readings on axis 0, 1, and 2,
    respectively.

    :param int ID: [default=-1] device ID (see multiple devices section
    for details)

    :raises ValueError: if any member of t is not implicitly convertible
    to C double.

    :raises ValueError: if any member of enc is not implicitly convertible to
    C int.

    :raises ValueError: if ID is not implicitly convertible to C char

    :rtype: Tuple[List[int], int]
    :returns: tuple of ([output0, output1, output2], err) where err is 0 on
    success, -1 otherwise and [output0, output1, output2] correspond to the
    motor commands on the wrist end-effector around wrist joint 0, 1, and 2,
    respectively.
    """

    output0 = c_ushort()
    output1 = c_ushort()
    output2 = c_ushort()

    return ([output0.value, output1.value, output2.value],
            _libdhd.dhdDeltaMotorToForce(
                t[0],
                t[1],
                t[2],
                enc[0],
                enc[1],
                enc[2],
                byref(output0),
                byref(output1),
                byref(output2)
        )
    )


_libdhd.dhdGripperEncoderToAngleRad.argtypes = [
    c_int,
    POINTER(c_double),
    c_byte
]
_libdhd.dhdGripperEncoderToAngleRad.restype = c_int
def gripperEncoderToAngleRad(enc: int, # NOQA
                             ID: int = -1) -> Tuple[float, int]:
    """
    This routine computes and returns the opening of the gripper as an angle in
    [rad] for a given encoder reading.

    This feature only applies to the following devices:
        dhd.bindings.constants.DeviceType.OMEGA331
        dhd.bindings.constants.DeviceType.OMEGA331_LEFT
        dhd.bindings.constants.DeviceType.SIGMA331
        dhd.bindings.constants.DeviceType.SIGMA331_LEFT

    :param int enc: gripper encoder reading

    :param int ID: [default=-1] device ID (see multiple devices section
    for details)

    :raises ValueError: if enc is not implicitly convertible to C int.
    :raises ValueError: if ID is not implicitly convertible to C char

    :rtype: Tuple[float, int]
    :returns: tuple of (angle, err) where err is 0 or
    dhd.bindings.constants.TIMEGUARD on success, -1 on otherwise and angle is
    the gripper opening in [rad]
    """

    angle = c_double()

    return (angle.value, _libdhd.dhdGripperEncoderToAngleRad(
                            enc,
                            byref(angle),
                            ID
                        )
            )


_libdhd.dhdGripperEncoderToGap.argtypes = [
    c_int,
    POINTER(c_double),
    c_byte
]
_libdhd.dhdGripperEncoderToGap.restype = c_int
def gripperEncoderToGap(enc: int, ID: int = -1) -> Tuple[float, int]: # NOQA
    """
    This routine computes and returns the opening of the gripper as a distance
    in [m] for a given encoder reading.

    This feature only applies to the following devices:
        dhd.bindings.constants.DeviceType.OMEGA331
        dhd.bindings.constants.DeviceType.OMEGA331_LEFT
        dhd.bindings.constants.DeviceType.SIGMA331
        dhd.bindings.constants.DeviceType.SIGMA331_LEFT


    :param int enc: gripper encoder reading

    :param int ID: [default=-1] device ID (see multiple devices section
    for details)

    :raises ValueError: if enc is not implicitly convertible to C int.
    :raises ValueError: if ID is not implicitly convertible to C char

    :rtype: Tuple[float, int]
    :returns: tuple of (gap, err) where err is 0 on success, -1 otherwise and
    gap is the gripper opening in [m]
    """

    gap = c_double()

    return (gap.value, _libdhd.dhdGripperEncoderToGap(
                            enc,
                            byref(gap),
                            ID
                        )
            )


_libdhd.dhdGripperAngleRadToEncoder.argtypes = [
    c_double,
    POINTER(c_int),
    c_byte
]
_libdhd.dhdGripperAngleRadToEncoder.restype = c_int
def gripperAngleRadToEncoder(angle: float, # NOQA
                             ID: int = -1) -> Tuple[int, int]:
    """
    This routine computes and returns the gripper encoder value for a given
    gripper opening distance in [rad].

    This feature only applies to the following devices:
        dhd.bindings.constants.DeviceType.OMEGA331
        dhd.bindings.constants.DeviceType.OMEGA331_LEFT
        dhd.bindings.constants.DeviceType.SIGMA331
        dhd.bindings.constants.DeviceType.SIGMA331_LEFT

    :param float angle: gripper opening as an angle in [rad]

    :param int ID: [default=-1] device ID (see multiple devices section
    for details)

    :raises ValueError: if angle is not implicitly convertible to C float
    :raises ValueError: if ID is not implicitly convertible to C char

    :rtype: Tuple[int, int]
    :returns: tuple of (enc, err) where err is 0 on success, -1 on otherwise
    and enc is the gripper encoder reading
    """

    enc = c_int()

    return (enc.value, _libdhd.dhdGripperEncoderToAngleRad(
                            angle,
                            byref(enc),
                            ID
                        )
            )


_libdhd.dhdGripperGapToEncoder.argtypes = [
    c_double,
    POINTER(c_int),
    c_byte
]
_libdhd.dhdGripperEncoderToGap.restype = c_int
def gripperGapToEncoder(gap: float, ID: int = -1) -> Tuple[int, int]: # NOQA
    """
    This routine computes and returns the gripper encoder value for a given
    gripper opening distance in [m].

    This feature only applies to the following devices:
        dhd.bindings.constants.DeviceType.OMEGA331
        dhd.bindings.constants.DeviceType.OMEGA331_LEFT
        dhd.bindings.constants.DeviceType.SIGMA331
        dhd.bindings.constants.DeviceType.SIGMA331_LEFT


    :param float gap: gripper opening distance in [m]

    :param int ID: [default=-1] device ID (see multiple devices section
    for details)

    :raises ValueError: if gap is not implicitly convertible to C double
    :raises ValueError: if ID is not implicitly convertible to C char

    :rtype: Tuple[int, int]
    :returns: tuple of (enc, err) where err is 0 on success, -1 otherwise and
    enc is the gripper encoder reading
    """

    enc = c_int()

    return (enc.value, _libdhd.dhdGripperEncoderToGap(
                            gap,
                            byref(enc),
                            ID
                        )
            )


_libdhd.dhdGripperMotorToForce.argtypes = [
    c_ushort,
    POINTER(c_double),
    c_int * 4,
    c_byte
]
_libdhd.dhdGripperMotorToForce.restype = c_int
def gripperMotorToForce(output: int,  # NOQA
                        enc_wrist: Tuple[int, int, int],
                        enc_gripper: int,
                        ID: int = -1) -> Tuple[float, int]:
    """
    This routine computes the force applied to the end-effector for a given
    motor command.

    This feature only applies to the following devices:
        dhd.bindings.constants.DeviceType.OMEGA331
        dhd.bindings.constants.DeviceType.OMEGA331_LEFT
        dhd.bindings.constants.DeviceType.SIGMA331
        dhd.bindings.constants.DeviceType.SIGMA331_LEFT


    :param int output: motor command on gripper axis

    :param Tuple[int, int, int] enc_wrist: tuple of (enc0, enc1, enc2) where
    (enc0, enc1, enc2) are encoder readings about joint 0, 1, and 2,
    respectively.

    :param int enc_gripper: encoder reading for the gripper.

    :param int ID: [default=-1] device ID (see multiple devices section
    for details)

    :raises ValueError: if output is not implicitly convertible to C ushort.

    :raises ValueError: if a member of enc_wrist is not implicitly convertible
    to C int

    :raises ValueError: if enc_gripper is not implicitly convertible to C int

    :raises ValueError: if ID is not implicitly convertible to C char

    :rtype: Tuple[float, int]
    :returns: tuple of (force, err) where err is 0 on success, -1 otherwise and
    force is the force on the gripper end-effector in [N]
    """

    force = c_double()

    enc = [enc_wrist[0], enc_wrist[1], enc_wrist[2], enc_gripper]

    return (force.value, _libdhd.dhdGripperMotorToForce(
                            output,
                            byref(force),
                            enc,
                            ID
                        )
            )


_libdhd.dhdGripperForceToMotor.argtypes = [
    c_double,
    POINTER(c_ushort),
    c_int * 4,
    c_byte
]
_libdhd.dhdGripperForceToMotor.restype = c_int
def gripperForceToMotor(force: float,  # NOQA
                        enc_wrist: Tuple[int, int, int],
                        enc_gripper: int,
                        ID: int = -1) -> Tuple[int, int]:
    """
    Given a desired force to be displayed by the force gripper, this routine
    computes and returns the corresponding motor command.

    This feature only applies to the following devices:
        dhd.bindings.constants.DeviceType.OMEGA331
        dhd.bindings.constants.DeviceType.OMEGA331_LEFT
        dhd.bindings.constants.DeviceType.SIGMA331
        dhd.bindings.constants.DeviceType.SIGMA331_LEFT


    :param int force: force on the gripper end-effector in [N]

    :param Tuple[int, int, int] enc_wrist: tuple of (enc0, enc1, enc2) where
    (enc0, enc1, enc2) are encoder readings about joint 0, 1, and 2,
    respectively.

    :param int enc_gripper: encoder reading for the gripper.

    :param int ID: [default=-1] device ID (see multiple devices section
    for details)

    :raises ValueError: if force is not implicitly convertible to C double

    :raises ValueError: if a member of enc_wrist is not implicitly convertible
    to C int

    :raises ValueError: if enc_gripper is not implicitly convertible to C int

    :raises ValueError: if ID is not implicitly convertible to C char

    :rtype: Tuple[int, int]
    :returns: tuple of (output, err) where err is 0 or
    dhd.bindings.constants.MOTOR_SATURATED on success, -1 otherwise and output
    is the motor command on the gripper axis.
    """

    output = c_ushort()

    enc = [enc_wrist[0], enc_wrist[1], enc_wrist[2], enc_gripper]

    return (output.value, _libdhd.dhdGripperMotorToForce(
                            force,
                            byref(output),
                            enc,
                            ID
                        )
            )


_libdhd.dhdSetMot.argtypes = [c_ushort * MAX_DOF, c_ubyte, c_byte]
_libdhd.dhdSetMot.restype = c_int
def setMot(outputs: DofTuple, mask: int = 0xff, ID: int = -1) -> int: # NOQA
    """
    Program motor commands to a selection of motor channels. Particularly
    useful when using the generic controller directly, without a device model
    attached.

    :param DofTuple outputs: list of motor command values
    :param int mask: [default=0xff] bitwise mask of which motor should be set
    :param int ID: [default=-1] device ID (see multiple devices section
    for details)

    :raises ValueError: if any member of outputs is not implicitly
    convertible to C int

    :raises ValueError: if mask is not implicitly convertible to C uchar

    :raises ValueError: if ID is not implicitly convertible to C char

    :rtype: int
    :returns: 0 on success, -1 otherwise
    """

    return _libdhd.dhdSetMot(outputs, mask, ID)


_libdhd.dhdPreloadMot.argtypes = [c_ushort * MAX_DOF, c_ubyte, c_byte]
_libdhd.dhdPreloadMot.restype = c_int
def preloadMot(outputs: DofTuple, mask: int = 0xff, ID: int = -1) -> int: # NOQA
    """
    Program motor commands to a selection of motor channels. Unlike
    dhd.bindings.expert.setMot, this function saves the requested commands
    internally for later application by calling dhd.bindings.standard.setForce
    and the likes.

    :param DofTuple outputs: list of motor command values
    :param int mask: [default=0xff] bitwise mask of which motor should be set
    :param int ID: [default=-1] device ID (see multiple devices section
    for details)

    :raises ValueError: if any member of outputs is not implicitly
    convertible to C int

    :raises ValueError: if mask is not implicitly convertible to C uchar

    :raises ValueError: if ID is not implicitly convertible to C char

    :rtype: int
    :returns: 0 on success, -1 otherwise
    """

    return _libdhd.dhdPreloadMot(outputs, mask, ID)


_libdhd.dhdGetEnc.argtypes = [c_int * MAX_DOF, c_ubyte, c_byte]
_libdhd.dhdGetEnc.restype = c_int
def getEnc(mask: int, ID: int = -1) -> Tuple[List[int], int]: # NOQA
    """
    Get a selective list of encoder values. Particularly useful when using the
    generic controller directly, without a device model attached.

    :param int mask: [default=0xff] bitwise mask of which motor should be set

    :param int ID: [default=-1] device ID (see multiple devices section
    for details)

    :raises ValueError: if mask is not implicitly convertible to C uchar
    :raises ValueError: if ID is not implicitly convertible to C char

    :rtype: Tuple[List[int], int]
    :returns: tuple of (enc, err) where err is 0 or
    dhd.bindings.constants.TIMEGUARD on success, -1 otherwise and enc is a list
    of encoder values.
    """
    enc = (c_int * MAX_DOF)()

    err = _libdhd.dhdGetEncoder(enc, mask, ID)

    return ([val for val in enc], err)


_libdhd.dhdGetEncRange.argtypes = [c_int * MAX_DOF, c_int * MAX_DOF, c_byte]
_libdhd.dhdGetEncRange.restype = c_int
def getEncRange(ID: int = -1) -> Tuple[List[int], List[int], int]: # NOQA
    """
    Get the expected min and max encoder values for all axes present on the
    current device. Axis indices that do not exist on the device will return
    a range of 0.

    :param int ID: [default=-1] device ID (see multiple devices section
    for details)

    :raises ValueError: if ID is not implicitly convertible to C char

    :rtype: Tuple[List[int], List[int], int]
    :returns: tuple of (encMin, encMax err) where err is 0 on success, -1
    otherwise and encMin and encMax are a list of minimum and maximum encoder
    values for each axis, respectively.
    """

    encMin = (c_int * MAX_DOF)()
    encMax = (c_int * MAX_DOF)()

    err = _libdhd.dhdGetEncRange(encMin, encMax, ID)

    return ([val for val in encMin], [val for val in encMax], err)


_libdhd.dhdSetBrk.argtypes = [c_ubyte, c_byte]
_libdhd.dhdSetBrk.restype = c_int
def setBrk(mask: int = 0xff, ID: int = -1) -> int: # NOQA
    """
    Set electromagnetic braking status on selective motor groups. Only applies
    when using the generic controller directly, without a device model
    attached.

    The motors on the generic controller are grouped as follows:
        group1 - [mot0, mot1, mot2]
        group2 - [mot3, mot4, mot5]
        group3 - [mot6]
        group4 - [mot7]

    The mask parameter addresses all 8 motors bitwise. If a single bit within
    the motor group is enabled, the entire motor group's electromagnetic
    brakes will be activated.

    :param mask: [default=0xff] bitwise mask of which motor groups should have
    their electromagnetic brakes be set on.

    :param int ID: [default=-1] device ID (see multiple devices section
    for details)

    :raises ValueError: if mask is not implicitly convertible to C uchar
    :raises ValueError: if ID is not implicitly convertible to C char

    :rtype: int
    :returns: 0 on success, -1 otherwise
    """

    return _libdhd.dhdSetBrk(mask, ID)


_libdhd.dhdGetDeltaJointAngles.argtypes = [
    POINTER(c_double),
    POINTER(c_double),
    POINTER(c_double),
    c_byte
]
_libdhd.dhdGetDeltaJointAngles.restype = c_int
def getDeltaJointAngles(ID: int = -1) -> Tuple[List[float], int]: # NOQA
    """
    Retrieve the joint angles in [rad] for the DELTA structure.

    :param int ID: [default=-1] device ID (see multiple devices section
    for details)

    :raises ValueError: if ID is not implicitly convertible to C char

    :rtype: Tuple[List[float], int]
    :returns: tuple of ([j0, j1, j2], err) where err is 0 or
    dhd.bindings.constants.TIMEGUARD on success, -1 otherwise and
    [j0, j1, j2] are the joint angles for axes 0, 1, and 2, respectively
    """

    j0 = c_double()
    j1 = c_double()
    j2 = c_double()

    return ([j0.value, j1.value, j2.value],
            _libdhd.dhdGetDeltaJointAngles(
                byref(j0),
                byref(j1),
                byref(j2),
                ID
        )
    )
