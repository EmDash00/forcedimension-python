"""
. module::expert
   :platform: Windows, Unix
   :synopsis: libdhd "Expert SDK" Python libdhd

. moduleauthor:: Ember "Emmy" Chow <emberchow.business@gmail.com>
"""


from typing import Tuple, MutableSequence, Optional, cast

from ctypes import c_int, c_uint, c_byte, c_ubyte, c_ushort, c_double

from ctypes import byref, POINTER

from forcedimension.dhd import _libdhd
from forcedimension.dhd.constants import ComMode, MAX_DOF
from forcedimension.dhd.adaptors import (
    DeviceTuple, DOFTuple, CartesianTuple
)


_libdhd.dhdEnableExpertMode.argtypes = []
_libdhd.dhdEnableExpertMode.restype = c_int


def enableExpertMode() -> int:
    """
    Enable expert mode.

    :rtype: int
    :returns: `0` on success `-1` otherwise.
    """
    return _libdhd.dhdEnableExpertMode()


_libdhd.dhdDisableExpertMode.argtypes = []
_libdhd.dhdDisableExpertMode.restype = c_int


def disableExpertMode() -> int:
    """
    Enable expert mode.

    :rtype: int
    :returns: `0` on success `-1` otherwise.
    """
    return _libdhd.dhdDisableExpertMode()


_libdhd.dhdPreset.argtypes = [c_int * MAX_DOF, c_ubyte, c_byte]
_libdhd.dhdPreset.restype = c_int


def preset(val: DOFTuple, mask: int = 0xff, ID: int = -1) -> int:
    """
    Set selected encoder offsets to a given value. Intended for use with the
    generic controller when no RESET button is available.

    :param DOFTuple val: Motor values array.

    :param int mask:
        Bitwise mask of which encoder should be set, defaults to `0xff`.

    :param int ID:
        Device ID (see multiple devices section for details), defaults to `-1`.

    :raises ValueError: If `val` is not implicitly convertible to C int[9].
    :raises ValueError: If `mask` is not implicitly convertible to C uchar.
    :raises ValueError: If `ID` is not implicitly convertible to C char.

    :rtype: int
    :returns: 0 on success, -1 otherwise
    """
    return _libdhd.dhdPreset(val, mask, ID)


# TODO add a page about TimeGuard
_libdhd.dhdSetTimeGuard.argtypes = [c_int, c_byte]
_libdhd.dhdSetTimeGuard.restype = c_int


def setTimeGuard(min_period: int, ID: int = -1) -> int:
    """
    Enable/disable the [TimeGuard] feature with an arbitrary minimum period.

    :param int min_period: minimum refresh period in [us]. A value of `0`.
    disables the TimeGuard feature, while a value of -1 resets the default
    recommended value.

    :param int ID:
        Device ID (see multiple devices section for details), defaults to `-1`.

    :raises ValueError: If `min_period` is not implicitly convertible to C int.
    :raises ValueError: If `ID` is not implicitly convertible to C char.

    :rtype: int
    :returns: `0` on success `-1` otherwise.
    """

    return _libdhd.dhdSetTimeGuard(min_period, ID)


# TODO Make a page for velocity threshold
_libdhd.dhdSetVelocityThreshold.argtypes = [c_uint, c_byte]
_libdhd.dhdSetVelocityThreshold.restype = c_int


def setVelocityThreshold(thresh: int, ID: int = -1) -> int:
    """
    Adjust the velocity threshold of the device. Velocity threshold is a safety
    feature that prevents the device from accelerating to high velocities
    without control. If the velocity of one of the device axis passes the
    threshold, the device enters BRAKE mode.

    Warning
    -------
    Since the range of threshold values is device dependent, it is
    recommended NOT to modify factory settings.

    :param int thresh: An arbitrary value of velocity threshold.

    :param int ID:
        Device ID (see multiple devices section for details), defaults to `-1`.

    :raises ValueError: If `thresh` is not implicitly convertible to C uint.
    :raises ValueError: If `ID` is not implicitly convertible to C char.

    :rtype: int
    :returns: `0` on success `-1` otherwise.
    """

    return _libdhd.dhdSetVelocityThreshold(thresh, ID)


_libdhd.dhdGetVelocityThreshold.argtypes = [POINTER(c_uint), c_byte]
_libdhd.dhdGetVelocityThreshold.restype = c_int


def getVelocityThreshold(ID: int = -1) -> Tuple[int, int]:
    """
    Get the velocity threshold of the device. Velocity threshold is a safety
    feature that prevents the device from accelerating to high velocities
    without control. If the velocity of one of the device axis passes the
    threshold, the device enters BRAKE mode.

    Warning
    -------
    Velocity thresholds are device dependent.

    :param int ID:
        Device ID (see multiple devices section for details), defaults to `-1`.

    :raises ValueError:
        If `ID` is not implicitly convertible to C char.

    :rtype: Tuple[int, int]
    :returns:
        Tuple of `(thresh, err)`. `err` is 0 on success, -1 otherwise.
        `thresh` is the value of the velocity threshold (in [m/s]).
    """

    thresh = c_uint()
    return (thresh.value, _libdhd.dhdGetVelocityThreshold(byref(thresh), ID))


_libdhd.dhdUpdateEncoders.argtypes = [c_byte]
_libdhd.dhdUpdateEncoders.restype = c_int


def updateEncoders(ID: int = -1) -> int:
    """
    Force an update of the internal encoder values in the state vector. This
    call retrieves the encoder readings from the device and places them into
    the state vector. No kinematic model is called.

    :param int ID:
        Device ID (see multiple devices section for details), defaults to `-1`.

    :raises ValueError: If `ID` is not implicitly convertible to C char.

    :rtype: Tuple[int, int]
    :returns: `0` on success `-1` otherwise.
    """
    return _libdhd.dhdUpdateEncoders(ID)


_libdhd.dhdGetDeltaEncoders.argtypes = [
    POINTER(c_int),
    POINTER(c_int),
    POINTER(c_int),
    c_byte
]
_libdhd.dhdGetDeltaEncoders.restypes = c_int


def getDeltaEncoders(
    ID: int = -1,
    out: Optional[MutableSequence[int]] = None
) -> Tuple[MutableSequence[int], int]:
    """
    Read all encoders values of the DELTA structure

    :param int ID:
        Device ID (see multiple devices section for details), defaults to `-1`.

    :param Optional[MutableSequence[int]] out:
        Optional mutable sequence to use instead of generating a new list. If
        this is specified, the mutable sequence provided will be updated with
        the new values and the return will be a reference to the same mutable
        sequence passed in.

    :raises TypeError: If `out` is specified and does not support item.
    assignment either because it's immutable or not subscriptable.

    :raises IndexError: If `out` is specified and `len(out) < 3`.
    :raises ValueError: If `ID` is not implicitly convertible to a C char type.

    :raises ValueError: If `ID` is not implicitly convertible to C char.

    :rtype: Tuple[MutableSequence[int], int]

    :returns:
        Tuple of `([enc0, enc1, enc2], err)`. `err` is `0` or
        :data:`forcedimension.dhd.constants.TIMEGUARD` on success, -1 otherwise
        and `enc0`, `enc1`, and `enc2` are the axis 0, axis 1, and axis 2
        encoder readings, respectively.
    """

    enc0 = c_int()
    enc1 = c_int()
    enc2 = c_int()

    err = _libdhd.dhdGetDeltaEncoders(
        byref(enc0),
        byref(enc1),
        byref(enc2),
        ID
    )

    if out is None:
        out = [enc0.value, enc1.value, enc2.value]
    else:
        out[0] = enc0.value
        out[1] = enc1.value
        out[2] = enc2.value

    return (out, err)


_libdhd.dhdGetWristEncoders.argtypes = [
    POINTER(c_int),
    POINTER(c_int),
    POINTER(c_int),
    c_byte
]
_libdhd.dhdGetWristEncoders.restypes = c_int


def getWristEncoders(
    ID: int = -1,
    out: Optional[MutableSequence[int]] = None
) -> Tuple[MutableSequence[int], int]:
    """
    Read all encoders values of the wrist structure.

    This feature only applies to the following devices:

    :data:`forcedimension.dhd.constants.DeviceType.OMEGA33`
    :data:`forcedimension.dhd.constants.DeviceType.OMEGA33_LEFT`
    :data:`forcedimension.dhd.constants.DeviceType.OMEGA331`
    :data:`forcedimension.dhd.constants.DeviceType.OMEGA331_LEFT`
    :data:`forcedimension.dhd.constants.DeviceType.SIGMA331`
    :data:`forcedimension.dhd.constants.DeviceType.SIGMA331_LEFT`

    :param int ID:
        Device ID (see multiple devices section for details), defaults to `-1`.

    :raises ValueError: If `ID` is not implicitly convertible to C char.

    :param Optional[MutableSequence[int]] out:
        Optional mutable sequence to use instead of generating a new list. If
        this is specified, the mutable sequence provided will be updated with
        the new values and the return will be a reference to the same mutable
        sequence passed in.

    :raises TypeError:
        If `out` is specified and does not support item assignment either
        because it's immutable or not subscriptable.

    :raises IndexError: If `out` is specified and `len(out) < 3`.
    :raises ValueError: If `ID` is not implicitly convertible to a C char type.

    :rtype: Tuple[MutableSequence[int], int]
    :returns:
        Tuple of `([enc0, enc1, enc2], err)`. `err` is 0 or
        :data:`forcedimension.dhd.constants.TIMEGUARD` on success,
        -1 otherwise. `[enc0, enc1, enc2]` are the axis 0, 1, and 2 raw encoder
        readings, respectively.
    """

    enc0 = c_int()
    enc1 = c_int()
    enc2 = c_int()

    err = _libdhd.dhdGetWristEncoders(
        byref(enc0),
        byref(enc1),
        byref(enc2),
        ID
    )

    if out is None:
        out = [enc0.value, enc1.value, enc2.value]
    else:
        out[0] = enc0.value
        out[1] = enc1.value
        out[2] = enc2.value

    return (out, err)


_libdhd.dhdGetGripperEncoder.argtypes = [POINTER(c_int), c_byte]
_libdhd.dhdGetGripperEncoder.argtypes = [c_int]


def getGripperEncoder(ID: int = -1) -> Tuple[int, int]:
    """
    Read the encoder value of the force gripper.

    This feature only applies to the following devices:
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA331`
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA331_LEFT`
        :data:`forcedimension.dhd.constants.DeviceType.SIGMA331`
        :data:`forcedimension.dhd.constants.DeviceType.SIGMA331_LEFT`

    :param int ID:
        Device ID (see multiple devices section for details), defaults to `-1`.

    :raises ValueError: If `ID` is not implicitly convertible to C char.

    :rtype: Tuple[int, int]
    :returns:
        Tuple of `(enc, err)`. `err` is `0` or
        :data:`forcedimension.dhd.constants.TIMEGUARD` on success, `-1`
        otherwise. `enc` is the gripper raw encoder reading.
    """

    enc = c_int()

    return (enc.value, _libdhd.dhdGetGripperEncoder(byref(enc), ID))


_libdhd.dhdGetEncoder.argtypes = [c_int, c_byte]
_libdhd.dhdGetEncoder.restype = c_int


def getEncoder(index: int, ID: int = -1) -> int:
    """
    Read a single encoder value from the haptic device

    :param int index:
        The motor index number as defined by
        :data:`forcedimension.dhd.constants.MAX_DOF`

    :param int ID:
        Device ID (see multiple devices section for details), defaults to `-1`.

    :raises ValueError: If `index` is not implicitly convertible to C int.
    :raises ValueError: If `ID` is not implicitly convertible to C char.

    :rtype: int
    :returns: The (positive) encoder reading on success, -1 otherwise.
    """

    return _libdhd.dhdGetEncoder(index, ID)


_libdhd.dhdSetMotor.argtypes = [c_int, c_ushort, c_byte]
_libdhd.dhdSetMotor.restype = c_int


def setMotor(index: int, output: int, ID: int = -1) -> int:
    """
    Program a command to a single motor channel.

    :param int index: The motor index number as defined by.
    :data:`forcedimension.dhd.constants.MAX_DOF`

    :param int output: The motor DAC value.

    :param int ID:
        Device ID (see multiple devices section for details), defaults to `-1`.

    :raises ValueError: If `index` is not implicitly convertible to C int.
    :raises ValueError: If `output` is not implicitly convertible to C ushort.
    :raises ValueError: If `ID` is not implicitly convertible to C char.

    :rtype: int
    :returns: `0` on success `-1` otherwise.
    """

    return _libdhd.dhdSetMotor(index, output, ID)


_libdhd.dhdSetDeltaMotor.argtypes = [c_ushort, c_ushort, c_ushort, c_byte]
_libdhd.dhdSetDeltaMotor.restype = c_int


def setDeltaMotor(output: DeviceTuple, ID: int = -1) -> int:
    """
    Set desired motor commands to the amplifier channels commanding the DELTA
    motors.

    :param DeviceTuple output: Tuple of (motor0, motor1, motor2) where.
    motor0, motor1, and motor2 are the axis 0, 1, and 2 DELTA motor commands,
    respectively.

    :param int ID:
        Device ID (see multiple devices section for details), defaults to `-1`.

    :raises ValueError:
        If any member of `output` is not implicitly convertible to C ushort.

    :raises ValueError: If `ID` is not implicitly convertible to C char.

    :rtype: int
    :returns: `0` on success `-1` otherwise.

    """
    return _libdhd.dhdSetDeltaMotor(output[0], output[1], output[2], ID)


_libdhd.dhdSetWristMotor.argtypes = [c_ushort, c_ushort, c_ushort, c_byte]
_libdhd.dhdSetWristMotor.restype = c_int


def setWristMotor(output: DeviceTuple, ID: int = -1) -> int:
    """
    Set desired motor commands to the amplifier channels commanding the wrist
    motors.

    :param DeviceTuple output:
        Tuple of (motor0, motor1, motor2) where `motor0`, `motor1`, and
        `motor2` are the axis 0, 1, and 2 wrist motor commands, respectively.

    :param int ID:
        Device ID (see multiple devices section for details), defaults to `-1`.

    :raises ValueError:
        If any member of `output` is not implicitly convertible to C ushort.

    :raises ValueError: If `ID` is not implicitly convertible to C char.

    :rtype: int
    :returns: `0` on success `-1` otherwise.

    """
    return _libdhd.dhdSetWristMotor(output[0], output[1], output[2], ID)


_libdhd.dhdSetWristMotor.argtypes = [c_ushort, c_byte]
_libdhd.dhdSetWristMotor.restype = c_int


def setGripperMotor(output: int, ID: int = -1) -> int:
    """
    Set desired motor commands to the amplifier channels commanding the force
    gripper.

    :param int output: gripper motor command.

    :param int ID:
        Device ID (see multiple devices section for details), defaults to `-1`.

    :raises ValueError: If `output` is not implicitly convertible to C ushort.

    :raises ValueError: If `ID` is not implicitly convertible to C char.

    :rtype: int
    :returns: `0` on success `-1` otherwise.
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


def deltaEncoderToPosition(
    enc: DeviceTuple,
    ID: int = -1,
    out: Optional[MutableSequence[float]] = None
) -> Tuple[MutableSequence[float], int]:
    """
    This routine computes and returns the position of the end-effector for a
    given set of encoder readings.

    :param DeviceTuple enc: tuple of (enc0, enc1, enc2) where enc0,.
    enc1, and enc2 coresspond to encoder readings on axis 0, 1, and 2,
    respectively.

    :param int ID:
        Device ID (see multiple devices section for details), defaults to `-1`.

    :param Optional[MutableSequence[float]] out:
        Optional mutable sequence to use instead of generating a new list. If
        this is specified, the mutable sequence provided will be updated with
        the new values and the return will be a reference to the same mutable
        sequence passed in.

    :raises TypeError:
        If `out` is specified and does not support item assignment either
        because it's immutable or not subscriptable.

    :raises IndexError: If `out` is specified and `len(out) < 3`.
    :raises ValueError: If `ID` is not implicitly convertible to a C char type.

    :raises ValueError:
        If any member of `enc` is not implicitly convertible to C int.

    :raises ValueError: If `ID` is not implicitly convertible to C char.

    :rtype: int
    :returns:
        Tuple of `([px, py, pz], err)` where `err` is `0` on success,
        `-1` otherwise and `[px, py, pz]` is a list of floats corresponding
        to the DELTA end-effector position on the X, Y, and Z axes,
        respectively in [m].
    """

    px = c_double()
    py = c_double()
    pz = c_double()

    err = _libdhd.dhdDeltaEncoderToPosition(
        enc[0],
        enc[1],
        enc[2],
        byref(px),
        byref(py),
        byref(pz),
        ID
    )

    if out is None:
        out = [px.value, py.value, pz.value]
    else:
        out[0] = px.value
        out[1] = py.value
        out[2] = pz.value

    return (out, err)


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


def deltaPositionToEncoder(
    pos: CartesianTuple,
    ID: int = -1,
    out: Optional[MutableSequence[int]] = None
) -> Tuple[MutableSequence[int], int]:
    """
    This routine computes and returns the encoder values of the end-effector
    for a given Cartesian end-effector position.

    :param CartesianTuple pos:
        Tuple of `(px, py, pz)` where `px`, `py`, and `pz` correspond to the
        end-effector position on the X, Y, and Z axes, respectively in [m].

    :param int ID:
        Device ID (see multiple devices section for details), defaults to `-1`.

    :param Optional[MutableSequence[int]] out:
        Optional mutable sequence to use instead of generating a new list. If
        this is specified, the mutable sequence provided will be updated with
        the new values and the return will be a reference to the same mutable
        sequence passed in.

    :raises TypeError:
        If `out` is specified and does not support item
        assignment either because it's immutable or not subscriptable.

    :raises IndexError: If `out` is specified and `len(out) < 3`.
    :raises ValueError: If `ID` is not implicitly convertible to a C char type.

    :raises ValueError:
        If any member of `pos` is not implicitly convertible to C double

    :raises ValueError: If `ID` is not implicitly convertible to C char.

    :rtype: Tuple[MutableSequence[int], int]

    :returns:
        Tuple of `([enc0, enc1, enc2], err). `err` is 0 on success, -1
        otherwise. `[enc0, enc1, enc2]` is a list of floats corresponding to
        the DELTA end-effector encoder readings on axis 0, 1, and 2,
        respectively.
    """

    enc0 = c_int()
    enc1 = c_int()
    enc2 = c_int()

    err = _libdhd.dhdDeltaEncoderToPosition(
        pos[0],
        pos[1],
        pos[2],
        byref(enc0),
        byref(enc1),
        byref(enc2),
        ID
    )

    if out is None:
        out = [enc0.value, enc1.value, enc2.value]
    else:
        out[0] = enc0.value
        out[1] = enc1.value
        out[2] = enc2.value

    return (out, err)


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


def deltaMotorToForce(
    output: DeviceTuple,
    enc: DeviceTuple,
    ID: int = -1,
    out: Optional[MutableSequence[float]] = None
) -> Tuple[MutableSequence[float], int]:
    """
    This routine computes and returns the force applied to the end-effector for
    a given set of motor commands at a given position (defined by encoder
    readings)

    :param DeviceTuple output: Tuple of (motor0, motor1, motor2) where.
    motor0, motor1, and motor2 are the axis 0, 1, and 2 DELTA motor commands,
    respectively.

    :param DeviceTuple enc:
        Sequence of `(enc0, enc1, enc2)` where `enc0`, `enc1`, and `enc2`
        coresspond to encoder readings on axis 0, 1, and 2, respectively.

    :param int ID:
        Device ID (see multiple devices section for details), defaults to `-1`.

    :param Optional[MutableSequence[float]] out:
        Optional mutable sequence to use instead of generating a new list. If
        this is specified, the mutable sequence provided will be updated with
        the new values and the return will be a reference to the same mutable
        sequence passed in.

    :raises TypeError: If `out` is specified and does not support item.
    assignment either because it's immutable or not subscriptable.

    :raises IndexError: If `out` is specified and `len(out) < 3`.
    :raises ValueError: If `ID` is not implicitly convertible to a C char type.

    :raises ValueError:
        If any member of `output` is not implicitly convertible to C ushort.

    :raises ValueError:
        If any member of `enc` is not implicitly convertible to C int.

    :raises ValueError: If `ID` is not implicitly convertible to C char.

    :rtype: Tuple[MutableSequence[float], int]
    :returns:
        Tuple of `([fx, fy, fz], err)`. `err` is `0` on success, `-1`
        otherwise and `[fx, fy, fz]` correspond to the translational force on
        the DELTA end-effector on the X, Y and Z axes, respectively in [N].
    """

    fx = c_double()
    fy = c_double()
    fz = c_double()

    err = _libdhd.dhdDeltaMotorToForce(
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

    if out is None:
        out = [fx.value, fy.value, fz.value]
    else:
        out[0] = fx.value
        out[1] = fy.value
        out[2] = fz.value

    return (out, err)


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


def deltaForceToMotor(
    f: CartesianTuple,
    enc: DeviceTuple,
    ID: int = -1,
    out: Optional[MutableSequence[int]] = None
) -> Tuple[MutableSequence[int], int]:
    """
    This routine computes and returns the motor commands necessary to obtain a
    given force on the end-effector at a given position (defined by encoder
    readings).

    :param CartesianTuple f:
        Tuple of `(fx, fy, fz)` where `fx`, `fy`, and `fz` are the force on the
        DELTA end-effector on the X, Y, and Z axes, respectively in [N]

    :param DeviceTuple enc:
        Tuple of `(enc0, enc1, enc2)` where `enc0`, `enc1`, and `enc2`
        coresspond to encoder readings on axis 0, 1, and 2, respectively.

    :param int ID:
        Device ID (see multiple devices section for details), defaults to `-1`.

    :param Optional[MutableSequence[int]] out:
        Optional mutable sequence to use instead of generating a new list. If
        this is specified, the mutable sequence provided will be updated with
        the new values and the return will be a reference to the same mutable
        sequence passed in.

    :raises TypeError: If `out` is specified and does not support item.
    assignment either because it's immutable or not subscriptable.

    :raises IndexError: If `out` is specified and `len(out) < 3`.

    :raises ValueError: If `ID` is not implicitly convertible to a C char type.

    :raises ValueError:
        If any member of `f` is not implicitly convertible to C double.

    :raises ValueError:
        If any member of `output` is not implicitly convertible to C ushort.

    :raises ValueError: If `ID` is not implicitly convertible to C char.

    :rtype: Tuple[MutableSequence[int], int]

    :returns:
        Tuple of `([output0, output1, output2], err)`. `err` is 0 or
        :data:`forcedimension.dhd.constants.MOTOR_SATURATED` on success, `-1`
        otherwise. `[output0, output1, output2]` correspond to the DELTA
        end-effector motor commands on axes 0, 1, and 2, respectively.
    """

    output0 = c_ushort()
    output1 = c_ushort()
    output2 = c_ushort()

    err = _libdhd.dhdDeltaMotorToForce(
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

    if out is None:
        out = [output0.value, output1.value, output2.value]
    else:
        out[0] = output0.value
        out[1] = output1.value
        out[2] = output2.value

    return (out, err)


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


def wristEncoderToOrientation(
    enc: DeviceTuple,
    ID: int = -1,
    out: Optional[MutableSequence[float]] = None,
) -> Tuple[MutableSequence[float], int]:
    """
    For devices with a wrist structure, compute the individual angle of each
    joint, starting with the one located nearest to the wrist base plate. For
    the :data:`forcedimension.dhd.constants.DeviceType.OMEGA33` and the
    :data:`forcedimension.dhd.constants.DeviceType.OMEGA33_LEFT` devices,
    angles are computed with respect to their internal reference frame, which
    is rotated 45 degrees or π/4 radians about the Y axis. Please refer to your
    device user manual for more information on your device coordinate system.

    This feature only applies to the following devices:

    :data:`forcedimension.dhd.constants.DeviceType.OMEGA33`
    :data:`forcedimension.dhd.constants.DeviceType.OMEGA33_LEFT`
    :data:`forcedimension.dhd.constants.DeviceType.OMEGA331`
    :data:`forcedimension.dhd.constants.DeviceType.OMEGA331_LEFT`
    :data:`forcedimension.dhd.constants.DeviceType.SIGMA331`
    :data:`forcedimension.dhd.constants.DeviceType.SIGMA331_LEFT`

    :param DeviceTuple enc:
        Tuple of `(enc0, enc1, enc2)` where `enc0`, `enc1`, and `enc2`
        correspond to wrist encoder readings on the first, second, and third
        joint, respectively

    :param int ID:
        Device ID (see multiple devices section for details), defaults to `-1`.

    :param Optional[MutableSequence[float]] out:
        Optional mutable sequence to use instead of generating a new list. If
        this is specified, the mutable sequence provided will be updated with
        the new values and the return will be a reference to the same mutable
        sequence passed in.

    :raises TypeError: If `out` is specified and does not support item.
    assignment either because it's immutable or not subscriptable.

    :raises IndexError: If `out` is specified and `len(out) < 3`.

    :raises ValueError: If any member of `enc` is not implicitly convertible.
    to C int.

    :raises ValueError: If `ID` is not implicitly convertible to C char.

    :rtype: Tuple[MutableSequence[float], int]
    :returns:
        Tuple of `([oa, ob, og], err)` where `err` is `0` on success,
        `-1` otherwise and `[oa, ob, og]` is a list of floats corresponding to
        the wrist end-effector's orientation about the first, second, and third
        wrist joint, respectively in [rad].
    """
    px = c_double()
    py = c_double()
    pz = c_double()

    err = _libdhd.dhdDeltaEncoderToPosition(
        enc[0],
        enc[1],
        enc[2],
        byref(px),
        byref(py),
        byref(pz),
        ID
    )

    if out is None:
        out = [px.value, py.value, pz.value]
    else:
        out[0] = px.value
        out[1] = py.value
        out[2] = pz.value

    return (out, err)


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


def wristOrientationToEncoder(
    orientation: CartesianTuple,
    ID: int = -1,
    out: Optional[MutableSequence[int]] = None,
) -> Tuple[MutableSequence[int], int]:
    """
    For devices with a wrist structure, compute the encoder values from the
    individual angle of each joint, starting witht he one located nearest to
    the wrist plate base. For the dhd.libdhd.DeviceType.OMEGA33 and
    dhd.libdhd.DeviceType.OMEGA33_LEFT devices, angles must be expressed
    withrespect to their internal reference frame, which is rotated 45 degrees
    or π/4 radians about the Y axis. Please refer to your device user manual
    for more information on your device coordinate system.

    This feature only applies to the following devices:
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA33`
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA33_LEFT`
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA331`
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA331_LEFT`
        :data:`forcedimension.dhd.constants.DeviceType.SIGMA331`
        :data:`forcedimension.dhd.constants.DeviceType.SIGMA331_LEFT`

    :param CartesianTuple orientation:
        Tuple of `(oa, ob, og)` where `(oa, ob, og)` coresspond to wrist end
        effector orientation around the X, Y, and Z axes, respectively in
        [rad].

    :param int ID:
        Device ID (see multiple devices section for details), defaults to `-1`.

    :param Optional[MutableSequence[int]] out:
        Optional mutable sequence to use instead of generating a new list. If
        this is specified, the mutable sequence provided will be updated with
        the new values and the return will be a reference to the same mutable
        sequence passed in.

    :raises TypeError: If `out` is specified and does not support item.
    assignment either because it's immutable or not subscriptable.

    :raises IndexError: If `out` is specified and `len(out) < 3`.

    :raises ValueError: If any member of `enc` is not implicitly convertible.
    to C int.

    :raises ValueError: If `ID` is not implicitly convertible to C char.

    :rtype: Tuple[MutableSequence[int], int]
    :returns: tuple of ([enc0, enc1, enc2], err) where err is 0 on success,
    -1 otherwise and [enc0, enc1, enc2] is a list of ints corresponding to
    the wrist end-effector orientation around the X, Y, and Z axes,
    respectively.
    """

    enc0 = c_int()
    enc1 = c_int()
    enc2 = c_int()

    err = _libdhd.dhdDeltaEncoderToPosition(
        orientation[0],
        orientation[1],
        orientation[2],
        byref(enc0),
        byref(enc1),
        byref(enc2),
        ID
    )

    if out is None:
        out = [enc0.value, enc1.value, enc2.value]
    else:
        out[0] = enc0.value
        out[1] = enc1.value
        out[2] = enc2.value

    return (out, err)


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


def wristMotorToTorque(
    cmd: DeviceTuple,
    enc: DeviceTuple,
    ID: int = -1,
    out: Optional[MutableSequence[float]] = None,
) -> Tuple[MutableSequence[float], int]:
    """
    This routine computes and returns the torque applied to the wrist
    end-effector for a given set of motor commands at a given orientation
    (defined by encoder readings)

    :param DeviceTuple cmd:
        Tuple of `(motor0, motor1, motor2)` where `motor0`, `motor1`, and
        `motor2` are the axis 0, 1, and 2 DELTA motor commands, respectively.

    :param DeviceTuple enc:
        Tuple of `(enc0, enc1, enc2)` where `enc0`, `enc1`, and `enc2`
        correspond to encoder readings on axis 0, 1, and 2, respectively.

    :param int ID:
        Device ID (see multiple devices section for details), defaults to `-1`.

    :param Optional[MutableSequence[float]] out:
        Optional mutable sequence to use instead of generating a new list. If
        this is specified, the mutable sequence provided will be updated with
        the new values and the return will be a reference to the same mutable
        sequence passed in.

    :raises TypeError:
        If `out` is specified and does not support item assignment either
        because it's immutable or not subscriptable.

    :raises IndexError: If `out` is specified and `len(out) < 3`.

    :raises ValueError:
        If any member of `output` is not implicitly convertible to C ushort.

    :raises ValueError:
        If any member of `enc` is not implicitly convertible to C int.

    :raises ValueError: If `ID` is not implicitly convertible to C char.

    :rtype: Tuple[MutableSequence[float], int]
    :returns:
        Tuple of ([tx, ty, tz], err) where `err` is `0` on success, `-1`,
        otherwise and [tx, ty, tz] correspond to the torque on the wrist
        end-effector around the X, Y, and Z axes respectively in [Nm].
    """

    tx = c_double()
    ty = c_double()
    tz = c_double()

    err = _libdhd.dhdDeltaMotorToForce(
        cmd[0],
        cmd[1],
        cmd[2],
        enc[0],
        enc[1],
        enc[2],
        byref(tx),
        byref(ty),
        byref(tz)
    )

    if out is None:
        out = [tx.value, ty.value, tz.value]
    else:
        out[0] = tx.value
        out[1] = ty.value
        out[2] = tz.value

    return (out, err)


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


def wristTorqueToMotor(
    t: CartesianTuple,
    enc: DeviceTuple,
    ID: int = -1,
    out: Optional[MutableSequence[int]] = None,
) -> Tuple[MutableSequence[int], int]:
    """
    This routine computes and returns the wrist motor commands necessary to
    obtain a given torque on the wrist end-effector at a given orientation
    (defined by encoder readings)

    :param DeviceTuple output:
        Tuple of `(motor0, motor1, motor2)` where `motor0`, `motor1`, and
        `motor2` are the axis 0, 1, and 2 DELTA motor commands, respectively.

    :param DeviceTuple enc:
        Tuple of `(enc0, enc1, enc2)` where `enc0`, `enc1`, and `enc2`
        correspond to encoder readings on axis 0, 1, and 2, respectively.

    :param int ID:
        Device ID (see multiple devices section for details), defaults to `-1`.

    :param Optional[MutableSequence[int]] out:
        Optional mutable sequence to use instead of generating a new list. If
        this is specified, the mutable sequence provided will be updated with
        the new values and the return will be a reference to the same mutable
        sequence passed in.

    :raises TypeError:
        If `out` is specified and does not support item assignment either
        because it's immutable or not subscriptable.

    :raises IndexError: If `out` is specified and `len(out) < 3`.

    :raises ValueError:
        If any member of `t` is not implicitly convertible to C double.

    :raises ValueError:
        If any member of `enc` is not implicitly convertible to C int.

    :raises ValueError: If `ID` is not implicitly convertible to C char.

    :rtype: Tuple[MutableSequence[int], int]

    :returns:
        Tuple of `([output0, output1, output2], err)`. `err` is `0` on
        success, `-1` otherwise and `[output0, output1, output2]` correspond to
        the motor commands on the wrist end-effector around wrist joint 0, 1,
        and 2, respectively.
    """

    output0 = c_ushort()
    output1 = c_ushort()
    output2 = c_ushort()

    err = _libdhd.dhdDeltaMotorToForce(
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

    if out is None:
        out = [output0.value, output1.value, output2.value]
    else:
        out[0] = output0.value
        out[1] = output1.value
        out[2] = output2.value

    return (out, err)


_libdhd.dhdGripperEncoderToAngleRad.argtypes = [
    c_int,
    POINTER(c_double),
    c_byte
]
_libdhd.dhdGripperEncoderToAngleRad.restype = c_int


def gripperEncoderToAngleRad(enc: int,
                             ID: int = -1) -> Tuple[float, int]:
    """
    This routine computes and returns the opening of the gripper as an angle in
    [rad] for a given encoder reading.

    This feature only applies to the following devices:

    :data:`forcedimension.dhd.constants.DeviceType.OMEGA331`
    :data:`forcedimension.dhd.constants.DeviceType.OMEGA331_LEFT`
    :data:`forcedimension.dhd.constants.DeviceType.SIGMA331`
    :data:`forcedimension.dhd.constants.DeviceType.SIGMA331_LEFT`

    :param int enc: Gripper encoder reading.

    :param int ID:
        Device ID (see multiple devices section for details), defaults to `-1`.

    :raises ValueError: If `enc` is not implicitly convertible to C int.
    :raises ValueError: If `ID` is not implicitly convertible to C char.

    :rtype: Tuple[float, int]
    :returns:
        Tuple of `(angle, err)` where err is `0` or
        :data:`forcedimension.dhd.constants.TIMEGUARD` on success, `-1` on
        otherwise and angle is the gripper opening in [rad]
    """

    angle = c_double()
    err = _libdhd.dhdGripperEncoderToAngleRad(
        enc,
        byref(angle),
        ID
    )

    return (angle.value, err)


_libdhd.dhdGripperEncoderToGap.argtypes = [
    c_int,
    POINTER(c_double),
    c_byte
]
_libdhd.dhdGripperEncoderToGap.restype = c_int


def gripperEncoderToGap(enc: int, ID: int = -1) -> Tuple[float, int]:
    """
    This routine computes and returns the opening of the gripper as a distance
    in [m] for a given encoder reading.

    This feature only applies to the following devices:

    :data:`forcedimension.dhd.constants.DeviceType.OMEGA331`
    :data:`forcedimension.dhd.constants.DeviceType.OMEGA331_LEFT`
    :data:`forcedimension.dhd.constants.DeviceType.SIGMA331`
    :data:`forcedimension.dhd.constants.DeviceType.SIGMA331_LEFT`

    :param int enc: gripper encoder reading.

    :param int ID:
        Device ID (see multiple devices section for details), defaults to `-1`.

    :raises ValueError: If `enc` is not implicitly convertible to C int.
    :raises ValueError: If `ID` is not implicitly convertible to C char.

    :rtype: Tuple[float, int]
    :returns:
        Tuple of `(gap, err)` where `err` is `0` on success, `-1` otherwise and
        gap is the gripper opening in [m]
    """

    gap = c_double()
    err = _libdhd.dhdGripperEncoderToGap(
        enc,
        byref(gap),
        ID
    )

    return (gap.value, err)


_libdhd.dhdGripperAngleRadToEncoder.argtypes = [
    c_double,
    POINTER(c_int),
    c_byte
]
_libdhd.dhdGripperAngleRadToEncoder.restype = c_int


def gripperAngleRadToEncoder(angle: float, ID: int = -1) -> Tuple[int, int]:
    """
    This routine computes and returns the gripper encoder value for a given
    gripper opening distance in [rad].

    This feature only applies to the following devices:
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA331`
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA331_LEFT`
        :data:`forcedimension.dhd.constants.DeviceType.SIGMA331`
        :data:`forcedimension.dhd.constants.DeviceType.SIGMA331_LEFT`

    :param float angle: gripper opening as an angle in [rad].

    :param int ID:
        Device ID (see multiple devices section for details), defaults to `-1`.

    :raises ValueError: If `angle` is not implicitly convertible to C float.
    :raises ValueError: If `ID` is not implicitly convertible to C char.

    :rtype: Tuple[int, int]
    :returns:
        Tuple of `(enc, err)` where `err` is `0` on success, `-1` otherwise and
        `enc` is the gripper encoder reading
    """

    enc = c_int()

    err = _libdhd.dhdGripperEncoderToAngleRad(
        angle,
        byref(enc),
        ID
    )
    return (enc.value, err)


_libdhd.dhdGripperGapToEncoder.argtypes = [
    c_double,
    POINTER(c_int),
    c_byte
]
_libdhd.dhdGripperEncoderToGap.restype = c_int


def gripperGapToEncoder(gap: float, ID: int = -1) -> Tuple[int, int]:
    """
    This routine computes and returns the gripper encoder value for a given
    gripper opening distance in [m].

    This feature only applies to the following devices:

    :data:`forcedimension.dhd.constants.DeviceType.OMEGA331`
    :data:`forcedimension.dhd.constants.DeviceType.OMEGA331_LEFT`
    :data:`forcedimension.dhd.constants.DeviceType.SIGMA331`
    :data:`forcedimension.dhd.constants.DeviceType.SIGMA331_LEFT`


    :param float gap: Gripper opening distance in [m].

    :param int ID:
        Device ID (see multiple devices section for details), defaults to `-1`.

    :raises ValueError: If `gap` is not implicitly convertible to C double.
    :raises ValueError: If `ID` is not implicitly convertible to C char.

    :rtype: Tuple[int, int]
    :returns:
        Tuple of `(enc, err)` where err is `0` on success, `-1` otherwise and
        `enc` is the gripper encoder reading.
    """

    enc = c_int()
    err = _libdhd.dhdGripperEncoderToGap(
        gap,
        byref(enc),
        ID
    )

    return (enc.value, err)


_libdhd.dhdGripperMotorToForce.argtypes = [
    c_ushort,
    POINTER(c_double),
    c_int * 4,
    c_byte
]
_libdhd.dhdGripperMotorToForce.restype = c_int


def gripperMotorToForce(output: int,
                        enc_wrist: DeviceTuple,
                        enc_gripper: int,
                        ID: int = -1) -> Tuple[float, int]:
    """
    This routine computes the force applied to the end-effector for a given
    motor command.

    This feature only applies to the following devices:

    :data:`forcedimension.dhd.constants.DeviceType.OMEGA331`
    :data:`forcedimension.dhd.constants.DeviceType.OMEGA331_LEFT`
    :data:`forcedimension.dhd.constants.DeviceType.SIGMA331`
    :data:`forcedimension.dhd.constants.DeviceType.SIGMA331_LEFT`


    :param int output: Motor command on gripper axis.

    :param DeviceTuple enc_wrist:
        Tuple of `(enc0, enc1, enc2)` where `enc0`, `enc1`, `enc2` are encoder
        readings about joint 0, 1, and 2, respectively.

    :param int enc_gripper: Encoder reading for the gripper.

    :param int ID:
        Device ID (see multiple devices section for details), defaults to `-1`.

    :raises ValueError: If `output` is not implicitly convertible to C ushort.

    :raises ValueError:
        If any member of `enc_wrist` is not implicitly convertible to C int.

    :raises ValueError:
        If `enc_gripper` is not implicitly convertible to C int.

    :raises ValueError:
        If `ID` is not implicitly convertible to C char.

    :rtype: Tuple[float, int]
    :returns:
        Tuple of (force, err) where err is 0 on success, -1 otherwise and
        force is the force on the gripper end-effector in [N].
    """

    force = c_double()

    enc = [enc_wrist[0], enc_wrist[1], enc_wrist[2], enc_gripper]
    err = _libdhd.dhdGripperMotorToForce(
        output,
        byref(force),
        enc,
        ID
    )

    return (force.value, err)


_libdhd.dhdGripperForceToMotor.argtypes = [
    c_double,
    POINTER(c_ushort),
    c_int * 4,
    c_byte
]
_libdhd.dhdGripperForceToMotor.restype = c_int


def gripperForceToMotor(force: float,
                        enc_wrist: DeviceTuple,
                        enc_gripper: int,
                        ID: int = -1) -> Tuple[int, int]:
    """
    Given a desired force to be displayed by the force gripper, this routine
    computes and returns the corresponding motor command.

    This feature only applies to the following devices:
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA331`
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA331_LEFT`
        :data:`forcedimension.dhd.constants.DeviceType.SIGMA331`
        :data:`forcedimension.dhd.constants.DeviceType.SIGMA331_LEFT`


    :param int force: force on the gripper end-effector in [N].

    :param DeviceTuple enc_wrist: tuple of (enc0, enc1, enc2) where.
    (enc0, enc1, enc2) are encoder readings about joint 0, 1, and 2,
    respectively.

    :param int enc_gripper: encoder reading for the gripper.

    :param int ID:
        Device ID (see multiple devices section for details), defaults to `-1`.

    :raises ValueError:
        If `force` is not implicitly convertible to C double.

    :raises ValueError:
        If `a` member of enc_wrist is not implicitly convertible to C int

    :raises ValueError:
        If `enc_gripper` is not implicitly convertible to C int.

    :raises ValueError: If `ID` is not implicitly convertible to C char.

    :rtype: Tuple[int, int]

    :returns:
        Tuple of `(output, err)` where err is `0` or
        :data:`forcedimension.dhd.constants.MOTOR_SATURATED` on success,
        `-1` otherwise and output is the motor command on the gripper axis.
    """

    output = c_ushort()

    enc = [enc_wrist[0], enc_wrist[1], enc_wrist[2], enc_gripper]

    err = _libdhd.dhdGripperMotorToForce(
        force,
        byref(output),
        enc,
        ID
    )

    return (output.value, err)


_libdhd.dhdSetMot.argtypes = [c_ushort * MAX_DOF, c_ubyte, c_byte]
_libdhd.dhdSetMot.restype = c_int


def setMot(outputs: DOFTuple, mask: int = 0xff, ID: int = -1) -> int:
    """
    Program motor commands to a selection of motor channels. Particularly
    useful when using the generic controller directly, without a device model
    attached.

    :param DOFTuple outputs: List of motor command values.

    :param int mask:
        Bitwise mask of which motor should be set, defaults to `0xff`

    :param int ID:
        Device ID (see multiple devices section for details), defaults to `-1`.

    :raises ValueError:
        If any member of `outputs` is not implicitly convertible to C int.

    :raises ValueError: If `mask` is not implicitly convertible to C uchar.

    :raises ValueError: If `ID` is not implicitly convertible to C char.

    :rtype: int
    :returns: 0 on success, -1 otherwise
    """

    return _libdhd.dhdSetMot(outputs, mask, ID)


_libdhd.dhdPreloadMot.argtypes = [c_ushort * MAX_DOF, c_ubyte, c_byte]
_libdhd.dhdPreloadMot.restype = c_int


def preloadMot(outputs: DOFTuple, mask: int = 0xff, ID: int = -1) -> int:
    """
    Program motor commands to a selection of motor channels. Unlike
    dhd.libdhd.expert.setMot, this function saves the requested commands
    internally for later application by calling dhd.libdhd.standard.setForce
    and the likes.

    :param DOFTuple outputs: List of motor command values.

    :param int mask:
        Bitwise mask of which motor should be set, defaults to `0xff`

    :param int ID:
        Device ID (see multiple devices section for details), defaults to `-1`.

    :raises ValueError: If any member of `outputs` is not implicitly.
    convertible to C int

    :raises ValueError: If `mask` is not implicitly convertible to C uchar.

    :raises ValueError: If `ID` is not implicitly convertible to C char.

    :rtype: int
    :returns: 0 on success, -1 otherwise
    """

    return _libdhd.dhdPreloadMot(outputs, mask, ID)


_libdhd.dhdGetEnc.argtypes = [c_int * MAX_DOF, c_ubyte, c_byte]
_libdhd.dhdGetEnc.restype = c_int


def getEnc(
    mask: int,
    ID: int = -1,
    out: Optional[MutableSequence[int]] = None,
) -> Tuple[MutableSequence[int], int]:
    """
    Get a selective list of encoder values. Particularly useful when using the
    generic controller directly, without a device model attached.

    :param int mask: [default=0xff] bitwise mask of which motor should be set.

    :param int ID:
        Device ID (see multiple devices section for details), defaults to `-1`.

    :param Optional[MutableSequence[int]] out:
        Optional mutable sequence to use instead of generating a new list. If
        this is specified, the mutable sequence provided will be updated with
        the new values and the return will be a reference to the same mutable
        sequence passed in.

    :raises TypeError: If `out` is specified and does not support item.
    assignment either because it's immutable or not subscriptable.

    :raises IndexError: If `out` is specified and.
    len(out) < :data:forcedimension.dhd.constants.MAX_DOF`


    :raises ValueError: If `mask` is not implicitly convertible to C uchar.
    :raises ValueError: If `ID` is not implicitly convertible to C char.

    :rtype: Tuple[MutableSequence[int], int]
    :returns:
        Tuple of `(enc, err)` where `err` is `0` or
        :data:`forcedimension.dhd.constants.TIMEGUARD` on success, `-1`
        otherwise and enc is a mutable sequence of encoder values.
    """
    enc = (c_int * MAX_DOF)()

    if out is None:
        return ([val for val in enc], _libdhd.dhdGetEnc(enc, mask, ID))
    else:
        err = _libdhd.dhdGetEnc(enc, mask, ID)

        for i in range(MAX_DOF):
            out[i] = enc[i]

        return (out, err)


_libdhd.dhdGetEncRange.argtypes = [c_int * MAX_DOF, c_int * MAX_DOF, c_byte]
_libdhd.dhdGetEncRange.restype = c_int


def getEncRange(
    ID: int = -1,
    encMin_out: Optional[MutableSequence[int]] = None,
    encMax_out: Optional[MutableSequence[int]] = None,
) -> Tuple[MutableSequence[int], MutableSequence[int], int]:
    """
    Get the expected min and max encoder values for all axes present on the
    current device. Axis indices that do not exist on the device will return
    a range of 0.

    :param int ID:
        Device ID (see multiple devices section for details), defaults to `-1`.

    :param Optional[MutableSequence[int]] encMin_out:
        Optional mutable sequence to use instead of generating a new list. If
        this is specified, the mutable sequence provided will be updated with
        the new values and the return will be a reference to the same mutable
        sequence passed in.

    :param Optional[MutableSequence[int]] encMax_out:
        Optional mutable sequence to use instead of generating a new list. If
        this is specified, the mutable sequence provided will be updated with
        the new values and the return will be a reference to the same mutable
        sequence passed in.

    :raises TypeError:
        If `encMax_out` is specified and does not support item assignment
        either because it's immutable or not subscriptable.

    :raises TypeError:
        If `encMin_out` is specified and does not support item assignment
        either because it's immutable or not subscriptable.

    :raises IndexError:
        If `encMin_out` is specified and
        len(out) < :data:forcedimension.dhd.constants.MAX_DOF`

    :raises IndexError:
        If `encMin_out` is specified and
        `len(encMin_out) < :data:forcedimension.dhd.constants.MAX_DOF`


    :raises ValueError: If `ID` is not implicitly convertible to a C char type.

    :raises ValueError: If `ID` is not implicitly convertible to C char.

    :rtype: Tuple[MutableSequence[int], MutableSequence[int], int]
    :returns:
        Tuple of `(encMin, encMax err)`. `err` is `0` on success, `-1`
        otherwise. `encMin` and `encMax` are a list of minimum and maximum
        encoder values for each axis, respectively.
    """

    encMin = (c_int * MAX_DOF)()
    encMax = (c_int * MAX_DOF)()

    err = _libdhd.dhdGetEncRange(encMin, encMax, ID)

    if encMin_out is None:
        encMin_out = [val for val in encMin]
    else:
        for i in range(MAX_DOF):
            encMin_out[i] = encMin[i]

    if encMax_out is None:
        encMax_out = [val for val in encMax]
    else:
        encMax_out[i] = encMax[i]

    return (encMin_out, encMax_out, err)


_libdhd.dhdSetBrk.argtypes = [c_ubyte, c_byte]
_libdhd.dhdSetBrk.restype = c_int


def setBrk(mask: int = 0xff, ID: int = -1) -> int:
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
    a motor group's is enabled, all motors in that motor's group's
    electromagnetic brakes will be activated.

    :param mask:
        Bitwise mask of which motor groups should have their electromagnetic
        brakes be set on, defaults to `0xff`

    :param int ID:
        Device ID (see multiple devices section for details), defaults to `-1`.

    :raises ValueError: If `mask` is not implicitly convertible to C uchar.
    :raises ValueError: If `ID` is not implicitly convertible to C char.

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


def getDeltaJointAngles(
    ID: int = -1,
    out: Optional[MutableSequence[float]] = None
) -> Tuple[MutableSequence[float], int]:
    """
    Retrieve the joint angles in [rad] for the DELTA structure.

    :param int ID:
        Device ID (see multiple devices section for details), defaults to `-1`.

    :param Optional[MutableSequence[float]] out:
        Optional mutable sequence to use instead of generating a new list. If
        this is specified, the mutable sequence provided will be updated with
        the new values and the return will be a reference to the same mutable
        sequence passed in.

    :raises TypeError: If `out` is specified and does not support item.
    assignment either because it's immutable or not subscriptable.

    :raises IndexError: If `out` is specified and `len(out) < 3`.
    :raises ValueError: If `ID` is not implicitly convertible to a C char type.

    :raises ValueError: If `ID` is not implicitly convertible to C char.

    :rtype: Tuple[MutableSequence[float], int]
    :returns:
        Tuple of `([j0, j1, j2], err)`. `err` is `0` or
        :data:`forcedimension.dhd.constants.TIMEGUARD` on success, `-1`
        otherwise and `[j0, j1, j2]` are a mutable sequence of joint angles for
        axes 0, 1, and 2, respectively.
    """

    j0 = c_double()
    j1 = c_double()
    j2 = c_double()

    err = _libdhd.dhdGetDeltaJointAngles(
        byref(j0),
        byref(j1),
        byref(j2),
        ID
    )

    if out is None:
        out = [j0.value, j1.value, j2.value]
    else:
        out[0] = j0.value
        out[1] = j1.value
        out[2] = j2.value

    return (out, err)


_libdhd.dhdGetDeltaJacobian.argtypes = [(c_double * 3) * 3, c_byte]
_libdhd.dhdGetDeltaJacobian.restype = c_int


def getDeltaJacobian(
    ID: int = -1,
    out: Optional[MutableSequence[MutableSequence[float]]] = None
) -> Tuple[MutableSequence[MutableSequence[float]], int]:
    """
    Retrieve the 3x3 jacobian matrix for the DELTA structure based on the
    current end-effector position. Please refer to your device user manual for
    more information on your device coordinate system.

    :param int ID:
        Device ID (see multiple devices section for details), defaults to `-1`.

    :param Optional[MutableSequence[MutableSequence[float]]] out:
         Mutable sequence of mutable sequences to use intead of generating a
         new list of lists. If this is speicfied, the list provided will be
         updated with the new values and the return will be a reference to the
         same mutable sequence.

    :raises TypeError: If `out` is specified and does not support item.
    assignment either because it's immutable or not subscriptable.

    :raises IndexError: If `any` dimension of out is less than 3.

    :raises ValueError: If `ID` is not implicitly convertible to a C char type.

    :raises ValueError: If `ID` is not implicitly convertible to C char.

    :rtype: Tuple[MutableSequence[MutableSequence[float]], int]
    :returns:
        Tuple of `(J, err)`. `err` is `0` or
        :data:`forcedimension.dhd.constants.TIMEGUARD` on success, `-1`
        otherwise. `J` is the 3x3 jacobian matrix.

    """

    J = ((c_double * 3) * 3)()
    err = _libdhd.dhdGetDeltaJacobian(J, ID)

    if out is None:
        out = cast(
            MutableSequence[MutableSequence[float]],
            [list(row) for row in J]
        )
    else:
        for i in range(3):
            for j in range(3):
                out[i][j] = J[i][j]

    return (out, err)


_libdhd.dhdDeltaJointAnglesToJacobian.argtypes = [
    c_double,
    c_double,
    c_double,
    (c_double * 3) * 3,
    c_byte
]
_libdhd.dhdDeltaJointAnglesToJacobian.restype = c_int


def deltaJointAnglesToJacobian(
    joint_angles: CartesianTuple,
    ID: int = -1,
    out: Optional[MutableSequence[MutableSequence[float]]] = None
) -> Tuple[MutableSequence[MutableSequence[float]], int]:
    """
    Retrieve the 3x3 jacobian matrix for the DELTA structure
    based on a given joint configuration. Please refer to your device user
    manual for more information on your device coordinate system.

    :param CartesianTuple joint_angles: tuple of (j0, j1, j2) where.
    (j0, j1, j2) refer to the joint angles for axis 0, 1, and 2, respectively.

    :param int ID:
        Device ID (see multiple devices section for details), defaults to `-1`.

    :param Optional[MutableSequence[MutableSequence[float]]] out:
         Mutable sequence of mutable sequences to use intead of generating a
         new list of lists. If this is speicfied, the list provided will be
         updated with the new values and the return will be a reference to the
         same mutable sequence.

    :raises TypeError:
        If `out` is specified and does not support item assignment either
        because it's immutable or not subscriptable.

    :raises IndexError: If `out` is specified and `len(out) < 3`.
    :raises ValueError: If `ID` is not implicitly convertible to a C char type.

    :raises ValueError:
        If any member of `joint_angles` is not implicitly convertible to C
        double.

    :raises ValueError: If `ID` is not implicitly convertible to C char.

    :rtype: Tuple[MutableSequence[MutableSequence[float]], int]
    :returns: tuple of (J, err) where err is 0 on success, -1 otherwise and
    J is the 3x3 jacobian matrix.
    """

    J = ((c_double * 3) * 3)()

    err = _libdhd.dhdDeltaJointAnglesToJacobian(
        joint_angles[0],
        joint_angles[1],
        joint_angles[2],
        J,
        ID
    )

    if out is None:
        out = cast(
            MutableSequence[MutableSequence[float]],
            [list(row) for row in J]
        )
    else:
        for i in range(3):
            for j in range(3):
                out[i][j] = J[i][j]

    return (out, err)


_libdhd.dhdDeltaJointTorquesExtrema.argtypes = [
    c_double,
    c_double,
    c_double,
    c_double * 3,
    c_double * 3,
    c_byte
]
_libdhd.dhdDeltaJointTorquesExtrema.restype = c_int


def deltaJointTorquesExtrema(
    joint_angles: CartesianTuple,
    ID: int = -1,
    minq_out: Optional[MutableSequence[float]] = None,
    maxq_out: Optional[MutableSequence[float]] = None,
) -> Tuple[MutableSequence[float], MutableSequence[float], int]:
    """
    Compute the range of applicable DELTA joint torques for a given DELTA joint
    angle configuration. Please refer to your device user manual for more
    information on your device coordinate system.

    :param CartesianTuple joint_angles: tuple of (j0, j1, j2) where.
    (j0, j1, j2) refer to the joint angles for axis 0, 1, and 2, respectively.

    :param int ID:
        Device ID (see multiple devices section for details), defaults to `-1`.

    :param Optional[MutableSequence[float]] minq_out:
        Optional mutable sequence to use instead of generating a new list. If
        this is specified, the mutable sequence provided will be updated with
        the new values and the return will be a reference to the same mutable
        sequence passed in.

    :param Optional[MutableSequence[float]] maxq_out:
        Optional mutable sequence to use instead of generating a new list. If
        this is specified, the mutable sequence provided will be updated with
        the new values and the return will be a reference to the same mutable
        sequence passed in.

    :raises TypeError: If `minq_out` is specified and does not support item.
    assignment either because it's immutable or not subscriptable.

    :raises IndexError:
        If `minq_out` is specified and
        `len(encMin_out) < :data:forcedimension.dhd.constants.MAX_DOF`

    :raises TypeError:
        If `maxq_out` is specified and does not support item assignment either
        because it's immutable or not subscriptable.

    :raises IndexError:
        If `maxq_out` is specified and
        `len(out) < :data:forcedimension.dhd.constants.MAX_DOF`

    :raises ValueError:
        If any member of `joint_angles` is not implicitly convertible to
        C double.

    :raises ValueError: If `ID` is not implicitly convertible to C char.

    :rtype: Tuple[MutableSequence[float], MutableSequence[float], int]

    :returns:
        Tuple of `(minq, maxq, err)`. `err` is `0` on success, `-1`
        otherwise, `minq` is a mutable sequence of floats.
        `[minq1, minq2, minq3]`, which correspond to the minimum applicable
        joint torque to axes 0, 1, and 2, respectively in [Nm], and `maxq` is
        a list of floats, `[maxq1, maxq2, maxq3]`, which correspond
        to the maximium applicable joint torque to axes 0, 1, and 2,
        respectively in [Nm]
    """

    minq = (c_double * 3)()
    maxq = (c_double * 3)()

    err = _libdhd.dhdDeltaJointTorquesExtrema(
        joint_angles[0],
        joint_angles[1],
        joint_angles[2],
        minq,
        maxq,
        ID
    )

    if minq_out is None:
        minq_out = [v for v in minq]
    else:
        minq_out[0] = minq[0]
        minq_out[1] = minq[1]
        minq_out[2] = minq[2]

    if maxq_out is None:
        maxq_out = [v for v in minq]
    else:
        maxq_out[0] = maxq[0]
        maxq_out[1] = maxq[1]
        maxq_out[2] = maxq[2]

    return (minq_out, maxq_out, err)


_libdhd.dhdDeltaGravityJointTorques.argtypes = [
    c_double,
    c_double,
    c_double,
    POINTER(c_double),
    POINTER(c_double),
    POINTER(c_double),
    c_byte
]
_libdhd.dhdDeltaGravityJointTorques.retype = c_int


def deltaGravityJointTorques(
    joint_angles: CartesianTuple,
    ID: int = -1,
    out: Optional[MutableSequence[float]] = None
) -> Tuple[MutableSequence[float], int]:
    """
    Compute the DELTA joint torques required to compensate for gravity in a
    given DELTA joint angle configuration. Please refer to your device user
    manual for more information on your device coordinate system.

    :param CartesianTuple joint_angles:
        Tuple of `(j0, j1, j2)` where `j0`, `j1`, `j2` refer to the joint
        angles for axis 0, 1, and 2, respectively.

    :param int ID:
        Device ID (see multiple devices section for details), defaults to `-1`.

    :param Optional[MutableSequence[float]] out:
        Optional mutable sequence to use instead of generating a new list. If
        this is specified, the mutable sequence provided will be updated with
        the new values and the return will be a reference to the same mutable
        sequence passed in.

    :raises TypeError: If `out` is specified and does not support item.
    assignment either because it's immutable or not subscriptable.

    :raises IndexError: If `out` is specified and `len(out) < 3`.

    :raises ValueError:
        If any member of `joint_angles` is not implicitly convertible to
        C double.

    :raises ValueError: If `ID` is not implicitly convertible to C char.

    :rtype: Tuple[MutableSequence[float], int]

    :returns:
        Tuple of `([q0, q1, q2], err)`. `err` is `0` on success, `-1`
        otherwise and `[q0, q1, q2]` is a mutable sequence of the gravity
        compensation joint torques for axes 0, 1, and 2, respectively in [Nm].
    """

    q0 = c_double()
    q1 = c_double()
    q2 = c_double()

    err = _libdhd.dhdDeltaGravityJointTorques(
        joint_angles[0],
        joint_angles[1],
        joint_angles[2],
        byref(q0),
        byref(q1),
        byref(q2),
        ID
    )

    if out is None:
        out = [q0.value, q1.value, q2.value]
    else:
        out[0] = q0.value
        out[1] = q1.value
        out[2] = q2.value

    return (out, err)


_libdhd.dhdSetDeltaJointTorques.argtypes = [
    c_double,
    c_double,
    c_double,
    c_byte
]
_libdhd.dhdSetDeltaJointTorques.restype = c_int


def setDeltaJointTorques(t: CartesianTuple,
                         ID: int = -1) -> int:
    """
    Set all joint torques of the DELTA structure.

    :param CartesianTuple t:
        Tuple of `(t0, t1, t2)` where `t0`, `t1`, and `t2` are the DELTA axis
        torque commands for axes 0, 1, and 2, respectively in [Nm].

    :raises ValueError:
        If any member of `t` is not implicitly convertible to C double

    :raises ValueError:
        If `ID` is not implicitly convertible to C char.

    :rtype: int
    :returns: 0 on success, -1 otherwise
    """
    return _libdhd.dhdSetDeltaJointTorques(t[0], t[1], t[2], ID)


_libdhd.dhdDeltaEncodersToJointAngles.argtypes = [
    c_int,
    c_int,
    c_int,
    POINTER(c_double),
    POINTER(c_double),
    POINTER(c_double),
    c_byte
]
_libdhd.dhdDeltaEncodersToJointAngles.restype = c_int


def deltaEncodersToJointAngles(
    enc: DeviceTuple,
    ID: int = -1,
    out: Optional[MutableSequence[float]] = None
) -> Tuple[MutableSequence[float], int]:
    """
    This routine computes and returns the DELTA joint angles for a given set of
    encoder readings.

    :param DeviceTuple enc:
        Tuple of `(enc0, enc1, enc2)` where `enc0`, `enc1`, and `enc2`
        coresspond to encoder readings on axis 0, 1, and 2, respectively.

    :param int ID:
        Device ID (see multiple devices section for details), defaults to `-1`.

    :param Optional[MutableSequence[float]] out:
        Optional mutable sequence to use instead of generating a new list. If
        this is specified, the mutable sequence provided will be updated with
        the new values and the return will be a reference to the same mutable
        sequence passed in.

    :raises TypeError: If `out` is specified and does not support item.
    assignment either because it's immutable or not subscriptable.

    :raises IndexError: If `out` is specified and `len(out) < 3`.

    :raises ValueError: If any member of `enc` is not implicitly convertible.
    to C int.

    :raises ValueError: If `ID` is not implicitly convertible to C char.

    :rtype: Tuple[MutableSequence[float], int]
    :returns:
        Tuple of `([j0, j1, j2], err)` where `err` is `0` on success,
        `-1` otherwise and `[j0, j1, j1]` is a mutable sequence of floats
        corresponding to the DELTA joint angles on axes 0, 1, and 2,
        respectively in [rad].
    """

    j0 = c_double()
    j1 = c_double()
    j2 = c_double()

    err = _libdhd.dhdDeltaEncodersToJointAngles(
        enc[0],
        enc[1],
        enc[2],
        byref(j0),
        byref(j1),
        byref(j2),
        ID
    )

    if out is None:
        out = [j0.value, j1.value, j2.value]
    else:
        out[0] = j0.value
        out[1] = j1.value
        out[2] = j2.value

    return (out, err)


_libdhd.dhdDeltaJointAnglesToEncoders.argtypes = [
    c_double,
    c_double,
    c_double,
    POINTER(c_int),
    POINTER(c_int),
    POINTER(c_int),
    c_byte
]
_libdhd.dhdDeltaJointAnglesToEncoders.restype = c_int


def deltaJointAnglesToEncoders(
    joint_angles: CartesianTuple,
    ID: int = -1,
    out: Optional[MutableSequence[int]] = None
) -> Tuple[MutableSequence[int], int]:
    """
    This routine computes and returns the DELTA encoder readings for a given
    set of joint angles.

    :param CartesianTuple enc:
        Tuple of `(j0, j1, j1)` where `j0`, `j1`, and `j2` coresspond to DELTA
        joint angles for axes 0, 1, and 2, respectively, in [rad].

    :param int ID:
        Device ID (see multiple devices section for details), defaults to `-1`.

    :param Optional[MutableSequence[float]] out:
        Optional mutable sequence to use instead of generating a new list. If
        this is specified, the mutable sequence provided will be updated with
        the new values and the return will be a reference to the same mutable
        sequence passed in.

    :raises TypeError: If `out` is specified and does not support item.
    assignment either because it's immutable or not subscriptable.

    :raises IndexError: If `out` is specified and `len(out) < 3`.

    :raises ValueError: If any member of `joint_angles` is not implicitly.
    convertible to C double.

    :raises ValueError: If `ID` is not implicitly convertible to C char.

    :rtype: Tuple[MutableSequence[int], int]
    :returns:
        Tuple of `([enc0, enc1, enc2], err)` where `err` is `0` on success,
        `-1` otherwise and `[enc0, enc1, enc2]` is a mutable sequence of floats
        which correspond to the DELTA joint angles (in [rad]) on axes 0, 1,
        and 2, respectively.
    """

    enc0 = c_int()
    enc1 = c_int()
    enc2 = c_int()

    err = _libdhd.dhdDeltaJointAnglesToEncoders(
        joint_angles[0],
        joint_angles[1],
        joint_angles[2],
        byref(enc0),
        byref(enc1),
        byref(enc2),
        ID
    )

    if out is None:
        out = [enc0.value, enc1.value, enc2.value]
    else:
        out[0] = enc0.value
        out[1] = enc1.value
        out[2] = enc2.value

    return (out, err)


_libdhd.dhdGetWristJointAngles.argtypes = [
    POINTER(c_double),
    POINTER(c_double),
    POINTER(c_double),
    c_byte
]
_libdhd.dhdGetWristJointAngles.restype = c_int


def getWristJointAngles(
    ID: int = -1,
    out: Optional[MutableSequence[float]] = None
) -> Tuple[MutableSequence[float], int]:
    """
    Retrieve the joint angles in [rad] for the wrist structure.

    :param int ID:
        Device ID (see multiple devices section for details), defaults to `-1`.

    :param Optional[MutableSequence[float]] out:
        Optional mutable sequence to use instead of generating a new list. If
        this is specified, the mutable sequence provided will be updated with
        the new values and the return will be a reference to the same mutable
        sequence passed in.

    :raises TypeError: If `out` is specified and does not support item.
    assignment either because it's immutable or not subscriptable.

    :raises IndexError: If `out` is specified and `len(out) < 3`.

    :raises ValueError: If `ID` is not implicitly convertible to C char.

    :rtype: Tuple[MutableSequence[float], int]

    :returns:
        Tuple of `([j0, j1, j2], err)`. `err` is `0` or
        :data:`forcedimension.dhd.constants.TIMEGUARD` on success, `-1`
        otherwise. `[j0, j1, j2]` is a mutable sequence of floats which
        corresponds to the joint angle for joint angles for joint 0, 1, and 2,
        respectively.
    """

    j0 = c_double()
    j1 = c_double()
    j2 = c_double()
    err = _libdhd.dhdGetWristJointAngles(
        byref(j0),
        byref(j1),
        byref(j2),
        ID
    )

    if out is None:
        out = [j0.value, j1.value, j2.value]
    else:
        out[0] = j0.value
        out[1] = j1.value
        out[2] = j2.value

    return (out, err)


_libdhd.dhdGetWristJacobian.argtypes = [(c_double * 3) * 3, c_byte]
_libdhd.dhdGetWristJacobian.restype = c_int


def getWristJacobian(
    ID: int = -1,
    out: Optional[MutableSequence[MutableSequence[float]]] = None
) -> Tuple[MutableSequence[MutableSequence[float]], int]:
    """
    Retrieve the 3x3 jacobian matrix for the wrist structure based on the
    current end-effector position. Please refer to your device user manual for
    more information on your device coordinate system.

    :param int ID:
        Device ID (see multiple devices section for details), defaults to `-1`.

    :param Optional[MutableSequence[float]] out:
        Optional mutable sequence to use instead of generating a new list. If
        this is specified, the mutable sequence provided will be updated with
        the new values and the return will be a reference to the same mutable
        sequence passed in.

    :raises TypeError:
        If `out` is specified and does not support item assignment either
        because it's immutable or not subscriptable.

    :raises IndexError: If any dimension of out is less than 3.
    :raises ValueError: If `ID` is not implicitly convertible to a C char type.

    :raises ValueError: If `ID` is not implicitly convertible to C char.

    :rtype: Tuple[MutableSequence[MutableSequence[float]], int]
    :returns:
        Tuple of `(J, err)` where `err` is `0` or
        :data:`forcedimension.dhd.constants.TIMEGUARD` on success, `-1`
        otherwise and `J` is the 3x3 jacobian matrix.

    """

    J = ((c_double * 3) * 3)()

    err = _libdhd.dhdGetWristJacobian(J, ID)

    if out is None:
        out = cast(
            MutableSequence[MutableSequence[float]],
            [list(row) for row in J]
        )
    else:
        for i in range(3):
            for j in range(3):
                out[i][j] = J[i][j]

    return (out, err)


_libdhd.dhdWristJointAnglesToJacobian.argtypes = [
    c_double,
    c_double,
    c_double,
    (c_double * 3) * 3,
    c_byte
]
_libdhd.dhdWristJointAnglesToJacobian.restype = c_int


def wristJointAnglesToJacobian(
    joint_angles: CartesianTuple,
    ID: int = -1,
    out: Optional[MutableSequence[MutableSequence[float]]] = None
) -> Tuple[MutableSequence[MutableSequence[float]], int]:
    """
    Retrieve the 3x3 jacobian matrix for the wrist structure
    based on a given joint configuration. Please refer to your device user
    manual for more information on your device coordinate system.

    :param CartesianTuple joint_angles:
        Tuple of `(j0, j1, j2)` where `j0`, `j1`, and `j2` refer to the joint
        angles for wrist axis 0, 1, and 2, respectively.

    :param int ID:
        Device ID (see multiple devices section for details), defaults to `-1`.

    :param Optional[MutableSequence[MutableSequence[float]]] out:
         Mutable sequence of mutable sequences to use intead of generating a
         new list of lists. If this is speicfied, the list provided will be
         updated with the new values and the return will be a reference to the
         same mutable sequence.

    :raises TypeError: If `out` is specified and does not support item.
    assignment either because it's immutable or not subscriptable.

    :raises IndexError: If `out` is specified and `len(out) < 3`.
    :raises ValueError: If `ID` is not implicitly convertible to a C char type.

    :raises ValueError:
        If any member of `joint_angles` is not implicitly convertible to
        C double.

    :raises ValueError: If `ID` is not implicitly convertible to C char.

    :rtype: Tuple[MutableSequence[MutableSequence[float]], int]
    :returns:
        Tuple of `(J, err)` where `err` is `0` on success, `-1` otherwise and
        `J` is the 3x3 jacobian matrix.
    """

    J = ((c_double * 3) * 3)()

    err = _libdhd.dhdWristJointAnglesToJacobian(
        joint_angles[0],
        joint_angles[1],
        joint_angles[2],
        J,
        ID
    )

    if out is None:
        out = cast(
            MutableSequence[MutableSequence[float]],
            [list(row) for row in J]
        )
    else:
        for i in range(3):
            for j in range(3):
                out[i][j] = J[i][j]

    return (out, err)


_libdhd.dhdWristJointTorquesExtrema.argtypes = [
    c_double,
    c_double,
    c_double,
    c_double * 3,
    c_double * 3,
    c_byte
]
_libdhd.dhdWristJointTorquesExtrema.restype = c_int


def wristJointTorquesExtrema(
    joint_angles: CartesianTuple,
    ID: int = -1,
    minq_out: Optional[MutableSequence[float]] = None,
    maxq_out: Optional[MutableSequence[float]] = None,
) -> Tuple[MutableSequence[float], MutableSequence[float], int]:
    """
    Compute the range of applicable wrist joint torques for a given wrist joint
    angle configuration. Please refer to your device user manual for more
    information on your device coordinate system.

    :param CartesianTuple joint_angles: tuple of (j0, j1, j2) where.
    (j0, j1, j2) refer to the joint angles for wrist axes 0, 1, and 2,
    respectively.

    :param int ID:
        Device ID (see multiple devices section for details), defaults to `-1`.

    :param Optional[MutableSequence[float]] minq_out:
        Optional mutable sequence to use instead of generating a new list. If
        this is specified, the mutable sequence provided will be updated with
        the new values and the return will be a reference to the same mutable
        sequence passed in.

    :param Optional[MutableSequence[float]] maxq_out:
        Optional mutable sequence to use instead of generating a new list. If
        this is specified, the mutable sequence provided will be updated with
        the new values and the return will be a reference to the same mutable
        sequence passed in.

    :raises TypeError:
        If `minq_out` is specified and does not support item assignment either
        because it's immutable or not subscriptable.

    :raises IndexError:
        If `minq_out` is specified and
        `len(encMin_out) < :data:forcedimension.dhd.constants.MAX_DOF`

    :raises TypeError:
        If `maxq_out` is specified and does not support item assignment either
        because it's immutable or not subscriptable.

    :raises IndexError:
        If `maxq_out` is specified and
        `len(out) < :data:forcedimension.dhd.constants.MAX_DOF`

    :raises ValueError:
        If any member of `joint_angles` is not implicitly convertible to C
        double.

    :raises ValueError: If `ID` is not implicitly convertible to C char.

    :rtype: Tuple[MutableSequence[float], MutableSequence[float], int]

    :returns:
        Tuple of `(minq, maxq, err)` where `err` is `0` on success, `-1`
        otherwise, `minq` is a mutable sequence of floats,
        `[minq1, minq2, minq3]`, which correspond to the minimum applicable
        joint torques (in [Nm]) to axes 0, 1, and 2, respectively in.
        `maxq` is a mutable sequence of floats [maxq1, maxq2, maxq3] which
        correspond to the maximium applicable joint torques (in [Nm]) to axes
        0, 1, and 2, respectively.
    """

    minq = (c_double * 3)()
    maxq = (c_double * 3)()

    err = _libdhd.dhdWristJointTorquesExtrema(
        joint_angles[0],
        joint_angles[1],
        joint_angles[2],
        minq,
        maxq,
        ID
    )

    if minq_out is None:
        minq_out = [v for v in minq]
    else:
        minq_out[0] = minq[0]
        minq_out[1] = minq[1]
        minq_out[2] = minq[2]

    if maxq_out is None:
        maxq_out = [v for v in minq]
    else:
        maxq_out[0] = maxq[0]
        maxq_out[1] = maxq[1]
        maxq_out[2] = maxq[2]

    return (minq_out, maxq_out, err)


_libdhd.dhdWristGravityJointTorques.argtypes = [
    c_double,
    c_double,
    c_double,
    POINTER(c_double),
    POINTER(c_double),
    POINTER(c_double),
    c_byte
]
_libdhd.dhdWristGravityJointTorques.retype = c_int


def wristGravityJointTorques(
    joint_angles: CartesianTuple,
    ID: int = -1,
    out: Optional[MutableSequence[float]] = None
) -> Tuple[MutableSequence[float], int]:
    """
    Compute the wrist joint torques required to compensate for gravity in a
    given wrist joint angle configuration. Please refer to your device user
    manual for more information on your device coordinate system.

    :param CartesianTuple joint_angles:
        Tuple of `(j0, j1, j2)` where `j0`, `j1`, and `j2` refer to the joint
        angles for wrist axes 0, 1, and 2, respectively.

    :param int ID:
        Device ID (see multiple devices section for details), defaults to `-1`.

    :param Optional[MutableSequence[float]] out:
        Optional mutable sequence to use instead of generating a new list. If
        this is specified, the mutable sequence provided will be updated with
        the new values and the return will be a reference to the same mutable
        sequence passed in.

    :raises TypeError:
        If `out` is specified and does not support item assignment either
        because it's immutable or not subscriptable.

    :raises IndexError: If `out` is specified and `len(out) < 3`.

    :raises ValueError:
        If any member of `joint_angles` is not implicitly convertible to
        C double.

    :raises ValueError: If `ID` is not implicitly convertible to C char.

    :rtype: Tuple[MutableSequence[float], int]
    :returns:
        Tuple of `([q0, q1, q2], err)`. `err` is `0` on success, `-1`
        otherwise and `[q0, q1, q2]` are the gravity compensation joint torques
        (in [Nm]) for axes 0, 1, and 2, respectively.
    """

    q0 = c_double()
    q1 = c_double()
    q2 = c_double()

    err = _libdhd.dhdWristGravityJointTorques(
        joint_angles[0],
        joint_angles[1],
        joint_angles[2],
        byref(q0),
        byref(q1),
        byref(q2),
        ID
    )

    if out is None:
        out = [q0.value, q1.value, q2.value]
    else:
        out[0] = q0.value
        out[1] = q1.value
        out[2] = q2.value

    return (out, err)


_libdhd.dhdSetWristJointTorques.argtypes = [
    c_double,
    c_double,
    c_double,
    c_byte
]
_libdhd.dhdSetWristJointTorques.restype = c_int


def setWristJointTorques(t: CartesianTuple,
                         ID: int = -1) -> int:
    """
    Set all joint torques of the wrist structure.

    :param CartesianTuple t:
        Sequence of (t0, t1, t2) where t0, t1, t2 are the wrist axis torque
        commands for axes 0, 1, and 2, respectively in [Nm].

    :param int ID:
        Device ID (see multiple devices section for details), defaults to `-1`.

    :raises ValueError:
        If any member of `t` is not implicitly convertible to C double

    :raises ValueError: If `ID` is not implicitly convertible to C char.

    :rtype: int
    :returns: `0` on success, `-1` otherwise.
    """
    return _libdhd.dhdSetWristJointTorques(t[0], t[1], t[2], ID)


_libdhd.dhdSetForceAndWristJointTorques.argtypes = [
    c_double,
    c_double,
    c_double,
    c_double,
    c_double,
    c_double,
    c_byte
]
_libdhd.dhdSetForceAndWristJointTorques.restype = c_int


def setForceAndWristJointTorques(
    f: CartesianTuple, t: CartesianTuple, ID: int = -1
) -> int:
    """
    Set Cartesian force and wrist joint torques

    :param CartesianTuple f:
        Sequence of `(fx, fy, fz)` where `fx`, `fy`, and `fz` are the force on
        the  DELTA end-effector on the X, Y, and Z axes, respectively in [N].

    :param CartesianTuple t: tuple of (t0, t1, t2) where.
    (t0, t1, t2) are the wrist axis torque commands for axes 0, 1, and 2,
    respectively in [Nm].

    :param int ID:
        Device ID (see multiple devices section for details), defaults to `-1`.

    :raises ValueError: If any member of `f` is not implicitly convertible to.
    c_double

    :raises ValueError: If any member of `t` is not implicitly convertible to.
    c_double

    :raises ValueError: If `ID` is not implicitly convertible to C char.

    :rtype: int
    :returns: 0 on success, -1 otherwise
    """
    return _libdhd.dhdSetForceAndWristJointTorques(
        f[0],
        f[1],
        f[2],
        t[0],
        t[1],
        t[2],
        ID
    )


_libdhd.dhdSetForceAndWristJointTorques.argtypes = [
    c_double,
    c_double,
    c_double,
    c_double,
    c_double,
    c_double,
    c_double,
    c_byte
]
_libdhd.dhdSetForceAndWristJointTorquesAndGripperForce.restype = c_int


def setForceAndWristJointTorquesAndGripperForce(
        f: CartesianTuple,
        t: CartesianTuple,
        fg: float,
        ID: int = -1) -> int:
    """
    Set Cartesian force, wrist joint torques, and gripper force

    :param CartesianTuple f:
        Sequence of `(fx, fy, fz)` where `fx`, `fy`, and `fz` (in [N]) are the
        forces on the DELTA end-effector on the X, Y, and Z axes respectively.

    :param CartesianTuple t:
        Tuple of (t0, t1, t2) where `t0`, `t1`, `t2` are the wrist axis torque
        (in [Nm]) commands for axes 0, 1, and 2, respectively.

    :param float fg: gripper force in [N].

    :param int ID:
        Device ID (see multiple devices section for details), defaults to `-1`.

    :raises ValueError:
        If any member of `f` is not implicitly convertible to C double.

    :raises ValueError:
        If any member of `t` is not implicitly convertible to C double.

    :raises ValueError:
        If `gripper_force` is not implicitly convertible to C double.

    :raises ValueError: If `ID` is not implicitly convertible to C char.

    :rtype: int
    :returns: 0 on success, -1 otherwise
    """
    return _libdhd.dhdSetForceAndWristJointTorquesAndGripperForce(
        f[0],
        f[1],
        f[2],
        t[0],
        t[1],
        t[2],
        fg,
        ID
    )


_libdhd.dhdWristEncodersToJointAngles.argtypes = [
    c_int,
    c_int,
    c_int,
    POINTER(c_double),
    POINTER(c_double),
    POINTER(c_double),
    c_byte
]
_libdhd.dhdWristEncodersToJointAngles.restype = c_int


def wristEncodersToJointAngles(
    enc: DeviceTuple,
    ID: int = -1,
    out: Optional[MutableSequence[float]] = None
) -> Tuple[MutableSequence[float], int]:
    """
    This routine computes and returns the wrist joint angles for a given set of
    encoder readings.

    :param DeviceTuple enc:
        Sequence of (enc0, enc1, enc2) where `enc0`, `enc1`, and `enc2`
        coresspond to encoder readings on wrist axes 0, 1, and 2, respectively.

    :param Optional[MutableSequence[float]] out:
        Optional mutable sequence to use instead of generating a new list. If
        this is specified, the mutable sequence provided will be updated with
        the new values and the return will be a reference to the same mutable
        sequence passed in.

    :param int ID:
        Device ID (see multiple devices section for details), defaults to `-1`.

    :raises ValueError:
        If any member of `enc` is not implicitly convertible to C int.

    :raises ValueError: If `ID` is not implicitly convertible to C char.

    :rtype: Tuple[MutableSequence[float], int]
    :returns:
        Tuple of `([j0, j1, j2], err)` where `err` is 0 on success,
        -1 otherwise and `[j0, j1, j1]` is a mutable sequence of floats
        corresponding to the wrist joint angles (in [rad]) on axes 0, 1, and 2,
        respectively.
    """

    j0 = c_double()
    j1 = c_double()
    j2 = c_double()

    err = _libdhd.dhdWristEncodersToJointAngles(
        enc[0],
        enc[1],
        enc[2],
        byref(j0),
        byref(j1),
        byref(j2),
        ID
    )

    if out is None:
        out = [j0.value, j1.value, j2.value]
    else:
        out[0] = j0.value
        out[1] = j1.value
        out[2] = j2.value

    return (out, err)


_libdhd.dhdWristJointAnglesToEncoders.argtypes = [
    c_double,
    c_double,
    c_double,
    POINTER(c_int),
    POINTER(c_int),
    POINTER(c_int),
    c_byte
]
_libdhd.dhdWristJointAnglesToEncoders.restype = c_int


def wristJointAnglesToEncoders(
    joint_angles: CartesianTuple,
    ID: int = -1,
    out: Optional[MutableSequence[int]] = None
) -> Tuple[MutableSequence[int], int]:
    """
    This routine computes and returns the wrist encoder readings for a given
    set of wrist joint angles.

    :param CartesianTuple enc:
        Tuple of `(j0, j1, j1)` where `j0`, `j1`, and `j2` coresspond to wrist
        joint angles for axes 0, 1, and 2, respectively, in [rad].

    :param int ID:
        Device ID (see multiple devices section for details), defaults to `-1`.

    :param Optional[MutableSequence[int]] out:
        Optional mutable sequence to use instead of generating a new list. If
        this is specified, the mutable sequence provided will be updated with
        the new values and the return will be a reference to the same mutable
        sequence passed in.

    :raises TypeError: If `out` is specified and does not support item.
    assignment either because it's immutable or not subscriptable.

    :raises IndexError: If `out` is specified and `len(out) < 3`.

    :raises ValueError:
        If any member of `joint_angles` is not implicitly convertible to
        C double.

    :raises ValueError: If `ID` is not implicitly convertible to C char.

    :rtype: Tuple[MutableSequence[int], int]
    :returns:
        Tuple of `([enc0, enc1, enc2], err)`. `err` is `0` on success,
        `-1` otherwise and `[enc0, enc1, enc2]` correspond to the wrist joint
        angles (in [rad]) on axes 0, 1, and 2, respectively.
    """

    enc0 = c_int()
    enc1 = c_int()
    enc2 = c_int()

    err = _libdhd.dhdWristJointAnglesToEncoders(
        joint_angles[0],
        joint_angles[1],
        joint_angles[2],
        byref(enc0),
        byref(enc1),
        byref(enc2),
        ID
    )

    if out is None:
        out = [enc0.value, enc1.value, enc2.value]
    else:
        out[0] = enc0.value
        out[1] = enc1.value
        out[2] = enc2.value

    return (out, err)


_libdhd.dhdGetJointAngles.argtypes = [c_double * MAX_DOF, c_byte]
_libdhd.dhdGetJointAngles.restype = c_int


def getJointAngles(
    ID: int = -1,
    out: Optional[MutableSequence[float]] = None
) -> Tuple[MutableSequence[float], int]:
    """
    Retrieve the joint angles in [rad] for all sensed degrees-of-freedom of the
    current device.

    :param int ID:
        Device ID (see multiple devices section for details), defaults to `-1`.

    :param Optional[MutableSequence[float]] out:
        Optional mutable sequence to use instead of generating a new list. If
        this is specified, the mutable sequence provided will be updated with
        the new values and the return will be a reference to the same mutable
        sequence passed in.

    :raises TypeError:
        If `out` is specified and does not support item assignment either
        because it's immutable or not subscriptable.

    :raises IndexError:
        If `out` is specified and.
        `len(out) < :data:forcedimension.dhd.constants.MAX_DOF`

    :raises ValueError: If `ID` is not implicitly convertible to C char.

    :rtype: Tuple[MutableSequence[float], int]
    :returns:
        Tuple of `(joint_angles, err)`. err is `0` or
        :data:`forcedimension.dhd.constants.TIMEGUARD` on success, `-1`
        otherwise. `joint_angles` is a mutable sequence of joint angles
        (in [rad]) with a length equal to
        :data:`forcedimension.dhd.constants.MAX_DOF`.
    """

    joint_angles = (c_int * MAX_DOF)()

    err = _libdhd.dhdGetJointAngles(joint_angles, ID)

    if out is None:
        out = [val for val in joint_angles]
    else:
        for i in range(MAX_DOF):
            out[i] = joint_angles[i]

    return (out, err)


_libdhd.dhdGetJointVelocities.argtypes = [c_double * MAX_DOF, c_byte]
_libdhd.dhdGetJointVelocities.restype = c_int


def getJointVelocities(
    ID: int = -1,
    out: Optional[MutableSequence[float]] = None
) -> Tuple[MutableSequence[float], int]:
    """
    Retrieve the joint angle velocities in [rad/s] for all sensed
    degrees-of-freedom of the current device.

    :param int ID:
        Device ID (see multiple devices section for details), defaults to `-1`.

    :param Optional[MutableSequence[float]] out:
        Optional mutable sequence to use instead of generating a new list. If
        this is specified, the mutable sequence provided will be updated with
        the new values and the return will be a reference to the same mutable
        sequence passed in.

    :raises TypeError: If `out` is specified and does not support item.
    assignment either because it's immutable or not subscriptable.

    :raises IndexError:
        If `out` is specified and
        `len(out) < :data:forcedimension.dhd.constants.MAX_DOF`

    :raises ValueError: If `ID` is not implicitly convertible to C char.

    :rtype: Tuple[MutableSequence[float], int]
    :returns:
        Tuple of `(v, err)`. `err` is `0` or
        :data:`forcedimension.dhd.constants.TIMEGUARD` on success,
        -1 otherwise. `v` is a mutable sequence of joint angle velocities
        (in [rad/s]) with length :data:`forcedimension.dhd.constants.MAX_DOF`.
    """

    w = (c_int * MAX_DOF)()

    err = _libdhd.dhdGetJointVelocities(w, ID)
    if out is None:
        out = [val for val in w]
    else:
        for i in range(MAX_DOF):
            out[i] = w[i]

    return (out, err)


_libdhd.dhdGetEncVelocities.argtypes = [c_double * MAX_DOF, c_byte]
_libdhd.dhdGetEncVelocities.restype = c_int


def getEncVelocities(
    ID: int = -1,
    out: Optional[MutableSequence[float]] = None
) -> Tuple[MutableSequence[float], int]:
    """
    Retrieve the joint angle velocities in [increments/s] for all sensed
    degrees-of-freedom of the current device

    :param int ID:
        Device ID (see multiple devices section for details), defaults to `-1`.

    :param Optional[MutableSequence[float]] out:
        Optional mutable sequence to use instead of generating a new list. If
        this is specified, the mutable sequence provided will be updated with
        the new values and the return will be a reference to the same mutable
        sequence passed in.

    :raises TypeError:
        If `out` is specified and does not support item assignment either
        because it's immutable or not subscriptable.

    :raises IndexError:
        If `out` is specified and
        `len(out) < :data:forcedimension.dhd.constants.MAX_DOF`

    :raises ValueError: If `ID` is not implicitly convertible to C char.

    :rtype: Tuple[MutableSequence[float], int]

    :returns:
        Tuple of `(v, err)`. `err` is `0` or
        :data:`forcedimension.dhd.constants.TIMEGUARD` on success, `-1`
        otherwise. `v` is a list of joint angle velocities in [increments/s].
    """

    v = (c_int * MAX_DOF)()

    err = _libdhd.dhdGetEncVelocities(v, ID)
    if out is None:
        out = [val for val in v]
    else:
        for i in range(MAX_DOF):
            out[i] = v[i]

    return (out, err)


_libdhd.dhdJointAnglesToInertiaMatrix.argtypes = [
    c_double * MAX_DOF,
    (c_double * 6) * 6,
    c_byte
]
_libdhd.dhdJointAnglesToInertiaMatrix.restype = c_int


def jointAnglesToIntertiaMatrix(
    joint_angles: DOFTuple,
    ID: int = -1,
    out: Optional[MutableSequence[MutableSequence[float]]] = None
) -> Tuple[MutableSequence[MutableSequence[float]], int]:
    """
    Retrieve the (Cartesian) inertia matrix based on a given joint
    configuration. Please refer to your device user manual for more information
    on your device coordinate system.

    :param int ID:
        Device ID (see multiple devices section for details), defaults to `-1`.

    :raises ValueError:
        If `joint_angles` is not implicitly convertible to C double[MAX_DOF]

    :param Optional[MutableSequence[MutableSequence[float]]] out:
         Mutable sequence of mutable sequences to use intead of generating a
         new list of lists. If this is speicfied, the list provided will be
         updated with the new values and the return will be a reference to the
         same mutable sequence.

    :raises TypeError:
        If `out` is specified and does not support item assignment either
        because it's immutable or not subscriptable.

    :raises IndexError:
        if `out` is specified and the length of the any dimension of `out` is
        less than 6.

    :raises ValueError: If `ID` is not implicitly convertible to C char.

    :rtype: Tuple[MutableSequence[MutableSequence[float]], int]

    :returns:
        Tuple of `(inertia, err)`. `err` is 0 on success, -1 otherwise.
        `inertia` is the 6x6 Cartesian inertia matrix.
    """

    inertia = ((c_double * 6) * 6)()
    err = _libdhd.dhdJointAnglesToInertiaMatrix(
        joint_angles,
        inertia,
        ID
    )

    if out is None:
        out = cast(
            MutableSequence[MutableSequence[float]],
            [list(row) for row in inertia]
        )
    else:
        for i in range(6):
            for j in range(6):
                out[i][j] = inertia[i][j]

    return (out, err)


_libdhd.dhdSetComMode.argtypes = [c_int, c_byte]
_libdhd.dhdSetComMode.restype = c_int


def setComMode(mode: ComMode, ID: int = -1) -> int:
    """
    Set the COM operation mode on compatible devices.

    :param ComMode mode: desired COM operation mode.

    :param int ID:
        Device ID (see multiple devices section for details), defaults to `-1`.

    :raises ValueError: If `mode` is not implicitly convertible to C int.
    :raises ValueError: If `ID` is not implicitly convertible to C char.

    :rtype: int

    :returns: `0` on success, `-1` otherwise.
    """

    return _libdhd.dhdSetComMode(mode, ID)


_libdhd.dhdSetWatchdog.argtypes = [c_ubyte, c_byte]
_libdhd.dhdSetWatchdog.restype = c_int


def setWatchdog(duration: int, ID: int = -1) -> int:
    """
    Set the watchdog duration in multiples of 125 microseconds on compatible
    devices. If the watchdog duration is exceeded before the device recieves a
    new force command, the device firmware will disable forces.

    A value of 0 disables the watchdog feature.

    See Also
    --------
    :func:`forcedimension.dhd.expert.getWatchdog()`

    :param int duration: watchdog duration in multiples of 125 us.

    :param int ID:
        Device ID (see multiple devices section for details), defaults to `-1`.

    :raises ValueError: If `duration` is not implicitly convertible to C uchar.
    :raises ValueError: If `ID` is not implicitly convertible to C char.

    :rtype: int

    :returns: `0` on success, `-1` otherwise.
    """

    return _libdhd.dhdSetWatchdog(duration, ID)


_libdhd.dhdGetWatchdog.argtypes = [POINTER(c_ubyte), c_byte]
_libdhd.dhdGetWatchdog.restype = c_int


def getWatchdog(ID: int = -1) -> Tuple[int, int]:
    """
    Get the watchdog duration in multiples of 125 us on compatible
    devices.

    See Also
    --------
    :func:`forcedimension.dhd.expert.setWatchdog()`

    :param int ID:
        Device ID (see multiple devices section for details), defaults to `-1`.

    :raises ValueError: If `ID` is not implicitly convertible to C char.

    :rtype: Tuple[int, int]

    :returns:
        Tuple of `(duration, err)` where `err` is 0 on success, -1 otherwise.
        `duration` is the watchdog duration in multiples of 125 us
    """

    duration = c_int()

    return (duration.value, _libdhd.dhdSetWatchdog(byref(duration), ID))
