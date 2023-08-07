import forcedimension.runtime as _runtime
from forcedimension.typing import (
    SupportsPtr, SupportsPtrs3, c_double_ptr, c_int_ptr, c_ushort_ptr,
    FloatVectorLike, IntVectorLike
)

from typing import Tuple

from ctypes import c_int, c_byte, c_double, c_ushort, c_ubyte

_libdhd = _runtime.load("libdrd")

if _libdhd is None:
    raise ImportError("There were problems loading libdhd.")

_libdhd.dhdGetDeltaEncoders.argtypes = [
    c_int_ptr,
    c_int_ptr,
    c_int_ptr,
    c_byte
]
_libdhd.dhdGetDeltaEncoders.restype = c_int


def getDeltaEncoders(out: SupportsPtrs3[c_int], ID: int = -1) -> int:
    """
    Read all encoders values of the DELTA structure.

    :param int ID:
        Device ID (see multiple devices section for details).

    :param SupportsPtrs3[c_int] out:
        An output buffer to store the delta encoder values.

    :raises IndexError:
        If ``len(out) < 3``.

    :raises AttributeError:
        If ``out.ptrs`` is not a valid attribute of ``out``

    :raises TypeError:
        If ``out.ptrs`` is not iterable.

    :raises ArgumentError:
        If ``out.ptrs`` does not expand into a tuple of 3
        ``Pointer[c_double]`` types.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to a C char.

    :returns:
        0 or :data:`forcedimension.dhd.constants.TIMEGUARD` on success, and
        -1 otherwise.
    """

    return _libdhd.dhdGetDeltaEncoders(*out.ptrs, ID)


_libdhd.dhdGetWristEncoders.argtypes = [
    c_int_ptr,
    c_int_ptr,
    c_int_ptr,
    c_byte
]
_libdhd.dhdGetWristEncoders.restype = c_int


def getWristEncoders(out: SupportsPtrs3[c_int], ID: int = -1) -> int:
    """
    Read all encoders values of the wrist structure.

    This feature only applies to the following devices
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA33`
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA33_LEFT`
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA331`
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA331_LEFT`
        :data:`forcedimension.dhd.constants.DeviceType.SIGMA331`
        :data:`forcedimension.dhd.constants.DeviceType.SIGMA331_LEFT`

    :param SupportsPtrs3[c_int] out:
        An output buffer to store the wrist encoder values.

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
        If ``ID`` is not implicitly convertible to a C char.

    :returns:
        0 or :data:`forcedimension.dhd.constants.TIMEGUARD` on success,
        -1 otherwise.
    """

    return _libdhd.dhdGetWristEncoders(*out.ptrs, ID)


_libdhd.dhdDeltaEncoderToPosition.argtypes = [
    c_int,
    c_int,
    c_int,
    c_double_ptr,
    c_double_ptr,
    c_double_ptr,
    c_byte
]
_libdhd.dhdDeltaEncoderToPosition.restype = c_int


def deltaEncoderToPosition(
    enc: IntVectorLike,
    out: SupportsPtrs3[c_double],
    ID: int = -1
) -> int:
    """
    This routine computes and returns the position of the end-effector about
    the X, Y, and Z axes (in [m]) for a given set of encoder values.

    :param IntVectorLike enc:
        Sequence of ``(enc0, enc1, enc2)`` where ``enc0``, ``enc1``, and
        ``enc2`` refer to raw encoder values on axis 0, 1, and 2,
        respectively.

    :param int ID:
        Device ID (see multiple devices section for details).

    :param SupportsPtrs3[c_double] out:
        An output buffer to store the position about the X, Y, and Z axes
        (in [m]).

    :raises ArgumentError:
        If any element of ``enc`` is not implicitly convertible to a C int.

    :raises IndexError:
        If ``len(enc) < 3``.

    :raises TypeError:
        If ``enc`` is not subscriptable.

    :raises AttributeError:
        If ``out.ptrs`` is not a valid attribute of ``out``

    :raises TypeError:
        If ``out.ptrs`` is not iterable.

    :raises ArgumentError:
        If ``out.ptrs`` does not expand into a tuple of 3
        ``Pointer[c_double]`` types.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to a C char.

    :returns:
        0 on success, -1 otherwise.
    """

    return _libdhd.dhdDeltaEncoderToPosition(
        enc[0], enc[1], enc[2], *out.ptrs, ID
    )


_libdhd.dhdDeltaPositionToEncoder.argtypes = [
    c_int,
    c_int,
    c_int,
    c_double_ptr,
    c_double_ptr,
    c_double_ptr,
    c_byte
]
_libdhd.dhdDeltaPositionToEncoder.restype = c_int


def deltaPositionToEncoder(
    pos: FloatVectorLike,
    out: SupportsPtrs3[c_int],
    ID: int = -1,
) -> int:
    """
    Computes and returns the encoder values of the end-effector
    for a given end-effector position.

    :param FloatVectorLike pos:
        Sequence of ``(px, py, pz)`` where ``px``, ``py``, and ``pz``
        refer to the end-effector position on the X, Y, and Z axes,
        respectively (in [m]).

    :param int ID:
        Device ID (see multiple devices section for details).

    :param SupportsPtrs3[c_int] out:
        An output buffer to store the delta encoder values.

    :raises AttributeError:
        If ``out.ptrs`` is not a valid attribute of ``out``

    :raises TypeError:
        If ``out.ptrs`` is not iterable.

    :raises ArgumentError:
        If ``out.ptrs`` does not expand into a tuple of 3
        ``Pointer[c_double]`` types.

    :raises ArgumentError:
        If any element of ``pos`` is not implicitly convertible to a C double.

    :raises IndexError:
        If ``len(pos) < 3``.

    :raises TypeError:
        If ``pos`` is not subscriptable.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to a C char.

    :returns:
        0 on success, -1 otherwise.
    """

    return _libdhd.dhdDeltaPositionToEncoder(
        pos[0], pos[1], pos[2], *out.ptrs, ID
    )


_libdhd.dhdDeltaMotorToForce.argtypes = [
    c_ushort,
    c_ushort,
    c_ushort,
    c_int,
    c_int,
    c_int,
    c_double_ptr,
    c_double_ptr,
    c_double_ptr,
    c_byte
]
_libdhd.dhdDeltaMotorToForce.restype = c_int


def deltaMotorToForce(
    mot: IntVectorLike,
    enc: IntVectorLike,
    out: SupportsPtrs3[c_double],
    ID: int = -1
) -> int:
    """
    This routine computes and returns the force applied to the end-effector
    about the X, Y, and Z axes (in [N]) for a given set of motor commands at a
    given position (defined by encoder readings)

    :param IntVectorLike mot:
        Sequence of ``(mot0, mot1, mot2)`` where ``mot0``, ``mot1``,
        and ``mot2`` are the axis 0, 1, and 2 DELTA motor commands,
        respectively.

    :param IntVectorLike enc:
        Sequence of ``(enc0, enc1, enc2)`` where ``enc0``, ``enc1``, and
        ``enc2`` refer to encoder values on axis 0, 1, and 2,
        respectively.

    :param SupportsPtrs3[c_double] out:
        An output buffer to store the force applied to the end effector about
        the X, Y, and Z axes (in [N])

    :param int ID:
        Device ID (see multiple devices section for details).

    :raises TypeError:
        If ``out`` does not support item assignment either
        because it's immutable or not subscriptable.

    :raises IndexError:
        If ``len(out) < 3``.

    :raises ArgumentError:
        If any element of ``output`` is not implicitly convertible to a C
        ushort.

    :raises IndexError:
        If ``len(output) < 3``.

    :raises TypeError:
        If ``output`` is not subscriptable.

    :raises ArgumentError:
        If any element of ``enc`` is not implicitly convertible to a C char.

    :raises IndexError:
        If ``len(enc) < 3``

    :raises TypeError:
        If ``enc`` is not subscriptable.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to a C char.

    :returns:
        0 on success, -1 otherwise.
    """

    return _libdhd.dhdDeltaMotorToForce(
        mot[0], mot[1], mot[2],
        enc[0], enc[1], enc[2],
        *out.ptrs,
        ID
    )


_libdhd.dhdDeltaForceToMotor.argtypes = [
    c_double,
    c_double,
    c_double,
    c_int,
    c_int,
    c_int,
    c_ushort_ptr,
    c_ushort_ptr,
    c_ushort_ptr,
    c_byte
]
_libdhd.dhdDeltaForceToMotor.restype = c_int


def deltaForceToMotor(
    f: FloatVectorLike,
    enc: IntVectorLike,
    out: SupportsPtrs3[c_ushort],
    ID = -1
) -> int:
    """
    This routine computes and returns the motor commands necessary to obtain a
    given force on the end-effector at a given position (defined by encoder
    readings).

    :param FloatVectorLike f:
        Sequence of ``(fx, fy, fz)`` where ``fx``, ``fy``, and ``fz`` are the
        force on the DELTA end-effector on the X, Y, and Z axes, respectively
        (in [N]).

    :param IntVectorLike enc:
        Sequence of ``(enc0, enc1, enc2)`` where ``enc0``, ``enc1``, and
        ``enc2`` refer to encoder values on axis 0, 1, and 2, respectively.

    :param SupportsPtrs3[c_ushort] out:
        An output buffer to store the motor commands on axes 0, 1, and 2.

    :param int ID:
        Device ID (see multiple devices section for details).

    :raises ArgumentError:
        If any element of ``f`` is not implicitly convertible to a C double.

    :raises IndexError:
        If ``len(f) < 3``.

    :raises TypeError:
        If ``f`` is not subscriptable.

    :raises ArgumentError:
        If any element of ``enc`` is not implicitly convertible to a C int.

    :raises IndexError:
        If ``len(enc) < 3``.

    :raises TypeError:
        If ``enc`` is not subscriptable.

    :raises AttributeError:
        If ``out.ptrs`` is not a valid attribute of ``out``

    :raises TypeError:
        If ``out.ptrs`` is not iterable.

    :raises ArgumentError:
        If ``out.ptrs`` does not expand into a tuple of 3
        ``Pointer[c_ushort]`` types.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to a C char.

    :returns:
        0 or :data:`forcedimension.dhd.constants.MOTOR_SATURATED` on success,
        and -1 otherwise.
    """

    return _libdhd.dhdDeltaMotorToForce(
        f[0], f[1], f[2],
        enc[0], enc[1], enc[2],
        *out.ptrs,
        ID
    )


_libdhd.dhdWristEncoderToOrientation.argtypes = [
    c_int,
    c_int,
    c_int,
    c_double_ptr,
    c_double_ptr,
    c_double_ptr,
    c_byte
]
_libdhd.dhdWristEncoderToOrientation.restype = c_int


def wristEncoderToOrientation(
    enc: IntVectorLike,
    out: SupportsPtrs3[c_double],
    ID: int = -1,
) -> int:
    """
    For devices with a wrist structure, compute the individual angle of each
    joint (in [rad]), starting with the one located nearest to the wrist base
    plate. For the :data:`forcedimension.dhd.constants.DeviceType.OMEGA33` and
    the :data:`forcedimension.dhd.constants.DeviceType.OMEGA33_LEFT` devices,
    angles are computed with respect to their internal reference frame, which
    is rotated 45 degrees or π/4 radians about the Y axis. Please refer to your
    device user manual for more information on your device coordinate system.

    This feature only applies to the following devices
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA33`
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA33_LEFT`
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA331`
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA331_LEFT`
        :data:`forcedimension.dhd.constants.DeviceType.SIGMA331`
        :data:`forcedimension.dhd.constants.DeviceType.SIGMA331_LEFT`

    :param IntVectorLike enc:
        Sequence of ``(enc0, enc1, enc2)`` where ``enc0``, ``enc1``, and
        ``enc2`` refer to wrist encoder values on the first, second, and
        third joint, respectively

    :param SupportsPtrs3[c_double] out:
        An output buffer to store the wrist end-effector orientation about the
        first, second, and third wrist joints (in [rad]).

    :raises AttributeError:
        If ``out.ptrs`` is not a valid attribute of ``out``

    :raises TypeError:
        If ``out.ptrs`` is not iterable.

    :raises ArgumentError:
        If ``out.ptrs`` does not expand into a tuple of 3
        ``Pointer[c_double]`` types.

    :raises ArgumentError:
        If any element of ``enc`` is not implicitly convertible
        to a C int.

    :raises IndexError:
        If ``len(enc) < 3``.

    :raises TypeError:
        If ``enc`` is not subscriptable.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to a C char.

    :returns:
        0 on success, -1 otherwise.

    """
    return _libdhd.dhdDeltaEncoderToPosition(
        enc[0], enc[1], enc[2],
        *out.ptrs,
        ID
    )


_libdhd.dhdWristOrientationToEncoder.argtypes = [
    c_double,
    c_double,
    c_double,
    c_int_ptr,
    c_int_ptr,
    c_int_ptr,
    c_byte
]
_libdhd.dhdWristOrientationToEncoder.restype = c_int


def wristOrientationToEncoder(
    orientation: FloatVectorLike,
    out: SupportsPtrs3[c_int],
    ID: int = -1,
) -> int:
    """
    For devices with a wrist structure, compute the encoder values from the
    individual angle of each joint, starting witht he one located nearest to
    the wrist plate base. For the dhd.libdhd.DeviceType.OMEGA33 and
    dhd.libdhd.DeviceType.OMEGA33_LEFT devices, angles must be expressed
    withrespect to their internal reference frame, which is rotated 45 degrees
    or π/4 radians about the Y axis. Please refer to your device user manual
    for more information on your device coordinate system.

    This feature only applies to the following devices
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA33`
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA33_LEFT`
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA331`
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA331_LEFT`
        :data:`forcedimension.dhd.constants.DeviceType.SIGMA331`
        :data:`forcedimension.dhd.constants.DeviceType.SIGMA331_LEFT`

    :param FloatVectorLike orientation:
        Sequence of ``(oa, ob, og)`` where ``oa``, ``ob``, and ``og`` refer to
        wrist end effector orientation (in [rad]) around the X, Y, and Z axes,
        respectively.

    :param SupportsPtrs3[c_int] out:
        An output buffer to store the encoder values.

    :param int ID:
        Device ID (see multiple devices section for details).


    :raises AttributeError:
        If ``out.ptrs`` is not a valid attribute of ``out``

    :raises TypeError:
        If ``out.ptrs`` is not iterable.

    :raises ArgumentError:
        If ``out.ptrs`` does not expand into a tuple of 3
        ``Pointer[c_int]`` types.

    :raises ArgumentError:
        If any element of ``orientation`` is not implicitly convertible to a C
        double.

    :raises IndexError:
        If ``len(orientation) < 3``.

    :raises TypeError:
        If ``orientation`` is not subscriptable.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to a C char.

    :returns:
        0 on success, -1 otherwise
    """

    return _libdhd.dhdDeltaEncoderToPosition(
        orientation[0], orientation[1], orientation[2],
        *out.ptrs,
        ID
    )


_libdhd.dhdWristMotorToTorque.argtypes = [
    c_ushort,
    c_ushort,
    c_ushort,
    c_int,
    c_int,
    c_int,
    c_double_ptr,
    c_double_ptr,
    c_double_ptr,
    c_byte
]
_libdhd.dhdWristMotorToTorque.restype = c_int


def wristMotorToTorque(
    output: IntVectorLike,
    enc: IntVectorLike,
    out: SupportsPtrs3[c_double],
    ID: int = -1
) -> int:
    """
    This routine computes and returns the torque applied to the wrist
    end-effector about the, X, Y, and Z axes (in [Nm]) for a given set of motor
    commands at a given orientation (defined by encoder values)

    :param IntVectorLike cmd:
        Sequence of ``(cmd0, cmd1, cmd2)`` where ``cmd0``, ``cmd1``,
        and ``cmd2`` are the axis 0, 1, and 2 DELTA motor commands,
        respectively.

    :param IntVectorLike enc:
        Sequence of ``(enc0, enc1, enc2)`` where ``enc0``, ``enc1``, and
        ``enc2`` refer to encoder values on axis 0, 1, and 2, respectively.

    :param int ID:
        Device ID (see multiple devices section for details).

    :param SupportsPtrs3[c_double] out:
        An output buffer to store the torques applied to the wrist about the
        X, Y, and Z axes (in [Nm]).

    :raises AttributeError:
        If ``out.ptrs`` is not a valid attribute of ``out``

    :raises TypeError:
        If ``out.ptrs`` is not iterable.

    :raises ArgumentError:
        If ``out.ptrs`` does not expand into a tuple of 3
        ``Pointer[c_double]`` types.

    :raises ArgumentError:
        If any element of ``cmd`` is not implicitly convertible to a C ushort.

    :raises IndexError:
        If ``len(cmd) < 3``.

    :raises TypeError:
        If ``cmd`` is not subscriptable.

    :raises ArgumentError:
        If any element of ``enc`` is not implicitly convertible to a C int.

    :raises IndexError:
        If ``len(enc) < 3``.

    :raises TypeError:
        If ``enc`` is not subscriptable.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to a C char.

    :returns:
        0 on success, -1, otherwise.
    """

    return _libdhd.dhdDeltaMotorToForce(
        output[0], output[1], output[2],
        enc[0], enc[1], enc[2],
        *out.ptrs,
        ID
    )


_libdhd.dhdWristTorqueToMotor.argtypes = [
    c_double,
    c_double,
    c_double,
    c_int,
    c_int,
    c_int,
    c_ushort_ptr,
    c_ushort_ptr,
    c_ushort_ptr,
    c_byte
]
_libdhd.dhdWristTorqueToMotor.restype = c_int


def wristTorqueToMotor(
    t: FloatVectorLike,
    enc: IntVectorLike,
    out: SupportsPtrs3[c_int],
    ID: int = -1
) -> int:
    """
    This routine computes and returns the wrist motor commands necessary to
    obtain a given torque on the wrist end-effector about the X, Y, and Z axes
    (in [Nm]) at a given orientation (defined by encoder values)

    :param FloatVectorLike t:
        Sequence of ``(t0, t1, t2)`` where ``t0``, ``t1``, and ``t2`` are the
        DELTA axis torque commands (in [Nm]) for axes 0, 1, and 2, respectively.

    :param IntVectorLike enc:
        Sequence of ``(enc0, enc1, enc2)`` where ``enc0``, ``enc1``, and
        ``enc2`` refer to encoder values on axis 0, 1, and 2, respectively.

    :param SupportsPtrs3[c_ushort] out:
        An output buffer to store the wrist motor commands.

    :param int ID:
        Device ID (see multiple devices section for details).

    :raises AttributeError:
        If ``out.ptrs`` is not a valid attribute of ``out``

    :raises TypeError:
        If ``out.ptrs`` is not iterable.

    :raises ArgumentError:
        If ``out.ptrs`` does not expand into a tuple of 3
        ``Pointer[c_ushort]`` types.

    :raises ArgumentError:
        If any element of ``t`` is not implicitly convertible to a C double.

    :raises IndexError:
        If ``len(t) < 3``.

    :raises TypeError:
        If ``t`` is not subscriptable.

    :raises ArgumentError:
        If any element of ``enc`` is not implicitly convertible to a C int.

    :raises IndexError:
        If ``len(enc) < 3``.

    :raises TypeError:
        If ``enc`` is not subscriptable.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to a C char.

    :returns:
        0 on success, -1 otherwise
    """

    return _libdhd.dhdDeltaMotorToForce(
        t[0], t[1], t[2],
        enc[0], enc[1], enc[2],
        *out.ptrs,
        ID
    )


_libdhd.dhdGripperMotorToForce.argtypes = [
    c_ushort,
    c_double_ptr,
    c_int_ptr,
    c_byte
]
_libdhd.dhdGripperMotorToForce.restype = c_int


def gripperMotorToForce(
    cmd: int,
    enc_wrist_grip: SupportsPtr[c_int],
    ID: int = -1
) -> Tuple[float, int]:
    """
    This routine computes the force applied to the end-effector for a given
    motor command.

    This feature only applies to the following devices
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA331`
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA331_LEFT`
        :data:`forcedimension.dhd.constants.DeviceType.SIGMA331`
        :data:`forcedimension.dhd.constants.DeviceType.SIGMA331_LEFT`

    :param int output:
        Motor command on gripper axis.

    :param IntVectorLike enc_wrist:
        Sequence of ``(enc0, enc1, enc2)`` where ``enc0``, ``enc1``, ``enc2``
        are encoder values about wrist joints 0, 1, and 2, respectively.

    :param int enc_gripper:
        Encoder reading for the gripper.

    :param int ID:
        Device ID (see multiple devices section for details).

    :raises ArgumentError:
        If ``cmd`` is not implicitly convertible to a C ushort.

    :raises ArgumentError:
        If any element of ``enc_wrist`` is not implicitly convertible to a C
        int.

    :raises IndexError:
        If ``len(enc_wrist) < 3``.

    :raises TypeError:
        If ``enc_wrist`` is not subscriptable.

    :raises ArgumentError:
        If ``enc_gripper`` is not implicitly convertible to a C int.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to a C char.

    :returns:
        A tuple in the form ``(force, err)``. ``force`` is the force on the
        gripper end-effector (in [N]). ``err`` is 0 on success, -1 otherwise.
    """

    force = c_double()

    err = _libdhd.dhdGripperMotorToForce(cmd, force, enc_wrist_grip.ptr, ID)

    return (force.value, err)


_libdhd.dhdGripperForceToMotor.argtypes = [
    c_double,
    c_ushort_ptr,
    c_int_ptr,
    c_byte
]
_libdhd.dhdGripperForceToMotor.restype = c_int


def gripperForceToMotor(
    f: float,
    enc_wrist_grip: SupportsPtr[c_int],
    ID: int = -1
) -> Tuple[int, int]:
    """
    Given a desired force to be displayed by the force gripper, this routine
    computes and returns the refering motor command.

    This feature only applies to the following devices
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA331`
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA331_LEFT`
        :data:`forcedimension.dhd.constants.DeviceType.SIGMA331`
        :data:`forcedimension.dhd.constants.DeviceType.SIGMA331_LEFT`

    :param int f:
        Force on the gripper end-effector (in [N]).

    :param IntVectorLike enc_wrist:
        An output buffer to store the wrist encoding readings.

    :param int ID:
        Device ID (see multiple devices section for details).

    :raises ArgumentError:
        If ``f`` is not implicitly convertible to a C double.

    :raises ArgumentError:
        If any member of ``enc_wrist`` is not implicitly convertible to a C
        ushort.

    :raises IndexError:
        If ``len(enc_wrist) < 3``.

    :raises TypeError:
        If ``enc_wrist`` is not subscriptable.

    :raises ArgumentError:
        If ``enc_gripper`` is not implicitly convertible to a C ushort.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to a C char.

    :returns:
        A tuple in the form ``(cmd, err)``.  ``cmd`` is the motor command on
        the gripper axis. ``err`` is 0 or
        :data:`forcedimension.dhd.constants.MOTOR_SATURATED` on success,
        -1 otherwise.
    """

    cmd = c_ushort()

    err = _libdhd.dhdGripperForceToMotor(f, cmd, enc_wrist_grip.ptr, ID)

    return (cmd.value, err)


_libdhd.dhdGetEnc.argtypes = [c_int_ptr, c_ubyte, c_byte]
_libdhd.dhdGetEnc.restype = c_int


def getEnc(out: SupportsPtr[c_int], mask: int=0xff, ID: int = -1) -> int:
    """
    Get a selective list of encoder values. Particularly useful when using the
    generic controller directly, without a device model attached.

    See Also
    --------
    :data:`forcedimension.dhd.constants.MAX_DOF`

    :param SupportsPtr[c_int] out:
        An output buffer to store the encoder values.

    :param int mask:
        Bitwise mask of which motor should be set.

    :param int ID:
        Device ID (see multiple devices section for details).

`    :raises AttributeError:
        If ``out.ptrs`` is not a valid attribute of ``out``

    :raises TypeError:
        If ``out.ptrs`` is not iterable.

    :raises ArgumentError:
        If ``out.ptrs`` does not expand into a tuple of
        :data:`forcedimension.dhd.constants.MAX_DOF` ``Pointer[c_int]`` types.

    :raises ArgumentError:
        If ``mask`` is not implicitly convertible to a C uchar.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to a C char.

    :returns:
        0 or :data:`forcedimension.dhd.constants.TIMEGUARD` on success, -1
        otherwise.
    """

    return  _libdhd.dhdGetEnc(out.ptr, mask, ID)


_libdhd.dhdGetDeltaJointAngles.argtypes = [
    c_double_ptr,
    c_double_ptr,
    c_double_ptr,
    c_byte
]
_libdhd.dhdGetDeltaJointAngles.restype = c_int


def getDeltaJointAngles(out: SupportsPtrs3[c_double], ID: int = -1) -> int:
    """
    Retrieve the joint angles (in [rad]) for the DELTA structure.

    :param int ID:
        Device ID (see multiple devices section for details).

    :param SupportsPtrs3[c_double] out:
        An output buffer to store the joint angles (in [rad]) of the DELTA
        structure for axes 0, 1, and 2.

    :raises AttributeError:
        If ``out.ptrs`` is not a valid attribute of ``out``

    :raises TypeError:
        If ``out.ptrs`` is not iterable.

    :raises ArgumentError:
        If ``out.ptrs`` does not expand into a tuple of 3
        ``Pointer[c_double]`` types.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to a C char.

    :returns:
        0 or :data:`forcedimension.dhd.constants.TIMEGUARD` on success,
        -1 otherwise.
    """

    return _libdhd.dhdGetDeltaJointAngles(*out.ptrs, ID)


_libdhd.dhdGetDeltaJacobian.argtypes = [c_double_ptr, c_byte]
_libdhd.dhdGetDeltaJacobian.restype = c_int


def getDeltaJacobian(out: SupportsPtr[c_double], ID: int = -1) -> int:
    """
    Retrieve the 3x3 jacobian matrix for the DELTA structure based on the
    current end-effector position. Please refer to your device user manual for
    more information on your device coordinate system.

    :param int ID:
        Device ID (see multiple devices section for details).

    :param SupportsPtr[c_double] out:
        An output buffer to store the Jacobian.

    :raises AttributeError:
        If ``out.ptr`` is not a valid attribute of ``out``

    :raises ArgumentError:
        If ``out.ptr`` is not of type Pointer[c_double]

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to a C char.

    :returns:
        0 or :data:`forcedimension.dhd.constants.TIMEGUARD` on success,
        -1 otherwise.
    """

    return _libdhd.dhdGetDeltaJacobian(out.ptr, ID)


_libdhd.dhdDeltaJointAnglesToJacobian.argtypes = [
    c_double,
    c_double,
    c_double,
    c_double_ptr,
    c_byte
]
_libdhd.dhdDeltaJointAnglesToJacobian.restype = c_int


def deltaJointAnglesToJacobian(
    joint_angles: FloatVectorLike,
    out: SupportsPtr[c_double],
    ID: int = -1,
) -> int:
    """
    Retrieve the 3x3 jacobian matrix for the DELTA structure
    based on a given joint configuration. Please refer to your device user
    manual for more information on your device coordinate system.

    :param FloatVectorLike joint_angles:
        Sequence of (j0, j1, j2) where ``j0``, ``j1``, and ``j2`` refer to the
        joint angles (in [rad]) for axis 0, 1, and 2, respectively.

    :param int ID:
        Device ID (see multiple devices section for details).

    :param SupportsPtr[c_double] out:
        An output buffer to store the Jacobian.

    :raises AttributeError:
        If ``out.ptr`` is not a valid attribute of ``out``

    :raises ArgumentError:
        If ``out.ptr`` is not of type Pointer[c_double]

    :raises ArgumentError:
        If any element of ``joint_angles`` is not implicitly convertible to C
        double.

    :raises IndexError:
        If ``len(joint_angles) < 3``.

    :raises TypeError:
        If ``joint_angles`` is not subscriptable.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to a C char.

    :returns:
        0 on success, -1 otherwise.
    """

    return _libdhd.dhdDeltaJointAnglesToJacobian(
        joint_angles[0], joint_angles[1], joint_angles[2],
        out.ptr,
        ID
    )


_libdhd.dhdDeltaJointTorquesExtrema.argtypes = [
    c_double,
    c_double,
    c_double,
    c_double_ptr,
    c_double_ptr,
    c_byte
]
_libdhd.dhdDeltaJointTorquesExtrema.restype = c_int


def deltaJointTorquesExtrema(
    joint_angles: FloatVectorLike,
    minq_out: SupportsPtr[c_double],
    maxq_out: SupportsPtr[c_double],
    ID: int = -1
) -> int:
    """
    Compute the range of applicable DELTA joint torques (in [Nm]) for a
    given DELTA joint angle configuration (in [rad]).
    Please refer to your device user manual for more information on your device
    coordinate system.

    See Also
    --------
    :data:`forcedimension.dhd.constants.MAX_DOF`

    :param FloatVectorLike joint_angles:
        Sequence of (j0, j1, j2) where ``j0``, ``j1``, and ``j2`` refer to the
        joint angles (in [rad]) for axis 0, 1, and 2, respectively.

    :param SupportsPtrs3[c_double] minq_out:
        An output buffer to store the minimum appliable DELTA joint torques
        about axes 0, 1, and 2 (in [Nm]).

    :param SupportsPtrs3[c_double] maxq_out:
        An output buffer to store the maximum appliable DELTA joint torques
        about axes 0, 1, and 2 (in [Nm]).

    :raises AttributeError:
        If ``minq_out.ptrs`` is not a valid attribute of ``minq_out``

    :raises TypeError:
        If ``minq_out.ptrs`` is not iterable.

    :raises ArgumentError:
        If ``minq_out.ptrs`` does not expand into a tuple of 3
        ``Pointer[c_double]`` types.

    :raises AttributeError:
        If ``maxq_out.ptrs`` is not a valid attribute of ``maxq_out``

    :raises TypeError:
        If ``maxq_out.ptrs`` is not iterable.

    :raises ArgumentError:
        If ``maxq_out.ptrs`` does not expand into a tuple of 3
        ``Pointer[c_double]`` types.

    :raises ArgumentError:
        If any element of ``joint_angles`` is not implicitly convertible to
        a C double.

    :raises IndexError:
        If ``len(joint_angles) < 3``.

    :raises TypeError:
        If ``joint_angles`` is not subscriptable.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to a C char.

    :returns:
        0 on success, -1 otherwise.
    """

    return _libdhd.dhdDeltaJointTorquesExtrema(
        joint_angles[0],
        joint_angles[1],
        joint_angles[2],
        minq_out.ptr,
        maxq_out.ptr,
        ID
    )


_libdhd.dhdDeltaGravityJointTorques.argtypes = [
    c_double,
    c_double,
    c_double,
    c_double_ptr,
    c_double_ptr,
    c_double_ptr,
    c_byte
]
_libdhd.dhdDeltaGravityJointTorques.restype = c_int


def deltaGravityJointTorques(
    joint_angles: FloatVectorLike,
    out: SupportsPtrs3[c_double],
    ID: int = -1
) -> int:
    """
    Compute the DELTA joint torques required to compensate for gravity in a
    given DELTA joint angle configuration. Please refer to your device user
    manual for more information on your device coordinate system.

    :param FloatVectorLike joint_angles:
        Sequence of ``(j0, j1, j2)`` where ``j0``, ``j1``, ``j2`` refer to the
        joint angles (in [rad]) for axis 0, 1, and 2, respectively.

    :param int ID:
        Device ID (see multiple devices section for details).

    :param SupportsPtrs3[c_double] out:
        An output buffer to store the gravity compensation joint torques for
        axes 0, 1, and 2 (in [Nm]).

    :raises ArgumentError:
        If any element of ``joint_angles`` is not implicitly convertible to
        a C double.

    :raises IndexError:
        If ``len(joint_angles) < 3``.

    :raises TypeError:
        If ``joint_angles`` is not subscriptable.

    :raises AttributeError:
        If ``out.ptrs`` is not a valid attribute of ``out``

    :raises TypeError:
        If ``out.ptrs`` is not iterable.

    :raises ArgumentError:
        If ``out.ptrs`` does not expand into a tuple of 3
        ``Pointer[c_double]`` types.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to a C char.

    :returns:
        0 on success, -1 otherwise.
    """

    return _libdhd.dhdDeltaGravityJointTorques(
        joint_angles[0],
        joint_angles[1],
        joint_angles[2],
        *out.ptrs,
        ID
    )


_libdhd.dhdDeltaEncodersToJointAngles.argtypes = [
    c_int,
    c_int,
    c_int,
    c_double_ptr,
    c_double_ptr,
    c_double_ptr,
    c_byte
]
_libdhd.dhdDeltaEncodersToJointAngles.restype = c_int


def deltaEncodersToJointAngles(
    enc: IntVectorLike,
    out: SupportsPtrs3[c_double],
    ID: int = -1
) -> int:
    """
    This routine computes and returns the DELTA joint angles (in [rad]) for a
    given set of encoder values.

    :param IntVectorLike enc:
        Sequence of ``(enc0, enc1, enc2)`` where ``enc0``, ``enc1``, and
        ``enc2`` refer to encoder values on axis 0, 1, and 2, respectively.

    :param int ID:
        Device ID (see multiple devices section for details).

    :param SupportsPtrs3[c_double] out:
        An output buffer to store the DELTA joint angles (in [rad]) about axes
        0, 1, and 2.

    :raises AttributeError:
        If ``out.ptrs`` is not a valid attribute of ``out``

    :raises TypeError:
        If ``out.ptrs`` is not iterable.

    :raises ArgumentError:
        If ``out.ptrs`` does not expand into a tuple of 3
        ``Pointer[c_double]`` types.

    :raises ArgumentError:
        If any element of ``enc`` is not implicitly convertible to a C int.

    :raises IndexError:
        If ``len(enc) < 3``.

    :raises TypeError:
        If ``enc`` is not subscriptable.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to a C char.

    :returns:
        0 on success, -1 otherwise.
    """

    return _libdhd.dhdDeltaEncodersToJointAngles(
        enc[0],
        enc[1],
        enc[2],
        *out.ptrs,
        ID
    )


_libdhd.dhdDeltaJointAnglesToEncoders.argtypes = [
    c_double,
    c_double,
    c_double,
    c_int_ptr,
    c_int_ptr,
    c_int_ptr,
    c_byte
]
_libdhd.dhdDeltaJointAnglesToEncoders.restype = c_int


def deltaJointAnglesToEncoders(
    joint_angles: FloatVectorLike,
    out: SupportsPtrs3[c_int],
    ID: int = -1,
) -> int:
    """
    This routine computes and returns the DELTA encoder values for a given
    set of joint angles (in [rad]).

    :param FloatVectorLike enc:
        Sequence of ``(j0, j1, j1)`` where ``j0``, ``j1``, and ``j2`` refer to
        DELTA joint angles (in [rad]) for axes 0, 1, and 2, respectively.

    :param int ID:
        Device ID (see multiple devices section for details).

    :param SupportsPtrs3[c_double] out:
        An output buffer to store the DELTA encoder values.

    :raises ArgumentError:
        If any element of ``joint_angles`` is not implicitly convertible to
        a C double.

    :raises IndexError:
        If ``len(joint_angles) < 3``.

    :raises TypeError:
        If ``joint_angles`` is not subscriptable.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to a C char.

    :returns:
        0 on success, -1 otherwise.
    """

    return _libdhd.dhdDeltaJointAnglesToEncoders(
        joint_angles[0],
        joint_angles[1],
        joint_angles[2],
        *out.ptrs,
        ID
    )


_libdhd.dhdGetWristJointAngles.argtypes = [
    c_double_ptr,
    c_double_ptr,
    c_double_ptr,
    c_byte
]
_libdhd.dhdGetWristJointAngles.restype = c_int


def getWristJointAngles(out: SupportsPtrs3[c_double], ID: int = -1) -> int:
    """
    Retrieve the joint angles (in [rad]) for the wrist structure.

    :param int ID:
        Device ID (see multiple devices section for details).

    :param SupportsPtrs3[c_double] out:
        An output buffer to store the wrist joint angles.

    :raises AttributeError:
        If ``out.ptrs`` is not a valid attribute of ``out``

    :raises TypeError:
        If ``out.ptrs`` is not iterable.

    :raises ArgumentError:
        If ``out.ptrs`` does not expand into a tuple of 3
        ``Pointer[c_double]`` types.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to a C char.

    :returns:
        0 or :data:`forcedimension.dhd.constants.TIMEGUARD` on success,
        -1 otherwise.
    """

    return _libdhd.dhdGetWristJointAngles(*out.ptrs, ID)


_libdhd.dhdGetWristJacobian.argtypes = [c_double_ptr, c_byte]
_libdhd.dhdGetWristJacobian.restype = c_int


def getWristJacobian(out: SupportsPtr[c_double], ID: int = -1) -> int:
    """
    Retrieve the 3x3 jacobian matrix for the wrist structure based on the
    current end-effector position. Please refer to your device user manual for
    more information on your device coordinate system.

    :param int ID:
        Device ID (see multiple devices section for details).

    :param SupportsPtr[c_double] out:
        An output buffer to store the Jacobian.

    :raises AttributeError:
        If ``out.ptr`` is not a valid attribute of ``out``

    :raises ArgumentError:
        If ``out.ptr`` is not a ``Pointer[c_double]`` type.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to a C char.

    :returns:
        0 or :data:`forcedimension.dhd.constants.TIMEGUARD` on success,
        and -1 otherwise.
    """

    return _libdhd.dhdGetWristJacobian(out.ptr, ID)


_libdhd.dhdWristJointAnglesToJacobian.argtypes = [
    c_double,
    c_double,
    c_double,
    c_double_ptr,
    c_byte
]
_libdhd.dhdWristJointAnglesToJacobian.restype = c_int


def wristJointAnglesToJacobian(
    joint_angles: FloatVectorLike,
    out: SupportsPtr[c_double],
    ID: int = -1,
) -> int:
    """
    Retrieve the 3x3 jacobian matrix for the wrist structure
    based on a given joint configuration. Please refer to your device user
    manual for more information on your device coordinate system.

    :param FloatVectorLike joint_angles:
        Sequence of ``(j0, j1, j2)`` where ``j0``, ``j1``, and ``j2`` refer to
        the joint angles for wrist axis 0, 1, and 2, respectively.

    :param SupportsPtr[c_double] out:
        An output buffer to store the Jacobian.

    :param int ID:
        Device ID (see multiple devices section for details).

    :raises AttributeError:
        If ``out.ptr`` is not a valid attribute of ``out``

    :raises ArgumentError:
        If ``out.ptr`` is not a ``Pointer[c_double]`` type.

    :raises ArgumentError:
        If any element of ``joint_angles`` is not implicitly convertible to
        a C double.

    :raises IndexError:
        If ``len(joint_angles) < 3``.

    :raises TypeError:
        If ``joint_angles`` is not subscriptable.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to a C char.

    :returns:
        0 on success, -1 otherwise.
    """

    return _libdhd.dhdWristJointAnglesToJacobian(
        joint_angles[0],
        joint_angles[1],
        joint_angles[2],
        out.ptr,
        ID
    )


_libdhd.dhdWristJointTorquesExtrema.argtypes = [
    c_double,
    c_double,
    c_double,
    c_double_ptr,
    c_double_ptr,
    c_byte
]
_libdhd.dhdWristJointTorquesExtrema.restype = c_int


def wristJointTorquesExtrema(
    joint_angles: FloatVectorLike,
    minq_out: SupportsPtr[c_double],
    maxq_out: SupportsPtr[c_double],
    ID: int = -1
) -> int:
    """
    Compute the range of applicable wrist joint torques for a given wrist joint
    angle configuration. Please refer to your device user manual for more
    information on your device coordinate system.

    :param FloatVectorLike joint_angles:
        Sequence of ``(j0, j1, j2)`` where ``j0``, ``j1``, ``j2`` refer to the
        joint angles for wrist axes 0, 1, and 2, respectively.

    :param SupportsPtrs3[c_double] minq_out:
        An output buffer to store the minimum appliable DELTA joint torques
        about axes 0, 1, and 2 (in [Nm]).

    :param SupportsPtrs3[c_double] maxq_out:
        An output buffer to store the maximum appliable DELTA joint torques
        about axes 0, 1, and 2 (in [Nm]).

    :param int ID:
        Device ID (see multiple devices section for details).

    :raises ArgumentError:
        If any element of ``joint_angles`` is not implicitly convertible to C
        double.

    :raises IndexError:
        If ``len(joint_angles) < 3``.

    :raises TypeError:
        If ``joint_angles`` is not subscriptable.

    :raises AttributeError:
        If ``minq_out.ptrs`` is not a valid attribute of ``minq_out``

    :raises TypeError:
        If ``minq_out.ptrs`` is not iterable.

    :raises ArgumentError:
        If ``minq_out.ptrs`` does not expand into a tuple of 3
        ``Pointer[c_double]`` types.

    :raises AttributeError:
        If ``maxq_out.ptrs`` is not a valid attribute of ``maxq_out``

    :raises TypeError:
        If ``maxq_out.ptrs`` is not iterable.

    :raises ArgumentError:
        If ``maxq_out.ptrs`` does not expand into a tuple of 3
        ``Pointer[c_double]`` types.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to a C char.

    :returns:
        0 on success, -1 otherwise.
    """

    return _libdhd.dhdWristJointTorquesExtrema(
        joint_angles[0],
        joint_angles[1],
        joint_angles[2],
        minq_out.ptr,
        maxq_out.ptr,
        ID
    )


_libdhd.dhdWristGravityJointTorques.argtypes = [
    c_double,
    c_double,
    c_double,
    c_double_ptr,
    c_double_ptr,
    c_double_ptr,
    c_byte
]
_libdhd.dhdWristGravityJointTorques.restype = c_int


def wristGravityJointTorques(
    joint_angles: FloatVectorLike,
    out: SupportsPtrs3[c_double],
    ID: int = -1
) -> int:
    """
    Compute the wrist joint torques required to compensate for gravity in a
    given wrist joint angle configuration. Please refer to your device user
    manual for more information on your device coordinate system.

    :param FloatVectorLike joint_angles:
        Sequence of ``(j0, j1, j2)`` where ``j0``, ``j1``, and ``j2`` refer to
        the joint angles (in [rad]) for wrist axes 0, 1, and 2, respectively.

    :param int ID:
        Device ID (see multiple devices section for details).

    :param SupportsPtrs3[c_double] out:
        An output buffer to store the gravity compensation wrist joint torques
        (in [Nm]) on axes 0, 1, and 2

    :raises AttributeError:
        If ``out.ptrs`` is not a valid attribute of ``out``

    :raises TypeError:
        If ``out.ptrs`` is not iterable.

    :raises ArgumentError:
        If ``out.ptrs`` does not expand into a tuple of 3
        ``Pointer[c_double]`` types.

    :raises ArgumentError:
        If any element of ``joint_angles`` is not implicitly convertible to
        a C double.

    :raises IndexError:
        If ``len(joint_angles) < 3``.

    :raises TypeError:
        If ``joint_angles`` is not subscriptable.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to a C char.

    :returns:
        0 on success, -1 otherwise
    """

    return _libdhd.dhdWristGravityJointTorques(
        joint_angles[0],
        joint_angles[1],
        joint_angles[2],
        *out.ptrs,
        ID
    )


_libdhd.dhdWristEncodersToJointAngles.argtypes = [
    c_int,
    c_int,
    c_int,
    c_double_ptr,
    c_double_ptr,
    c_double_ptr,
    c_byte
]
_libdhd.dhdWristEncodersToJointAngles.restype = c_int


def wristEncodersToJointAngles(
    enc: IntVectorLike,
    out: SupportsPtrs3[c_double],
    ID: int = -1
) -> int:
    """
    This routine computes and returns the wrist joint angles (in [rad])
    for a given set of encoder values.

    :param IntVectorLike enc:
        Sequence of (enc0, enc1, enc2) where ``enc0``, ``enc1``, and ``enc2``
        refer to encoder values on wrist axes 0, 1, and 2, respectively.

    :param SupportsPtrs3[c_double] out:
        An output buffer to store the joint angles (in [rad]) about wrist
        joint axes 0, 1, and 2.

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
        If any element of ``enc`` is not implicitly convertible to a C int.

    :raises IndexError:
        If ``len(enc) < 3``.

    :raises TypeError:
        If ``enc`` is not subscriptable.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to a C char.

    :returns:
        0 on success, -1 otherwise.
    """

    return _libdhd.dhdWristEncodersToJointAngles(
        enc[0], enc[1], enc[2], *out.ptrs, ID
    )


_libdhd.dhdWristJointAnglesToEncoders.argtypes = [
    c_double,
    c_double,
    c_double,
    c_int_ptr,
    c_int_ptr,
    c_int_ptr,
    c_byte
]
_libdhd.dhdWristJointAnglesToEncoders.restype = c_int


def wristJointAnglesToEncoders(
    joint_angles: FloatVectorLike,
    out: SupportsPtrs3[c_int],
    ID: int = -1
) -> int:
    """
    This routine computes and returns the wrist encoder values for a given
    set of wrist joint angles (in [rad]).

    :param FloatVectorLike enc:
        Sequence of ``(j0, j1, j1)`` where ``j0``, ``j1``, and ``j2`` refer to
        wrist joint angles (in [rad]) for axes 0, 1, and 2, respectively.

    :param IntVectorLike out:
        An output buffer to store the wrist encoder values.

    :param int ID:
        Device ID (see multiple devices section for details).

    :raises AttributeError:
        If ``out.ptrs`` is not a valid attribute of ``out``

    :raises TypeError:
        If ``out.ptrs`` is not iterable.

    :raises ArgumentError:
        If ``out.ptrs`` does not expand into a tuple of 3
        ``Pointer[c_int]`` types.

    :raises ArgumentError:
        If any element of ``joint_angles`` is not implicitly convertible to
        a C double.

    :raises IndexError:
        If ``len(joint_angles) < 3``.

    :raises TypeError:
        If ``joint_angles`` is not subscriptable.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to a C char.

    :returns:
        0 on success, -1 otherwise.
    """

    return _libdhd.dhdWristJointAnglesToEncoders(
        joint_angles[0],
        joint_angles[1],
        joint_angles[2],
        *out.ptrs,
        ID
    )


_libdhd.dhdGetJointAngles.argtypes = [c_double_ptr, c_byte]
_libdhd.dhdGetJointAngles.restype = c_int


def getJointAngles(out: SupportsPtr[c_double], ID: int = -1) -> int:
    """
    Retrieve the joint angles (in [rad]) for all sensed degrees-of-freedom of
    the current device.

    See Also
    --------
    :data:`forcedimension.dhd.constants.MAX_DOF`


    :param int ID:
        Device ID (see multiple devices section for details).

    :param SupportsPtr[c_double] out:
        An output buffer to store the joint angles (in [rad]) for all sensed
        degrees-of-freedom for the current device.

    :raises AttributeError:
        If ``out.ptrs`` is not a valid attribute of ``out``

    :raises TypeError:
        If ``out.ptrs`` is not iterable.

    :raises ArgumentError:
        If ``out.ptrs`` does not expand into a tuple of
        :data:`forcedimension.dhd.constants.MAX_DOF`
        ``Pointer[c_double]`` types.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to a C char.

    :returns:
        0 or :data:`forcedimension.dhd.constants.TIMEGUARD` on success,
        -1 otherwise.
    """

    return _libdhd.dhdGetJointAngles(out.ptr, ID)


_libdhd.dhdGetJointVelocities.argtypes = [c_double_ptr, c_byte]
_libdhd.dhdGetJointVelocities.restype = c_int


def getJointVelocities(out: SupportsPtr[c_double], ID: int = -1) -> int:
    """
    Retrieve the joint angle velocities (in [rad/s]) for all sensed
    degrees-of-freedom of the current device.

    See Also
    --------
    :data:`forcedimension.dhd.constants.MAX_DOF`


    :param SupportsPtr[c_double] out:
        An output buffer to store the joint velocities (in [rad/s]) for all
        sensed degrees-of-freedom of the current device.

    :raises AttributeError:
        If ``out.ptrs`` is not a valid attribute of ``out``

    :raises TypeError:
        If ``out.ptrs`` is not iterable.

    :raises ArgumentError:
        If ``out.ptrs`` does not expand into a tuple of
        :data:`forcedimension.dhd.constants.MAX_DOF`
        ``Pointer[c_double]`` types.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to a C char.

    :returns:
        0 or :data:`forcedimension.dhd.constants.TIMEGUARD` on success,
        -1 otherwise.
    """

    return _libdhd.dhdGetJointVelocities(out.ptr, ID)


_libdhd.dhdGetEncVelocities.argtypes = [c_double_ptr, c_byte]
_libdhd.dhdGetEncVelocities.restype = c_int


def getEncVelocities(out: SupportsPtr[c_double], ID: int = -1) -> int:
    """
    Retrieve the encoder angle velocities (in [inc/s]) for all sensed
    degrees-of-freedom of the current device

    See Also
    --------
    :data:`forcedimension.dhd.constants.MAX_DOF`


    :param int ID:
        Device ID (see multiple devices section for details).

    :param SupportsPtr[c_double] out:
        An output buffer to store the encoder velocities (in [inc/s]) for all
        sensed degrees-of-freedom of the current device.

    :raises AttributeError:
        If ``out.ptrs`` is not a valid attribute of ``out``

    :raises TypeError:
        If ``out.ptrs`` is not iterable.

    :raises ArgumentError:
        If ``out.ptrs`` does not expand into a tuple of
        :data:`forcedimension.dhd.constants.MAX_DOF`
        ``Pointer[c_double]`` types.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to a C char.

    :returns:
       0 or :data:`forcedimension.dhd.constants.TIMEGUARD` on success,
       -1 otherwise.
    """

    return _libdhd.dhdGetEncVelocities(out.ptr, ID)


_libdhd.dhdJointAnglesToInertiaMatrix.argtypes = [
    c_double_ptr,
    c_double_ptr,
    c_byte
]
_libdhd.dhdJointAnglesToInertiaMatrix.restype = c_int


def jointAnglesToIntertiaMatrix(
    joint_angles: SupportsPtr[c_double],
    out: SupportsPtr[c_double],
    ID: int = -1,
) -> int:
    """
    Retrieve the inertia matrix based on a given joint
    configuration. Please refer to your device user manual for more information
    on your device coordinate system.

    See Also
    --------
    :data:`forcedimension.dhd.constants.MAX_DOF`


    :param int ID:
        Device ID (see multiple devices section for details).

    :param SupportsPtr[c_double] out:
        An array of joint angles (in [rad]) containing joint angles for all
        degrees of freedom.

    :param SupportsPtr[c_double] out:
        An output buffer for the 6x6 inertia matrix.

    :raises AttributeError:
        If ``joint_angles.ptrs`` is not a valid attribute of ``joint_angles``

    :raises TypeError:
        If ``joint_angles.ptrs`` is not subscriptable.

    :raises ArgumentError:
        If ``joint_angles.ptrs[0]`` is not of type ``Pointer[c_double]``.

    :raises AttributeError:
        If ``out.ptr`` is not a valid attribute of ``out``

    :raises ArgumentError:
        If ``out.ptr`` is not a ``Pointer[c_double]`` type.


    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to a C char.

    :returns:
        0 on success, -1 otherwise.
    """

    return _libdhd.dhdJointAnglesToInertiaMatrix(joint_angles.ptr, out.ptr, ID)

