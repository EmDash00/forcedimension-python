from ctypes import c_byte, c_double, c_int, c_ubyte, c_uint, c_ushort
from typing import Tuple

from forcedimension.dhd.constants import ComMode, MAX_DOF
import forcedimension.runtime as runtime
from forcedimension.typing import (
    DOFTuple,
    FloatVectorLike,
    MutableIntVectorLike,
    MutableFloatVectorLike,
    MutableFloatMatrixLike,
    IntVectorLike,
    c_double_ptr,
    c_int_ptr,
    c_ushort_ptr,
    c_ubyte_ptr,
    c_uint_ptr
)

from . import direct_expert as direct

_libdhd = runtime.load("libdrd")

if _libdhd is None:
    raise ImportError("There were problems loading libdhd.")

_libdhd.dhdEnableExpertMode.argtypes = []
_libdhd.dhdEnableExpertMode.restype = c_int


def enableExpertMode() -> int:
    """
    Enable expert mode.

    :returns: 0 on success, -1 otherwise.
    """
    return _libdhd.dhdEnableExpertMode()


_libdhd.dhdDisableExpertMode.argtypes = []
_libdhd.dhdDisableExpertMode.restype = c_int


def disableExpertMode() -> int:
    """
    Enable expert mode.

    :returns: 0 on success, -1 otherwise.
    """
    return _libdhd.dhdDisableExpertMode()


_libdhd.dhdPreset.argtypes = [c_int * MAX_DOF, c_ubyte, c_byte]
_libdhd.dhdPreset.restype = c_int


def preset(val: IntVectorLike, mask: int = 0xff, ID: int = -1) -> int:
    """
    Set selected encoder offsets to a given value. Intended for use with the
    generic controller when no RESET button is available.

    See Also
    --------
    :data:`forcedimension.dhd.constants.MAX_DOF`


    :param IntVectorLike val:
        Vector of encoder offsets refering to each DOF with length
        :data:`forcedimension.dhd.constants.MAX_DOF`.

    :param int mask:
        Bitwise mask of which encoder should be set, defaults to ``0xff``.

    :param int ID:
        Device ID (see multiple devices section for details), defaults to -1.

    :raises ArgumentError:
        If ``val`` is not implicitly convertible to a C char.

    :raises IndexError:
        If ``len(val) < MAX_DOF``.

    :raises TypeError:
        If ``val`` is not subscriptable.

    :raises ArgumentError:
        If ``mask`` is not implicitly convertible to a C char.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to a C char.

    :returns: 0 on success, -1 otherwise
    """
    return _libdhd.dhdPreset(
        (c_int * MAX_DOF)(
            val[0], val[1], val[2], val[3], val[4], val[5], val[6], val[7]
        ),
        mask,
        ID
    )

# TODO add a page about TimeGuard
_libdhd.dhdSetTimeGuard.argtypes = [c_int, c_byte]
_libdhd.dhdSetTimeGuard.restype = c_int


def setTimeGuard(min_period: int, ID: int = -1) -> int:
    """
    Enable/disable the [TimeGuard] feature with an arbitrary minimum period.

    :param int min_period:
        Minimum refresh period (in [us]). A value of 0.
        disables the TimeGuard feature, while a value of -1 resets the default
        recommended value.

    :param int ID:
        Device ID (see multiple devices section for details), defaults to -1.

    :raises ArgumentError:
        If ``min_period`` is not implicitly convertible to a C char.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to a C char.

    :returns: 0 on success, -1 otherwise.
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


    :param int thresh:
        An arbitrary value of velocity threshold (in [m/s]).

    :param int ID:
        Device ID (see multiple devices section for details), defaults to -1.

    :raises ArgumentError:
        If ``thresh`` is not implicitly convertible to a C char.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to a C char.

    :returns: 0 on success, -1 otherwise.
    """

    return _libdhd.dhdSetVelocityThreshold(thresh, ID)


_libdhd.dhdGetVelocityThreshold.argtypes = [c_uint_ptr, c_byte]
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
        Device ID (see multiple devices section for details), defaults to -1.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to a C char.

    :returns:
        0 on success, -1 otherwise
    """

    thresh = c_uint()
    return (thresh.value, _libdhd.dhdGetVelocityThreshold(thresh, ID))


_libdhd.dhdUpdateEncoders.argtypes = [c_byte]
_libdhd.dhdUpdateEncoders.restype = c_int


def updateEncoders(ID: int = -1) -> int:
    """
    Force an update of the internal encoder values in the state vector. This
    call retrieves the encoder values from the device and places them into
    the state vector. No kinematic model is called.

    :param int ID:
        Device ID (see multiple devices section for details), defaults to -1.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to a C char.

    :returns: 0 on success, -1 otherwise.
    """
    return _libdhd.dhdUpdateEncoders(ID)


_libdhd.dhdGetDeltaEncoders.argtypes = [
    c_int_ptr,
    c_int_ptr,
    c_int_ptr,
    c_byte
]
_libdhd.dhdGetDeltaEncoders.restype = c_int


def getDeltaEncoders(out: MutableIntVectorLike, ID: int = -1) -> int:
    """
    Read all encoders values of the DELTA structure

    :param int ID:
        Device ID (see multiple devices section for details), defaults to -1.

    :param IntVectorLike out:
        An output buffer to store the raw encoder values.

    :raises TypeError:
        If ``out`` does not support item assignment either
        because it's immutable or not subscriptable.

    :raises IndexError:
        If ``len(out) < 3``.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to a C char.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to a C char.

    :returns:
        0 or :data:`forcedimension.dhd.constants.TIMEGUARD` on success, and
        -1 otherwise.
    """

    enc0 = c_int()
    enc1 = c_int()
    enc2 = c_int()

    err = _libdhd.dhdGetDeltaEncoders(enc0, enc1, enc2, ID)

    out[0] = enc0.value
    out[1] = enc1.value
    out[2] = enc2.value

    return err


_libdhd.dhdGetWristEncoders.argtypes = [
    c_int_ptr,
    c_int_ptr,
    c_int_ptr,
    c_byte
]
_libdhd.dhdGetWristEncoders.restype = c_int


def getWristEncoders(out: MutableIntVectorLike, ID: int = -1) -> int:
    """
    Read all encoders values of the wrist structure.

    This feature only applies to the following devices
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA33`
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA33_LEFT`
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA331`
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA331_LEFT`
        :data:`forcedimension.dhd.constants.DeviceType.SIGMA331`
        :data:`forcedimension.dhd.constants.DeviceType.SIGMA331_LEFT`

    :param int ID:
        Device ID (see multiple devices section for details), defaults to -1.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to a C char.

    :param IntVectorLike out:
        An output buffer to store the raw wrist encoder values.

    :raises TypeError:
        If ``out`` does not support item assignment either
        because it's immutable or not subscriptable.

    :raises IndexError:
        If ``len(out) < 3``.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to a C char.

    :returns:
        0 or :data:`forcedimension.dhd.constants.TIMEGUARD` on success,
        -1 otherwise.
    """

    enc0 = c_int()
    enc1 = c_int()
    enc2 = c_int()

    err = _libdhd.dhdGetWristEncoders(
        enc0,
        enc1,
        enc2,
        ID
    )

    out[0] = enc0.value
    out[1] = enc1.value
    out[2] = enc2.value

    return err


_libdhd.dhdGetGripperEncoder.argtypes = [c_int_ptr, c_byte]
_libdhd.dhdGetGripperEncoder.argtypes = [c_int]


def getGripperEncoder(ID: int = -1) -> Tuple[int, int]:
    """
    Read the encoder value of the force gripper.

    This feature only applies to the following devices
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA331`
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA331_LEFT`
        :data:`forcedimension.dhd.constants.DeviceType.SIGMA331`
        :data:`forcedimension.dhd.constants.DeviceType.SIGMA331_LEFT`

    :param int ID:
        Device ID (see multiple devices section for details), defaults to -1.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to a C char.

    :returns:
        0 or :data:`forcedimension.dhd.constants.TIMEGUARD` on success,
        -1 otherwise.
    """

    enc = c_int()

    return (enc.value, _libdhd.dhdGetGripperEncoder(enc, ID))


_libdhd.dhdGetEncoder.argtypes = [c_int, c_byte]
_libdhd.dhdGetEncoder.restype = c_int


def getEncoder(index: int, ID: int = -1) -> int:
    """
    Read a single encoder value from the haptic device.

    See Also
    --------
    :data:`forcedimension.dhd.constants.MAX_DOF`


    :param int index:
        The motor index number as defined by
        :data:`forcedimension.dhd.constants.MAX_DOF`

    :param int ID:
        Device ID (see multiple devices section for details), defaults to -1.

    :raises ArgumentError:
        If ``index`` is not implicitly convertible to a C char.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to a C char.

    :returns: The (positive) encoder reading on success, -1 otherwise.
    """

    return _libdhd.dhdGetEncoder(index, ID)


_libdhd.dhdSetMotor.argtypes = [c_int, c_ushort, c_byte]
_libdhd.dhdSetMotor.restype = c_int


def setMotor(index: int, output: int, ID: int = -1) -> int:
    """
    Program a command to a single motor channel.

    See Also
    --------
    :data:`forcedimension.dhd.constants.MAX_DOF`


    :param int index:
        The motor index number as defined by
        :data:`forcedimension.dhd.constants.MAX_DOF`

    :param int output:
        The motor DAa C char.

    :param int ID:
        Device ID (see multiple devices section for details), defaults to -1.

    :raises ArgumentError:
        If ``index`` is not implicitly convertible to a C char.

    :raises ArgumentError:
        If ``output`` is not implicitly convertible to a C char.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to a C char.

    :returns: 0 on success, -1 otherwise.
    """

    return _libdhd.dhdSetMotor(index, output, ID)


_libdhd.dhdSetDeltaMotor.argtypes = [c_ushort, c_ushort, c_ushort, c_byte]
_libdhd.dhdSetDeltaMotor.restype = c_int


def setDeltaMotor(mot: IntVectorLike, ID: int = -1) -> int:
    """
    Set desired motor commands to the amplifier channels commanding the DELTA
    motors.

    :param IntVectorLike mot:
        Sequence of ``(mot0, mot1, mot2)`` where ``mot0``, ``mot1``,
        and ``mot2`` are the axis 0, 1, and 2 DELTA motor commands,
        respectively.

    :param int ID:
        Device ID (see multiple devices section for details), defaults to -1.

    :raises ArgumentError:
        If any element of ``mot`` is not implicitly convertible to a C ushort.

    :raises IndexError:
        If ``len(mot) < 3``.

    :raises TypeError:
        If ``mot`` is not subscriptable.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to a C char.

    :returns: 0 on success, -1 otherwise.

    """
    return _libdhd.dhdSetDeltaMotor(mot[0], mot[1], mot[2], ID)


_libdhd.dhdSetWristMotor.argtypes = [c_ushort, c_ushort, c_ushort, c_byte]
_libdhd.dhdSetWristMotor.restype = c_int


def setWristMotor(output: IntVectorLike, ID: int = -1) -> int:
    """
    Set desired motor commands to the amplifier channels commanding the wrist
    motors.

    :param IntVectorLike output:
        Sequence of (output0, output1, output2) where ``output0``, ``output1``,
        and ``output2`` are the axis 0, 1, and 2 wrist motor commands,
        respectively.

    :param int ID:
        Device ID (see multiple devices section for details), defaults to -1.

    :raises ArgumentError:
        If any element of ``output`` is not implicitly convertible to a C char.

    :raises IndexError:
        If ``len(output) < 3``.

    :raises TypeError:
        If ``output`` is not subscriptable.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to a C char.

    :returns: 0 on success, -1 otherwise.

    """
    return _libdhd.dhdSetWristMotor(output[0], output[1], output[2], ID)


_libdhd.dhdSetWristMotor.argtypes = [c_ushort, c_byte]
_libdhd.dhdSetWristMotor.restype = c_int


def setGripperMotor(output: int, ID: int = -1) -> int:
    """
    Set desired motor commands to the amplifier channels commanding the force
    gripper.

    :param int output:
        Gripper motor command.

    :param int ID:
        Device ID (see multiple devices section for details), defaults to -1.

    :raises ArgumentError:
        If ``output`` is not implicitly convertible to a C ushort.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to a C char.

    :returns: 0 on success, -1 otherwise.
    """
    return _libdhd.dhdSetWristMotor(output, ID)


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
    out: MutableFloatVectorLike,
    ID: int = -1
) -> int:
    """
    This routine computes and returns the position of the end-effector for a
    given set of encoder values.

    :param IntVectorLike enc:
        Sequence of ``(enc0, enc1, enc2)`` where ``enc0``, ``enc1``, and
        ``enc2`` refer to raw encoder values on axis 0, 1, and 2,
        respectively.

    :param int ID:
        Device ID (see multiple devices section for details), defaults to -1.

    :param IntVectorLike out:
        An output buffer to store the end-effector position (in [m]).

    :raises TypeError:
        If ``out`` does not support item assignment either
        because it's immutable or not subscriptable.

    :raises IndexError:
        If ``len(out) < 3``.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to a C char.

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

    px = c_double()
    py = c_double()
    pz = c_double()

    err = _libdhd.dhdDeltaEncoderToPosition(
        enc[0],
        enc[1],
        enc[2],
        px,
        py,
        pz,
        ID
    )

    out[0] = px.value
    out[1] = py.value
    out[2] = pz.value

    return err


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
    out: MutableIntVectorLike,
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
        Device ID (see multiple devices section for details), defaults to -1.

    :param IntVectorLike out:
        An output buffer to store the raw encoder values.

    :raises TypeError:
        If ``out`` does not support item
        assignment either because it's immutable or not subscriptable.

    :raises IndexError:
        If ``len(out) < 3``.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to a C char.

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

    enc0 = c_int()
    enc1 = c_int()
    enc2 = c_int()

    err = _libdhd.dhdDeltaEncoderToPosition(
        pos[0],
        pos[1],
        pos[2],
        enc0,
        enc1,
        enc2,
        ID
    )

    out[0] = enc0.value
    out[1] = enc1.value
    out[2] = enc2.value

    return err


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
    out: MutableFloatVectorLike,
) -> int:
    """
    This routine computes and returns the force applied to the end-effector for
    a given set of motor commands at a given position (defined by encoder
    readings)

    :param IntVectorLike mot:
        Sequence of ``(mot0, mot1, mot2)`` where ``mot0``, ``mot1``,
        and ``mot2`` are the axis 0, 1, and 2 DELTA motor commands,
        respectively.

    :param IntVectorLike enc:
        Sequence of ``(enc0, enc1, enc2)`` where ``enc0``, ``enc1``, and
        ``enc2`` refer to encoder values on axis 0, 1, and 2,
        respectively.

    :param int ID:
        Device ID (see multiple devices section for details), defaults to -1.

    :param MutableFloatVectorLike out:
        An output buffer to store the applied force to the end effector.

    :raises TypeError:
        If ``out`` does not support item assignment either
        because it's immutable or not subscriptable.

    :raises IndexError:
        If ``len(out) < 3``.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to a C char.

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

    fx = c_double()
    fy = c_double()
    fz = c_double()

    err = _libdhd.dhdDeltaMotorToForce(
        mot[0],
        mot[1],
        mot[2],
        enc[0],
        enc[1],
        enc[2],
        fx,
        fy,
        fz
    )

    out[0] = fx.value
    out[1] = fy.value
    out[2] = fz.value

    return err


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
    out: MutableIntVectorLike
) -> int:
    """
    This routine computes and returns the motor commands necessary to obtain a
    given force on the end-effector at a given position (defined by encoder
    readings).

    :param FloatVectorLike f:
        Sequence of ``(fx, fy, fz)`` where ``fx``, ``fy``, and ``fz`` are the
        force on the DELTA end-effector on the X, Y, and Z axes, respectively
        in [N].

    :param IntVectorLike enc:
        Sequence of ``(enc0, enc1, enc2)`` where ``enc0``, ``enc1``, and
        ``enc2`` refer to encoder values on axis 0, 1, and 2, respectively.

    :param int ID:
        Device ID (see multiple devices section for details), defaults to -1.

    :param IntVectorLike out:
        An output buffer to store the applied force to the end-effector.

    :raises TypeError:
        If ``out`` does not support item assignment either
        because it's immutable or not subscriptable.

    :raises IndexError:
        If ``len(out) < 3``.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to a C char.

    :raises ArgumentError:
        If any element of ``f`` is not implicitly convertible to a C char.

    :raises IndexError:
        If ``len(f) < 3``.

    :raises TypeError:
        If ``f`` is not subscriptable.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to a C char.

    :returns:
        0 or :data:`forcedimension.dhd.constants.MOTOR_SATURATED` on success,
        and -1 otherwise.
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
        output0,
        output1,
        output2
    )

    out[0] = output0.value
    out[1] = output1.value
    out[2] = output2.value

    return err


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
    out: MutableFloatVectorLike,
    ID: int = -1,
) -> int:
    """
    For devices with a wrist structure, compute the individual angle of each
    joint, starting with the one located nearest to the wrist base plate. For
    the :data:`forcedimension.dhd.constants.DeviceType.OMEGA33` and the
    :data:`forcedimension.dhd.constants.DeviceType.OMEGA33_LEFT` devices,
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

    :param int ID:
        Device ID (see multiple devices section for details), defaults to -1.

    :param MutableFloatVectorLike out:
        An output buffer to store the joint angles.

    :raises TypeError:
        If ``out`` does not support item assignment either
        because it's immutable or not subscriptable.

    :raises IndexError:
        If ``len(out) < 3``.

    :raises ArgumentError:
        If any element of ``enc`` is not implicitly convertible
        to a C char.

    :raises IndexError:
        If ``len(enc) < 3``.

    :raises TypeError:
        If ``enc`` is not subscriptable.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to a C char.

    :returns:
        0 on success, -1 otherwise.
    """
    px = c_double()
    py = c_double()
    pz = c_double()

    err = _libdhd.dhdDeltaEncoderToPosition(
        enc[0],
        enc[1],
        enc[2],
        px,
        py,
        pz,
        ID
    )

    out[0] = px.value
    out[1] = py.value
    out[2] = pz.value

    return err


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
    out: MutableIntVectorLike,
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
        wrist end effector orientation around the X, Y, and Z axes,
        respectively (in [rad]).

    :param int ID:
        Device ID (see multiple devices section for details), defaults to -1.

    :param IntVectorLike out:
        An output buffer to store the encoder values.

    :raises TypeError:
        If ``out`` does not support item assignment either
        because it's immutable or not subscriptable.

    :raises IndexError:
        If ``len(out) < 3``.

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

    enc0 = c_int()
    enc1 = c_int()
    enc2 = c_int()

    err = _libdhd.dhdDeltaEncoderToPosition(
        orientation[0],
        orientation[1],
        orientation[2],
        enc0,
        enc1,
        enc2,
        ID
    )

    out[0] = enc0.value
    out[1] = enc1.value
    out[2] = enc2.value

    return err


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
    out: MutableFloatVectorLike
) -> int:
    """
    This routine computes and returns the torque applied to the wrist
    end-effector for a given set of motor commands at a given orientation
    (defined by encoder values)

    :param IntVectorLike cmd:
        Sequence of ``(cmd0, cmd1, cmd2)`` where ``cmd0``, ``cmd1``,
        and ``cmd2`` are the axis 0, 1, and 2 DELTA motor commands,
        respectively.

    :param IntVectorLike enc:
        Sequence of ``(enc0, enc1, enc2)`` where ``enc0``, ``enc1``, and
        ``enc2`` refer to encoder values on axis 0, 1, and 2, respectively.

    :param int ID:
        Device ID (see multiple devices section for details), defaults to -1.

    :param MutableFloatVectorLike out:
        An output buffer to store the torques applied to the wrist.

    :raises TypeError:
        If ``out`` does not support item assignment either
        because it's immutable or not subscriptable.

    :raises IndexError:
        If ``len(out) < 3``.

    :raises ArgumentError:
        If any element of ``cmd`` is not implicitly convertible to a C ushort.

    :raises IndexError:
        If ``len(cmd) < 3``.

    :raises TypeError:
        If ``cmd`` is not subscriptable.

    :raises ArgumentError:
        If any element of ``enc`` is not implicitly convertible to a C char.

    :raises IndexError:
        If ``len(enc) < 3``.

    :raises TypeError:
        If ``enc`` is not subscriptable.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to a C char.

    :returns:
        0 on success, -1, otherwise.
    """

    tx = c_double()
    ty = c_double()
    tz = c_double()

    err = _libdhd.dhdDeltaMotorToForce(
        output[0],
        output[1],
        output[2],
        enc[0],
        enc[1],
        enc[2],
        tx,
        ty,
        tz
    )

    out[0] = tx.value
    out[1] = ty.value
    out[2] = tz.value

    return err


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
    out: MutableIntVectorLike
) -> int:
    """
    This routine computes and returns the wrist motor commands necessary to
    obtain a given torque on the wrist end-effector at a given orientation
    (defined by encoder values)

    :param FloatVectorLike t:
        Sequence of ``(t0, t1, t2)`` where ``t0``, ``t1``, and ``t2`` are the
        DELTA axis torque commands for axes 0, 1, and 2, respectively in [Nm].

    :param IntVectorLike enc:
        Sequence of ``(enc0, enc1, enc2)`` where ``enc0``, ``enc1``, and
        ``enc2`` refer to encoder values on axis 0, 1, and 2, respectively.

    :param int ID:
        Device ID (see multiple devices section for details), defaults to -1.

    :param IntVectorLike out:
        An output buffer to store the wrist motor commands.

    :raises TypeError:
        If ``out`` does not support item assignment either
        because it's immutable or not subscriptable.

    :raises IndexError:
        If ``len(out) < 3``.

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
        output0,
        output1,
        output2
    )

    out[0] = output0.value
    out[1] = output1.value
    out[2] = output2.value

    return err


_libdhd.dhdGripperEncoderToAngleRad.argtypes = [
    c_int,
    c_double_ptr,
    c_byte
]
_libdhd.dhdGripperEncoderToAngleRad.restype = c_int


def gripperEncoderToAngleRad(
    enc: int,
    ID: int = -1
) -> Tuple[float, int]:
    """
    This routine computes and returns the opening of the gripper as an angle in
    [rad] for a given encoder reading.

    This feature only applies to the following devices
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA331`
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA331_LEFT`
        :data:`forcedimension.dhd.constants.DeviceType.SIGMA331`
        :data:`forcedimension.dhd.constants.DeviceType.SIGMA331_LEFT`

    :param int enc:
        Gripper encoder reading.

    :param int ID:
        Device ID (see multiple devices section for details), defaults to -1.

    :raises ArgumentError:
        If ``enc`` is not implicitly convertible to a C char.

    :raises IndexError:
        If ``len(enc) < 3``

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to a C char.

    :returns:
        0 or :data:`forcedimension.dhd.constants.TIMEGUARD` on success,
        -1 otherwise.
    """

    angle = c_double()
    err = _libdhd.dhdGripperEncoderToAngleRad(
        enc,
        angle,
        ID
    )

    return (angle.value, err)


_libdhd.dhdGripperEncoderToGap.argtypes = [
    c_int,
    c_double_ptr,
    c_byte
]
_libdhd.dhdGripperEncoderToGap.restype = c_int


def gripperEncoderToGap(enc: int, ID: int = -1) -> Tuple[float, int]:
    """
    This routine computes and returns the opening of the gripper as a distance
    (in [m]) for a given encoder reading.

    This feature only applies to the following devices
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA331`
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA331_LEFT`
        :data:`forcedimension.dhd.constants.DeviceType.SIGMA331`
        :data:`forcedimension.dhd.constants.DeviceType.SIGMA331_LEFT`

    :param int enc:
        Gripper encoder reading.

    :param int ID:
        Device ID (see multiple devices section for details), defaults to -1.

    :raises ArgumentError:
        If ``enc`` is not implicitly convertible to a C char.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to a C char.

    :returns:
        A tuple in the form ``(gap, err)``. ``gap`` is the gripper opening in
        [m] ``err`` is 0 on success, -1 otherwise.
    """

    gap = c_double()
    err = _libdhd.dhdGripperEncoderToGap(
        enc,
        gap,
        ID
    )

    return (gap.value, err)


_libdhd.dhdGripperAngleRadToEncoder.argtypes = [
    c_double,
    c_int_ptr,
    c_byte
]
_libdhd.dhdGripperAngleRadToEncoder.restype = c_int


def gripperAngleRadToEncoder(angle: float, ID: int = -1) -> Tuple[int, int]:
    """
    This routine computes and returns the gripper encoder value for a given
    gripper opening distance (in [rad]).

    This feature only applies to the following devices
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA331`
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA331_LEFT`
        :data:`forcedimension.dhd.constants.DeviceType.SIGMA331`
        :data:`forcedimension.dhd.constants.DeviceType.SIGMA331_LEFT`

    :param float angle:
        Gripper opening as an angle (in [rad]).

    :param int ID:
        Device ID (see multiple devices section for details), defaults to -1.

    :raises ArgumentError:
        If ``angle`` is not implicitly convertible to a C char.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to a C char.

    :returns:
        A tuple in the form ``(enc, err)``. ``enc`` is the gripper raw encoder
        reading. ``err`` is 0 on success, -1 otherwise.

    """

    enc = c_int()

    err = _libdhd.dhdGripperEncoderToAngleRad(
        angle,
        enc,
        ID
    )
    return (enc.value, err)


_libdhd.dhdGripperGapToEncoder.argtypes = [
    c_double,
    c_int_ptr,
    c_byte
]
_libdhd.dhdGripperEncoderToGap.restype = c_int


def gripperGapToEncoder(gap: float, ID: int = -1) -> Tuple[int, int]:
    """
    This routine computes and returns the gripper encoder value for a given
    gripper opening distance (in [m]).

    This feature only applies to the following devices
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA331`
        :data:`forcedimension.dhd.constants.DeviceType.OMEGA331_LEFT`
        :data:`forcedimension.dhd.constants.DeviceType.SIGMA331`
        :data:`forcedimension.dhd.constants.DeviceType.SIGMA331_LEFT`

    :param float gap:
        Gripper opening distance (in [m]).

    :param int ID:
        Device ID (see multiple devices section for details), defaults to -1.

    :raises ArgumentError:
        If ``gap`` is not implicitly convertible to a C char.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to a C char.

    :returns:
        A tuple in the form ``(enc, err)``. ``enc`` is the gripper encoder
        reading. ``err`` is 0 on success, -1 otherwise.
    """

    enc = c_int()
    err = _libdhd.dhdGripperEncoderToGap(
        gap,
        enc,
        ID
    )

    return (enc.value, err)


_libdhd.dhdGripperMotorToForce.argtypes = [
    c_ushort,
    c_double_ptr,
    c_int * 4,
    c_byte
]
_libdhd.dhdGripperMotorToForce.restype = c_int


def gripperMotorToForce(
    cmd: int,
    enc_wrist: IntVectorLike,
    enc_gripper: int,
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
        Device ID (see multiple devices section for details), defaults to -1.

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
        gripper end-effector in [N]. ``err`` is 0 on success, -1 otherwise.
    """

    force = c_double()

    enc = [enc_wrist[0], enc_wrist[1], enc_wrist[2], enc_gripper]
    err = _libdhd.dhdGripperMotorToForce(
        cmd,
        force,
        enc,
        ID
    )

    return (force.value, err)


_libdhd.dhdGripperForceToMotor.argtypes = [
    c_double,
    c_ushort_ptr,
    c_int * 4,
    c_byte
]
_libdhd.dhdGripperForceToMotor.restype = c_int


def gripperForceToMotor(
    f: float,
    enc_wrist: IntVectorLike,
    enc_gripper: int,
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
        Force on the gripper end-effector in [N].

    :param IntVectorLike enc_wrist:
        An output buffer to store the wrist encoding readings.

    :param int ID:
        Device ID (see multiple devices section for details), defaults to -1.

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
    enc = [enc_wrist[0], enc_wrist[1], enc_wrist[2], enc_gripper]

    err = _libdhd.dhdGripperForceToMotor(
        f,
        enc,
        enc,
        ID
    )

    return (cmd.value, err)


_libdhd.dhdSetMot.argtypes = [c_ushort * MAX_DOF, c_ubyte, c_byte]
_libdhd.dhdSetMot.restype = c_int


def setMot(cmds: IntVectorLike, mask: int = 0xff, ID: int = -1) -> int:
    """
    Program motor commands to a selection of motor channels. Particularly
    useful when using the generic controller directly, without a device model
    attached.

    See Also
    --------
    :data:`forcedimension.dhd.constants.MAX_DOF`


    :param IntVectorLike cmds:
        List of motor command values.

    :param int mask:
        Bitwise mask of which motor should be set, defaults to ``0xff``

    :param int ID:
        Device ID (see multiple devices section for details), defaults to -1.

    :raises ArgumentError:
        If any element of ``cmds`` is not implicitly convertible to a C
        ushort.

    :raises IndexError:
        If ``len(cmds) < MAX_DOF``.

    :raises TypeError:
        If ``cmds`` is not subscriptable.

    :raises ArgumentError:
        If ``mask`` is not implicitly convertible to a C char.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to a C char.

    :returns: 0 on success, -1 otherwise
    """

    return _libdhd.dhdSetMot(
        (c_int * MAX_DOF)(
            cmds[0],
            cmds[1],
            cmds[2],
            cmds[3],
            cmds[4],
            cmds[5],
            cmds[6],
            cmds[7]
        ),
        mask,
        ID
    )


_libdhd.dhdPreloadMot.argtypes = [c_ushort * MAX_DOF, c_ubyte, c_byte]
_libdhd.dhdPreloadMot.restype = c_int


def preloadMot(cmds: IntVectorLike, mask: int = 0xff, ID: int = -1) -> int:
    """
    Program motor commands to a selection of motor channels. Unlike
    dhd.libdhd.expert.setMot, this function saves the requested commands
    internally for later application by calling dhd.libdhd.standard.setForce
    and the likes.

    See Also
    --------
    :data:`forcedimension.dhd.constants.MAX_DOF`


    :param IntVectorLike outputs:
        List of motor command values.

    :param int mask:
        Bitwise mask of which motor should be set, defaults to ``0xff``

    :param int ID:
        Device ID (see multiple devices section for details), defaults to -1.

    :raises ArgumentError:
        If any element of ``cmds`` is not implicitly convertible to a C ushort.

    :raises IndexError:
        If ``len(cmds) < MAX_DOF``.

    :raises TypeError:
        If ``cmds`` is not subscriptable.

    :raises ArgumentError:
        If ``mask`` is not implicitly convertible to a C char.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to a C char.

    :returns: 0 on success, -1 otherwise
    """

    return _libdhd.dhdPreloadMot(
        (c_int * MAX_DOF)(
            cmds[0],
            cmds[1],
            cmds[2],
            cmds[3],
            cmds[4],
            cmds[5],
            cmds[6],
            cmds[7]
        ),
        mask,
        ID
    )


_libdhd.dhdGetEnc.argtypes = [c_int * MAX_DOF, c_ubyte, c_byte]
_libdhd.dhdGetEnc.restype = c_int


def getEnc(mask: int, out: MutableIntVectorLike, ID: int = -1) -> int:
    """
    Get a selective list of encoder values. Particularly useful when using the
    generic controller directly, without a device model attached.

    See Also
    --------
    :data:`forcedimension.dhd.constants.MAX_DOF`


    :param int mask:
        Bitwise mask of which motor should be set, default to ``0xff``

    :param int ID:
        Device ID (see multiple devices section for details), defaults to -1.

    :param IntVectorLike out:
        An output buffer to store the encoder values.

    :raises TypeError:
        If ``out`` does not support item assignment either
        because it's immutable or not subscriptable.

    :raises IndexError:
        If ``len(out) < MAX_DOF``

    :raises ArgumentError:
        If ``mask`` is not implicitly convertible to a C char.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to a C char.

    :returns:
        0 or :data:`forcedimension.dhd.constants.TIMEGUARD` on success, -1
        otherwise.
    """
    enc = (c_int * MAX_DOF)()

    err = _libdhd.dhdGetEnc(enc, mask, ID)

    for i in range(MAX_DOF):
        out[i] = enc[i]

    return err


_libdhd.dhdGetEncRange.argtypes = [c_int * MAX_DOF, c_int * MAX_DOF, c_byte]
_libdhd.dhdGetEncRange.restype = c_int


def getEncRange(ID: int = -1) -> Tuple[DOFTuple, DOFTuple, int]:
    """
    Get the expected min and max encoder values for all axes present on the
    current device. Axis indices that do not exist on the device will return
    a range of 0.

    See Also
    --------
    :data:`forcedimension.dhd.constants.MAX_DOF`


    :param int ID:
        Device ID (see multiple devices section for details), defaults to -1.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to a C char.

    :returns:
        0 on success, -1 otherwise.
    """

    encMin = (c_int * MAX_DOF)()
    encMax = (c_int * MAX_DOF)()

    err = _libdhd.dhdGetEncRange(encMin, encMax, ID)

    return (tuple(encMin), tuple(encMax), err)


_libdhd.dhdSetBrk.argtypes = [c_ubyte, c_byte]
_libdhd.dhdSetBrk.restype = c_int


def setBrk(mask: int = 0xff, ID: int = -1) -> int:
    """
    Set electromagnetic braking status on selective motor groups. Only applies
    when using the generic controller directly, without a device model
    attached.

    Generic control motor groups
        group1 - [mot0, mot1, mot2]

        group2 - [mot3, mot4, mot5]

        group3 - [mot6]

        group4 - [mot7]

    The mask parameter addresses all 8 motors bitwise. If a single bit within
    a motor group's is enabled, all motors in that motor's group's
    electromagnetic brakes will be activated.

    :param mask:
        Bitwise mask of which motor groups should have their electromagnetic
        brakes be set on, defaults to ``0xff``

    :param int ID:
        Device ID (see multiple devices section for details), defaults to -1.

    :raises ArgumentError:
        If ``mask`` is not implicitly convertible to a C char.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to a C char.

    :returns: 0 on success, -1 otherwise
    """

    return _libdhd.dhdSetBrk(mask, ID)


_libdhd.dhdGetDeltaJointAngles.argtypes = [
    c_double_ptr,
    c_double_ptr,
    c_double_ptr,
    c_byte
]
_libdhd.dhdGetDeltaJointAngles.restype = c_int


def getDeltaJointAngles(out: MutableFloatVectorLike, ID: int = -1) -> int:
    """
    Retrieve the joint angles (in [rad]) for the DELTA structure.

    :param int ID:
        Device ID (see multiple devices section for details), defaults to -1.

    :param MutableFloatVectorLike out:
        An output buffer to store the joint angles.

    :raises TypeError:
        If ``out`` does not support item assignment either
        because it's immutable or not subscriptable.

    :raises IndexError:
        If ``len(out) < 3``.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to a C char.

    :returns:
        0 or :data:`forcedimension.dhd.constants.TIMEGUARD` on success,
        -1 otherwise.
    """

    j0 = c_double()
    j1 = c_double()
    j2 = c_double()

    err = _libdhd.dhdGetDeltaJointAngles(
        j0,
        j1,
        j2,
        ID
    )

    out[0] = j0.value
    out[1] = j1.value
    out[2] = j2.value

    return err


_libdhd.dhdGetDeltaJacobian.argtypes = [(c_double * 3) * 3, c_byte]
_libdhd.dhdGetDeltaJacobian.restype = c_int


def getDeltaJacobian(out: MutableFloatMatrixLike, ID: int = -1) -> int:
    """
    Retrieve the 3x3 jacobian matrix for the DELTA structure based on the
    current end-effector position. Please refer to your device user manual for
    more information on your device coordinate system.

    :param int ID:
        Device ID (see multiple devices section for details), defaults to -1.

    :param MutableFloatMatrixLike out:
        An output buffer to store the Jacobian.

    :raises TypeError:
        If ``out`` does not support item assignment either
        because it's immutable or not subscriptable.

    :raises IndexError:
        If any dimension of ``out`` is less than 3.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to a C char.

    :returns:
        0 or :data:`forcedimension.dhd.constants.TIMEGUARD` on success,
        -1 otherwise.
    """

    J = ((c_double * 3) * 3)()
    err = _libdhd.dhdGetDeltaJacobian(J, ID)

    for i in range(3):
        for j in range(3):
            out[i][j] = J[i][j]

    return err


_libdhd.dhdDeltaJointAnglesToJacobian.argtypes = [
    c_double,
    c_double,
    c_double,
    (c_double * 3) * 3,
    c_byte
]
_libdhd.dhdDeltaJointAnglesToJacobian.restype = c_int


def deltaJointAnglesToJacobian(
    joint_angles: FloatVectorLike,
    out: MutableFloatMatrixLike,
    ID: int = -1,
) -> int:
    """
    Retrieve the 3x3 jacobian matrix for the DELTA structure
    based on a given joint configuration. Please refer to your device user
    manual for more information on your device coordinate system.

    :param FloatVectorLike joint_angles:
        Sequence of (j0, j1, j2) where ``j0``, ``j1``, and ``j2`` refer to the
        joint angles for axis 0, 1, and 2, respectively.

    :param int ID:
        Device ID (see multiple devices section for details), defaults to -1.

    :param MutableFloatMatrixLike out:
        An output buffer to store the return.

    :raises TypeError:
        If ``out`` does not support item assignment either
        because it's immutable or not subscriptable.

    :raises IndexError:
        If ``len(out) < 3``.

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

    J = ((c_double * 3) * 3)()

    err = _libdhd.dhdDeltaJointAnglesToJacobian(
        joint_angles[0],
        joint_angles[1],
        joint_angles[2],
        J,
        ID
    )

    for i in range(3):
        for j in range(3):
            out[i][j] = J[i][j]

    return err


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
    joint_angles: FloatVectorLike,
    minq_out: MutableFloatVectorLike,
    maxq_out: MutableFloatVectorLike,
    ID: int = -1
) -> int:
    """
    Compute the range of applicable DELTA joint torques for a given DELTA joint
    angle configuration. Please refer to your device user manual for more
    information on your device coordinate system.

    See Also
    --------
    :data:`forcedimension.dhd.constants.MAX_DOF`


    :param FloatVectorLike joint_angles:
        Sequence of ``(j0, j1, j2)`` where ``j0``, ``j1``, ``j2`` refer to the
        joint angles for axis 0, 1, and 2, respectively.

    :param int ID:
        Device ID (see multiple devices section for details), defaults to -1.

    :param MutableFloatVectorLike minq_out:
        An output buffer to store the return.

    :param MutableFloatVectorLike maxq_out:
        An output buffer to store the return.

    :raises TypeError:
        If ``minq_out`` does not support item assignment
        either because it's immutable or not subscriptable.

    :raises IndexError:
        If ``len(encMin_out) < MAX_DOF``

    :raises TypeError:
        If ``maxq_out`` does not support item assignment
        either because it's immutable or not subscriptable.

    :raises IndexError:
        If ``len(out) < MAX_DOF``

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

    minq_out[0] = minq[0]
    minq_out[1] = minq[1]
    minq_out[2] = minq[2]

    maxq_out[0] = maxq[0]
    maxq_out[1] = maxq[1]
    maxq_out[2] = maxq[2]

    return err


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
    out: MutableFloatVectorLike,
    ID: int = -1
) -> int:
    """
    Compute the DELTA joint torques required to compensate for gravity in a
    given DELTA joint angle configuration. Please refer to your device user
    manual for more information on your device coordinate system.

    :param FloatVectorLike joint_angles:
        Sequence of ``(j0, j1, j2)`` where ``j0``, ``j1``, ``j2`` refer to the
        joint angles for axis 0, 1, and 2, respectively.

    :param int ID:
        Device ID (see multiple devices section for details), defaults to -1.

    :param MutableFloatVectorLike out:
        An output buffer to store the return.

    :raises TypeError:
        If ``out`` does not support item.
        assignment either because it's immutable or not subscriptable.

    :raises IndexError:
        If ``len(out) < 3``.

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

    q0 = c_double()
    q1 = c_double()
    q2 = c_double()

    err = _libdhd.dhdDeltaGravityJointTorques(
        joint_angles[0],
        joint_angles[1],
        joint_angles[2],
        q0,
        q1,
        q2,
        ID
    )

    out[0] = q0.value
    out[1] = q1.value
    out[2] = q2.value

    return err


_libdhd.dhdSetDeltaJointTorques.argtypes = [
    c_double,
    c_double,
    c_double,
    c_byte
]
_libdhd.dhdSetDeltaJointTorques.restype = c_int


def setDeltaJointTorques(
    t: FloatVectorLike,
    ID: int = -1
) -> int:
    """
    Set all joint torques of the DELTA structure.

    :param FloatVectorLike t:
        Sequence of ``(t0, t1, t2)`` where ``t0``, ``t1``, and ``t2`` are the
        DELTA axis torque commands for axes 0, 1, and 2, respectively in [Nm].

    :raises ArgumentError:
        If any element of ``t`` is not implicitly convertible to a C char

    :raises IndexError:
        If ``len(t) < 3``.

    :raises TypeError:
        If ``t`` is not subscriptable.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to a C char.

    :returns: 0 on success, -1 otherwise
    """
    return _libdhd.dhdSetDeltaJointTorques(t[0], t[1], t[2], ID)


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
    out: MutableFloatVectorLike,
    ID: int = -1
) -> int:
    """
    This routine computes and returns the DELTA joint angles for a given set of
    encoder values.

    :param IntVectorLike enc:
        Sequence of ``(enc0, enc1, enc2)`` where ``enc0``, ``enc1``, and
        ``enc2`` refer to encoder values on axis 0, 1, and 2, respectively.

    :param int ID:
        Device ID (see multiple devices section for details), defaults to -1.

    :param MutableFloatVectorLike out:
        An output buffer to store the return.

    :raises TypeError:
        If ``out`` does not support item
        assignment either because it's immutable or not subscriptable.

    :raises IndexError:
        If ``len(out) < 3``.

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

    j0 = c_double()
    j1 = c_double()
    j2 = c_double()

    err = _libdhd.dhdDeltaEncodersToJointAngles(
        enc[0],
        enc[1],
        enc[2],
        j0,
        j1,
        j2,
        ID
    )

    out[0] = j0.value
    out[1] = j1.value
    out[2] = j2.value

    return err


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
    out: MutableIntVectorLike,
    ID: int = -1,
) -> int:
    """
    This routine computes and returns the DELTA encoder values for a given
    set of joint angles.

    :param FloatVectorLike enc:
        Sequence of ``(j0, j1, j1)`` where ``j0``, ``j1``, and ``j2`` refer to
        DELTA joint angles for axes 0, 1, and 2, respectively, (in [rad]).

    :param int ID:
        Device ID (see multiple devices section for details), defaults to -1.

    :param MutableFloatVectorLike out:
        An output buffer to store the return.

    :raises TypeError:
        If ``out`` does not support item
        assignment either because it's immutable or not subscriptable.

    :raises IndexError:
        If ``len(out) < 3``.

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

    enc0 = c_int()
    enc1 = c_int()
    enc2 = c_int()

    err = _libdhd.dhdDeltaJointAnglesToEncoders(
        joint_angles[0],
        joint_angles[1],
        joint_angles[2],
        enc0,
        enc1,
        enc2,
        ID
    )

    out[0] = enc0.value
    out[1] = enc1.value
    out[2] = enc2.value

    return err


_libdhd.dhdGetWristJointAngles.argtypes = [
    c_double_ptr,
    c_double_ptr,
    c_double_ptr,
    c_byte
]
_libdhd.dhdGetWristJointAngles.restype = c_int


def getWristJointAngles(out: MutableFloatVectorLike, ID: int = -1) -> int:
    """
    Retrieve the joint angles (in [rad]) for the wrist structure.

    :param int ID:
        Device ID (see multiple devices section for details), defaults to -1.

    :param MutableFloatVectorLike out:
        An output buffer to store the wrist joint angles.

    :raises TypeError:
        If ``out`` does not support item assignment either
        because it's immutable or not subscriptable.

    :raises IndexError:
        If ``len(out) < 3``.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to a C char.

    :returns:
        0 or :data:`forcedimension.dhd.constants.TIMEGUARD` on success,
        -1 otherwise.
    """

    j0 = c_double()
    j1 = c_double()
    j2 = c_double()
    err = _libdhd.dhdGetWristJointAngles(
        j0,
        j1,
        j2,
        ID
    )

    out[0] = j0.value
    out[1] = j1.value
    out[2] = j2.value

    return err


_libdhd.dhdGetWristJacobian.argtypes = [(c_double * 3) * 3, c_byte]
_libdhd.dhdGetWristJacobian.restype = c_int


def getWristJacobian(out: MutableFloatMatrixLike,ID: int = -1) -> int:
    """
    Retrieve the 3x3 jacobian matrix for the wrist structure based on the
    current end-effector position. Please refer to your device user manual for
    more information on your device coordinate system.

    :param int ID:
        Device ID (see multiple devices section for details), defaults to -1.

    :param MutableFloatVectorLike out:
        An output buffer to store the Jacobian.

    :raises TypeError:
        If ``out`` does not support item assignment either
        because it's immutable or not subscriptable.

    :raises IndexError:
        If any dimension of out is less than 3.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to a C char.

    :returns:
        0 or :data:`forcedimension.dhd.constants.TIMEGUARD` on success,
        and -1 otherwise.
    """

    J = ((c_double * 3) * 3)()

    err = _libdhd.dhdGetWristJacobian(J, ID)

    for i in range(3):
        for j in range(3):
            out[i][j] = J[i][j]

    return err


_libdhd.dhdWristJointAnglesToJacobian.argtypes = [
    c_double,
    c_double,
    c_double,
    (c_double * 3) * 3,
    c_byte
]
_libdhd.dhdWristJointAnglesToJacobian.restype = c_int


def wristJointAnglesToJacobian(
    joint_angles: FloatVectorLike,
    out: MutableFloatMatrixLike,
    ID: int = -1,
) -> int:
    """
    Retrieve the 3x3 jacobian matrix for the wrist structure
    based on a given joint configuration. Please refer to your device user
    manual for more information on your device coordinate system.

    :param FloatVectorLike joint_angles:
        Sequence of ``(j0, j1, j2)`` where ``j0``, ``j1``, and ``j2`` refer to
        the joint angles for wrist axis 0, 1, and 2, respectively.

    :param int ID:
        Device ID (see multiple devices section for details), defaults to -1.

    :param MutableFloatMatrixLike out:
        An output buffer to store the Jacobian.

    :raises TypeError:
        If ``out`` does not support item assignment either
        because it's immutable or not subscriptable.

    :raises IndexError:
        If ``len(out) < 3``.

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

    J = ((c_double * 3) * 3)()

    err = _libdhd.dhdWristJointAnglesToJacobian(
        joint_angles[0],
        joint_angles[1],
        joint_angles[2],
        J,
        ID
    )

    for i in range(3):
        for j in range(3):
            out[i][j] = J[i][j]

    return err


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
    joint_angles: FloatVectorLike,
    minq_out: MutableFloatVectorLike,
    maxq_out: MutableFloatVectorLike,
    ID: int = -1
) -> int:
    """
    Compute the range of applicable wrist joint torques for a given wrist joint
    angle configuration. Please refer to your device user manual for more
    information on your device coordinate system.

    :param FloatVectorLike joint_angles:
        Sequence of ``(j0, j1, j2)`` where ``j0``, ``j1``, ``j2`` refer to the
        joint angles for wrist axes 0, 1, and 2, respectively.

    :param int ID:
        Device ID (see multiple devices section for details), defaults to -1.

    :param MutableFloatVectorLike minq_out:
        An output buffer to store the return.

    :param MutableFloatVectorLike maxq_out:
        An output buffer to store the return.

    :raises TypeError:
        If ``minq_out`` does not support item assignment
        either because it's immutable or not subscriptable.

    :raises IndexError:
        If ``len(minq_out) < MAX_DOF``

    :raises TypeError:
        If ``maxq_out`` does not support item assignment
        either because it's immutable or not subscriptable.

    :raises IndexError:
        If ``len(maxq_out) < MAX_DOF``

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

    minq_out[0] = minq[0]
    minq_out[1] = minq[1]
    minq_out[2] = minq[2]

    maxq_out[0] = maxq[0]
    maxq_out[1] = maxq[1]
    maxq_out[2] = maxq[2]

    return err


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
    out: MutableFloatVectorLike,
    ID: int = -1
) -> int:
    """
    Compute the wrist joint torques required to compensate for gravity in a
    given wrist joint angle configuration. Please refer to your device user
    manual for more information on your device coordinate system.

    :param FloatVectorLike joint_angles:
        Sequence of ``(j0, j1, j2)`` where ``j0``, ``j1``, and ``j2`` refer to
        the joint angles for wrist axes 0, 1, and 2, respectively.

    :param int ID:
        Device ID (see multiple devices section for details), defaults to -1.

    :param MutableFloatVectorLike out:
        An output buffer to store the joint torques.

    :raises TypeError:
        If ``out`` does not support item assignment either
        because it's immutable or not subscriptable.

    :raises IndexError:
        If ``len(out) < 3``.

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

    q0 = c_double()
    q1 = c_double()
    q2 = c_double()

    err = _libdhd.dhdWristGravityJointTorques(
        joint_angles[0],
        joint_angles[1],
        joint_angles[2],
        q0,
        q1,
        q2,
        ID
    )

    out[0] = q0.value
    out[1] = q1.value
    out[2] = q2.value

    return err


_libdhd.dhdSetWristJointTorques.argtypes = [
    c_double,
    c_double,
    c_double,
    c_byte
]
_libdhd.dhdSetWristJointTorques.restype = c_int


def setWristJointTorques(
    t: FloatVectorLike,
    ID: int = -1
) -> int:
    """
    Set all joint torques of the wrist structure.

    :param FloatVectorLike t:
        Sequence of (t0, t1, t2) where t0, t1, t2 are the wrist axis torque
        commands for axes 0, 1, and 2, respectively in [Nm].

    :param int ID:
        Device ID (see multiple devices section for details), defaults to -1.

    :raises ArgumentError:
        If any element of ``t`` is not implicitly convertible to a C double.

    :raises IndexError:
        If ``len(t) < 3``.

    :raises TypeError:
        If ``t`` is not subscriptable.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to a C char.

    :returns: 0 on success, -1 otherwise.
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
    f: FloatVectorLike,
    t: FloatVectorLike,
    ID: int = -1
) -> int:
    """
    Set force (in [N]) and wrist joint torques (in [Nm]) about the
    x, y, and z axes.

    :param FloatVectorLike f:
        Sequence of ``(fx, fy, fz)`` where ``fx``, ``fy``, and ``fz`` are the
        translation forces (in [N]) to be applied to the DELTA end-effector on
        the X, Y, and Z axes respectively.

    :param FloatVectorLike t:
        Sequence of (t0, t1, t2) where ``t0``, ``t1``, ``t2`` are the wrist
        joint torques (in [Nm]) to be applied to the wrist end-effector
        for axes 0, 1, and 2, respectively.

    :param int ID:
        Device ID (see multiple devices section for details), defaults to -1.

    :raises ArgumentError:
        If any element of ``f`` is not implicitly convertible to a C double.

    :raises IndexError:
        If ``len(f) < 3``.

    :raises TypeError:
        If ``f`` is not subscriptable.

    :raises ArgumentError:
        If any element of ``t`` is not implicitly convertible to a C double.

    :raises IndexError:
        If ``len(t) < 3``.

    :raises TypeError:
        If ``t`` is not subscriptable.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to a C char.

    :returns: 0 on success, -1 otherwise
    """
    return _libdhd.dhdSetForceAndWristJointTorques(
        f[0], f[1], f[2], t[0], t[1], t[2], ID
    )


_libdhd.dhdSetForceAndWristJointTorquesAndGripperForce.argtypes = [
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
        f: FloatVectorLike,
        t: FloatVectorLike,
        fg: float,
        ID: int = -1) -> int:
    """
    Set force (in [N]) and wrist joint torques (in [Nm]) about the x, y and z
    axes as well as the and gripper force

    :param FloatVectorLike f:
        Sequence of ``(fx, fy, fz)`` where ``fx``, ``fy``, and ``fz`` are the
        translation forces (in [N]) to be applied to the DELTA end-effector on
        the X, Y, and Z axes respectively.

    :param FloatVectorLike t:
        Sequence of (t0, t1, t2) where ``t0``, ``t1``, ``t2`` are the wrist
        joint torques (in [Nm]) to be applied to the wrist end-effector
        for axes 0, 1, and 2, respectively.

    :param float fg:
        Gripper force in [N].

    :param int ID:
        Device ID (see multiple devices section for details), defaults to -1.

    :raises ArgumentError:
        If any element of ``f`` is not implicitly convertible to a C double.

    :raises IndexError:
        If ``len(f) < 3``.

    :raises TypeError:
        If ``f`` is not subscriptable.

    :raises ArgumentError:
        If any element of ``t`` is not implicitly convertible to a C double.

    :raises IndexError:
        If ``len(t) < 3``.

    :raises TypeError:
        If ``t`` is not subscriptable.

   :raises ArgumentError:
        If ``gripper_force`` is not implicitly convertible to a C double.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to a C char.

    :returns: 0 on success, -1 otherwise
    """
    return _libdhd.dhdSetForceAndWristJointTorquesAndGripperForce(
        f[0], f[1], f[2], t[0], t[1], t[2], fg, ID
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
    out: MutableFloatVectorLike,
    ID: int = -1
) -> int:
    """
    This routine computes and returns the wrist joint angles (in [rad])
    for a given set of encoder values.

    :param IntVectorLike enc:
        Sequence of (enc0, enc1, enc2) where ``enc0``, ``enc1``, and ``enc2``
        refer to encoder values on wrist axes 0, 1, and 2, respectively.

    :param MutableFloatVectorLike out:
        An output buffer to store the joint angles (in [rad]).

    :param int ID:
        Device ID (see multiple devices section for details), defaults to -1.

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

    j0 = c_double()
    j1 = c_double()
    j2 = c_double()

    err = _libdhd.dhdWristEncodersToJointAngles(
        enc[0], enc[1], enc[2], j0, j1, j2, ID
    )

    out[0] = j0.value
    out[1] = j1.value
    out[2] = j2.value

    return err


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
    out: MutableIntVectorLike,
    ID: int = -1
) -> int:
    """
    This routine computes and returns the wrist encoder values for a given
    set of wrist joint angles (in [rad]).

    :param FloatVectorLike enc:
        Sequence of ``(j0, j1, j1)`` where ``j0``, ``j1``, and ``j2`` refer to
        wrist joint angles for axes 0, 1, and 2, respectively, (in [rad]).

    :param int ID:
        Device ID (see multiple devices section for details), defaults to -1.

    :param IntVectorLike out:
        An output buffer to store the raw encoder values.

    :raises TypeError:
        If ``out`` does not support item assignment either
        because it's immutable or not subscriptable.

    :raises IndexError:
        If ``len(out) < 3``.

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

    enc0 = c_int()
    enc1 = c_int()
    enc2 = c_int()

    err = _libdhd.dhdWristJointAnglesToEncoders(
        joint_angles[0],
        joint_angles[1],
        joint_angles[2],
        enc0,
        enc1,
        enc2,
        ID
    )

    out[0] = enc0.value
    out[1] = enc1.value
    out[2] = enc2.value

    return err


_libdhd.dhdGetJointAngles.argtypes = [c_double * MAX_DOF, c_byte]
_libdhd.dhdGetJointAngles.restype = c_int


def getJointAngles(out: MutableFloatVectorLike, ID: int = -1) -> int:
    """
    Retrieve the joint angles (in [rad]) for all sensed degrees-of-freedom of
    the current device.

    See Also
    --------
    :data:`forcedimension.dhd.constants.MAX_DOF`


    :param int ID:
        Device ID (see multiple devices section for details), defaults to -1.

    :param MutableFloatVectorLike out:
        An output buffer to store the joint angles (in [rad]).

    :raises TypeError:
        If ``out`` does not support item assignment either
        because it's immutable or not subscriptable.

    :raises IndexError:
        If ``len(out) < MAX_DOF``

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to a C char.

    :returns:
        0 or :data:`forcedimension.dhd.constants.TIMEGUARD` on success,
        -1 otherwise.
    """

    joint_angles = (c_int * MAX_DOF)()

    err = _libdhd.dhdGetJointAngles(joint_angles, ID)

    for i in range(MAX_DOF):
        out[i] = joint_angles[i]

    return err


_libdhd.dhdGetJointVelocities.argtypes = [c_double * MAX_DOF, c_byte]
_libdhd.dhdGetJointVelocities.restype = c_int


def getJointVelocities(out: MutableFloatVectorLike, ID: int = -1) -> int:
    """
    Retrieve the joint angle velocities (in [rad/s]) for all sensed
    degrees-of-freedom of the current device.

    See Also
    --------
    :data:`forcedimension.dhd.constants.MAX_DOF`


    :param int ID:
        Device ID (see multiple devices section for details), defaults to -1.

    :param MutableFloatVectorLike out:
        An output buffer to store the joint velocities (in [rad/s]).

    :raises TypeError:
        If ``out`` does not support item assignment either
        because it's immutable or not subscriptable.

    :raises IndexError:
        If ``len(out) < :data:forcedimension.dhd.constants.MAX_DOF``

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to a C char.

    :returns:
        0 or :data:`forcedimension.dhd.constants.TIMEGUARD` on success,
        -1 otherwise.
    """

    w = (c_int * MAX_DOF)()

    err = _libdhd.dhdGetJointVelocities(w, ID)

    for i in range(MAX_DOF):
        out[i] = w[i]

    return err


_libdhd.dhdGetEncVelocities.argtypes = [c_double * MAX_DOF, c_byte]
_libdhd.dhdGetEncVelocities.restype = c_int


def getEncVelocities(out: MutableFloatVectorLike, ID: int = -1) -> int:
    """
    Retrieve the encoder angle velocities (in [inc/s]) for all sensed
    degrees-of-freedom of the current device

    See Also
    --------
    :data:`forcedimension.dhd.constants.MAX_DOF`


    :param int ID:
        Device ID (see multiple devices section for details), defaults to -1.

    :param MutableFloatVectorLike out:
        An output buffer to store the encoder velocities (in [inc/s]).

    :raises TypeError:
        If ``out`` does not support item assignment either
        because it's immutable or not subscriptable.

    :raises IndexError:
        If ``len(out) < MAX_DOF``

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to a C char.

    :returns:
       0 or :data:`forcedimension.dhd.constants.TIMEGUARD` on success,
       -1 otherwise.
    """

    v = (c_int * MAX_DOF)()

    err = _libdhd.dhdGetEncVelocities(v, ID)

    for i in range(MAX_DOF):
        out[i] = v[i]

    return err


_libdhd.dhdJointAnglesToInertiaMatrix.argtypes = [
    c_double * MAX_DOF,
    (c_double * 6) * 6,
    c_byte
]
_libdhd.dhdJointAnglesToInertiaMatrix.restype = c_int


def jointAnglesToIntertiaMatrix(
    joint_angles: FloatVectorLike,
    out: MutableFloatMatrixLike,
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
        Device ID (see multiple devices section for details), defaults to -1.

    :param MutableFloatMatrixLike out:
        Output buffer for the inertia matrix.

    :raises ArgumentError:
        If ``joint_angles`` is not implicitly convertible to a C char

    :raises IndexError:
        If ``len(joint_angles) < 3``.

    :raises TypeError:
        If ``joint_angles`` is not subscriptable.

    :raises TypeError:
        If ``out`` does not support item assignment either
        because it's immutable or not subscriptable.

    :raises IndexError:
        If of the any dimension of ``out`` is less than 6.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to a C char.

    :returns:
        0 on success, -1 otherwise.
    """

    inertia = ((c_double * 6) * 6)()
    err = _libdhd.dhdJointAnglesToInertiaMatrix(
        (c_double * MAX_DOF)(
            joint_angles[0],
            joint_angles[1],
            joint_angles[2],
            joint_angles[3],
            joint_angles[4],
            joint_angles[5],
            joint_angles[6],
            joint_angles[7]
        ),
        inertia,
        ID
    )

    for i in range(6):
        for j in range(6):
            out[i][j] = inertia[i][j]

    return err


_libdhd.dhdSetComMode.argtypes = [c_int, c_byte]
_libdhd.dhdSetComMode.restype = c_int


def setComMode(mode: ComMode, ID: int = -1) -> int:
    """
    Set the COM operation mode on compatible devices.

    :param ComMode mode:
        desired COM operation mode.

    :param int ID:
        Device ID (see multiple devices section for details), defaults to -1.

    :raises ArgumentError:
        If ``mode`` is not implicitly convertible to a C char.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to a C char.

    :returns: 0 on success, -1 otherwise.
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


    :param int duration:
        watchdog duration in multiples of 125 us.

    :param int ID:
        Device ID (see multiple devices section for details), defaults to -1.

    :raises ArgumentError:
        If ``duration`` is not implicitly convertible to a C char.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to a C char.

    :returns: 0 on success, -1 otherwise.
    """

    return _libdhd.dhdSetWatchdog(duration, ID)


_libdhd.dhdGetWatchdog.argtypes = [c_ubyte_ptr, c_byte]
_libdhd.dhdGetWatchdog.restype = c_int


def getWatchdog(ID: int = -1) -> Tuple[int, int]:
    """
    Get the watchdog duration in multiples of 125 us on compatible
    devices.

    See Also
    --------
    :func:`forcedimension.dhd.expert.setWatchdog()`


    :param int ID:
        Device ID (see multiple devices section for details), defaults to -1.

    :raises ArgumentError:
        If ``ID`` is not implicitly convertible to a C char.

    :returns:
        0 on success, -1 otherwise.
    """

    duration = c_int()

    return (duration.value, _libdhd.dhdSetWatchdog(duration, ID))
