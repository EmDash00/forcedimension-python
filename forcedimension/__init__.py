__all__ = ['libdhd', 'libdrd', 'runtime']

from threading import Thread
from typing import MutableSequence, Optional, Callable, List
from typing import cast
from math import nan

from time import sleep, monotonic

from forcedimension.dhd.libdhd.adaptors import CartesianTuple, DeviceTuple
import forcedimension.dhd.libdhd as libdhd
import forcedimension.dhd.libdhd.expert  # NOQA

from forcedimension.util import ( # NOQA
    NamedSequence,
    EuclidianVector,
    JacobianMatrix,
    ImmutableWrapper,
    UpdateOpts,
)

from forcedimension.dhd.libdhd import DeviceType
from forcedimension.dhd.libdhd.adaptors import (
    StatusTuple,
    DHDIOError,
    DHDErrorNoDeviceFound,
    ErrorNum,
    errno_to_exception
)

DefaultVecType = EuclidianVector  # type: ignore

try:
    from forcedimension.util import NumpyVector, NumpyJacobian  # NOQA
    DefaultVecType = NumpyVector  # type: ignore
except ImportError:
    pass


MutFSeq = MutableSequence[float]
MutFSeqSeq = MutableSequence[MutableSequence[float]]
MutISeq = MutableSequence[int]

libdhd.expert.enableExpertMode()


class HapticDevice:
    """
    A HapticDevice is a high-level wrapper for any compatible ForceDimension
    device. It abstracts away low level implementation details bound in the
    libdhd and provides a peformant portable Pythonic interface for doing
    high-level control.
    """
    def __init__(
            self,
            ID: Optional[int] = None,
            devtype: Optional[DeviceType] = None,
            vecgen: Callable[[], MutFSeq] = DefaultVecType,
            matgen: Callable[[], MutFSeqSeq] = cast(Callable[[], MutFSeqSeq],
                                                    JacobianMatrix)
    ):
        """
        Create a handle to a ForceDimension haptic device.

        :param Optional[int] ID: optional ID to open. If no ID is provided, the
        first available device available is opened.

        :param Optional[DeviceType]: optional requirement for the type of
        device opened. If specified with ID, a RuntimeError will happen if the
        device at the given ID is not the required type.

        :param vecgen: A method to generate vectors for use in the haptic
        device. For maximum portability, the default is just a
        :class:`forcedimension.dhd.EuclidianVector`.
        If your system supports numpy, the default is
        :class:`forcedimension.dhd.EuclidianVector`.

        You can also provide it with any class so long as len(vecgen()) >= 3

        While you can construct a HapticDevice object this way, it is usually
        recommended to use the object with a "with" statement so that the
        device is always properly closed.
        """

        if ID is not None:
            if (ID < 0):
                raise ValueError("ID must be greater than 0.")

        self._id: Optional[int] = ID
        self._vecgen = vecgen

        if (len(vecgen()) < 3):
            raise ValueError("vecgen did not create a container with at least "
                             "length 3.")

        self._open = True

        self._enc = [0, 0, 0]
        self._joint_angles = [0, 0, 0]

        self._pos: MutFSeq = vecgen()
        self._w: MutFSeq = vecgen()
        self._v: MutFSeq = vecgen()
        self._f: MutFSeq = vecgen()
        self._t: MutFSeq = vecgen()
        self._J: MutFSeqSeq = cast(MutFSeqSeq, JacobianMatrix())

        self._req = False
        self._f_req = [0.0] * 3
        self._t_req = [0.0] * 3
        self._vibration_req: List[float] = [0] * 2
        self._buttons = 0

        self._pos_view: ImmutableWrapper[MutFSeq] = (
            ImmutableWrapper(data=self._pos)
        )
        self._w_view: ImmutableWrapper[MutFSeq] = (
            ImmutableWrapper(data=self._w)
        )
        self._v_view: ImmutableWrapper[MutFSeq] = (
            ImmutableWrapper(data=self._v)
        )
        self._f_view: ImmutableWrapper[MutFSeq] = (
            ImmutableWrapper(data=self._f)
        )
        self._t_view: ImmutableWrapper[MutFSeq] = (
            ImmutableWrapper(data=self._t)
        )
        self._J_view: ImmutableWrapper[MutFSeqSeq] = (
            ImmutableWrapper(data=self._J)
        )

        self._mass = None

        self.gripper = None

        self._devtype = devtype

        self._thread_exception = None

        self._haptic_deamon: Optional[HapticDaemon] = None

        if (libdhd.getDeviceCount() > 0):
            if self._id is None:
                if (self._devtype is None):
                    self._id = libdhd.open()
                    self._devtype = libdhd.getSystemType()
                else:
                    self._id = libdhd.openType(self._devtype)

                if (self._devtype == -1):
                    raise errno_to_exception(libdhd.errorGetLast())(
                        ID=cast(int, self._id),
                        op=libdhd.getSystemType
                    )
            else:
                libdhd.openID(cast(int, self._id))
                if (cast(int, self._id) == -1):
                    raise errno_to_exception(libdhd.errorGetLast())

                devtype = libdhd.getSystemType()

                if (devtype == -1):
                    raise errno_to_exception(libdhd.errorGetLast())(
                        ID=cast(int, self._id),
                        op=libdhd.getSystemType
                    )

                if self.devtype is not None:
                    if (self.devtype != devtype):
                        raise Exception(
                            "Device is not of type {}".format(self.devtype))

            self._left_handed = libdhd.isLeftHanded(cast(int, self._id))

            if (libdhd.hasGripper(cast(int, self._id))):
                self.gripper = Gripper(
                    self,
                    cast(int, self._id),
                    vecgen
                )

            self.update_enc_and_calculate()
            self.update_velocity()
            # self.update_angular_velocity()

            if self.gripper is not None:
                self.update_force_and_torque_and_gripper_force()
            else:
                self.update_force_and_torque()
        else:
            raise DHDErrorNoDeviceFound()

    def check_threadex(self):
        if self._thread_exception is not None:
            raise self._thread_exception

    @property
    def ID(self) -> Optional[int]:
        """
        Provides a read-only accessor to the ID of the HapticDevice.
        Thread-safe.

        :rtype: Optional[int]
        :returns: The ID of the HapticDevices
        """
        self.check_threadex()
        return cast(int, self._id)

    @property
    def devtype(self) -> Optional[DeviceType]:
        return self._devtype

    @property
    def pos(self) -> Optional[ImmutableWrapper[MutFSeq]]:
        """
        Provides a copy of the last-known position of the HapticDevice's end
        effector. Thread-safe.

        :rtype: Optional[MutableSequence[float]]

        :returns: A mutable sequence of [x, y, z] where x, y, and z are the
        end-effector's position given in [m].
        """
        self.check_threadex()
        return self._pos_view

    @property
    def mass(self) -> Optional[float]:
        """
        Get the mass of the end-effector used for gravity compensation in [kg].

        :rtype: Optional[float]

        :returns: the set mass of the end-effector in [kg]
        """
        return self._mass

    def set_mass(self, m: float):
        libdhd.setEffectorMass(m, ID=cast(int, self._id))

    @property
    def v(self) -> Optional[ImmutableWrapper[MutFSeq]]:
        """
        Provides a copy of the last-known linear velocity of the HapticDevice's
        end-effector. Thread-safe.

        :rtype: Optional[MutableSequence[float]]

        :returns: A mutable sequence of [vx, vy, vz] where vx, vy, and vz are
        the end-effector's linear velocity given in [m/s].
        """
        self.check_threadex()
        return self._v_view

    @property
    def w(self) -> Optional[ImmutableWrapper[MutFSeq]]:
        """
        Provides a copy of the last-known angular velocity of the
        HapticDevice's end-effector. Thread-safe.

        :rtype: Optional[MutableSequence[float]]

        :returns: A mutable sequence of [wx, wy, wz] where wx, wy, and wz are
        the end-effector's linear velocity given in [rad/s].
        """
        self.check_threadex()
        return self._w_view

    @property
    def t(self) -> Optional[ImmutableWrapper[MutFSeq]]:
        """
        Provides a copy of the last-known applied torque of the HapticDevice's
        end-effector. Thread-safe.

        :rtype: Optional[MutableSequence[float]]

        :returns: A mutable sequence of [tx, ty, tz] where tx, ty, and tz are
        the torque experienced by the end-effector in [Nm]
        """
        self.check_threadex()
        return self._t_view

    @property
    def f(self) -> Optional[ImmutableWrapper[MutFSeq]]:
        """
        Provides a copy of the last-known applied force of the HapticDevice's
        end-effector. Thread-safe.

        :rtype: Optional[MutableSequence[float]]

        :returns: A mutable sequence of [fx, fy, fz] where fx, fy, and fz are
        the torque experienced by the end-effector in [N]
        """

        self.check_threadex()
        return self._f_view

    @property
    def left_handed(self) -> Optional[bool]:
        return self._left_handed

    def get_status(self) -> StatusTuple:
        """
        Perform a blocking read to the HapticDevice, requesting all pertinent
        status information.

        :rtype: StatusTuple
        :returns: StatusTuple containing all status information.
        """
        status, err = libdhd.getStatus(ID=cast(int, self._id))

        if (err):
            raise errno_to_exception(libdhd.errorGetLast())

        return status

    def calculate_pos(self) -> None:
        """
        Calculates and stores the position of the device given the current
        end-effector position.
        """

        libdhd.expert.deltaEncoderToPosition(
            ID=cast(int, self._id),
            enc=cast(DeviceTuple, self._enc),
            out=self._pos
        )

    def calculate_joint_angles(self) -> None:
        """
        Calculates and stores the joint angles of the device given the current
        end-effector encoder readings.
        """
        libdhd.expert.deltaEncodersToJointAngles(
            ID=cast(int, self._id),
            enc=cast(DeviceTuple, self._enc),
            out=cast(MutableSequence[float], self._joint_angles)
        )

    def calculate_jacobian(self) -> None:
        """
        Calculates and stores the Jacobian matrix of the device given the
        current end-effector position.
        """
        self.calculate_joint_angles()

        libdhd.expert.deltaJointAnglesToJacobian(
            ID=cast(int, self._id),
            joint_angles=cast(CartesianTuple, self._joint_angles),
            out=self._J
        )

    # TODO: Actually implement this
    """
    def calculate_velocity(self) -> None:
        Uses the internal velocity estimator to provide an estimated velocity
        using position data.

        :raises: ValueError if there is no internal velocity estimator set.
        if (self._velocity_estimator is not None):
            self._velocity_estimator.feed(self._pos_view)
            self._velocity_estimator.update(self._v)
        else:
            raise ValueError("There is no velocity estimator configured.")

    def calculate_angular_velocity(self):
        self.calculate_jacobian()

    """
    def update_enc_and_calculate(self) -> None:
        self.update_enc()
        self.calculate_joint_angles()
        self.calculate_pos()
        self.calculate_jacobian()

    def update_enc(self) -> None:
        """
        Performs a blocking read to the HapticDevice, requesting the current
        encoder readers of the (DELTA structure that controls the) end-effector
        and updates the last-known position  with the response.

        :rtype: None
        """

        _, err = libdhd.expert.getDeltaEncoders(
                    ID=cast(int, self._id),
                    out=self._enc
                )

        if err and err != libdhd.TIMEGUARD:
            raise errno_to_exception(
                ErrorNum(libdhd.errorGetLast()))(
                    ID=cast(int, self.ID),
                    feature=libdhd.expert.getDeltaEncoders
                )

    def update_position(self) -> None:
        """
        Performs a blocking read to the HapticDevice, requesting the current
        position of the end-effector and updates the last-known position with
        the response.

        :rtype: None
        """

        _, err = libdhd.getPosition(ID=cast(int, self._id), out=self._pos)

        if err and err != libdhd.TIMEGUARD:
            raise errno_to_exception(
                ErrorNum(libdhd.errorGetLast()))(
                    ID=cast(int, self.ID),
                    feature=libdhd.getPosition
                )

    def update_velocity(self) -> None:
        """
        Performs a blocking read to the HapticDevice, requesting the current
        linear velocity of the end-effector and updates the last-known linear
        velocity with the response.

        :rtype: None
        """
        _, err = libdhd.getLinearVelocity(ID=cast(int, self._id), out=self._v)

        if err and err != libdhd.TIMEGUARD:
            if libdhd.errorGetLast() != ErrorNum.TIMEOUT:
                raise errno_to_exception(ErrorNum(libdhd.errorGetLast()))(
                    ID=cast(int, self._id),
                    feature=libdhd.getLinearVelocity
                )
            else:
                self._v = [float('nan'), float('nan'), float('nan')]

    def update_angular_velocity(self):
        """
        Performs a blocking read to the HapticDevice, requesting the current
        angular velocity of the end-effector and updates the last-known
        angular velocity with the response.

        :rtype: None
        """
        _, err = libdhd.getAngularVelocityRad(
                    ID=cast(int, self._id),
                    out=self._w
        )

        if (err):
            if libdhd.errorGetLast() != ErrorNum.TIMEOUT:
                raise errno_to_exception(ErrorNum(libdhd.errorGetLast()))(
                    ID=cast(int, self._id),
                    feature=libdhd.getAngularVelocityRad
                )
            else:
                self._v = [float('nan'), float('nan'), float('nan')]

    def update_force(self):
        """
        Performs a blocking read to the HapticDevice, requesting the current
        force the end end-effector is experiencing and updates the last-known
        force with the response.

        :rtype: None
        """

        _, err = libdhd.getForce(ID=cast(int, self._id), out=self._f)

        if err and err != libdhd.TIMEGUARD:
            raise errno_to_exception(ErrorNum(
                libdhd.errorGetLast())(
                    ID=cast(int, self._id),
                    feature=libdhd.getForce
                )
            )

    def update_force_and_torque(self):
        """
        Performs a blocking read to the HapticDevice, requesting in parallel
        the current force and torque applied to the end-effector and
        updates the last-known force and torque with the response.

        :rtype: None
        """
        _, _, err = libdhd.getForceAndTorque(
            ID=cast(int, self._id),
            f_out=self._f,
            t_out=self._t
        )

        if (err):
            raise errno_to_exception(libdhd.errorGetLast())(
                    ID=cast(int, self._id),
                    feature=libdhd.getForceAndTorque
                )

    def update_force_and_torque_and_gripper_force(self):
        """
        Performs a blocking read to the HapticDevice, requesting in parallel
        the force and torque applied to the end-effector as well as the force
        applied to the gripper. Then, the last known force, torque, and gripper
        force are updated.

        The reason this is in HapticDevice is largely due to an implement
        detail in libdhd and as a way to optimize requests to the device.
        """
        if self.gripper is not None:
            _, _, fg, err = libdhd.getForceAndTorqueAndGripperForce(
                ID=cast(int, self._id),
                f_out=self._f,
                t_out=self._t
            )
            if (err):
                raise errno_to_exception(ErrorNum(libdhd.errorGetLast()))(
                    ID=cast(int, self._id),
                    feature=libdhd.getForceAndTorqueAndGripperForce
                )

            self.gripper._fg = fg

    def update_buttons(self):
        """
        Performs a blocking read and gets the state of all buttons on the
        device into a int bit vector buffer.

        See Also
        --------
        :func:`HapticDevice.get_button`
        """
        self._buttons = libdhd.getButtonMask(ID=self._id)

    def submit(self):
        """
        Push the requested forces and torques to the device in a blocking send.

        See Also
        --------
        :func:HapticDevice.req

        """
        self._req = False
        err = libdhd.setForceAndTorque(
                self._f_req,
                self._t_req,
                cast(int, self._id)
            )

        if err:
            if err == libdhd.MOTOR_SATURATED:
                pass
            else:
                raise errno_to_exception(ErrorNum(libdhd.errorGetLast()))(
                        ID=cast(int, self._id),
                        feature=libdhd.setForceAndTorqueAndGripperForce
                    )

    def req(
        self,
        f: CartesianTuple,
        t: CartesianTuple = CartesianTuple(0, 0, 0)
    ) -> None:
        """
        Load the request force and request torque buffer for this device.
        This won't send the request to the device. This is used by the
        HapticDaemon.

        :param CartesianTuple f:
            The force in [N] to apply to the end effector about
            the x, y, and z axes.

        :param CartesianTuple t:
            The torque in [Nm] to apply to the end effector about the
            x, y,and z axes.

        See Also
        --------
        :func:`HapticDevice.submit`
        """
        self._req = True
        self._f_req[0:3] = f
        self._t_req[0:3] = t

    def req_vibration(self, f: float, A: float):
        """
        Load the requested vibration into the vibration buffer. This won't
        send the request to the device. This is used by the HapticDaemon. This
        vibration will be added on top of the force requested to the device.

        :param float f: frequency of the vibration in [Hz]
        :param float A: amplitude of the vibration

        See Also
        --------
        :func:`HapticDevice.submit_vibration`
        """

        self._vibration_req[0:2] = [f, A]

    def neutral(self):
        """
        Disable forces and put the device in IDLE mode. Brakes will be turned
        off and gravity compensation disabled. No forces will be allowed to be
        put on the device.
        """
        self.enable_force(enabled=False)
        self.enable_brakes(enabled=False)
        self.enable_gravity_compensation(enabled=False)

    def stop(self):
        """
        Disable force and put the device in BRAKE mode. You may feel a viscous
        force that keeps that device from moving too quickly in this mode.
        """
        libdhd.stop(cast(int, self._id))

    def submit_vibration(self):
        err = libdhd.setVibration(
                    ID=cast(int, self._id),
                    f=self._vibration_req[0],
                    A=self._vibration_req[1],
                    device_type=self.devtype
                )

        if err:
            if err == libdhd.MOTOR_SATURATED:
                pass
            raise errno_to_exception(ErrorNum(libdhd.errorGetLast()))(
                    ID=cast(int, self._id),
                    feature=libdhd.setVibration
                )

    def submit_enc(self):
        _, err = libdhd.expert.getEnc(
            ID=cast(int, self._id),
            mask=self._enc_req,
            out=self._enc
        )

        if err:
            raise errno_to_exception(ErrorNum(libdhd.errorGetLast()))(
                    ID=cast(int, self._id),
                    feature=libdhd.getEnc
                )

    def enable_force(self, enabled: bool = True):
        """
        Enable/disable force on the end-effector.

        :param bool enabled: [default=True] true to enable, false to disable
        """
        libdhd.enableForce(enabled, ID=cast(int, self._id))

    def enable_brakes(self, enabled: bool = True):
        """
        Enable electromagnetic braking on the device.

        :param enabled bool:
            True to enable electromagnetic braking, False to disable.
        """
        err = libdhd.setBrakes(enabled)
        if err:
            raise errno_to_exception(ErrorNum(libdhd.errorGetLast()))(
                    ID=cast(int, self._id),
                    feature=libdhd.setVibration
                )

    def get_max_force(self) -> Optional[float]:
        """
        Retrieve the current limit (in N) to the force magnitude that can be
        applied by the haptic device.

        :returns: The current limit (in N) to the force magnitude that can be
        applied by the haptic device to the end-effector. If there is no limit,
        None is returned instead.

        rtype: Optional[float]
        """

        limit = libdhd.getMaxForce(ID=cast(int, self._id))

        return limit if limit > 0 else None

    def set_max_force(self, limit: Optional[float]) -> None:
        """
        Define or disable a limit (in N) to the force magnitude that can be
        applied by the haptic device.

        :param Optional[float] limit: The desired limit (in N) to the force
        magnitude that can be applied. If the limit is None, the force limit is
        disabled.
        """
        if limit is None:
            err = libdhd.setMaxForce(ID=cast(int, self._id), limit=-1.0)
        else:
            if limit < 0:
                raise ValueError

            err = libdhd.setMaxForce(ID=cast(int, self._id), limit=limit)

        if err:
            raise errno_to_exception(ErrorNum(libdhd.errorGetLast()))

    def get_max_torque(self) -> Optional[float]:
        """
        Retrieve the current limit (in Nm) to the torque magnitude that can be
        applied by the haptic device.

        :returns: The current limit (in Nm) to the force magnitude that can be
        applied by the haptic device to the end-effector. If there is no limit,
        None is returned instead.

        rtype: Optional[float]
        """

        limit = libdhd.getMaxTorque(ID=cast(int, self._id))

        return limit if limit > 0 else None

    def set_max_torque(self, limit: Optional[float]) -> None:
        """
        Define or disable a limit (in N) to the force magnitude that can be
        applied by the haptic device.

        :param Optional[float] limit: The desired limit (in N) to the force
        magnitude that can be applied. If the limit is None, the force limit is
        disabled.
        """
        if limit is None:
            err = libdhd.setMaxTorque(ID=cast(int, self._id), limit=-1.0)
        else:
            if limit < 0:
                raise ValueError

            err = libdhd.setMaxTorque(ID=cast(int, self._id), limit=limit)

        if err:
            raise errno_to_exception(ErrorNum(libdhd.errorGetLast()))

    def enable_gravity_compensation(self, enabled: bool = True):
        """
        Enable built-in gravity compensation for the end-effector that will be
        added on top of the force request.

        :param bool enabled: True to enable, False to disable
        """
        libdhd.setGravityCompensation(enabled, ID=cast(int, self._id))

        self._req = True
        self._f_req[0:3] = [0.0, 0.0, 0.0]
        self._t_req[0:3] = [0.0, 0.0, 0.0]

        libdhd.stop(cast(int, self._id))

    def get_button(self, button_id: int = 0) -> bool:
        """
        See if the button on the device is being pressed.

        :param int button_id: The button to check

        :rtype: bool
        :returns: True if the button is being pressed, False otherwise.

        See Also
        --------
        :class:forcedimension.dhd.libdhd.constants.NovintButtonID
        """
        return bool(self._buttons & cast(int, 1 << button_id))

    def close(self):
        self.open = False
        if self._haptic_deamon is not None:
            self._haptic_deamon.stop()
        libdhd.close(cast(int, self._id))

    def __enter__(self):
        return self

    def __exit__(self, t, value, traceback):
        self.close()


class Gripper:
    """
    A high-level wrapper for methods and kinematic data of a Gripper on a
    HapticDevice.

    Certain kinds of HapticDevices opened will have grippers. If that is the
    case, a Gripper object will be instantiated as well containing methods to
    get kinematic information about the Gripper.
    """

    def __init__(
            self,
            parent: HapticDevice,
            ID: Optional[int] = None,
            vecgen: Callable[[], MutableSequence[float]] = EuclidianVector
    ):

        if ID is not None:
            if (ID < 0):
                raise ValueError("ID must be greater than 0.")

            self._id: int = ID

        VecType = vecgen

        self._enc: int = 0
        self._angle: float = nan
        self._gap: float = nan
        self._v: float = nan
        self._w: float = nan
        self._fg: float = nan

        self._thumb_pos: MutFSeq = VecType()
        self._finger_pos: MutFSeq = VecType()

        self._parent: HapticDevice = parent

        self._thumb_pos_view: ImmutableWrapper[MutFSeq] = (
            ImmutableWrapper(self._thumb_pos)
        )

        self._finger_pos_view: ImmutableWrapper[MutFSeq] = (
            ImmutableWrapper(self._finger_pos)
        )

        self._fg_req: float = 0.0

    def request(self, fg: float):
        self._fg_req = fg

    def submit(self):
        self._req = False
        libdhd.setForceAndTorqueAndGripperForce(
            f=self._parent._f_req,
            t=self._parent._t_req,
            fg=self
        )

    def req(self, fg: float):
        self._parent._req = True
        self._fg_req = fg

    # def submit_enc(self) -> NoReturn:
        """
        _, err = libdhd.expert.getEnc(
            ID=cast(int, self._id),
            mask=self._enc_req,
            out=self._enc
        )
        """
        # raise NotImplementedError

    # def request_enc(self, enc_mask: int) -> NoReturn:
        # self._enc_req &= enc_mask
        # raise NotImplementedError

    def get_max_force(self) -> Optional[float]:
        """
        Retrieve the current limit (in N) to the force magnitude that can be
        applied by the haptic device.

        :returns: The current limit (in N) to the force magnitude that can be
        applied by the haptic device to the end-effector. If there is no limit,
        None is returned instead.

        rtype: Optional[float]
        """

        limit = libdhd.getMaxGripperForce(ID=cast(int, self._id))

        return limit if limit > 0 else None

    def set_max_force(self, limit: Optional[float]) -> None:
        """
        Define or disable a limit (in N) to the force magnitude that can be
        applied by the haptic device.

        :param Optional[float] limit: The desired limit (in N) to the force
        magnitude that can be applied. If the limit is None, the force limit is
        disabled.
        """
        if limit is None:
            err = libdhd.setMaxGripperForce(ID=cast(int, self._id), limit=-1.0)
        else:
            if limit < 0:
                raise ValueError

            err = libdhd.setMaxGripperForce(
                    ID=cast(int, self._id),
                    limit=limit
                )

        if err:
            raise errno_to_exception(ErrorNum(libdhd.errorGetLast()))

    @property
    def thumb_pos(self):
        self.check_threadex()
        return self._thumb_pos_view

    @property
    def finger_pos(self):
        self.check_threadex()
        return self._finger_pos_view

    @property
    def gap(self):
        self.check_threadex()
        return self._gap

    @property
    def angle(self):
        self.check_threadex()
        return self._angle

    @property
    def v(self):
        self.check_threadex()
        return self._v

    @property
    def w(self):
        self.check_threadex()
        return self._w

    @property
    def fg(self):
        self.check_threadex()
        return self._fg

    def calculate_gap(self):
        if self._enc is not None:
            self._gap = libdhd.expert.gripperEncoderToGap(
                            ID=cast(int, self._id),
                            enc=self._enc
                        )
        else:
            raise ValueError

    def calculate_angle(self):
        if self._enc is not None:
            self._angle = libdhd.expert.gripperEncoderToAngleRad(
                            ID=cast(int, self._id),
                            enc=self._enc
                        )
        else:
            raise ValueError

    def update_enc(self):
        self._enc, err = libdhd.expert.getGripperEncoder(
                            ID=cast(int, self._id)
                        )

        if err and err != libdhd.TIMEGUARD:
            raise errno_to_exception(libdhd.errorGetLast())

    def update_enc_and_calculate(self):
        self.update_enc()
        self.calculate_angle()
        self.calculate_gap()

    def update_linear_velocity(self):
        self._v, err = libdhd.getGripperLinearVelocity(ID=cast(int, self._id))
        if err and err != libdhd.TIMEGUARD:
            raise errno_to_exception(libdhd.errorGetLast())

    def update_angular_velocity(self):
        self._w, err = libdhd.getGripperAngularVelocityRad(
                            ID=cast(int, self._id)
                        )

        if err and err != libdhd.TIMEGUARD:
            raise errno_to_exception(libdhd.errorGetLast())

    def update_angle(self):
        self._angle, err = libdhd.getGripperAngleRad(cast(int, self._id))
        if err and err != libdhd.TIMEGUARD:
            raise errno_to_exception(libdhd.errorGetLast())

    def update_gap(self):
        self._gap, err = libdhd.getGripperGap(cast(int, self._id))
        if err and err != libdhd.TIMEGUARD:
            raise errno_to_exception(libdhd.errorGetLast())

    def update_thumb_pos(self):
        _, err = libdhd.getGripperThumbPos(
                    cast(int, self._id),
                    out=self._thumb_pos
                )

        if err and err != libdhd.TIMEGUARD:
            raise errno_to_exception(libdhd.errorGetLast())

    def update_finger_pos(self):
        _, err = libdhd.getGripperFingerPos(
                    cast(int, self._id),
                    out=self._finger_pos
                )
        if err and err != libdhd.TIMEGUARD:
            raise errno_to_exception(libdhd.errorGetLast())

    def check_threadex(self):
        self._parent.check_threadex()


class Poller(Thread):
    def __init__(
        self,
        f: Callable[[], None],
        min_period: Optional[float] = None,
    ):
        self._min_period = min_period
        self.ex = None

        self._stopped = False
        self._paused = False
        self._f = f

        super().__init__()

    def stop(self):
        if not self._stopped:
            self._stopped = True
            self.join()

    def pause(self, paused=True):
        self._paused = paused

    def start(self, paused=False, *args, **kwargs):
        self._paused = paused
        super().start(*args, **kwargs)

    def run(self):
        try:
            while not self._stopped:
                while not self._paused and not self._stopped:
                    t = monotonic()

                    self._f()

                    sleep(self._min_period * 0.9)
                    while (monotonic() - t) < self._min_period:
                        pass

                while self._paused and not self._stopped:
                    sleep(self._min_period)

        except DHDIOError as ex:
            self.ex = ex
            self._paused = True


class HapticDaemon(Thread):
    def __init__(
                self,
                dev: HapticDevice,
                update_list: UpdateOpts = UpdateOpts()
            ):

        super().__init__()

        if not isinstance(dev, HapticDevice):
            raise TypeError("Daemon needs an instance of HapticDevice")

        self._paused = False
        self._dev = dev
        self._dev._haptic_deamon = self
        self.daemon = True

        self._set_pollers(update_list)

        if (self._dev.gripper is not None):
            self._req_func: Callable[[], None] = self._dev.gripper.submit
        else:
            self._req_func = self._dev.submit

    def _set_pollers(self, update_list):
        pollers = []

        if update_list is not None:
            """
            not implemented yet.

            if update_list.enc:
                funcs.append(self._dev.update_enc_and_calculate)
            """

            funcs = (
                self._dev.update_enc_and_calculate,
                self._dev.update_velocity,
                self._dev.update_angular_velocity,
                self._dev.update_buttons
            )

            pollers.extend(
                Poller(update, 1/freq)
                for freq, update in zip(update_list[:-2], funcs)
                if freq is not None
            )

            if update_list.ft is not None:
                if (
                    update_list.gripper is not None and
                    self.gripper is not None
                ):
                    pollers.append(
                        Poller(
                            self._dev.update_force_and_torque,
                            1/update_list.ft
                        )
                    )
                else:
                    f = self._dev.update_force_and_torque_and_gripper_force
                    pollers.append(
                        Poller(f, 1/update_list.ft)
                    )
        if update_list.gripper is not None:
            funcs = (
                self._dev.gripper.update_enc_and_calculate,
                self._dev.gripper.update_finger_pos,
                self._dev.gripper.update_thumb_pos,
                self._dev.gripper.update_linear_velocity,
                self._dev.gripper.update_angular_velocity
            )

            pollers.extend(
                Poller(update, 1/freq)
                for freq, update in zip(update_list.gripper, funcs)
                if freq is not None
            )

            self._force_poller = (
                Poller(self._dev.gripper.submit, 1/update_list.req)
            )
        else:
            self._force_poller = (
                Poller(self._dev.submit, 1/update_list.req)
            )

        self._pollers = pollers

    def stop(self):
        self._paused = True
        if self._pollers is not None:
            for poller in self._pollers:
                poller.stop()

        self._force_poller.stop()

        self.join()

    def forcepoll(self, poll=True):
        if poll:
            self._force_poller.pause(not poll)

    def run(self, forceon=True):
        self._paused = False
        try:
            for poller in self._pollers:
                poller.start()

            self._force_poller.start(not forceon)

            while not self._paused:
                for poller in self._pollers:
                    if poller.ex is not None:
                        raise poller.ex

                if self._force_poller.ex is not None:
                    raise self._force_poller.ex

                sleep(0.01)

        except DHDIOError as ex:
            self._paused = True

            for poller in self._pollers:
                poller.stop()
            self._dev._thread_exception = ex
