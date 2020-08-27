"""
.. module::dhd
   :platform: Windows, Unix
   :synopsis: libdhd Python bindings

.. moduleauthor:: Drason Chow <drasonchow@gmail.com>
"""

__all__ = ['bindings']

from threading import Thread
from concurrent.futures import ThreadPoolExecutor

from typing import MutableSequence, Optional, Callable
from typing import cast
from time import monotonic

from forcedimension.dhd.bindings.adaptors import CartesianTuple, DeviceTuple
import forcedimension.dhd.bindings as libdhd
import forcedimension.dhd.bindings.expert  # NOQA

from forcedimension.dhd.util import ( # NOQA
    Euclidian,
    EuclidianVector,
    JacobianMatrix,
    ImmutableWrapper,
    UpdateTuple,
    GripperUpdateTuple
)

try:
    from forcedimension.dhd.util import NumpyVector # NOQA
except ImportError:
    pass

from forcedimension.dhd.bindings import DeviceType
from forcedimension.dhd.bindings.adaptors import (
    StatusTuple,
    DHDIOError,
    DHDErrorNoDeviceFound,
    errno_to_exception
)


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
            parent_f_req,
            parent_t_req,
            parent_enc_req,
            ID: Optional[int] = None,
            vecgen: Callable[[], MutableSequence[float]] = EuclidianVector
    ):

        if ID is not None:
            if (ID < 0):
                raise ValueError("ID must be greater than 0.")

            self._id = ID

        self.VecType = vecgen
        VecType = vecgen

        self._enc = 0
        self._angle: MutableSequence[float] = VecType()
        self._gap: MutableSequence[float] = VecType()

        self._thumb_pos: MutableSequence[float] = VecType()
        self._finger_pos: MutableSequence[float] = VecType()

        self._v: MutableSequence[float] = VecType()
        self._w: MutableSequence[float] = VecType()
        self._fg: float = 0

        self._parent_f_req = parent_f_req
        self._parent_t_req = parent_t_req
        self._fg_req: float = 0.0

    def request_ft(self, fg: float):
        self._fg_req = fg

    def submit_ft(self):
        libdhd.setForceAndTorqueAndGripperForce(
            f=self._parent_f_req,
            t=self._parent_t_req,
            fg=self
        )

    # def submit_enc(self) -> NoReturn:
        """
        _, err = libdhd.expert.getEnc(
            ID=self._id,
            mask=self._enc_req,
            out=self._enc
        )
        """
        # raise NotImplementedError

    # def request_enc(self, enc_mask: int) -> NoReturn:
        # self._enc_req &= enc_mask
        # raise NotImplementedError

    def calculate_gap(self):
        if self._enc is not None:
            self._gap = libdhd.expert.gripperEncoderToGap(
                            ID=self._id,
                            enc=self._enc
                        )
        else:
            raise ValueError

    def calculate_angle(self):
        if self._enc is not None:
            self._angle = libdhd.expert.gripperEncoderToAngleRad(
                            ID=self._id,
                            enc=self._enc
                        )
        else:
            raise ValueError

    def update_enc(self):
        self._enc, err = libdhd.expert.getGripperEncoder(ID=self._id)

    def update_enc_and_calculate(self):
        self.update_enc()
        self.calculate_angle()
        self.calculate_gap()

    def update_linear_velocity(self):
        libdhd.getAngularVelocityRad(self._id, out=self._v)

    def update_angular_velocity(self):
        libdhd.getAngularVelocityRad(self._id, out=self._w)

    def update_angle(self):
        libdhd.getGripperAngleRad(self._id, out=self._angle)

    def update_gap(self):
        libdhd.getGripperGap(self._id, out=self._gap)

    def update_thumb_pos(self):
        libdhd.getGripperThumbPos(self._id, out=self._thumb_pos)

    def update_finger_pos(self):
        libdhd.getGripperFingerPos(self._id, out=self._finger_pos)

    def update_force(self):
        pass

    def check_threadex(self):
        if self._thread_exception is not None:
            raise self._thread_exception


MutFSeq = MutableSequence[float]
MutFSeqSeq = MutableSequence[MutableSequence[float]]
MutISeq = MutableSequence[int]


class HapticDevice:
    """
    A HapticDevice is a high-level wrapper for any compatible ForceDimension
    device. It abstracts away low level implementation details bound in the
    bindings and provides a peformant portable Pythonic interface for doing
    high-level control.
    """
    def __init__(
            self,
            ID: Optional[int] = None,
            devtype: Optional[DeviceType] = None,
            vecgen: Callable[[], MutFSeq] = EuclidianVector,
            matgen: Callable[[], MutFSeqSeq] = cast(Callable[[], MutFSeqSeq],
                                                    JacobianMatrix)
    ):
        """
        Create a handle to a ForceDimension haptic device.

        :param Optional[int] ID: optional ID to open. If no ID is provided, the
        first device available is opened.

        :param Optional[DeviceType]: optional requirement for the type of
        device opened. If specified with ID, a RuntimeError will happen if the
        device at the given ID is not the required type.

        :param vecgen: A method to generate vectors for use in the haptic
        device. For maximum portability, the default is just a EuclideanVector.

        If your system supports numpy, you can provide it with the NumpyVector
        function.

        You can also provide it with any class so long as len(vecgen()) >= 3
        """

        if ID is not None:
            if (ID < 0):
                raise ValueError("ID must be greater than 0.")

            self._id = ID

        self._vecgen = vecgen

        if (len(vecgen()) < 3):
            raise ValueError("vecgen did not create a container with at least "
                             "length 3.")

        self._enc: Optional[MutISeq] = None
        self._joint_angles: Optional[MutFSeq] = None
        self._pos: Optional[MutFSeq] = None
        self._w: Optional[MutFSeq] = None
        self._v: Optional[MutFSeq] = None
        self._f: Optional[MutFSeq] = None
        self._t: Optional[MutFSeq] = None
        self._J: Optional[MutFSeqSeq] = None

        self._pos_view: Optional[ImmutableWrapper[MutFSeq]] = None
        self._w_view: Optional[ImmutableWrapper[MutFSeq]] = None
        self._v_view: Optional[ImmutableWrapper[MutFSeq]] = None
        self._f_view: Optional[ImmutableWrapper[MutFSeq]] = None
        self._t_view: Optional[ImmutableWrapper[MutFSeq]] = None
        self._J_view: Optional[ImmutableWrapper[MutFSeqSeq]] = None

        self._left_handed = None

        self._f_req = [float('nan')] * 3
        self._t_req = [float('nan')] * 3
        self._enc_req = 0
        self._buttons = 0

        self._mass = None

        self.gripper = None

        self.devtype = devtype

        self._thread_exception = None

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
        return self._id

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
        status, err = libdhd.getStatus(ID=self._id)

        if (err):
            raise errno_to_exception(libdhd.errorGetLast())

        return status

    def calculate_pos(self) -> None:
        """
        Calculates and stores the position of the device given the current
        end-effector position.
        """

        libdhd.expert.deltaEncodersToJointAngles(
            ID=self._id,
            enc=cast(DeviceTuple, self._enc),
            out=self._pos
        )

    def calculate_joint_angles(self) -> None:
        """
        Calculates and stores the joint angles of the device given the current
        end-effector encoder readings.
        """
        libdhd.expert.deltaEncodersToJointAngles(
            ID=self._id,
            enc=cast(DeviceTuple, self._enc),
            out=self._joint_angles
        )

    def calculate_jacobian(self) -> None:
        """
        Calculates and stores the Jacobian matrix of the device given the
        current end-effector position.
        """
        self.calculate_joint_angles()

        libdhd.expert.deltaJointAnglesToJacobian(
            ID=self._id,
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
    def update_enc_and_calculate(self, bitmask: int = 0xff) -> None:
        self.update_enc(bitmask)
        self.calculate_joint_angles()
        self.calculate_jacobian()

    def update_enc(self, bitmask: int = 0xff) -> None:
        """
        Performs a blocking read to the HapticDevice, requesting the current
        encoder readers of the (DELTA structure that controls the) end-effector
        and updates the last-known position  with the response.

        :rtype: None
        """

        _, err = libdhd.expert.getEnc(ID=self._id, mask=bitmask, out=self._enc)

        if (err):
            raise errno_to_exception(libdhd.errorGetLast())

    def update_position(self) -> None:
        """
        Performs a blocking read to the HapticDevice, requesting the current
        position of the end-effector and updates the last-known position with
        the response.

        :rtype: None
        """

        _, err = libdhd.getPosition(ID=self._id, out=self._pos)

        if (err):
            raise errno_to_exception(libdhd.errorGetLast())

    def update_velocity(self) -> None:
        """
        Performs a blocking read to the HapticDevice, requesting the current
        linear velocity of the end-effector and updates the last-known linear
        velocity with the response.

        :rtype: None
        """
        _, err = libdhd.getLinearVelocity(ID=self._id, out=self._v)

        if (err):
            if libdhd.errorGetLast() != libdhd.ErrorNum.TIMEOUT:
                raise errno_to_exception(libdhd.ErrorNum(err))(ID=self._id)
            else:
                self._v = [float('nan'), float('nan'), float('nan')]

    def update_angular_velocity(self):
        """
        Performs a blocking read to the HapticDevice, requesting the current
        angular velocity of the end-effector and updates the last-known
        angular velocity with the response.

        :rtype: None
        """
        _, err = libdhd.getAngularVelocityRad(ID=self._id, out=self._w)

        if (err):
            if libdhd.errorGetLast() != libdhd.ErrorNum.TIMEOUT:
                raise errno_to_exception(libdhd.ErrorNum(err))(ID=self._id)
            else:
                self._v = [float('nan'), float('nan'), float('nan')]

    def update_force(self):
        """
        Performs a blocking read to the HapticDevice, requesting the current
        force the end end-effector is experiencing and updates the last-known
        force with the response.

        :rtype: None
        """

        _, err = libdhd.getForce(ID=self._id, out=self._f)

        if (err):
            raise errno_to_exception(libdhd.errorGetLast())

    def update_torque(self):
        """
        Performs a blocking read to the HapticDevice, requesting the current
        torque the end end-effector is experiencing and updates the last-known
        torque with the response.

        :rtype: None
        """
        _, err = libdhd.getForce(ID=self._id, out=self._t)

        if (err):
            raise errno_to_exception(libdhd.errorGetLast())

    def update_force_and_torque(self):
        """
        Performs a blocking read to the HapticDevice, requesting in parallel
        the current force and torque the end end-effector is experiencing and
        updates the last-known force and torque with the response.

        :rtype: None
        """
        _, _, err = libdhd.getForceAndTorque(
            ID=self._id,
            f_out=self._f,
            t_out=self._t
        )

        if (err):
            raise errno_to_exception(libdhd.errorGetLast())

    def update_force_and_torque_and_gripper_force(self):
        if self.gripper is not None:
            _, _, fg, err = libdhd.getForceAndTorqueAndGripperForce(
                ID=self._id,
                f_out=self._f,
                t_out=self._t
            )
            if (err):
                raise errno_to_exception(err)
            self.gripper._fg = fg

    def update_buttons(self):
        self._buttons = libdhd.getButtonMask()

        err = libdhd.errorGetLast()
        if err:
            raise errno_to_exception(libdhd.ErrorNum(err))

    def submit_ft(self):
        libdhd.setForceAndTorque(self._f_req, self._t_req, self._id)

    def set(self, f: CartesianTuple, t: CartesianTuple):
        self._f_req[0:2] = f
        self._t_req[0:2] = t

    def submit_enc(self):
        _, err = libdhd.expert.getEnc(
            ID=self._id,
            mask=self._enc_req,
            out=self._enc
        )

    def request_enc(self, enc_mask: int = 0x07):  # 0...111
        self._enc_req &= enc_mask

    def enable_force(self, enabled: bool = True):
        libdhd.enableForce(enabled, ID=self._id)

    def enable_gravity_compensation(self, enabled: bool = False):
        libdhd.setGravityCompensation(enabled, ID=self._id)

    def neutral(self):
        self._f_req[0:2] = [0.0, 0.0, 0.0]
        self._t_req[0:2] = [0.0, 0.0, 0.0]

    def brake(self):
        libdhd.stop(self._id)

    def get_button(self, button_id: int = 0) -> bool:
        return bool(self._buttons & self._id)

    def __enter__(self):
        VecGen = self._vecgen

        self._enc = [0, 0, 0]
        self._joint_angles = [0, 0, 0]
        self._pos = VecGen()
        self._w = VecGen()
        self._v = VecGen()
        self._f = VecGen()
        self._t = VecGen()

        self._pos_view = ImmutableWrapper(data=self._pos)
        self._w_view = ImmutableWrapper(data=self._w)
        self._v_view = ImmutableWrapper(data=self._v)
        self._f_view = ImmutableWrapper(data=self._f)
        self._t_view = ImmutableWrapper(data=self._t)
        self._J_view = ImmutableWrapper(data=self._J)

        if (len(self._pos) < 3):
            raise ValueError("vecgen did not create a container with at least "
                             "length 3.")

        self._f_req[0:2] = [0.0, 0.0, 0.0]
        self._t_req[0:2] = [0.0, 0.0, 0.0]

        if (libdhd.getDeviceCount() > 0):
            if self._id is None:
                if (self.devtype is None):
                    self._id = libdhd.open()
                    self.devtype = libdhd.getSystemType()
                else:
                    self._id = libdhd.openType(self.devtype)

                if (self.devtype == -1):
                    raise errno_to_exception(libdhd.errorGetLast())(
                        ID=self._id,
                        op=libdhd.getSystemType
                    )
            else:
                libdhd.openID(self._id)
                if (self._id == -1):
                    raise errno_to_exception(libdhd.errorGetLast())

                devtype = libdhd.getSystemType()

                if (devtype == -1):
                    raise errno_to_exception(libdhd.errorGetLast())(
                        ID=self._id,
                        op=libdhd.getSystemType
                    )

                if (self.devtype != devtype):
                    raise Exception(
                        "Device is not of type {}".format(self.devtype))

            self._left_handed = libdhd.isLeftHanded(self._id)
            if (devtype == -1):
                raise errno_to_exception(libdhd.errorGetLast())(
                    ID=self._id,
                    op=libdhd.isLeftHanded
                )

            if (libdhd.hasGripper(self._id)):
                self.gripper = Gripper(
                    self._f_req,
                    self._t_req,
                    self._id,
                    self._vecgen
                )

            self.update()
        else:
            raise DHDErrorNoDeviceFound()

        return self

    def __exit__(self):
        self.neutral()
        self._active_close_req = True

        while self._active_close_req:
            pass

        libdhd.close(self._id)


class HapticDeviceDaemon(Thread):
    def __init__(
                self,
                dev: HapticDevice,
                update_list: Optional[UpdateTuple] = UpdateTuple(),
                gripper_update_list: Optional[GripperUpdateTuple] = None,
                max_freq: Optional[float] = 4000
            ):

        if not isinstance(dev, HapticDevice):
            raise TypeError("Poller acts on an instance of HapticDevice")

        self._dev = dev
        self.daemon = True

        self._set_funcs(update_list, gripper_update_list)
        self._num_updates = len(self._funcs)

        if self._num_updates == 0:
            raise ValueError("There must be at least one parameter to update.")

        if (max_freq is not None):
            self._min_period = 1 / max_freq

    def _set_funcs(self, update_list, gripper_update_list):
        funcs = []

        if update_list is not None:
            if update_list.enc:
                funcs.append(self._dev.update_enc_and_calculate)

            if update_list.v:
                funcs.append(self._dev.update_velocity)

            if update_list.w:
                funcs.append(self._dev.update_angular_velocity)

            if update_list.f and update_list.t:
                if self.gripper_update_list:
                    funcs.append(self._dev.update_force_and_torque)
                else:
                    funcs.append(
                        self._dev.update_force_and_torque_and_gripper_force
                    )
            elif update_list.f and not update_list.t:
                funcs.append(self._dev.update_force)
            elif not update_list.f and update_list.t:
                funcs.append(self._dev.update_torque)

        if gripper_update_list is not None:
            if gripper_update_list.v:
                funcs.append(self._dev.gripper.update_linear_velocity)

            if gripper_update_list.w:
                funcs.append(self._dev.gripper.update_angular_velocity)

            if gripper_update_list.gap:
                funcs.append(self._dev.gripper.update_gap)

            if gripper_update_list.finger_pos:
                funcs.append(self._dev.gripper.update_finger_pos)

            if gripper_update_list.thumb_pos:
                funcs.append(self._dev.gripper.update_thumb_pos)

            funcs.append(self._dev.gripper.submit)
        else:
            funcs.append(self._dev.submit)

        self._funcs = funcs

    def sync(self):
        self.t = monotonic()
        while True:
            while (monotonic() - self.t < self.min_period):
                pass

            self.t = monotonic()

    def run(self):
        try:
            if self._num_updates == 1:
                func = self.funcs[0]
                if self.min_period is not None:
                    while True:
                        self.sync()
                        func()
                else:
                    while True:
                        func()
            else:
                num = self._num_updates

                # poll each requested parameter in its own thread
                with ThreadPoolExecutor(max_workers=num) as executor:
                    if self._min_period is not None:
                        num_done = 0
                        futures = []

                        for func in self._funcs:
                            futures.append(executor.submit(func))

                        # max frequency one needs to synchronize
                        # by waiting for each one to finish and then
                        # synchronizing with the max frequency.
                        while True:
                            for future in futures:
                                if future.done():
                                    num_done += 1

                            if num_done == self._num_updates:
                                futures.clear()
                                self.sync()
                                for func in self._funcs:
                                    futures.append(executor.submit(func))
                    else:
                        future_map = {}

                        for func in self._funcs:
                            future_map[func] = executor.submit(func)

                        while True:
                            for func in future_map:
                                if future_map[func].done():
                                    func = future_map[func]
                                    future_map[func] = executor.submit(func)

        except DHDIOError as ex:
            self._dev.thread_exception = ex
