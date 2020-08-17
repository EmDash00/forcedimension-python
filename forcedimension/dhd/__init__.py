"""
.. module::dhd
   :platform: Windows, Unix
   :synopsis: libdhd Python bindings

.. moduleauthor:: Drason Chow <drasonchow@gmail.com>
"""

__all__ = ['bindings']

from threading import Thread
from concurrent.futures import ThreadPoolExecutor

from typing import List, MutableSequence, Optional, Callable
from typing import NamedTuple

from time import monotonic

from forcedimension.dhd.bindings.adaptors import CartesianTuple
import forcedimension.dhd.bindings as libdhd
import forcedimension.dhd.bindings.expert  # NOQA

from forcedimension.dhd.bindings import DeviceType
from forcedimension.dhd.bindings.adaptors import (
    StatusTuple,
    DHDIOError,
    DHDErrorNoDeviceFound,
    errno_to_exception
)

libdhd.expert.enableExpertMode()


class Euclidian(type):
    """
    Euclidian metaclass for MutableSequence types. Using this metaclass will
    automatically create convience 3 read-write properties "x", "y", and "z",
    which correspond to the 0th, 1st, and 2nd elements of an object of the
    MutableSequence class.
    """
    def __new__(cls, name, bases, dct):

        x = super().__new__(cls, name, bases, dct)

        def mutable_accessor(i):
            def getter(self):
                return self.__getitem__(i)

            def setter(self, value):
                self.__setitem__(i, value)

            return property(getter, setter)

        for i, prop in enumerate(['x', 'y', 'z']):
            setattr(x, prop, mutable_accessor(i))

        return x


class EuclidianVector(List[float], metaclass=Euclidian):
    """
    A List[float] providing convience "x", "y", "z" read-write accessor
    properties. This class subclasses List[float]; therefore, for all intents
    and purposes, you can treat it as a list. This allows for it to be
    compatible with Python's standard library functions.
    """
    def __init__(self, data: List[float] = [0, 0, 0]):
        if (len(data) != 3):
            raise ValueError

        super().__init__(data)


try:
    from numpy import ndarray, array  # type: ignore

    class NumpyEuclidianView(ndarray, metaclass=Euclidian):
        """
        A view over a numpy ndarry, which provides convience "x", "y", and "z"
        read-write accessor properties. This class subclasses ndarray;
        therefore, for all intents and purposes you can treat it as an ndarray.
        This allows you to simply pass in this class to any and all numpy
        methods.
        """
        def __new__(cls, data: MutableSequence[float]):
            if len(data) != 3:
                raise ValueError

            return array(data, dtype=float).view(cls)
except ImportError:
    pass


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
            ID: Optional[int] = None,
            vecgen: Callable[[], MutableSequence[float]] = EuclidianVector
    ):

        if ID is not None:
            if (ID < 0):
                raise ValueError("ID must be greater than 0.")

            self._id = ID

        self.VecType = vecgen
        VecType = vecgen

        self._angle: MutableSequence[float] = VecType()
        self._gap: MutableSequence[float] = VecType()

        self._thumb_pos: Optional[MutableSequence[float]] = VecType()
        self._finger_pos: Optional[MutableSequence[float]] = VecType()

        self._v: Optional[MutableSequence[float]] = VecType()
        self._w: Optional[MutableSequence[float]] = VecType()

        self._parent_f_req = parent_f_req
        self._parent_t_req = parent_t_req
        self._fg_req: float = 0.0

    def set(self, fg: float):
        self._fg_req = fg

    def submit(self):
        if self.gripper is not None:
            libdhd.setForceAndTorqueAndGripperForce(
                f=self._parent_f_req,
                t=self._parent_t_req,
                fg=self
            )
        else:
            libdhd.setForceAndTorque(
                f=self._parent_f_req,
                t=self._parent_t_req,
            )

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
            vecgen: Callable[[], MutableSequence[float]] = EuclidianVector
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
        self._active_close_req: bool = False

        self._pos: Optional[MutableSequence[float]] = None
        self._w: Optional[MutableSequence[float]] = None
        self._v: Optional[MutableSequence[float]] = None
        self._f: Optional[MutableSequence[float]] = None
        self._t: Optional[MutableSequence[float]] = None

        self._f_req = [0.0, 0.0, 0.0]
        self._t_req = [0.0, 0.0, 0.0]

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
    def pos(self) -> Optional[MutableSequence[float]]:
        """
        Provides a read-only accessor to the last-known position of the
        HapticDevice's end effector. Thread-safe.

        :rtype: Optional[MutableSequence[float]]

        :returns: A mutable sequence of [x, y, z] where x, y, and z are the
        end-effector's position given in [m].
        """
        self.check_threadex()
        return self._pos

    @property
    def v(self) -> Optional[MutableSequence[float]]:
        """
        Provides a read-only accessor to the last-known linear velocity of the
        HapticDevice's end-effector. Thread-safe.

        :rtype: Optional[MutableSequence[float]]

        :returns: A mutable sequence of [vx, vy, vz] where vx, vy, and vz are
        the end-effector's linear velocity given in [m/s].
        """
        self.check_threadex()
        return self._v

    @property
    def w(self) -> Optional[MutableSequence[float]]:
        """
        Provides a read-only accessor to the last-known angular velocity of the
        HapticDevice's end-effector. Thread-safe.

        :rtype: Optional[MutableSequence[float]]

        :returns: A mutable sequence of [wx, wy, wz] where wx, wy, and wz are
        the end-effector's linear velocity given in [rad/s].
        """
        self.check_threadex()
        return self._w

    @property
    def t(self) -> Optional[MutableSequence[float]]:
        """
        Provides a read-only accessor to the last-known torque experienced by
        the HapticDevice's end-effector. Thread-safe.

        :rtype: Optional[MutableSequence[float]]

        :returns: A mutable sequence of [tx, ty, tz] where tx, ty, and tz are
        the torque experienced by the end-effector in [Nm]
        """
        self.check_threadex()
        return self._t

    @property
    def f(self) -> Optional[MutableSequence[float]]:
        """
        Provides a read-only accessor to the current linear force experienced
        by the HapticDevice's end-effector.

        :rtype: Optional[MutableSequence[float]]

        :returns: A mutable sequence of [fx, fy, fz] where fx, fy, and fz are
        the torque experienced by the end-effector in [N]
        """

        self.check_threadex()
        return self._f

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
            raise errno_to_exception(libdhd.errorGetLast())

    def update_angular_velocity(self):
        """
        Performs a blocking read to the HapticDevice, requesting the current
        angular velocity of the end-effector and updates the last-known
        angular velocity with the response.

        :rtype: None
        """
        _, err = libdhd.getAngularVelocityRad(ID=self._id, out=self._w)

        if (err):
            raise errno_to_exception(libdhd.errorGetLast())

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

    def submit(self):
        libdhd.setForceAndTorque(self._f_req, self._t_req, self._id)

    def set(self, f: CartesianTuple, t: CartesianTuple):
        self._f_req[0:2] = f
        self._t_req[0:2] = t

    def neutral(self):
        libdhd.setForceAndTorque(
            f=CartesianTuple(x=0, y=0, z=0),
            t=CartesianTuple(x=0, y=0, z=0),
            ID=self._id
        )

    def brake(self):
        libdhd.stop(self._id)

    def __enter__(self):
        VecGen = self._vecgen

        self._pos = VecGen()
        self._w = VecGen()
        self._v = VecGen()
        self._f = VecGen()
        self._t = VecGen()

        if (len(self._pos) < 3):
            raise ValueError("vecgen did not create a container with at least "
                             "length 3.")

        if (libdhd.getDeviceCount() > 0):
            if self._id is None:
                if (self.devtype is None):
                    self._id = libdhd.open()
                    self.devtype = libdhd.getSystemType()
                else:
                    self._id = libdhd.openType(self.devtype)

                if (self.devtype == -1):
                    raise errno_to_exception(libdhd.errorGetLast())

            else:
                libdhd.openID(self._id)
                if (self._id == -1):
                    raise errno_to_exception(libdhd.errorGetLast())

                devtype = libdhd.getSystemType()

                if (devtype == -1):
                    raise errno_to_exception(libdhd.errorGetLast())

                if (self.devtype != devtype):
                    raise Exception(
                        "Device is not of type {}".format(self.devtype))

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


class GripperUpdateTuple(NamedTuple):
    gap: bool = True
    thumb_pos: bool = True
    finger_pos: bool = True
    v: bool = True
    w: bool = True


class UpdateTuple(NamedTuple):
    pos: bool = True
    v: bool = True
    w: bool = True
    f: bool = True
    t: bool = True


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
            if update_list.pos:
                funcs.append(self._dev.update_position)

            if update_list.v:
                funcs.append(self._dev.update_velocity)

            if update_list.w:
                funcs.append(self._dev.update_angular_velocity)

            if update_list.f and update_list.t:
                funcs.append(self._dev.update_force_and_torque)
            elif update_list.f and not update_list.t:
                funcs.append(self._dev.update_force)
            elif not update_list.f and update_list.t:
                funcs.append(self._dev.update_torque)

            funcs.append(self._dev.submit)

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
