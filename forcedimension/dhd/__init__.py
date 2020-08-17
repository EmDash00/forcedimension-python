"""
.. module::dhd
   :platform: Windows, Unix
   :synopsis: libdhd Python bindings

.. moduleauthor:: Drason Chow <drasonchow@gmail.com>
"""

__all__ = ['bindings']

from threading import Thread
from typing import List, MutableSequence, Optional, Callable

from time import monotonic

import forcedimension.dhd.bindings as libdhd
from forcedimension.dhd.bindings import DeviceType
from forcedimension.dhd.bindings.adaptors import (
    StatusTuple,
    DHDErrorNoDeviceFound,
    errno_to_exception
)

libdhd.expert.enableExpertMode()


class Euclidian(type):
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
    def __init__(self, data: List[float] = [0, 0, 0]):
        if (len(data) != 3):
            raise ValueError

        super().__init__(data)


try:
    from numpy import ndarray, array  # type: ignore

    class NumpyEuclidianView(ndarray, metaclass=Euclidian):
        def __new__(cls, data: MutableSequence[float]):
            if len(data) != 3:
                raise ValueError

            return array(data, dtype=float).view(cls)
except ImportError:
    pass


class Gripper:
    def __init__(
            self,
            ID: Optional[int] = None,
            vecgen: Callable[[], MutableSequence[float]] = EuclidianVector
    ):

        if ID is not None:
            if (ID < 0):
                raise ValueError("ID must be greater than 0.")

            self._id = ID

        self.VecType = vecgen
        VecType = vecgen

        self.angle: MutableSequence[float] = VecType()
        self.gap: Optional[MutableSequence[float]] = VecType()

        self.thumb_pos: Optional[MutableSequence[float]] = VecType()
        self.finger_pos: Optional[MutableSequence[float]] = VecType()

        self.v: Optional[MutableSequence[float]] = VecType()
        self.w: Optional[MutableSequence[float]] = VecType()

    def update_linear_velocity(self):
        libdhd.getAngularVelocityRad(self._id, out=self.v)

    def update_angular_velocity(self):
        libdhd.getAngularVelocityRad(self._id, out=self.w)

    def update_angle(self):
        libdhd.getGripperAngleRad(self._id, out=self.angle)

    def update_gap(self):
        libdhd.getGripperGap(self._id, out=self.gap)

    def update_thumb_pos(self):
        libdhd.getGripperThumbPos(self._id, out=self.thumb_pos)

    def update_finger_pos(self):
        libdhd.getGripperFingerPos(self._id, out=self.finger_pos)

    def update_all(self):
        self.update_angle()
        self.update_gap()

        self.update_thumb_pos()
        self.update_finger_pos()

        self.update_linear_velocity()
        self.update_angular_velocity()


class HapticDevice:
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

        self._pos: Optional[MutableSequence[float]] = None
        self._w: Optional[MutableSequence[float]] = None
        self._v: Optional[MutableSequence[float]] = None
        self._f: Optional[MutableSequence[float]] = None
        self._t: Optional[MutableSequence[float]] = None

        self.gripper = None

        self.devtype = devtype

        self._thread_exception = None

    @property
    def ID(self):
        self.check_threadex()
        return self._id

    @property
    def pos(self):
        self.check_threadex()
        return self._pos

    @property
    def v(self):
        self.check_threadex()
        return self._v

    @property
    def w(self):
        self.check_threadex()
        return self._w

    @property
    def t(self):
        self.check_threadex()
        return self._t

    @property
    def f(self):
        self.check_threadex()
        return self._f

    def get_status(self) -> StatusTuple:
        status, err = libdhd.getStatus(ID=self._id)

        if (err):
            raise errno_to_exception(libdhd.errorGetLast())

        return status

    def update_position(self) -> None:
        if (libdhd.getPosition(ID=self._id, out=self._pos)):
            raise errno_to_exception(libdhd.errorGetLast())

    def update_velocity(self):
        if (libdhd.getLinearVelocity(ID=self._id, out=self._v)):
            raise errno_to_exception(libdhd.errorGetLast())

    def update_angular_velocity(self):
        if (libdhd.getAngularVelocityRad(ID=self._id, out=self._w)):
            raise errno_to_exception(libdhd.errorGetLast())

    def update_force(self):
        if (libdhd.getForce(ID=self._id, out=self._f)):
            raise errno_to_exception(libdhd.errorGetLast())

    def update_torque(self):
        if(libdhd.getForce(ID=self._id, out=self._t)):
            raise errno_to_exception(libdhd.errorGetLast())

    def update_force_and_torque(self):
        if (libdhd.getForceAndTorque(ID=self._id,
                                     f_out=self._f, t_out=self._t)):

            raise errno_to_exception(libdhd.errorGetLast())

    def update_all(self):
        self.update_position()
        self.update_velocity()
        self.update_angular_velocity()
        self.update_force_and_torque()

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
                self.gripper = Gripper(self._id, self._vecgen)

            self.update()
        else:
            raise DHDErrorNoDeviceFound()

        return self

    def __exit__(self):
        libdhd.close(self._id)


class Poller(Thread):

    def __init__(
                self,
                dev: HapticDevice,
                update_list=None,
                max_freq: Optional[float] = 4000
            ):

        self._dev = dev
        self.daemon = True

        if (max_freq is not None):
            self.min_period = 1 / max_freq

    def sync(self):
        self.t = monotonic()
        while True:
            while (monotonic() - self.t < self.min_period):
                pass

            self.t = monotonic()

    def run(self):
        try:

            if self.min_period is not None:
                while True:
                    self.sync()
                    self._dev.update_all()
            else:
                while True:
                    self._dev.update_all()

        except RuntimeError as ex:
            self._dev.thread_exception = ex
