"""
.. module::dhd
   :platform: Windows, Unix
   :synopsis: libdhd Python bindings

.. moduleauthor:: Drason Chow <drasonchow@gmail.com>
"""

__all__ = ['bindings']

from threading import Thread
from collections import UserList
from typing import List, Iterable, Optional

import forcedimension.dhd.bindings as libdhd
from forcedimension.dhd.bindings import DeviceType
from forcedimension.dhd.bindings.adaptors import StatusTuple


class EuclideanVector(UserList):
    def __init__(self, data: List[float] = [0, 0, 0]):
        if (len(data) != 3):
            raise ValueError("EuclideanVector must take in a list of 3 floats")

        self.data = data

    @property
    def x(self):
        return self.data[0]

    @x.setter
    def x(self, val):
        self.data[0] = val

    @property
    def y(self):
        return self.data[1]

    @y.setter
    def y(self, val):
        self.data[1] = val

    @property
    def z(self):
        return self.data[2]

    @z.setter
    def z(self, val):
        self.data[2] = val


def NumpyVector():
    import numpy as np  # type: ignore

    return (np.zeros(3))


class Gripper:
    def __init__(self, ID: Optional[int] = None):
        if ID is not None:
            if (ID < 0):
                raise ValueError("ID must be greater than 0.")

            self.id = ID

    def update(self):
        pass


class HapticDevice:
    def __init__(
            self,
            ID: Optional[int] = None,
            devtype: Optional[DeviceType] = None,
            vecgen=EuclideanVector,
            safe=True):
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

        self._pos: Optional[Iterable[float]] = None
        self._w: Optional[Iterable[float]] = None
        self._v: Optional[Iterable[float]] = None
        self._f: Optional[Iterable[float]] = None
        self._t: Optional[Iterable[float]] = None

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
            raise RuntimeError(libdhd.errorGetLastStr())

        return status

    def update_position(self) -> None:
        if (libdhd.getPosition(ID=self._id, out=self._pos)):
            raise RuntimeError(libdhd.errorGetLastStr())

    def update_velocity(self):
        if (libdhd.getLinearVelocity(ID=self._id, out=self._v)):
            raise RuntimeError(libdhd.errorGetLastStr())

    def update_angular_velocity(self):
        if (libdhd.getAngularVelocityRad(ID=self._id, out=self._w)):
            raise RuntimeError(libdhd.errorGetLastStr())

    def update_force(self):
        if (libdhd.getForce(ID=self._id, out=self._f)):
            raise RuntimeError(libdhd.errorGetLastStr())

    def update_torque(self):
        if(libdhd.getForce(ID=self._id, out=self._t)):
            raise RuntimeError(libdhd.errorGetLastStr())

    def update_force_and_torque(self):
        if (libdhd.getForceAndTorque(ID=self._id,
                                     f_out=self._f, t_out=self._t)):

            raise RuntimeError(libdhd.errorGetLastStr())

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
            raise ValueError("vecgen did not create a container with at least"
                             " length 3.")

        if (libdhd.getDeviceCount() > 0):
            if self._id is None:
                if (self.devtype is None):
                    self._id = libdhd.open()
                    self.devtype = libdhd.getSystemType()
                else:
                    self._id = libdhd.openType(self.devtype)

                if (self.devtype == -1):
                    raise RuntimeError(libdhd.errorGetLastStr())

            else:
                libdhd.openID(self._id)
                if (self._id == -1):
                    raise IOError(libdhd.errorGetLastStr())

                devtype = libdhd.getSystemType()

                if (devtype == -1):
                    raise RuntimeError(libdhd.errorGetLastStr())

                if (self.devtype != devtype):
                    raise RuntimeError(
                        "Device is not of type {}".format(self.devtype))

            if (libdhd.hasGripper(self._id)):
                self.gripper = Gripper(self._id, self._vecgen)

            self.update()
        else:
            raise RuntimeError("There are no devices connected to the system.")

        return self

    def __exit__(self):
        libdhd.close(self._id)


class Poller(Thread):
    def __init__(self, dev: HapticDevice, update_list=None):
        self._dev = dev
        self.daemon = True

    def run(self):
        try:
            self._dev.update()
        except RuntimeError as ex:
            self._dev.thread_exception = ex
