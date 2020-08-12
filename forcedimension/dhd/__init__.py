"""
.. module::dhd
   :platform: Windows, Unix
   :synopsis: libdhd Python bindings

.. moduleauthor:: Drason Chow <drasonchow@gmail.com>
"""

__all__ = ['bindings']

from collections import UserList
from typing import List, Optional
from abc import ABC, abstractmethod

import forcedimension.dhd.bindings as libdhd
from forcedimension.dhd.bindings import DeviceType


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
    import numpy as np

    return (np.zeros(3))


class Gripper:
    def __init__(self, ID: Optional[int] = None):
        if ID is not None:
            if (ID < 0):
                raise ValueError("ID must be greater than 0.")

            self.id = ID

    def update(self):
        pass


class EndEffector:
    def __init__(self, ID: Optional[int] = None):
        if ID is not None:
            if (ID < 0):
                raise ValueError("ID must be greater than 0.")

            self.id = ID


class HapticDevice:
    def __init__(
            self,
            ID: Optional[int] = None,
            devtype: Optional[DeviceType] = None,
            vecgen=EuclideanVector):
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

        self._pos = None
        self._w = None
        self._v = None
        self._f = None
        self._t = None

        self.gripper = None

        self.devtype = devtype

    @property
    def ID(self):
        return self._id

    @property
    def pos(self):
        return self._pos

    @property
    def v(self):
        return self._v

    @property
    def w(self):
        return self._w

    @property
    def t(self):
        return self._t

    @property
    def f(self):
        return self._f

    def get_status(self):
        return libdhd.getStatus(ID=self._id)

    def update_position(self):
        libdhd.getPosition(ID=self._id, out=self.pos)

    def update_velocity(self):
        libdhd.getLinearVelocity(ID=self._id, out=self.v)

    def update_angular_velocity(self):
        libdhd.getAngularVelocityRad(ID=self._id, out=self.v)

    def update_force(self):
        libdhd.getForce(ID=self._id, out=self._f)

    def update_torque(self):
        libdhd.getForce(ID=self._id, out=self._t)

    def update_force_and_torque(self):
        libdhd.getForceAndTorque(ID=self._id, f_out=self._f, t_out=self._t)

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
                self._id = libdhd.openType(self.devtype)
                self.devtype = libdhd.getSystemType()

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


