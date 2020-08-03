"""
.. module::adaptors
   :platform: Windows, Unix
   :synopsis: pythonic adaptors for libdhd args and returns.

.. moduleauthor:: Drason Chow <drasonchow@gmail.com>
"""

from typing import NamedTuple


class VersionTuple(NamedTuple):
    """
    Adapts the four seperate number return into a single grouped
    NamedTuple.
    """
    major: int
    minor: int
    release: int
    revision: int


class StatusTuple(NamedTuple):
    """
    Named tuple adapting the status array returned by
    forcedimension.bindings.dhd.getStatus()
    """
    power: int
    connected: int
    started: int
    reset: int
    idle: int
    force: int
    brake: int
    torque: int
    wrist_detected: int
    error: int
    gravity: int
    timeguard: int
    wrist_init: int
    redundancy: int
    forceoffcause: int


class CartesianTuple(NamedTuple):
    """
    Named tuple adapting an immutable triple of (x, y, z) floats.

    Used to meaningfully talk about 3-Vectors.
    """
    x: float
    y: float
    z: float


class DeviceTuple(NamedTuple):
    """
    Named tuple adapting an immutable triple of (axis0, axis1, axis2) ints.

    Used to meaningfully talk about device motors or encoders as a group.
    """
    axis0: int
    axis1: int
    axis2: int


class DOFTuple(NamedTuple):
    """
    Named tuple adapting a way to specify an int assignment to the device
    associated with that degree-of-freedom up to but not including
    dhd.bindings.MAX_DOF
    """
    dof0: int
    dof1: int
    dof2: int
    dof3: int
    dof4: int
    dof5: int
    dof6: int
    dof7: int
