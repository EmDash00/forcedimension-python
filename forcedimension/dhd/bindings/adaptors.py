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
