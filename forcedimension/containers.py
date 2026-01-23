from __future__ import annotations

import ctypes
import functools
import operator
from collections.abc import Callable
from typing import Any, NamedTuple
from typing import cast as _cast

import numpy as np
import numpy.typing as npt
import pydantic as pyd
import pydantic_core as pyd_core
import typing_extensions
from forcedimension_core.constants import MAX_DOF
from forcedimension_core.containers import Status
from forcedimension_core.containers.numpy import (
    DOFFloat,
    DOFInt,
    Mat3x3,
    Mat6x6,
    Vec3,
)
from forcedimension_core.typing import c_double_ptr

from forcedimension.util import ImmutableWrapper

__all__ = ["DOFInt", "DOFFloat", "Mat3x3", "Mat6x6", "Status"]


@typing_extensions.deprecated(
    "GripperUpdateOpts is deprecated and will be removed in v0.3.0"
)
class GripperUpdateOpts(NamedTuple):
    """Encodes the update rates for various gripperproperties with HapticDaemon."""

    enc: float = 1000
    thumb_pos: float = 1000
    finger_pos: float = 1000
    v: float = 4000
    w: float = 4000


@typing_extensions.deprecated("UpdateOpts is deprecated and will be removed in v0.3.0")
class UpdateOpts(NamedTuple):
    """Encodes the update rates for various device properties with HapticDaemon."""

    enc: float | None = 1000
    v: float | None = 4000
    w: float | None = None
    buttons: float | None = 100
    ft: float | None = 4000
    req: float | None = 4000
    gripper: GripperUpdateOpts | None = None


class PollGroup(NamedTuple):
    """Encodes a group of functions that should be polled together.

    :param list[Callable[[], Any]] targets: List of functions to be polled

    :param Callable[[], Any] | float wait_for:
        Either a function to wait for completion or a timeout in seconds.

    :param bool high_precision:
        Whether to use high-precision timing. High precision timing uses
        busy-waiting that does not release the thread for the last millisecond
        of the wait. Note that this is more CPU intensive.

    :param str | None name: Optional name for the poll group.
    """

    targets: list[Callable[[], Any]]
    wait_for: Callable[[], Any] | float
    high_precision: bool = True
    name: str | None = None


class State3(np.ndarray):
    """Represents position and velocity in 3D Cartesian space as a (3, 2)
    ndarray as: [[x, y, z], [vx, vy, vz]].

    This class provides a typed view over a numpy array for operations
    involving 3D Cartesian states in physics simulations, orbital mechanics,
    or robotics.

    Examples:
        >>> data = np.array([1.0, 2.0, 3.0, 0.1, 0.2, 0.3])
        >>> state = State3(data)
        >>> state.pos  # array([1., 2., 3.])
        >>> state.v  # array([0.1, 0.2, 0.3])
        >>> state.x  # array([1, 0.1])
        >>> state.y  # array([2, 0.2])
        >>> state.z  # array([3, 0.3])

    """

    def __new__(cls, data: npt.ArrayLike = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)):
        """Create a new State3.

        :param numpy.typing.ArrayLike data:
            An array of (x, y, z, vx, vy, vy) that represents
            the initial value of the array.
        """
        arr = np.asarray(data, dtype=ctypes.c_double)

        if functools.reduce(operator.mul, arr.shape) != 6:
            raise ValueError

        return arr.reshape(2, 3).view(cls)

    def __init__(self, *args, **kwargs):
        self._pos_view = ImmutableWrapper(self[0])
        self._v_view = ImmutableWrapper(self[1])

        self._x_view = ImmutableWrapper(self[:, 0])
        self._y_view = ImmutableWrapper(self[:, 1])
        self._z_view = ImmutableWrapper(self[:, 2])

    @classmethod
    def __get_pydantic_core_schema__(
        cls, source_type: Any, handler: pyd.GetCoreSchemaHandler
    ) -> pyd_core.CoreSchema:
        return pyd_core.core_schema.no_info_after_validator_function(
            cls, handler(cls.__init__)
        )

    @property
    def pos(self) -> Vec3:
        """A read-only view on the position part of the state."""
        return _cast(Vec3, self._pos_view)

    @property
    def v(self) -> Vec3:
        """A read-only view on the velocity part of the state."""
        return _cast(Vec3, self._v_view)

    @property
    def x(self) -> np.ndarray:
        """A read-only view on the x part of the state."""
        return _cast(np.ndarray, self._x_view)

    @property
    def y(self) -> np.ndarray:
        """A read-only view on the y part of the state."""
        return _cast(np.ndarray, self._y_view)

    @property
    def z(self) -> np.ndarray:
        """A read-only view on the z part of the state."""
        return _cast(np.ndarray, self._z_view)


class DOFFloatState(np.ndarray):
    """Represents a state vector for each degree of freedom for the device
    as a numpy array view over an array of shape (2, MAX_DOF):
    [[dof_0, ...,  dof_n], [velocity_dof_0, ..., velocity_dof_n]].

    Each dof's position is encoded as an actuation angle in [rad] and each
    dof's velocity is encoded as an actuation angular velocity in [rad/s]

    DOF Mapping (assuming MAX_DOF = 8):
        Index 0-2: DELTA structure (end-effector position)
        Index 3-5: WRIST structure (end-effector orientation)
        Index 6:   Reserved/unused
        Index 7:   Gripper

    Example:
        >>> state = DOFFloatState()  # Zero state
        >>> state.delta    # DELTA angles/velocities
        >>> state.wrist    # WRIST angles/velocities
        >>> state.gripper  # Gripper angle/veloci
    """

    def __new__(cls, data: npt.ArrayLike = tuple(0.0 for _ in range(2 * MAX_DOF))):
        """Create a new DOFFloatState instance.

        :param ArrayLike data:
           Array-like input that can be converted to shape (2, MAX_DOF).
           Defaults to zeros.

        :raises ValueError: If total elements in data != 2 * MAX_DOF
        """
        arr = np.ascontiguousarray(data, dtype=ctypes.c_double)

        if functools.reduce(operator.mul, arr.shape) != MAX_DOF * 2:
            raise ValueError

        return arr.reshape(2, MAX_DOF).view(cls)

    def __init__(self, *args, **kwargs):
        self._delta = State3(self[:, :3])
        self._wrist = State3(self[:, 3:6])
        self._gripper_view = ImmutableWrapper(self[:, 7])

        self._gripper_pos = ctypes.cast(
            self.ctypes.data + (MAX_DOF - 1) * self.itemsize, c_double_ptr
        ).contents

        self._gripper_v = ctypes.cast(
            self.ctypes.data + (2 * MAX_DOF - 1) * self.itemsize, c_double_ptr
        ).contents

        self._delta_view = ImmutableWrapper(self._delta)
        self._wrist_view = ImmutableWrapper(self._wrist)

    @classmethod
    def __get_pydantic_core_schema__(
        cls, source_type: Any, handler: pyd.GetCoreSchemaHandler
    ) -> pyd_core.CoreSchema:
        return pyd_core.core_schema.no_info_after_validator_function(
            cls,
            handler(cls.__init__),
            serialization=pyd_core.core_schema.plain_serializer_function_ser_schema(
                lambda arr: arr.tolist()
            ),
        )

    @property
    def delta(self) -> np.ndarray:
        """Read-only view of DELTA structure as a (2, 3) ndarray.

        delta[0, :] = [delta_dof0, delta_dof1, delta_dof2] angles in [rad]
        delta[1, :] = [delta_dof0, delta_dof1, delta_dof2] angular velocities in [rad/s].

        Info
        ----
        The DELTA structure is the part of the device that controls the
        end-effector's position.
        """
        return _cast(np.ndarray, self._delta_view)

    @property
    def wrist(self) -> np.ndarray:
        """Read-only view of WRIST structure state as (2, 3) ndarray.

        wrist[0, :] = [wrist_dof0, wrist_dof1, wrist_dof2] angles in [rad]
        wrist[1, :] = [wrist_dof0, wrist_dof1, wrist_dof2] angular velocities in [rad/s].

        Info
        ----
        The WRIST structure is the part of the device that controls the
        end-effector's orientation.
        """
        return _cast(np.ndarray, self._wrist_view)

    @property
    def gripper(self) -> np.ndarray:
        """A read only view of the gripper's state as a (2,) narray.

        griper[0] = gripper_angle [rad]
        griper[1] = gripper_angluar_velocity [rad/s]

        """
        return self[:, 7].squeeze()
