from __future__ import annotations

import ctypes
import functools
import operator
from typing import Any, Callable, List, NamedTuple, Optional, Union
from typing import cast as _cast

import numpy as np
import numpy.typing as npt
import pydantic as pyd
import pydantic_core as pyd_core
from forcedimension_core.constants import MAX_DOF
from forcedimension_core.containers import Status
from forcedimension_core.containers.numpy import (
    DOFFloat, DOFInt, DOFMotor, Enc3, Enc4, Mat3x3, Mat6x6, Mot3, Vec3
)
from forcedimension_core.typing import c_double_ptr

from forcedimension.util import ImmutableWrapper


class GripperUpdateOpts(NamedTuple):
    """
    Deprecated. Will be removed in v1.0.0
    """
    enc: float = 1000
    thumb_pos: float = 1000
    finger_pos: float = 1000
    v: float = 4000
    w: float = 4000


class UpdateOpts(NamedTuple):
    """
    Deprecated. Will be removed in v1.0.0
    """
    enc: Optional[float] = 1000
    v: Optional[float] = 4000
    w: Optional[float] = None
    buttons: Optional[float] = 100
    ft: Optional[float] = 4000
    req: Optional[float] = 4000
    gripper: Optional[GripperUpdateOpts] = None


class PollGroup(NamedTuple):
    targets: List[Callable[[], Any]]
    wait_for: Union[Callable[[], Any], float]
    high_precision: bool = True
    name: Optional[str] = None


class State3(np.ndarray):
    """
    Represents a vector in Cartesian coordinates as a view over a
    `numpy.ndarray`. The "x" , "y", and "z" properties
    corresponding to the 0th, 1st, and 2nd elements, respectively.
    """

    def __new__(cls, data: npt.ArrayLike = (0., 0., 0., 0., 0., 0.)):
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
        return _cast(Vec3, self._pos_view)

    @property
    def v(self) -> Vec3:
        return _cast(Vec3, self._v_view)

    @property
    def x(self) -> np.ndarray:
        return _cast(np.ndarray, self._x_view)

    @property
    def y(self) -> np.ndarray:
        return _cast(np.ndarray, self._y_view)

    @property
    def z(self) -> np.ndarray:
        return _cast(np.ndarray, self._z_view)


class DOFFloatState(np.ndarray):
    def __new__(
        cls, data: npt.ArrayLike = tuple(0. for _ in range(2 * MAX_DOF))
    ):
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
            )
        )

    @property
    def delta(self) -> np.ndarray:
        return _cast(np.ndarray, self._delta_view)

    @property
    def wrist(self) -> np.ndarray:
        return _cast(np.ndarray, self._wrist_view)

    @property
    def gripper(self) -> np.ndarray:
        return self[:, 7]
