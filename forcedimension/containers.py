from __future__ import annotations

import ctypes
import functools
import operator
from ctypes import c_double, c_int, c_ushort
from typing import Any, Callable, List, NamedTuple, Optional, Tuple, Union
from typing import cast as _cast

import numpy as np
import numpy.typing as npt
import pydantic as pyd
import pydantic_core as pyd_core
from typing_extensions import NotRequired

from forcedimension.dhd.constants import MAX_DOF
from forcedimension.runtime import VersionTuple
from forcedimension.typing import (
    FloatArray, IntArray, SupportsPtr,
    SupportsPtrs3, c_double_ptr, c_int_ptr,
    c_ushort_ptr
)
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


class _HapticPollerOptions(NamedTuple):
    interval: float
    high_precision: bool


class Vector3(
    np.ndarray, SupportsPtrs3[c_double], SupportsPtr[c_double]
):
    """
    Represents a vector in Cartesian coordinates as a view over a
    `numpy.ndarray`. The "x" , "y", and "z" properties
    corresponding to the 0th, 1st, and 2nd elements, respectively.
    """

    def __new__(cls, data: npt.ArrayLike = (0., 0., 0.)):
        arr = np.ascontiguousarray(data, dtype=c_double).view(cls)

        if len(arr) != 3:
            raise ValueError

        return arr

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._ptrs = (
            ctypes.cast(self.ctypes.data, c_double_ptr),
            ctypes.cast(self.ctypes.data + self.itemsize, c_double_ptr),
            ctypes.cast(self.ctypes.data + 2 * self.itemsize, c_double_ptr)
        )

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
    def ptr(self) -> c_double_ptr:
        return self._ptrs[0]

    @property
    def ptrs(self) -> Tuple[c_double_ptr, c_double_ptr, c_double_ptr]:
        return self._ptrs

    @property
    def x(self) -> float:
        return self[0]

    @x.setter
    def x(self, value: float):
        self[0] = value

    @property
    def y(self) -> c_double:
        return self[1]

    @y.setter
    def y(self, value: float):
        self[1] = value

    @property
    def z(self) -> c_double:
        return self[2]

    @z.setter
    def z(self, value: float):
        self[2] = value


class State3(np.ndarray):
    """
    Represents a vector in Cartesian coordinates as a view over a
    `numpy.ndarray`. The "x" , "y", and "z" properties
    corresponding to the 0th, 1st, and 2nd elements, respectively.
    """

    def __new__(cls, data: npt.ArrayLike = (0., 0., 0., 0., 0., 0.)):
        arr = np.asarray(data, dtype=c_double)

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
    def pos(self) -> Vector3:
        return _cast(Vector3, self._pos_view)

    @property
    def v(self) -> Vector3:
        return _cast(Vector3, self._v_view)

    @property
    def x(self) -> np.ndarray:
        return _cast(np.ndarray, self._x_view)

    @property
    def y(self) -> np.ndarray:
        return _cast(np.ndarray, self._y_view)

    @property
    def z(self) -> np.ndarray:
        return _cast(np.ndarray, self._z_view)


class Enc3(np.ndarray, SupportsPtr[c_int], SupportsPtrs3[c_int]):
    """
    Represents a 3 axis array of encoders
    (e.g. the delta encoders or wrist encoders) as a view over a
    `numpy.ndarray`.
    """

    def __new__(cls, data: npt.ArrayLike = (0., 0., 0.)):
        arr = np.ascontiguousarray(data, dtype=c_int).view(cls)

        if len(arr) != 3:
            raise ValueError

        return arr

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._ptrs = (
            ctypes.cast(self.ctypes.data, c_int_ptr),
            ctypes.cast(self.ctypes.data + self.itemsize, c_int_ptr),
            ctypes.cast(self.ctypes.data + 2 * self.itemsize, c_int_ptr)
        )

    @classmethod
    def __get_pydantic_core_schema__(
        cls, source_type: Any, handler: pyd.GetCoreSchemaHandler
    ) -> pyd_core.CoreSchema:
        return pyd_core.core_schema.no_info_after_validator_function(
            cls, handler(cls.__init__),
            serialization=pyd_core.core_schema.plain_serializer_function_ser_schema(
                lambda arr: arr.tolist()
            )
        )

    @property
    def ptr(self) -> c_int_ptr:
        return self._ptrs[0]

    @property
    def ptrs(self) -> Tuple[c_int_ptr, c_int_ptr, c_int_ptr]:
        return self._ptrs


class Enc4(np.ndarray, SupportsPtr[c_int]):
    """
    Represents an array containing 3 wrist encoders and a gripper encoder
    (in that order) as a view over a `numpy.ndarray`.
    """

    def __new__(cls, data: npt.ArrayLike = (0., 0., 0., 0.)):
        arr = np.ascontiguousarray(data, dtype=c_int).view(cls)

        if len(arr) != 4:
            raise ValueError

        return arr

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._ptr = ctypes.cast(self.ctypes.data, c_int_ptr)

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
    def ptr(self) -> c_int_ptr:
        return self._ptr


class DOFIntArray(np.ndarray, SupportsPtr[c_int]):
    """
    Represents an array of encoder values for each degree-of-freedom as a
    view over a `numpy.ndarray`.
    """

    def __new__(cls, data: IntArray = tuple(0 for _ in range(MAX_DOF))):
        arr = np.ascontiguousarray(data, dtype=c_int).view(cls)

        if len(arr) != MAX_DOF:
            raise ValueError()

        return arr

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self._ptr = ctypes.cast(self.ctypes.data, c_int_ptr)
        self._delta = Enc3(self[:3])
        self._wrist = Enc3(self[3:6])
        self._wrist_grip = Enc4(self[3:7])

        self._gripper = ctypes.cast(
            self.ctypes.data + 7 * self.itemsize, c_int_ptr
        ).contents

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
    def ptr(self) -> c_int_ptr:
        return self._ptr

    @property
    def delta(self) -> Enc3:
        return self._delta

    @property
    def wrist(self) -> Enc3:
        return self._wrist

    @property
    def wrist_grip(self) -> Enc4:
        return self._wrist_grip

    @property
    def gripper(self) -> c_int:
        return self._gripper


class DOFMotorArray(np.ndarray, SupportsPtr[c_ushort]):
    """
    Represents an array of motor commands for each degree-of-freedom as
    a view over a `numpy.ndarray`.
    """

    def __new__(cls, data: IntArray = tuple(0 for _ in range(MAX_DOF))):
        arr = np.ascontiguousarray(data, dtype=c_ushort).view(cls)

        if len(arr) != MAX_DOF:
            raise ValueError()

        return arr

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._ptr = ctypes.cast(self.ctypes.data, c_ushort_ptr)

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
    def ptr(self) -> c_ushort_ptr:
        return self._ptr


class DOFFloatArray(np.ndarray, SupportsPtr[c_double]):
    """
    A class representing an array of joint angles for all
    degree-of-freedoms as a view over a `numpy.ndarray`.
    """

    def __new__(
        cls, data: npt.ArrayLike = tuple(0. for _ in range(MAX_DOF))
    ):
        arr = np.ascontiguousarray(data, dtype=c_double).view(cls)

        if len(arr) != MAX_DOF:
            raise ValueError()

        return arr

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._ptr = ctypes.cast(self.ctypes.data, c_double_ptr)

        self._delta = Vector3(self[:3])
        self._wrist = Vector3(self[3:6])
        self._gripper = ctypes.cast(
            self.ctypes.data + 7 * self.itemsize, c_double_ptr
        ).contents

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
    def ptr(self) -> c_double_ptr:
        return self._ptr

    @property
    def delta(self) -> Vector3:
        return self._delta

    @property
    def wrist(self) -> Vector3:
        return self._wrist

    @property
    def gripper(self) -> c_double:
        return self._gripper


class DOFFloatState(np.ndarray):
    def __new__(
        cls, data: npt.ArrayLike = tuple(0. for _ in range(2 * MAX_DOF))
    ):
        arr = np.ascontiguousarray(data, dtype=c_double)

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


class Mat3x3(np.ndarray, SupportsPtr[c_double]):
    """
    Represents a 3x3 orientation frame matrix as a view over a
    `numpy.ndarray`.
    """

    def __new__(cls, data: npt.ArrayLike = tuple(0. for _ in range(9))):
        arr = np.ascontiguousarray(data, dtype=c_double)

        if len(arr) != 9:
            raise ValueError()

        return arr.reshape((3, 3)).view(cls)

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._ptr = ctypes.cast(self.ctypes.data, c_double_ptr)

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
    def ptr(self) -> c_double_ptr:
        return self._ptr


class Mat6x6(np.ndarray, SupportsPtr[c_double]):
    """
    Represents a 6x6 inertia matrix as a view over a `numpy.ndarray`.
    """

    def __new__(cls, data: npt.ArrayLike = tuple(0. for _ in range(36))):
        arr = np.ascontiguousarray(data, dtype=c_double)

        if len(arr) != 36:
            raise ValueError()

        return arr.reshape((6, 6)).view(cls)

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._ptr = ctypes.cast(self.ctypes.data, c_double_ptr)

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
    def ptr(self) -> c_double_ptr:
        return self._ptr
