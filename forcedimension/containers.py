import ctypes
from array import array
from ctypes import c_double, c_int, c_ushort
from typing import Any, Iterable, List, NamedTuple, Optional, Tuple

import pydantic
import pydantic_core

from forcedimension.dhd.constants import MAX_DOF
from forcedimension.typing import (
    IntArray, SupportsPtr, SupportsPtrs3, c_double_ptr, c_int_ptr, c_ushort_ptr
)


class VersionTuple(NamedTuple):
    """
    Adapts the four seperate number return into a single grouped
    NamedTuple.
    """
    major: int
    minor: int
    release: int
    revision: int

    def __str__(self):
        return f"{self.major}.{self.minor}.{self.release}-{self.revision}"


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


class _HapticPollerOptions(NamedTuple):
    interval: float
    high_precision: bool


class Vector3(array, SupportsPtrs3[c_double], SupportsPtr[c_double_ptr]):
    """
    Represents a vector with attributes x, y, and z corresponding to the 0th,
    1st, and 2nd indicies, respectively, as a Python `array.array`.
    """

    def __new__(
        cls, initializer: Iterable[float] = (0., 0., 0.)
    ):
        if isinstance(initializer, array):
            return initializer

        arr = super(Vector3, cls).__new__(
            cls, 'd', initializer  # type: ignore
        )

        if len(arr) != 3:
            raise ValueError()

        return arr

    def __init__(self, *args, **kwargs):
        ptr = self.buffer_info()[0]
        self._ptrs = (
            ctypes.cast(ptr, c_double_ptr),
            ctypes.cast(ptr + self.itemsize, c_double_ptr),
            ctypes.cast(ptr + 2 * self.itemsize, c_double_ptr),
        )

    @classmethod
    def __get_pydantic_core_schema__(
        cls, source_type: Any, handler: pydantic.GetCoreSchemaHandler
    ) -> pydantic_core.CoreSchema:
        return core_schema.no_info_after_validator_function(  # type: ignore
            cls, handler(cls.__init__)
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
    def y(self) -> float:
        return self[1]

    @y.setter
    def y(self, value: float):
        self[1] = value

    @property
    def z(self) -> float:
        return self[2]

    @z.setter
    def z(self, value: float):
        self[2] = value


class Enc3(array, SupportsPtrs3[c_int], SupportsPtr[c_int]):
    """
    Represents the type of a 3-axis encoder array (e.g. delta or wrist encoder
    arrays) as a Python `array.array`.
    """

    def __new__(
        cls, initializer: Iterable[float] = (0., 0., 0.)
    ):
        if isinstance(initializer, array):
            return initializer

        arr = super(Enc3, cls).__new__(
            cls, 'd', initializer  # type: ignore
        )

        if len(arr) != 3:
            raise ValueError()

        return arr

    def __init__(self, *args, **kwargs):
        ptr = self.buffer_info()[0]
        self._ptrs = (
            ctypes.cast(ptr, c_int_ptr),
            ctypes.cast(ptr + self.itemsize, c_int_ptr),
            ctypes.cast(ptr + 2 * self.itemsize, c_int_ptr),
        )

    @classmethod
    def __get_pydantic_core_schema__(
        cls, source_type: Any, handler: pydantic.GetCoreSchemaHandler
    ) -> pydantic_core.CoreSchema:
        return core_schema.no_info_after_validator_function(  # type: ignore
            cls, handler(cls.__init__)
        )

    @property
    def ptr(self) -> c_int_ptr:
        return self._ptrs[0]

    @property
    def ptrs(self) -> Tuple[c_int_ptr, c_int_ptr, c_int_ptr]:
        return self._ptrs


class Enc4(array, SupportsPtr[c_int]):
    """
    Represents an array of wrist encoders and a gripper encoder as a Python
    `array.array`
    """

    def __new__(
        cls, initializer: Iterable[float] = (0., 0., 0., 0.)
    ):
        if isinstance(initializer, array):
            return initializer

        arr = super(Enc4, cls).__new__(
            cls, 'd', initializer  # type: ignore
        )

        if len(arr) != 4:
            raise ValueError()

        return arr

    def __init__(self, *args, **kwargs):
        self._ptr = ctypes.cast(self.buffer_info()[0], c_int_ptr)

    @classmethod
    def __get_pydantic_core_schema__(
        cls, source_type: Any, handler: pydantic.GetCoreSchemaHandler
    ) -> pydantic_core.CoreSchema:
        return core_schema.no_info_after_validator_function(  # type: ignore
            cls, handler(cls.__init__)
        )

    @property
    def ptr(self) -> c_int_ptr:
        return self._ptr


class DOFInt(array, SupportsPtr[c_int]):
    """
    Represents an array of encoders for each degree-of-freedom as a Python
    `array.array`
    """

    def __new__(
        cls, initializer: Iterable[int] = (0 for _ in range(MAX_DOF))
    ):
        arr = super(DOFInt, cls).__new__(
            cls, 'i', initializer  # type: ignore
        )

        if len(arr) != MAX_DOF:
            raise ValueError()

        return arr

    def __init__(self):
        self._ptr = ctypes.cast(self.buffer_info()[0], c_int_ptr)
        self._delta = Enc3(self[:3])
        self._wrist = Enc3(self[3:6])
        self._wrist_grip = Enc4(self[3:7])
        self._gripper = ctypes.cast(
            self.buffer_info()[0] + self.itemsize * (MAX_DOF - 1), c_int_ptr
        ).contents

    @classmethod
    def __get_pydantic_core_schema__(
        cls, source_type: Any, handler: pydantic.GetCoreSchemaHandler
    ) -> pydantic_core.CoreSchema:
        return core_schema.no_info_after_validator_function(  # type: ignore
            cls, handler(cls.__init__)
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


class DOFMotorArray(array, SupportsPtr[c_ushort]):
    """
    Represents an array of motor commands as a Python `array.array`.
    """

    def __new__(
        cls, initializer: Iterable[int] = (0 for _ in range(MAX_DOF))
    ):
        arr = super(DOFMotorArray, cls).__new__(
            cls, 'H', initializer  # type: ignore
        )

        if len(arr) != MAX_DOF:
            raise ValueError()

        return arr

    def __init__(self):
        self._ptr = ctypes.cast(self.buffer_info()[0], c_ushort_ptr)

    @classmethod
    def __get_pydantic_core_schema__(
        cls, source_type: Any, handler: pydantic.GetCoreSchemaHandler
    ) -> pydantic_core.CoreSchema:
        return core_schema.no_info_after_validator_function(  # type: ignore
            cls, handler(cls.__init__)
        )

    @property
    def ptr(self) -> c_ushort_ptr:
        return self._ptr


class DOFFloat(array, SupportsPtr[c_double]):
    """
    Represents an array of joint angles as a Python `array.array`.
    """

    def __new__(
        cls, initializer: Iterable[float] = (0 for _ in range(MAX_DOF))
    ):
        arr = super(DOFFloat, cls).__new__(
            cls, 'd', initializer  # type: ignore
        )

        if len(arr) != MAX_DOF:
            raise ValueError()

        return arr

    def __init__(self):
        self._ptr = ctypes.cast(self.buffer_info()[0], c_double_ptr)

        self._delta = Vector3(self[:3])
        self._wrist = Vector3(self[3:6])
        self._gripper = self._gripper = ctypes.cast(
            self.buffer_info()[0] + 7 * self.itemsize, c_double_ptr
        ).contents

    @classmethod
    def __get_pydantic_core_schema__(
        cls, source_type: Any, handler: pydantic.GetCoreSchemaHandler
    ) -> pydantic_core.CoreSchema:
        return core_schema.no_info_after_validator_function(  # type: ignore
            cls, handler(cls.__init__)
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


class Mat3x3(List[array], SupportsPtr[c_double]):
    """
    Represents the type of a coordinate frame matrix as a list of Python
    `array.array`.
    """

    def __init__(
        self, iterable: Iterable[float] = (0. for _ in range(MAX_DOF))
    ):
        self._arr = array('d', iterable)
        if len(self._arr) != 9:
            raise ValueError()

        self._ptr = ctypes.cast(self._arr.buffer_info()[0], c_double_ptr)
        super().__init__(
            self._arr[i:i + 3] for i in range(3)
        )

    @classmethod
    def __get_pydantic_core_schema__(
        cls, source_type: Any, handler: pydantic.GetCoreSchemaHandler
    ) -> pydantic_core.CoreSchema:
        return core_schema.no_info_after_validator_function(  # type: ignore
            cls, handler(cls.__init__)
        )


    @property
    def ptr(self) -> c_double_ptr:
        return self._ptr


class Mat6x6(List[array], SupportsPtr[c_double]):
    """
    Represents the type of an inertia matrix as a list of Python `array.array`.
    """

    def __init__(
        self, iterable: Iterable[float] = (0. for _ in range(36))
    ):
        self._arr = array('d', iterable)
        if len(self._arr) != 36:
            raise ValueError()

        self._ptr = ctypes.cast(self._arr.buffer_info()[0], c_double_ptr)
        super().__init__(
            self._arr[i:i + 6] for i in range(6)
        )

    @classmethod
    def __get_pydantic_core_schema__(
        cls, source_type: Any, handler: pydantic.GetCoreSchemaHandler
    ) -> pydantic_core.CoreSchema:
        return core_schema.no_info_after_validator_function(  # type: ignore
            cls, handler(cls.__init__)
        )

    @property
    def ptr(self) -> c_double_ptr:
        return self._ptr


DefaultVecType = Vector3
DefaultEnc3Type = Enc3
DefaultEnc4Type = Enc4
DefaultIntDOFType = DOFInt
DefaultFloatDOFType = DOFFloat
DefaultMat3x3Type = Mat3x3
DefaultMat6x6Type = Mat6x6


try:
    import numpy as np
    import numpy.typing as npt

    from forcedimension.typing import FloatArray

    class NumpyVector3(
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
            cls, source_type: Any, handler: pydantic.GetCoreSchemaHandler
        ) -> pydantic_core.CoreSchema:
            return core_schema.no_info_after_validator_function(  # type: ignore
                cls, handler(cls.__init__)
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

    class NumpyEnc3(np.ndarray, SupportsPtr[c_int], SupportsPtrs3[c_int]):
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
            cls, source_type: Any, handler: pydantic.GetCoreSchemaHandler
        ) -> pydantic_core.CoreSchema:
            return core_schema.no_info_after_validator_function(  # type: ignore
                cls, handler(cls.__init__)
            )

        @property
        def ptr(self) -> c_int_ptr:
            return self._ptrs[0]

        @property
        def ptrs(self) -> Tuple[c_int_ptr, c_int_ptr, c_int_ptr]:
            return self._ptrs

    class NumpyEnc4(np.ndarray, SupportsPtr[c_int]):
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
            cls, source_type: Any, handler: pydantic.GetCoreSchemaHandler
        ) -> pydantic_core.CoreSchema:
            return core_schema.no_info_after_validator_function(  # type: ignore
                cls, handler(cls.__init__)
            )

        @property
        def ptr(self) -> c_int_ptr:
            return self._ptr

    class NumpyDOFInt(np.ndarray, SupportsPtr[c_int]):
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
            self._delta = NumpyEnc3(self[:3])
            self._wrist = NumpyEnc3(self[3:6])
            self._wrist_grip = NumpyEnc4(self[3:7])

            self._gripper = ctypes.cast(
                self.ctypes.data + 7 * self.itemsize, c_int_ptr
            ).contents

        @classmethod
        def __get_pydantic_core_schema__(
            cls, source_type: Any, handler: pydantic.GetCoreSchemaHandler
        ) -> pydantic_core.CoreSchema:
            return core_schema.no_info_after_validator_function(  # type: ignore
                cls, handler(cls.__init__)
            )

        @property
        def ptr(self) -> c_int_ptr:
            return self._ptr

        @property
        def delta(self) -> NumpyEnc3:
            return self._delta

        @property
        def wrist(self) -> NumpyEnc3:
            return self._wrist

        @property
        def wrist_grip(self) -> NumpyEnc4:
            return self._wrist_grip

        @property
        def gripper(self) -> c_int:
            return self._gripper

    class NumpyDOFMotorArray(np.ndarray, SupportsPtr[c_ushort]):
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
            cls, source_type: Any, handler: pydantic.GetCoreSchemaHandler
        ) -> pydantic_core.CoreSchema:
            return core_schema.no_info_after_validator_function(  # type: ignore
                cls, handler(cls.__init__)
            )

        @property
        def ptr(self) -> c_ushort_ptr:
            return self._ptr

    class NumpyDOFFloat(np.ndarray, SupportsPtr[c_double]):
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

            self._delta = NumpyVector3(self[:3])
            self._wrist = NumpyVector3(self[3:6])
            self._gripper = ctypes.cast(
                self.ctypes.data + 7 * self.itemsize, c_double_ptr
            ).contents

        @classmethod
        def __get_pydantic_core_schema__(
            cls, source_type: Any, handler: pydantic.GetCoreSchemaHandler
        ) -> pydantic_core.CoreSchema:
            return core_schema.no_info_after_validator_function(  # type: ignore
                cls, handler(cls.__init__)
            )

        @property
        def ptr(self) -> c_double_ptr:
            return self._ptr

        @property
        def delta(self) -> NumpyVector3:
            return self._delta

        @property
        def wrist(self) -> NumpyVector3:
            return self._wrist

        @property
        def gripper(self) -> c_double:
            return self._gripper

    class NumpyMat3x3(np.ndarray, SupportsPtr[c_double]):
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
            cls, source_type: Any, handler: pydantic.GetCoreSchemaHandler
        ) -> pydantic_core.CoreSchema:
            return core_schema.no_info_after_validator_function(  # type: ignore
                cls, handler(cls.__init__)
            )

        @property
        def ptr(self) -> c_double_ptr:
            return self._ptr

    class NumpyMat6x6(np.ndarray, SupportsPtr[c_double]):
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
            cls, source_type: Any, handler: pydantic.GetCoreSchemaHandler
        ) -> pydantic_core.CoreSchema:
            return core_schema.no_info_after_validator_function(  # type: ignore
                cls, handler(cls.__init__)
            )

        @property
        def ptr(self) -> c_double_ptr:
            return self._ptr

    DefaultVecType = NumpyVector3
    DefaultEnc3Type = NumpyEnc3
    DefaultEnc4Type = NumpyEnc4
    DefaultIntDOFType = NumpyDOFInt
    DefaultFloatDOFType = NumpyDOFFloat
    DefaultMat3x3Type = NumpyMat3x3
    DefaultMat6x6Type = NumpyMat6x6
except ImportError:
    pass
