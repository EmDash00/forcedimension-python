import ctypes
from array import ArrayType, array
from ctypes import c_double, c_int, c_ushort
from typing import Iterable, List, Tuple, NamedTuple

from forcedimension.dhd.constants import MAX_DOF
from forcedimension.typing import (
    SupportsPtr, SupportsPtrs3,
    c_double_ptr, c_int_ptr, c_ushort_ptr
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


class Vector3(array, SupportsPtrs3[c_double], SupportsPtr[c_double_ptr]):
    """
    A List[float] providing convience "x", "y", "z" read-write accessor
    properties. This class subclasses List[float]; therefore, for all intents
    and purposes, you can treat it as a list. This allows for it to be
    compatible with Python's standard library functions.
    """
    def __new__(
        cls, initializer: Iterable[float] = (0., 0., 0.)
    ):
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
    def __new__(
        cls, initializer: Iterable[float] = (0., 0., 0.)
    ):
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

    @property
    def ptr(self) -> c_int_ptr:
        return self._ptrs[0]

    @property
    def ptrs(self) -> Tuple[c_int_ptr, c_int_ptr, c_int_ptr]:
        return self._ptrs


class Enc4(array, SupportsPtr[c_int]):
    def __new__(
        cls, initializer: Iterable[float] = (0., 0., 0., 0.)
    ):
        arr = super(Enc4, cls).__new__(
            cls, 'd', initializer  # type: ignore
        )

        if len(arr) != 4:
            raise ValueError()

        return arr

    def __init__(self, *args, **kwargs):
        self._ptr = ctypes.cast(self.buffer_info()[0], c_int_ptr)

    @property
    def ptr(self) -> c_int_ptr:
        return self._ptr


class DOFIntArray(array, SupportsPtr[c_int]):
    """
    A List[float] providing convience "x", "y", "z" read-write accessor
    properties. This class subclasses List[float]; therefore, for all intents
    and purposes, you can treat it as a list. This allows for it to be
    compatible with Python's standard library functions.
    """
    def __new__(
        cls, initializer: Iterable[int] = (0 for _ in range(MAX_DOF))
    ):
        arr = super(DOFIntArray, cls).__new__(
            cls, 'i', initializer  # type: ignore
        )

        if len(arr) != MAX_DOF:
            raise ValueError()

        return arr

    def __init__(self):
        self._ptr = ctypes.cast(self.buffer_info()[0], c_int_ptr)

    @property
    def ptr(self) -> c_int_ptr:
        return self._ptr

class DOFMotorArray(array, SupportsPtr[c_ushort]):
    """
    A List[float] providing convience "x", "y", "z" read-write accessor
    properties. This class subclasses List[float]; therefore, for all intents
    and purposes, you can treat it as a list. This allows for it to be
    compatible with Python's standard library functions.
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

    @property
    def ptr(self) -> c_ushort_ptr:
        return self._ptr


class DOFFloatArray(array, SupportsPtr[c_double]):
    """
    A List[float] providing convience "x", "y", "z" read-write accessor
    properties. This class subclasses List[float]; therefore, for all intents
    and purposes, you can treat it as a list. This allows for it to be
    compatible with Python's standard library functions.
    """
    def __new__(
        cls, initializer: Iterable[float] = (0 for _ in range(MAX_DOF))
    ):
        arr = super(DOFFloatArray, cls).__new__(
            cls, 'd', initializer  # type: ignore
        )

        if len(arr) != MAX_DOF:
            raise ValueError()

        return arr

    def __init__(self):
        self._ptr = ctypes.cast(self.buffer_info()[0], c_double_ptr)

    @property
    def ptr(self) -> c_double_ptr:
        return self._ptr


class FrameMatrix(List[ArrayType[float]], SupportsPtr[c_double]):
    """
    Represents the type of a coordinate frame matrix.
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

    @property
    def ptr(self) -> c_double_ptr:
        return self._ptr


class InertiaMatrix(List[ArrayType[float]], SupportsPtr[c_double]):
    """
    Represents the type of a coordinate frame matrix.
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

    @property
    def ptr(self) -> c_double_ptr:
        return self._ptr


DefaultVecType = Vector3
DefaultEnc3Type = Enc3
DefaultEnc4Type = Enc4
DefaultDOFIntType = DOFIntArray
DefaultDOFFloatType = DOFFloatArray
DefaultFrameMatrixType = FrameMatrix
DefaultInertiaMatrixType = InertiaMatrix


try:
    from numpy import asarray, ndarray

    from forcedimension.typing import FloatArray

    class NumpyVector3(
        ndarray, SupportsPtrs3[c_double_ptr], SupportsPtr[c_double_ptr]
    ):
        """
        A view over a numpy ndarry, which provides convenience "x", "y", and "z"
        read-write accessor properties. This class subclasses ndarray;
        therefore, for all intents and purposes you can treat it as an ndarray.
        This allows you to simply pass in this class to any and all numpy
        methods.
        """
        def __new__(cls, data: FloatArray = [0., 0., 0.]):
            if len(data) != 3:
                raise ValueError

            return asarray(data, dtype=c_double).view(cls)

        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            self._ptrs = (
                ctypes.cast(self.ctypes.data, c_double_ptr),
                ctypes.cast(self.ctypes.data + self.itemsize, c_double_ptr),
                ctypes.cast(self.ctypes.data + 2 * self.itemsize, c_double_ptr)
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

    class NumpyEnc3(ndarray, SupportsPtr[c_int], SupportsPtrs3[c_int]):
        """
        A view over a numpy ndarry, which provides convenience "x", "y", and "z"
        read-write accessor properties. This class subclasses ndarray;
        therefore, for all intents and purposes you can treat it as an ndarray.
        This allows you to simply pass in this class to any and all numpy
        methods.
        """
        def __new__(cls, data: FloatArray = [0., 0., 0.]):
            if len(data) != 3:
                raise ValueError

            return asarray(data, dtype=c_int).view(cls)

        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            self._ptrs = (
                ctypes.cast(self.ctypes.data, c_int_ptr),
                ctypes.cast(self.ctypes.data + self.itemsize, c_int_ptr),
                ctypes.cast(self.ctypes.data + 2 * self.itemsize, c_int_ptr)
            )

        @property
        def ptr(self) -> c_int_ptr:
            return self._ptrs[0]

        @property
        def ptrs(self) -> Tuple[c_int_ptr, c_int_ptr, c_int_ptr]:
            return self._ptrs


    class NumpyEnc4(ndarray, SupportsPtr[c_int]):
        """
        A view over a numpy ndarry, which provides convenience "x", "y", and "z"
        read-write accessor properties. This class subclasses ndarray;
        therefore, for all intents and purposes you can treat it as an ndarray.
        This allows you to simply pass in this class to any and all numpy
        methods.
        """
        def __new__(cls, data: FloatArray = [0., 0., 0., 0.]):
            if len(data) != 4:
                raise ValueError

            return asarray(data, dtype=c_int).view(cls)

        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            self._ptr = ctypes.cast(self.ctypes.data, c_int_ptr)

        @property
        def ptr(self) -> c_int_ptr:
            return self._ptr


    class NumpyDOFIntArray(ndarray, SupportsPtr[c_int]):
        def __new__(cls, data: Iterable[int] = (0 for _ in range(MAX_DOF))):
            arr = asarray(data, dtype=c_int).view(cls)

            if len(arr) != MAX_DOF:
                raise ValueError()

            return arr

        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            self._ptr = ctypes.cast(self.ctypes.data, c_int_ptr)

        @property
        def ptr(self) -> c_int_ptr:
            return self._ptr

    class NumpyDOFMotorArray(ndarray, SupportsPtr[c_ushort]):
        def __new__(cls, data: Iterable[int] = (0 for _ in range(MAX_DOF))):
            arr = asarray(data, dtype=c_ushort).view(cls)

            if len(arr) != MAX_DOF:
                raise ValueError()

            return arr

        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            self._ptr = ctypes.cast(self.ctypes.data, c_ushort_ptr)

        @property
        def ptr(self) -> c_ushort_ptr:
            return self._ptr

    class NumpyDOFFloatArray(ndarray, SupportsPtr[c_double]):
        def __new__(cls, data: Iterable[float] = (0 for _ in range(MAX_DOF))):
            arr = asarray(data, dtype=c_double).view(cls)

            if len(arr) != MAX_DOF:
                raise ValueError()

            return arr

        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            self._ptr = ctypes.cast(self.ctypes.data, c_double_ptr)

        @property
        def ptr(self) -> c_double_ptr:
            return self._ptr



    class NumpyFrameMatrix(ndarray, SupportsPtr[c_double_ptr]):
        """
        A view over a numpy ndarry, which provides convenience "x", "y", and "z"
        read-write accessor properties. This class subclasses ndarray;
        therefore, for all intents and purposes you can treat it as an ndarray.
        This allows you to simply pass in this class to any and all numpy
        methods.
        """
        def __new__(cls, data: Iterable[float] = (0. for _ in range(MAX_DOF))):
            arr = asarray(data, dtype=c_double).view(cls).reshape((3, 3))

            if len(arr) != 9:
                raise ValueError()

            return arr.reshape((3, 3))

        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            self._ptr = ctypes.cast(self.ctypes.data, c_double_ptr)

        @property
        def ptr(self) -> c_double_ptr:
            return self._ptr

    class NumpyInertiaMatrix(ndarray, SupportsPtr[c_double_ptr]):
        """
        A view over a numpy ndarry, which provides convenience "x", "y", and "z"
        read-write accessor properties. This class subclasses ndarray;
        therefore, for all intents and purposes you can treat it as an ndarray.
        This allows you to simply pass in this class to any and all numpy
        methods.
        """
        def __new__(cls, data: Iterable[float] = (0. for _ in range(36))):
            arr = asarray(data, dtype=c_int).view(cls)

            if len(arr) != 36:
                raise ValueError()

            return arr.reshape((6, 6))

        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            self._ptr = ctypes.cast(self.ctypes.data, c_double_ptr)

        @property
        def ptr(self) -> c_double_ptr:
            return self._ptr

    DefaultVecType = NumpyVector3
    DefaultEnc3Type = NumpyEnc3
    DefaultEnc4Type = NumpyEnc4
    DefaultDOFIntType = NumpyDOFIntArray
    DefaultDOFFloatType = NumpyDOFFloatArray
    DefaultFrameMatrixType = NumpyFrameMatrix
    DefaultInertiaMatrixType = NumpyInertiaMatrix
except ImportError:
    pass
