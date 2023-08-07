import ctypes as ct
from ctypes import c_double, c_int, c_ubyte, c_uint, c_ushort
from typing import (
    TYPE_CHECKING, Container, Generic, List, Protocol, Sized, Tuple,
    TypeVar, Union
)

_KT_contra = TypeVar("_KT_contra", contravariant=True)
_VT = TypeVar("_VT")
_VT_co = TypeVar("_VT_co", covariant=True)

try:
    from numpy import float64, int64
    from numpy.typing import NDArray

    MutableFloatReturnArray = Union[List[float], NDArray[float64]]
    MutableFloatReturnArray2D = Union[List[List[float]], NDArray[float64]]
    MutableIntReturnArray = Union[List[int], NDArray[int64]]
except ImportError:
    MutableFloatReturnArray = List[float]
    MutableFloatReturnArray2D = List[List[float]]
    MutableIntReturnArray = List[int]


class _Array(Sized, Container[_VT_co], Protocol[_KT_contra, _VT_co]):
    def __getitem__(self, __k: _KT_contra) -> _VT_co: ...


class _MutableArray(
    _Array[_KT_contra, _VT], Protocol[_KT_contra, _VT]
):
    def __setitem__(self, __k: _KT_contra, __v: _VT) -> None: ...


if not TYPE_CHECKING:
    class Pointer:
        @classmethod
        def __class_getitem__(cls, item):

            # Don't try to resolve generic types at runtime.
            # They only matter for typing anyways.
            if isinstance(item, TypeVar):
                return None

            return ct.POINTER(item)

    CType = TypeVar('CType')
else:
    #: Generic type representing a C pointer
    Pointer = ct._Pointer

    #: Generic type representing a C data type (e.g. int, float, etc.)
    CType = TypeVar('CType', bound=ct._CData)

PointerType = Pointer[CType]

c_double_ptr = Pointer[c_double]
c_ubyte_ptr = Pointer[c_ubyte]
c_ushort_ptr = Pointer[c_ushort]
c_int_ptr = Pointer[c_int]
c_uint_ptr = Pointer[c_uint]


class SupportsPtr(Protocol, Generic[CType]):
    @property
    def ptr(self) -> Pointer[CType]:
        ...


class SupportsPtrs3(Protocol, Generic[CType]):
    @property
    def ptrs(self) -> Tuple[
        Pointer[CType], Pointer[CType], Pointer[CType]
    ]:
        ...

#: Represents the type of a homogenous array of floats
#: specifically, it implements ``__getitem__`` and ``__len__``
FloatArray = _Array[int, float]

#: Represents the type of a homogenous array of ints
#: specifically, it implements ``__getitem__`` and ``__len__``
IntArray = _Array[int, int]

#: Represents the type of a mutable homogenous array of floats
#: specifically, it implements ``__setitem__``, ``__getitem__``, and
#: ``__len__``
MutableFloatArray = _MutableArray[int, float]

#: Represents the type of a mutable homogenous array of ints
#: specifically, it implements ``__setitem__``, ``__getitem__``, and
#: ``__len__``
MutableIntArray = _MutableArray[int, int]

#: Represents the type of a homogenous array of floats.
#: Used specifically to simplify the typing of various returns since
#: Functions that take in FloatArray are assumed to return the same kind
#: of float array.
FloatVectorLike = FloatArray

#: Represents the type of a mutable homogenous array of floats.
#: Used specifically to simplify the typing of various returns since
#: Functions that take in FloatArray are assumed to return the same kind
#: of float array.
MutableFloatVectorLike = Union[MutableFloatArray, MutableFloatReturnArray]

#: Represents the type of a homogenous array of ints.
#: Used specifically to simplify the typing of various returns since
#: functions that take in FloatArray are assumed to return the same kind
#: of float array.
IntVectorLike = IntArray


#: Represents the type of a mutable homogenous array of ints.
#: Used specifically to simplify the typing of various returns since
#: functions that take in FloatArray are assumed to return the same kind
#: of float array.

MutableIntVectorLike = Union[MutableIntArray, MutableIntReturnArray]


#: Represents a 2D array of floats
MutableFloatArray2D = _MutableArray[int, MutableFloatVectorLike]

#: Represents the type of a homogenous 2D array of floats.
#: Used specifically to simplify the typing of various returns since
#: functions that take in MutableFloatArray2D are assumed to return the same
#: of 2D float array.
MutableFloatMatrixLike = Union[MutableFloatArray2D, MutableFloatReturnArray2D]

IntDOFTuple = Tuple[int, int, int, int, int, int, int, int]
FloatDOFTuple = Tuple[float, float, float, float, float, float, float, float]

class GenericVec(_MutableArray[int, float], Protocol):
    def __getitem__(self, __k: int) -> float: ...
    def __setitem__(self, __k: int, __v: float) -> None: ...
    @property
    def ptrs(self) -> Tuple[
        Pointer[c_double], Pointer[c_double], Pointer[c_double]
    ]:
        ...

