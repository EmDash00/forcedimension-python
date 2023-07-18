from typing import Any, Container, List, Tuple, TypeVar, Protocol, Sized

_KT_contra = TypeVar("_KT_contra", contravariant=True)
_VT = TypeVar("_VT")
_VT_co = TypeVar("_VT_co", covariant=True)

from typing import Union
from numpy import float64, int64
from numpy.typing import NDArray


class _Array(Sized, Container[_KT_contra], Protocol[_KT_contra, _VT_co]):
    def __getitem__(self, __k: _KT_contra) -> _VT_co: ...


class _MutableArray(
    _Array[_KT_contra, _VT], Protocol[_KT_contra, _VT]
):
    def __setitem__(self, __k: _KT_contra, __v: _VT) -> None: ...

#: Represents the type of a homogenous array of floats
#: specifically, it implements ``__getitem__`` and ``__len__``
FloatArray = TypeVar('FloatArray', bound=_Array[int, float])

#: Represents the type of a homogenous array of ints
#: specifically, it implements ``__getitem__`` and ``__len__``
IntArray = TypeVar('IntArray', bound=_Array[int, int])

#: Represents the type of a mutable homogenous array of floats
#: specifically, it implements ``__setitem__``, ``__getitem__``, and
#: ``__len__``
MutableFloatArray = TypeVar(
    'MutableFloatArray', bound=_MutableArray[int, float]
)

#: Represents the type of a homogenous array of floats.
#: Used specifically to simplify the typing of various returns since
#: Functions that take in FloatArray are assumed to return the same kind
#: of float array.
FloatVectorLike = Union[FloatArray, NDArray[float64], List[float]]

#: Represents the type of a mutable homogenous array of floats.
#: Used specifically to simplify the typing of various returns since
#: Functions that take in FloatArray are assumed to return the same kind
#: of float array.
MutableFloatVectorLike = Union[
    MutableFloatArray, NDArray[float64], List[float]
]

#: Represents the type of a homogenous array of ints.
#: Used specifically to simplify the typing of various returns since
#: functions that take in FloatArray are assumed to return the same kind
#: of float array.
IntVectorLike = Union[IntArray, NDArray[int64], List[int]]

#: Represents a 2D array of floats
MutableFloatArr2D = TypeVar(
    'MutableFloatArr2D', bound=_MutableArray[int, MutableFloatVectorLike]
)

MutableFloatMatrixLike = Union[
    MutableFloatArr2D, List[List[float]], NDArray[float64]
]

#: Represents a tuple of MAX_DOF ints, one for each DOF
DOFTuple = Tuple[int, int, int, int, int, int, int, int]
