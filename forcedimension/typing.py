from typing import Container, List, Protocol, Sized, Tuple, TypeVar, Union

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
FloatVectorLike = Union[FloatArray, MutableFloatReturnArray]

#: Represents the type of a mutable homogenous array of floats.
#: Used specifically to simplify the typing of various returns since
#: Functions that take in FloatArray are assumed to return the same kind
#: of float array.
MutableFloatVectorLike = Union[MutableFloatArray, MutableFloatReturnArray]

#: Represents the type of a homogenous array of ints.
#: Used specifically to simplify the typing of various returns since
#: functions that take in FloatArray are assumed to return the same kind
#: of float array.
IntVectorLike = Union[IntArray, MutableIntReturnArray]

#: Represents a 2D array of floats
MutableFloatArray2D = TypeVar(
    'MutableFloatArray2D', bound=_MutableArray[int, MutableFloatVectorLike]
)

#: Represents the type of a homogenous 2D array of floats.
#: Used specifically to simplify the typing of various returns since
#: functions that take in MutableFloatArray2D are assumed to return the same
#: of 2D float array.
MutableFloatMatrixLike = Union[MutableFloatArray2D, MutableFloatReturnArray2D]

#: Represents a tuple of MAX_DOF ints, one for each DOF
DOFTuple = Tuple[int, int, int, int, int, int, int, int]
