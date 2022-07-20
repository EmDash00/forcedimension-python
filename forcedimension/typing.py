from typing import Any, Container, Tuple, TypeVar, Protocol, Sized

_KT_contra = TypeVar("_KT_contra", contravariant=True)
_VT = TypeVar("_VT")
_VT_co = TypeVar("_VT_co", covariant=True)


class _Array(Sized, Container[_KT_contra], Protocol[_KT_contra, _VT_co]):
    def __getitem__(self, __k: _KT_contra) -> _VT_co: ...


class _MutableArray(
    _Array[_KT_contra, _VT], Protocol[_KT_contra, _VT]
):
    def __setitem__(self, __k: _KT_contra, __v: _VT) -> None: ...

#: Represents the type of a vectors of floats
#: specifically, it implements ``__getitem__`` and ``__len__``
FloatVectorLike = TypeVar('FloatVectorLike', bound=_Array[int, float])

#: Represents vectors of ints
#: specifically, it implements ``__getitem__`` and ``__len__``
IntVectorLike = TypeVar('IntVectorLike', bound=_Array[int, int])

#: Represents the type of a mutable vectors of floats
#: specifically, it implements ``__setitem__``, ``__getitem__``, and
#: ``__len__``
MutableFloatVectorLike = TypeVar(
    'MutableFloatVectorLike', bound=_MutableArray[int, float]
)

# I would like to type this as
# TypeVar('MatrixLike', bound=SupportsGetSetItem[int, VectorLike])
# however, generics cannot be used in bounds. See mypy#2756.


#: Represents a mutable matrix. WIP
MutableFloatMatrixLike = TypeVar(
    'MutableFloatMatrixLike', bound=_MutableArray[int, Any]
)

#: Represents a tuple of MAX_DOF ints, one for each DOF
DOFTuple = Tuple[int, int, int, int, int, int, int, int]
