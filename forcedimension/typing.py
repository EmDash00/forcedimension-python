from typing import Any, Container, Tuple, TypeVar, Protocol, Sized

_KT_contra = TypeVar("_KT_contra", contravariant=True)
_VT = TypeVar("_VT")
_VT_co = TypeVar("_VT_co", covariant=True)


class Array(Sized, Container[_KT_contra], Protocol[_KT_contra, _VT_co]):
    def __getitem__(self, __k: _KT_contra) -> _VT_co: ...


class MutableArray(
    Array[_KT_contra, _VT], Protocol[_KT_contra, _VT]
):
    def __setitem__(self, __k: _KT_contra, __v: _VT) -> None: ...


FloatVectorLike = TypeVar('FloatVectorLike', bound=Array[int, float])
IntVectorLike = TypeVar('IntVectorLike', bound=Array[int, int])
MutableFloatVectorLike = TypeVar(
    'MutableFloatVectorLike', bound=MutableArray[int, float]
)

# I would like to type this as
# TypeVar('MatrixLike', bound=SupportsGetSetItem[int, VectorLike])
# however, generics cannot be used in bounds. See mypy #2756.
MutableFloatMatrixLike = TypeVar(
    'MutableFloatMatrixLike', bound=MutableArray[int, Any]
)

DOFTuple = Tuple[int, int, int, int, int, int, int, int]
