from typing import Any, Container, TypeVar, Protocol

_KT_contra = TypeVar("_KT_contra", contravariant=True)
_VT = TypeVar("_VT")
_VT_co = TypeVar("_VT_co", covariant=True)


class SupportsGetItem(Container[_KT_contra], Protocol[_KT_contra, _VT_co]):
    def __getitem__(self, __k: _KT_contra) -> _VT_co: ...


class SupportsGetSetItem(
    SupportsGetItem[_KT_contra, _VT], Protocol[_KT_contra, _VT]
):
    def __setitem__(self, __k: _KT_contra, __v: _VT) -> None: ...


VectorLike = TypeVar('VectorLike', bound=SupportsGetSetItem[int, float])

# I would like to type this as
# TypeVar('MatrixLike', bound=SupportsGetSetItem[int, VectorLike])
# however, generics cannot be used in bounds. See mypy #2756.
MatrixLike = TypeVar(
    'MatrixLike', bound=SupportsGetSetItem[int, Any]
)
