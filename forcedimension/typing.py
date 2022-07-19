from typing import Any, TypeVar, Protocol
from _typeshed import SupportsGetItem

_KT_contra = TypeVar("_KT_contra", contravariant=True)
_VT = TypeVar("_VT")


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

