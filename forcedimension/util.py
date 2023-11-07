import ctypes as ct
import functools
import inspect
import sys
from copy import deepcopy
from ctypes import Structure
from typing import Any, Callable, Generic, NoReturn, Optional, TypeVar, Union

from forcedimension_core.runtime import _libdhd
from forcedimension_core.typing import MutableArray

T = TypeVar('T')


if sys.platform == 'win32':
    from ctypes import WINFUNCTYPE as _NATIVE_FUNCTYPE
else:
    from ctypes import CFUNCTYPE as _NATIVE_FUNCTYPE


_libdhd.dhdGetTime.argtypes = []
_libdhd.dhdGetTime.restype = ct.c_double


@_NATIVE_FUNCTYPE(None, ct.c_double)
def spin(time: float):
    """
    Busy wait while minimally holding the Python GIL. Allows multiple Python
    threads to busy wait simultaneously. For best results use time > 10 us.

    :param float time:
        Time (in [s]) to busy wait.
    """
    t0 = _libdhd.dhdGetTime()
    while _libdhd.dhdGetTime() - t0 < time:
        pass


def function_chain(*funcs: Callable[..., Any]) -> Callable[..., Any]:
    def ret():
        for func in funcs:
            func()

    return ret


class ImmutableWrapper(Generic[T]):
    __slots__ = ['_data']

    @classmethod
    def _generate_wrapper_cls(cls, wrapper_cls):
        namespace = {}

        copy_methods = [
            '__concat__',
            '__iconcat__',
        ]

        removed_methods = [
            '__reverse__',
            'sort',
            'append',
            'pop',
            'clear',
            'remove'
        ]

        proxy_operators = [
            '__lt__',
            '__le__',
            '__eq__',
            '__ne__',
            '__ge__',
            '__gt__',
            '__length_hint__',
            '__len__',
            '__iter__',
            '__missing__',
            '__contains__'
            '__index__',
            '__hash__',
            '__nonzero__',
            '__abs__',
            '__invert__',
            '__neg__',
            '__pos__',
            '__round__',
            '__floor__',
            '__ceil__',
            '__trunc__',
            '__call__',
            '__hex__',
            '__oct__'
            '__int__',
            '__long__',
            '__float__'
            '__reduce__',
            '__reduce_ex__',
            'next'
        ]

        lri_operators = [
            '__{}add__',
            '__{}and__',
            '__{}floordiv__',
            '__{}mod__',
            '__{}mul__',
            '__{}matmul__',
            '__{}or__',
            '__{}pow__',
            '__{}rshift__',
            '__{}lshift__',
            '__{}sub__',
            '__{}truediv__',
            '__{}divmod__',
            '__{}xor__',
        ]

        def make_method(name):
            def f(self, *args, **kwargs):
                return (
                    (getattr(object.__getattribute__(self, '_data'), name))(
                        *args,
                        **kwargs
                    )
                )

            return f

        def make_copy_method(name):
            def f(self, *args):
                data_cpy = deepcopy(object.__getattribute__(self, '_data'))

                return (getattr(data_cpy, name))(*args)

            return f

        def bad_method(name):
            def f(self, *args, **kwargs):
                raise ValueError(
                    "ImmutableWrapper prevents mutation on the underlying "
                    "(possibly mutable) data."
                )

            return f

        for op in proxy_operators:
            if hasattr(wrapper_cls, op):
                namespace[op] = make_method(op)

        # for op in copy_methods:
            # if hasattr(cls_name, op):
                # namespace[op] = make_copy_method(op)

        for op in lri_operators:
            l_op = op.format('')
            if hasattr(wrapper_cls, l_op):
                namespace[l_op] = make_copy_method(l_op)

            r_op = op.format('r')
            if hasattr(wrapper_cls, r_op):
                namespace[r_op] = make_copy_method(r_op)

            i_op = op.format('i')
            if hasattr(wrapper_cls, i_op):
                namespace[i_op] = bad_method(i_op)

        for op in copy_methods:
            if hasattr(wrapper_cls, op):
                namespace[op] = make_copy_method(op)

        for op in removed_methods:
            if hasattr(wrapper_cls, op):
                namespace[op] = bad_method(op)

        return type(
            "{}({})".format(cls.__name__, wrapper_cls),
            (cls,),
            namespace
        )

    def __new__(cls, data, *args, **kwargs):

        # TODO: manually override truth
        # TODO: manually overrride reverse

        data_cls = data.__class__
        try:
            cache = cls.__dict__["_class_proxy_cache"]
        except KeyError:
            cls._class_proxy_cache = cache = {}
        try:
            generated_cls = cache[data.__class__]
        except KeyError:
            generated_cls = cls._generate_wrapper_cls(data_cls)
            cache[data.__class__] = generated_cls

        instance = object.__new__(generated_cls)
        generated_cls.__init__(instance, data, *args, **kwargs)

        return instance

    def __init__(self, data: Union[MutableArray, Structure]):
        object.__setattr__(self, '_data', data)

    def __getattribute__(self, name: str):
        return getattr(object.__getattribute__(self, '_data'), name)

    def __setattr__(self, name: str, value: Any):
        raise TypeError(
            "ImmutableWrapper prevents mutation over the underlying "
            "(possibly mutable) data."
        )

    def __delattr__(self, name: str) -> NoReturn:
        raise TypeError(
            "ImmutableWrapper prevents mutation over the underlying "
            "(possibly mutable) data."
        )

    def __getitem__(self, index: Union[int, slice]):
        return (object.__getattribute__(self, '_data'))[index]

    def __setitem__(self, index: Union[int, slice], value) -> NoReturn:
        raise TypeError(
            "ImmutableWrapper prevents mutation over the underlying "
            "(possibly mutable) data."
        )

    def __delitem__(self, i) -> NoReturn:
        raise TypeError(
            "ImmutableWrapper prevents mutation over the underlying "
            "(possibly mutable) data."
        )

    def __repr__(self) -> str:
        return "ImmutableWrapper({})".format(
            (object.__getattribute__(self, '_data')).__repr__()
        )

    def __str__(self) -> str:
        return (object.__getattribute__(self, '_data')).__str__()


def full_method_name(method: Callable[..., Any]):
    if (parent_class := _get_impl_class(method)) is None:
        raise ValueError("Could not find parent class name.")

    return f"{parent_class.__name__}.{method.__name__}"


def _get_impl_class(method) -> Optional[type]:
    if isinstance(method, functools.partial):
        return _get_impl_class(method.func)

    if inspect.ismethod(method) or (
        inspect.isbuiltin(method) and
        getattr(method, '__self__', None) is not None and
        getattr(method.__self__, '__class__', None)
    ):
        for cls in inspect.getmro(method.__self__.__class__):
            if method.__name__ in cls.__dict__:
                return cls

        # fallback to __qualname__ parsing
        method = getattr(method, '__func__', method)

    if inspect.isfunction(method):
        cls = getattr(
            inspect.getmodule(method),
            method.__qualname__.split('.<locals>', 1)[0].rsplit('.', 1)[0],
            None
        )

        if isinstance(cls, type):
            return cls

    # handle special descriptor objects
    return getattr(method, '__objclass__', None)
