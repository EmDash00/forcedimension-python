from typing import List, NoReturn, Union, Any, MutableSequence, NamedTuple
from typing import TypeVar, Generic, Optional
from copy import deepcopy

T = TypeVar('T')


class NamedSequence(type):
    """
    Euclidian metaclass for MutableSequence types. Using this metaclass will
    automatically create convience 3 read-write properties "x", "y", and "z",
    which correspond to the 0th, 1st, and 2nd elements of an object of the
    MutableSequence class.
    """
    def __new__(cls, name, bases, dct, names=['x', 'y', 'z']):

        x = super().__new__(cls, name, bases, dct)

        def mutable_accessor(i):
            def getter(self):
                return self.__getitem__(i)

            def setter(self, value):
                self.__setitem__(i, value)

            return property(getter, setter)

        for i, prop in enumerate(names):
            setattr(x, prop, mutable_accessor(i))

        return x


class EuclidianVector(List[float], metaclass=NamedSequence):
    """
    A List[float] providing convience "x", "y", "z" read-write accessor
    properties. This class subclasses List[float]; therefore, for all intents
    and purposes, you can treat it as a list. This allows for it to be
    compatible with Python's standard library functions.
    """
    def __init__(self, data: List[float] = [0, 0, 0]):
        if (len(data) != 3):
            raise ValueError

        super().__init__(data)


class EncoderVector(List[float], metaclass=NamedSequence,
                    names=['enc1', 'enc2', 'enc3']):
    """
    A List[float] providing convience "x", "y", "z" read-write accessor
    properties. This class subclasses List[float]; therefore, for all intents
    and purposes, you can treat it as a list. This allows for it to be
    compatible with Python's standard library functions.
    """
    def __init__(self, data: List[float] = [0, 0, 0]):
        if (len(data) != 3):
            raise ValueError

        super().__init__(data)


class JacobianMatrix(List[List[float]]):
    """
    Used by the library backend to create the default type of the
    3x3 jacobian matrix.
    """
    def __init__(self):
        super().__init__(
            [
                [0, 0, 0],
                [0, 0, 0],
                [0, 0, 0]
            ]
        )


try:
    from numpy import ndarray, asarray, zeros  # type: ignore

    class NumpyVector(ndarray, metaclass=NamedSequence):
        """
        A view over a numpy ndarry, which provides convience "x", "y", and "z"
        read-write accessor properties. This class subclasses ndarray;
        therefore, for all intents and purposes you can treat it as an ndarray.
        This allows you to simply pass in this class to any and all numpy
        methods.
        """
        def __new__(cls, data: MutableSequence[float] = [0, 0, 0]):
            if len(data) != 3:
                raise ValueError

            return asarray(data, dtype=float).view(cls)

    # not sure why but throws a call-arg error in mypy
    class NumpyEncVec(ndarray, metaclass=NamedSequence,  # type: ignore
                      names=['enc0', 'enc1', 'enc2']):
        """
        A view over a numpy ndarry, which provides convience "enc0", "enc1",
        and "enc2" read-write accessor properties. This class subclasses
        ndarray; therefore, for all intents and purposes you can treat it as an
        ndarray. This allows you to simply pass in this class to any and all
        numpy methods.
        """
        def __new__(cls, data: MutableSequence[float] = [0, 0, 0]):
            if len(data) != 3:
                raise ValueError

            return asarray(data, dtype=float).view(cls)

    class NumpyJacobian(ndarray):
        """
        A view over a JacobianMatrix.
        """
        def __new__(cls):
            return zeros(shape=(3, 3), dtype=float).view(cls)

except ImportError:
    pass


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

    def __init__(self, data: MutableSequence[Any]):
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


class GripperUpdateOpts(NamedTuple):
    enc: float = 1000
    thumb_pos: float = 1000
    finger_pos: float = 1000
    v: float = 4000
    w: float = 4000


class UpdateOpts(NamedTuple):
    enc: Optional[float] = 1000
    v: Optional[float] = 4000
    w: Optional[float] = None
    buttons: Optional[float] = 100
    ft: Optional[float] = 4000
    req: Optional[float] = 4000
    gripper: Optional[GripperUpdateOpts] = None
