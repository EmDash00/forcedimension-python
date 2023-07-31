from copy import deepcopy
from threading import Event, Thread
from time import perf_counter_ns, sleep
from typing import Any, Generic, NoReturn,  TypeVar, Union
from forcedimension.typing import _MutableArray


from forcedimension import HapticDevice


T = TypeVar('T')


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

    def __init__(self, data: _MutableArray):
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


class HapticPoller(Thread):
    def __init__(
        self,  h: HapticDevice, interval: float, *args,
        high_prec: bool = False, **kwargs
    ):
        super().__init__(*args, **kwargs)

        self._h = h

        self._interval = int(interval * 1E9)

        self._stop_event = Event()
        self._pause_event = Event()
        self._pause_sync = Event()

        self._target_args = kwargs.get('args', ())
        self._target_kwargs = kwargs.get('kwargs', {})
        self._target = kwargs.get('target', None)

        self._high_prec = high_prec

    def stop(self):
        self._stop_event.set()
        self.join()

    def pause(self):
        self._pause_event.set()
        self._pause_sync.wait()

    def unpause(self):
        self._pause_event.clear()
        self._pause_sync.clear()

    def _execute_target(self):
        if self._target:
            self._target(*self._target_args, **self._target_kwargs)

    def _run_zero_interval(self):
        while not self._stop_event.is_set():
            if not self._pause_event.is_set():
                self._execute_target()
            else:
                self._pause_sync.set()
                sleep(0.001)

    def _run_low_prec(self):
        while not self._stop_event.wait(self._interval):

            if not self._pause_event.is_set():
                self._pause_sync.set()
                sleep(0.001)
                continue

            self._execute_target()

    def _run_high_prec(self):
        t0 = perf_counter_ns()

        wait_period = self._interval
        if self._interval > 1_000_000:
            sleep_period = (self._interval - 1_000_000) / 1E9
        else:
            sleep_period = 0

        if not self._stop_event.is_set():
            self._execute_target()

        t_sleep = perf_counter_ns()
        while not self._stop_event.wait(sleep_period):
            while (perf_counter_ns() - t_sleep) < wait_period:
                pass

            t0 = perf_counter_ns()
            if not self._pause_event.is_set():
                self._execute_target()
            else:
                self._pause_sync.set()
                sleep_period = 0.001
                wait_period = 0.001
                continue

            wait_period = max(self._interval - (perf_counter_ns() - t0), 0)

            if wait_period > 1_000_000:
                sleep_period = (wait_period - 1_000_000) / 1E9
            else:
                sleep_period = 0

            t_sleep = perf_counter_ns()

    def run(self):
        try:
            if self._interval == 0:
                self._run_zero_interval()
                return

            if self._high_prec:
                self._run_high_prec()
            else:
                self._run_low_prec()

        finally:
            del self._target
