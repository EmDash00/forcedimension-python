from ctypes import c_bool, c_byte, c_double
from forcedimension import runtime

_libdhd = runtime.load("libdrd")

if _libdhd is None:
    raise ImportError("There were problems loading libdhd.")


_libdhd.dhdKbHit.argtypes = []
_libdhd.dhdKbHit.restype = c_bool


def kbHit() -> bool:
    """
    Check keyboard for a key hit. This function is OS independent.

    :rtype: bool

    :returns:
        `True` if a key on the keyboard was hit, and `False` otherwise.
    """
    return _libdhd.dhdKbHit()


_libdhd.dhdKbGet.argtypes = []
_libdhd.dhdKbGet.restype = c_byte


def kbGet() -> str:
    """
    Retrieve a character from the keyboard. This function is OS independent.

    :rtype: str

    :returns:
        The character hit on the keyboard.
    """
    return chr(_libdhd.dhdKbGet())


_libdhd.dhdGetTime.argtypes = []
_libdhd.dhdGetTime.restype = c_double


def getTime() -> float:
    """
    Returns the current value from the high-resolution system counter in [s].
    The resolution of the system counter may be machine-dependent, as it is
    usually derived from one of the CPU clock signals. The time returned,
    however, is guarunteed to be monotonic.

    :rtype: float

    :returns:
        The current monotonic time in [s] from the high-resolution system
        counter.
    """
    return _libdhd.dhdGetTime()


_libdhd.dhdSleep.argtypes = [c_double]
_libdhd.dhd.restype = None


def sleep(sec: float) -> None:
    """
    Sleep for a given period of time in [s]. This function is OS independent.

    :rtype: None
    """
    _libdhd.dhdSleep(sec)


def startThread():
    # TODO implement startThread
    pass
