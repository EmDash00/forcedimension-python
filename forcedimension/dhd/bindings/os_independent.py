"""
.. module::expert
   :platform: Windows, Unix
   :synopsis: libdhd "OS Independent SDK" Python bindings

.. moduleauthor:: Drason Chow <drasonchow@gmail.com>
"""

from forcedimension.dhd.bindings import _libdhd
from ctypes import c_bool, c_byte, c_double

_libdhd.dhdKbHit.argtypes = []
_libdhd.dhdKbHit.restype = c_bool
def kbHit() -> bool: # NOQA
    """
    Check keyboard for a key hit. This function is OS independent.

    :rtype: bool
    :returns: if a key on the keyboard was hit.
    """
    return _libdhd.dhdKbHit()


_libdhd.dhdKbGet.argtypes = []
_libdhd.dhdKbGet.restype = c_byte
def kbGet() -> str: # NOQA
    """
    Retrieve a character from the keyboard. This function is OS independent.

    :rtype: str
    :returns: the character hit on the keyboard.
    """
    return _libdhd.dhdKbGet().decode('utf-8')


_libdhd.dhdGetTime.argtypes = []
_libdhd.dhdGetTime.restype = c_double
def getTime() -> float: # NOQA
    """
    Returns the current value from the high-resolution system counter in [s].
    The resolution of the system counter may be machine-dependent, as it is
    usually derived from one of the CPU clock signals. The time returned,
    however, is guarunteed to be monotonic.

    :rtype: float
    :returns: the current monotonic time in [s] from the high-resolution system
    counter
    """
    return _libdhd.dhdGetTime()


_libdhd.dhdSleep.argtypes = [c_double]
_libdhd.dhd.restype = None
def sleep(sec: float) -> None: # NOQA
    """
    Sleep for a given period of time in [s]. This function is OS independent.

    :rtype: None
    """
    _libdhd.dhdSleep(sec)


def startThread():
    # TODO implement startThread
    pass
