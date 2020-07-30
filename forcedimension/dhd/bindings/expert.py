"""
.. module::expert
   :platform: Windows, Unix
   :synopsis: libdhd "Expert SDK" Python bindings

.. moduleauthor:: Drason Chow <drasonchow@gmail.com>
"""


from typing import Tuple, List, Union, Optional, cast

from ctypes import (
    c_int, c_uint, c_bool, c_byte, c_ushort, c_char_p, c_double
)

from ctypes import byref, POINTER

from forcedimension.dhd.bindings import _libdhd

_libdhd.dhdEnableExpertMode.argtypes = []
_libdhd.dhdEnableExpertMode.restype = c_int
def enableExpertMode() -> int: # NOQA
    """
    Enable expert mode.

    :rtype: int
    :returns: 0 on success, -1 otherwise.
    """
    return _libdhd.dhdEnableExpertMode()


_libdhd.dhdDisableExpertMode.argtypes = []
_libdhd.dhdDisableExpertMode.restype = c_int
def disableExpertMode() -> int: # NOQA
    """
    Enable expert mode.

    :rtype: int
    :returns: 0 on success, -1 otherwise.
    """
    return _libdhd.dhdDisableExpertMode()
