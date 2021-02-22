"""
.. module::adaptors
   :platform: Windows, Unix
   :synopsis: pythonic adaptors for libdhd args and returns.

.. moduleauthor:: Drason Chow <drasonchow@gmail.com>
"""

from typing import NamedTuple, Callable, Optional, Any
from forcedimension.dhd.bindings.constants import ErrorNum


class VersionTuple(NamedTuple):
    """
    Adapts the four seperate number return into a single grouped
    NamedTuple.
    """
    major: int
    minor: int
    release: int
    revision: int


class StatusTuple(NamedTuple):
    """
    Named tuple adapting the status array returned by
    forcedimension.bindings.dhd.getStatus()
    """
    power: int
    connected: int
    started: int
    reset: int
    idle: int
    force: int
    brake: int
    torque: int
    wrist_detected: int
    error: int
    gravity: int
    timeguard: int
    wrist_init: int
    redundancy: int
    forceoffcause: int
    unknown_status: int  # TODO: figure out what this is


class CartesianTuple(NamedTuple):
    """
    Named tuple adapting an immutable triple of (x, y, z) floats.

    Used to meaningfully talk about 3-Vectors.
    """
    x: float
    y: float
    z: float


class DeviceTuple(NamedTuple):
    """
    Named tuple adapting an immutable triple of (axis0, axis1, axis2) ints.

    Used to meaningfully talk about device motors or encoders as a group.
    """
    axis0: int
    axis1: int
    axis2: int


class DOFTuple(NamedTuple):
    """
    Named tuple adapting a way to specify an int assignment to the device
    associated with that degree-of-freedom up to but not including
    dhd.bindings.MAX_DOF
    """
    dof0: int
    dof1: int
    dof2: int
    dof3: int
    dof4: int
    dof5: int
    dof6: int
    dof7: int


class DHDError(Exception):
    def __init__(self, msg: Optional[str] = "Undocumented error."):
        if msg is not None:
            return super().__init__(msg)
        else:
            return super().__init__()


class DHDFeatureError(DHDError):
    def __init__(
        self,
        *,
        reason: str,
        ID: Optional[int] = None,
        feature: Optional[Callable[[Any], Any]]
    ):
        err_msg = "{} is not available {} because {}."
        feature_seg = "A particular feature" if feature is None else feature
        id_seg = "" if ID is None else "on device {}".format(ID)

        return super().__init__(err_msg.format(feature_seg, id_seg, reason))


class DHDErrorExpertModeDisabled(DHDFeatureError):
    def __init__(
        self,
        *,
        feature: Optional[Callable[[Any], Any]] = None,
        **kwargs
    ):
        return super().__init__(
            reason="expert mode is disabled",
            ID=None,
            feature=feature,
        )


class DHDErrorFeatureNotAvailable(DHDFeatureError):
    def __init__(
        self,
        *,
        feature: Optional[Callable[[Any], Any]],
        ID: Optional[int] = None,
        **kwargs
    ):

        return super().__init__(
            reason="it is not supported on this device",
            ID=ID,
            feature=feature
        )


class DHDErrorFeatureNotEnabled(DHDFeatureError):
    def __init__(
        self,
        *,
        feature: Optional[Callable[[Any], Any]] = None,
        ID: Optional[int] = None,
        **kwargs
    ):

        return super().__init__(
            reason="it was previously disabled for this device",
            ID=ID,
            feature=feature
        )


class DHDErrorDeviceNotReady(DHDFeatureError):
    def __init__(
        self,
        *,
        feature: Optional[Callable[[Any], Any]],
        ID: Optional[int] = None,
        **kwargs
    ):
        return super().__init__(
            reason="the device isn't ready to proccess a new command",
            feature=feature,
            ID=ID
        )


class DHDErrorRedundantFail(DHDError):
    def __init__(self, *, ID: Optional[int] = None, **kwargs):
        if ID is not None:
            spec = " on device ID {}".format(ID)
        else:
            spec = ""

        super().__init__("The redundant encoder integrity "
                         "test failed{}.".format(spec))


class DHDIOError(DHDError, OSError):
    def __init__(
        self,
        *,
        err: str,
        ID: Optional[int] = None,
        op: Optional[Callable[[Any], Any]] = None
    ):
        err_msg = "{}{}{}."
        op_seg = "" if op is None else "{} failed.".format(op)
        id_seg = "" if ID is None else " occured on device {}".format(ID)

        return super().__init__(err_msg.format(op_seg, err, id_seg))


class DHDErrorTimeout(DHDIOError):
    def __init__(
        self,
        *,
        op: Optional[Callable[[Any], Any]] = None,
        ID: Optional[int] = None,
        **kwargs
    ):

        return super().__init__(
            err="timeout",
            ID=ID,
            op=op
        )


class DHDErrorCom(DHDIOError):
    def __init__(
        self,
        *,
        ID: Optional[int] = None,
        **kwargs
    ):
        return super().__init__(
            err="A communication error between the host and the HapticDevice",
            ID=ID
        )


class DHDErrorDHCBusy(DHDIOError):
    def __init__(self, *, ID: Optional[int] = None, **kwargs):
        return super().__init__(
            err="The device controller is busy.",
            ID=ID
        )


class DHDErrorNoDeviceFound(DHDIOError):
    def __init__(self, **kwargs):
        return super().__init__(
            err="No compatible ForceDimension devices found"
        )


class DHDErrorDeviceInUse(DHDIOError):
    def __init__(self, *, ID: Optional[int] = None, **kwargs):
        super().__init__(
            err="Open error (because the device is already in use)",
            ID=ID
        )


class DHDErrorNoDriverFound(DHDIOError):
    def __init__(self, *, ID: Optional[int] = None, **kwargs):
        return super().__init__(
            err="A required driver is not installed (see device manual for"
                "details)",
            ID=ID)


class DHDErrorConfiguration(DHDIOError):
    def __init__(self, *, ID: Optional[int] = None, **kwargs):
        super().__init__(
            err="IO error trying to read/write calibration data from "
                "device memory",
            ID=ID
        )


class DHDErrorGeometry(DHDError):
    def __init__(self, ID: Optional[int] = None):

        if (ID is not None):
            spec = "device ID {}".format(ID)
        else:
            spec = "the device"

        return super().__init__("An error has occured within {} "
                                "geometric model".format(spec))


_error = [
    None,
    DHDError,
    DHDErrorCom,
    DHDErrorDHCBusy,
    DHDErrorNoDriverFound,
    DHDErrorNoDeviceFound,
    DHDErrorFeatureNotAvailable,
    DHDErrorTimeout,
    DHDErrorGeometry,
    DHDErrorExpertModeDisabled,
    NotImplementedError,
    MemoryError,
    DHDErrorDeviceNotReady,
    FileNotFoundError,
    DHDErrorConfiguration,
    ValueError,
    DHDErrorRedundantFail,
    DHDErrorFeatureNotEnabled,
    DHDErrorDeviceInUse
]


def errno_to_exception(errno: ErrorNum):
    return _error[errno]
