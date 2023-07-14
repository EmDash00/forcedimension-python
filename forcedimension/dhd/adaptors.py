"""
.. module::adaptors
   :platform: Windows, Unix
   :synopsis: pythonic adaptors for libdhd args and returns.

.. moduleauthor:: Ember Chow <emberchow.business@gmail.com>
"""

from typing import NamedTuple, Callable, Optional, Any
from forcedimension.dhd.constants import ErrorNum


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
    #: This flag indicates if the device is powered or not.
    power: int

    #: This flag indicates if the device is connected or not.
    connected: int

    #: This flag indicates if the device controller is running.
    started: int

    #: This flag indicates if the device is in RESET mode or not.
    #: See device modes for details.
    reset: int

    #: This flag indicates if the device is in IDLE mode or not.
    #: See device modes for details.
    idle: int

    #: This flag indicates if the device is in FORCE mode or not.
    #: See device modes for details.
    force: int

    #: This flag indicates if the device is in BRAKE mode or not.
    #: See device modes for details.
    brake: int

    #: This flag indicates if the torques are active or not when the device is
    #: in FORCE mode. See device modes for details.
    torque: int

    #: This flag indicates if the device has a wrist or not.
    #: See device types for details.
    wrist_detected: int

    #: This flag indicates if the an error happened on the device controller.
    error: int

    #: This flag indicates if the gravity compensation option is enabled or
    #: not.
    gravity: int

    #: This flag indicates if the TimeGuard feature is enabled or not.
    #: See TimeGuard feature for details.
    timeguard: int

    #: This flag indicates if the device wrist is initialized or not.
    #: See device types for details.
    wrist_init: int

    #: The status of the redundant encoder consistency check. For devices
    #: equipped with redundant encoders, a value of 1 indicates that the
    #: redundancy check is successful. A value of 0 is reported otherwise, or
    # if the device does not feature redundant encoders.
    redundancy: int

    #: The event that caused forces to be disabled on the device (the last time
    #: forces were turned off).
    forceoffcause: int

    unknown_status: int  # TODO: figure out what this is


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
        err_msg =
        feature_seg = (
            "A particular feature" if feature is None else str(feature)
        )
        id_seg = "" if ID is None else f" on device {ID} "

        return super().__init__(
            f"{feature_seg} is not available{id_seg}because {reason}."
        )


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
            **kwargs
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
            feature=feature,
            **kwargs
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
            feature=feature,
            **kwargs
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
            ID=ID,
            **kwargs
        )


class DHDErrorRedundantFail(DHDError):
    def __init__(self, *, ID: Optional[int] = None, **kwargs):
        if ID is not None:
            spec = f" on device ID {ID}"
        else:
            spec = ""

        super().__init__(
            f"The redundant encoder integrity test failed{spec}",
            **kwargs
        )


class DHDIOError(DHDError, OSError):
    def __init__(
        self,
        *,
        err: str,
        ID: Optional[int] = None,
        op: Optional[Callable[[Any], Any]] = None
    ):
        op_seg = "" if op is None else f"{op} failed."
        id_seg = "" if ID is None else f" occured on device {ID}"

        return super().__init__(f"{op_seg}{err}{id_seg}")


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
            op=op,
            **kwargs
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
            ID=ID,
            **kwargs
        )


class DHDErrorDHCBusy(DHDIOError):
    def __init__(self, *, ID: Optional[int] = None, **kwargs):
        return super().__init__(
            err="The device controller is busy.",
            ID=ID,
            **kwargs
        )


class DHDErrorNoDeviceFound(DHDIOError):
    def __init__(self, **kwargs):
        return super().__init__(
            err="No compatible ForceDimension devices found",
            **kwargs
        )


class DHDErrorDeviceInUse(DHDIOError):
    def __init__(self, *, ID: Optional[int] = None, **kwargs):
        super().__init__(
            err="Open error (because the device is already in use)",
            ID=ID,
            **kwargs
        )


class DHDErrorNoDriverFound(DHDIOError):
    def __init__(self, *, ID: Optional[int] = None, **kwargs):
        return super().__init__(
            err="A required driver is not installed (see device manual for"
                "details)",
            ID=ID,
            **kwargs
        )


class DHDErrorConfiguration(DHDIOError):
    def __init__(self, *, ID: Optional[int] = None, **kwargs):
        super().__init__(
            err="IO error trying to read/write calibration data from "
                "device memory",
            ID=ID,
            **kwargs
        )


class DHDErrorGeometry(DHDError):
    def __init__(self, ID: Optional[int] = None):

        if (ID is not None):
            spec = f"device ID {ID}'s"
        else:
            spec = "the device's"

        return super().__init__(
            f"An error has occured within {spec} geometric model"
        )


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
    """
    Convert a DHD error number to an exception.
    """
    return _error[errno]
