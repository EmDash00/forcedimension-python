import ctypes
from ctypes import ArgumentError, Structure, c_int, pointer
from typing import Any, Callable, Optional

from forcedimension.dhd.constants import ErrorNum
from forcedimension.typing import Pointer, SupportsPtr, c_int_ptr


class Status(Structure):
    """
    Adapts the status array returned by
    :func:`forcedimension.bindings.dhd.getStatus()`
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._ptr = ctypes.cast(pointer(self), c_int_ptr)

    @property
    def ptr(self) -> Pointer[c_int]:
        return self._ptr

    _fields_ = (
        ('power', c_int),
        ('connected', c_int),
        ('started', c_int),
        ('reset', c_int),
        ('idle', c_int),
        ('force', c_int),
        ('brake', c_int),
        ('torque', c_int),
        ('wrist_detected', c_int),
        ('error', c_int),
        ('gravity', c_int),
        ('timeguard', c_int),
        ('wrist_init', c_int),
        ('redundancy', c_int),
        ('forceoffcause', c_int),
        ('locks', c_int),
        ('axis_checked', c_int),
    )

    #: Indicates if the device is powered or not.
    power: int
    "Indicates if the device is powered or not."

    #: Indicates if the device is connected or not.
    connected: int
    "Indicates if the device is connected or not."

    #: Indicates if the device controller is running or not.
    started: int
    "Indicates if the device controller is running or not."

    #: Indicates if the device is in RESET mode or not.
    #: See device modes for details.
    reset: int
    "Indicates if the device controller is in RESET mode or not."

    #: Indicates if the device is in IDLE mode or not.
    #: see device modes for details.
    idle: int
    "Indicates if the device controller is in idle mode or not."

    #: Indicates if the device is in force mode or not.
    #: see device modes for details.
    force: int
    "Indicates if the device controller is in force mode or not."

    #: Indicates if the device is in brake mode or not.
    #: see device modes for details.
    brake: int
    "Indicates if device controller is in break mode or not."

    #: indicates if the torques are active or not when the device is
    #: in force mode. see device modes for details.
    torque: int
    "indicates if torques are active when the device is in force mode."

    #: Indicates if the device has a wrist or not.
    #: see device types for details.
    wrist_detected: int
    "Indicates if the device has a wrist or not."

    #: Indicates if the an error happened on the device controller.
    error: int
    "Indicates if an error happend on the device controller."

    #: Indicates if the gravity compensation option is enabled or not.
    gravity: int
    "Indicates if the gravity compensation option is enabled or not."

    #: Indicates if the TimeGuard feature is enabled or not.
    #: See TimeGuard feature for details.
    timeguard: int
    "Indicates if the TimeGuard option is enabled or not."

    #: Indicates if the device wrist is initialized or not.
    #: See device types for details.
    wrist_init: int
    "Indicates if the device wrist is initialized or not."

    #: The status of the redundant encoder consistency check. For devices
    #: equipped with redundant encoders, a value of 1 indicates that the
    #: redundancy check is successful. A value of 0 is reported otherwise, or
    # if the device does not feature redundant encoders.
    redundancy: int
    """
    Indicates if the redundant encoder check was successful. For devices that
    don't feature redundant encoders, this value is 0.
    """

    #: The event that caused forces to be disabled on the device (the last time
    #: forces were turned off).
    forceoffcause: int
    """
    The event that caused forces to be disabled on the device (the last time
    forces were turned off).
    """

    unknown_status: int  # TODO: figure out what this is


class DHDError(Exception):
    def __init__(
        self, msg: Optional[str] = "An undocumented error has occured."
    ):
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
        feature_seg = (
            "A feature" if feature is None else str(feature)
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


class DHDErrorMemory(DHDError, MemoryError):
    def __init__(self, *args, **kwargs):
        return super().__init__(
            "DHD ran out of memory."
        )


class DHDErrorNotImplemented(DHDError, NotImplementedError):
    def __init__(self, *args, **kwargs):
        return super().__init__(
            "The command or feature is currently not implemented."
        )


class DHDErrorFileNotFound(DHDError, FileNotFoundError):
    def __init__(self, *args, **kwargs):
        return super().__init__()


class DHDErrorDeprecated(DHDError):
    def __init__(self):
        super().__init__(
            "This feature, function, or current device is marked as "
            "deprecated."
        )


class DHDErrorInvalidIndex(DHDError, IndexError):
    def __init__(self):
        super().__init__(
            "An index passed to the function is outside the expected valid "
            "range. "
        )


class DHDErrorArgument(DHDError, ValueError):
    def __init__(self, null=False):
        if not null:
            super().__init__(
                "The function producing this error was passed an invalid or "
                "argument."
            )
        else:
            super().__init__(
                "The function producing this error was passed an unexpected "
                "null pointer argument."
            )


class DHDErrorNullArgument(DHDErrorArgument):
    def __init__(self):
        super().__init__(null=True)


class DHDErrorNoRegulation(DHDError):
    def __init__(self):
        super().__init__(
            "The robotic regulation thread is not running. This only applies "
            "to functions from the robotics SDK."
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
    DHDErrorNotImplemented,
    DHDErrorMemory,
    DHDErrorDeviceNotReady,
    DHDErrorFileNotFound,
    DHDErrorConfiguration,
    DHDErrorInvalidIndex,
    DHDErrorDeprecated,
    DHDErrorNullArgument,
    DHDErrorRedundantFail,
    DHDErrorFeatureNotEnabled,
    DHDErrorDeviceInUse,
    DHDErrorArgument,
    DHDErrorNoRegulation
]


def errno_to_exception(errno: int):
    """
    Convert a DHD error number to an exception.
    """
    return _error[errno]
