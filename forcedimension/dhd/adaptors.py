import ctypes
from ctypes import Structure, c_int, pointer
from enum import IntEnum
from typing import Any, Callable, Dict, Literal, Optional
import typing

import pydantic
import pydantic_core

from forcedimension.dhd.constants import (
    DEFAULT_VELOCITY_WINDOW, MAX_STATUS,
    ComMode, DeviceType, ErrorNum, VelocityEstimatorMode
)
from forcedimension.typing import ComModeStr, Pointer, c_int_ptr


class VelocityEstimatorConfig(pydantic.BaseModel):
    window_size: int = DEFAULT_VELOCITY_WINDOW
    mode: VelocityEstimatorMode = VelocityEstimatorMode.WINDOWING

    @pydantic.field_validator('window_size')
    @classmethod
    def validate_window(cls, val: Optional[int]):
        if val is None:
            return

        if val < 0:
            raise ValueError("window must be greater than 0")


class Status(Structure):
    """
    Adapts the status array returned by
    :func:`forcedimension.bindings.dhd.getStatus()`
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._ptr = ctypes.cast(pointer(self), c_int_ptr)

    @classmethod
    def __get_pydantic_core_schema__(
        cls, source_type: Any, handler: pydantic.GetCoreSchemaHandler
    ) -> pydantic_core.CoreSchema:
        return pydantic_core.core_schema.no_info_after_validator_function(
            cls, handler(cls.__init__)
        )

    @property
    def ptr(self) -> Pointer[c_int]:
        return self._ptr

    def __len__(self) -> int:
        return MAX_STATUS

    def __getitem__(self, i) -> int:
        return getattr(self, self._fields_[i][0])

    def __iter__(self):
        for field_name, _ in self._fields_:
            yield getattr(self, field_name)

    def __str__(self) -> str:
        return (
            f"Status(power={self.power}, connected={self.connected}, "
            f"started={self.started}, reset={self.reset}, idle={self.idle}, "
            f"force={self.force}, brake={self.brake}, torque={self.torque}, "
            f"wrist_detected={self.wrist_detected}, error={self.error}, "
            f"gravity={self.gravity}, timeguard={self.timeguard}, "
            f"redundancy={self.redundancy}, "
            f"forceoffcause={self.forceoffcause}, locks={self.locks}, "
            f"axis_checked={self.axis_checked})"
        )

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
    "1 if the device controller is in RESET mode, 0 otherwise"

    #: Indicates if the device is in IDLE mode or not.
    #: see device modes for details.
    idle: int
    "1 if the device controller is in idle mode or not, 0 otherwise"

    #: Indicates if the device is in force mode or not.
    #: see device modes for details.
    force: int
    "1 if the device controller is in force mode or not, 0 otherwise"

    #: Indicates if the device is in brake mode or not.
    #: see device modes for details.
    brake: int
    "1 if device controller is in break mode or not, 0 otherwise"

    #: indicates if the torques are active or not when the device is
    #: in force mode. see device modes for details.
    torque: int
    "1 if torques are active when the device is in force mode, 0 otherwise"

    #: Indicates if the device has a wrist or not.
    #: see device types for details.
    wrist_detected: int
    "1 if the device has a wrist, 0 otherwise."

    #: Indicates if the an error happened on the device controller.
    error: int
    "1 if an error happend on the device controller, 0 otherwise"

    #: Indicates if the gravity compensation option is enabled or not.
    gravity: int
    "1 if the gravity compensation option is enabled, 0 otherwise"

    #: Indicates if the TimeGuard feature is enabled or not.
    #: See TimeGuard feature for details.
    timeguard: int
    "1 if the TimeGuard option is enabled, 0 otherwise"

    #: Indicates if the device wrist is initialized or not.
    #: See device types for details.
    wrist_init: int
    "1 if the device wrist is initialized, 0 otherwise."

    #: The status of the redundant encoder consistency check. For devices
    #: equipped with redundant encoders, a value of 1 indicates that the
    #: redundancy check is successful. A value of 0 is reported otherwise, or
    # if the device does not feature redundant encoders.
    redundancy: int
    """
    1 if the redundant encoder check was successful. For devices that
    don't feature redundant encoders, this value is 0.
    """

    #: The event that caused forces to be disabled on the device (the last time
    #: forces were turned off).
    forceoffcause: int
    """
    The event that caused forces to be disabled on the device (the last time
    forces were turned off).
    """

    #: The status of the locks on supported devices. The value can be either
    #: 1 if the locks are engaged, 0 if the locks are disengagned,
    #: or -1 if the status of the locks is unknown.
    locks: int
    """
    The status of the locks on supported devices. The value can be either
    1 if the locks are engaged, 0 if the locks are disengagned, or -1 if the
    status of the locks is unknown.
    """

    #: A bit vector that indicates the validation status of each axis. The
    #: validation status of all device axes can be assessed by calling the
    #: :func:`forcedimension.drd.checkInit()` function in the Force Dimension
    #: Robotic SDK (DRD). Each bit of the status value returned corresponds
    #: to the validation status of the corresponding axis.
    axis_checked: int
    """
    A bit vector that indicates the validation status of each axis. The
    validation status of all device axes can be assessed by calling the
    drd.checkInit() function in the Force Dimension Robotic SDK (DRD). Each bit
    of the status value returned corresponds to a the validation status of the
    corresponding axis:
    """

class Handedness(IntEnum):
    NONE = 0
    LEFT = 1
    RIGHT = 2


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
        op: Optional[str] = None
    ):
        op_seg = "" if op is None else f"{op} failed. "
        id_seg = "" if ID is None else f" occured on device {ID}"

        return super().__init__(f"{op_seg}{err}{id_seg}")


class DHDErrorTimeout(DHDIOError):
    def __init__(
        self,
        *,
        op: Optional[str] = None,
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
            err=(
                "A communication error occured between the host and the "
                "HapticDevice"
            ),
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
            err="No compatible Force Dimension devices found",
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
            err="The firmware or internal configuration health check failed",
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

_com_mode_strs = [
    'sync',
    'async',
    'virtual',
    'network'
]

_com_modes: Dict[ComModeStr, ComMode] = {
    'sync': ComMode.SYNC,
    'async': ComMode.ASYNC,
    'virtual': ComMode.VIRTUAL,
    'network': ComMode.NETWORK,
}

_devtype_strs = {
    DeviceType.NONE: 'None',
    DeviceType.DELTA3: 'DELTA.3',
    DeviceType.OMEGA3: 'OMEGA.3',
    DeviceType.OMEGA6_RIGHT: 'OMEGA.6 right',
    DeviceType.OMEGA6_LEFT: 'OMEGA.6 Left',
    DeviceType.OMEGA7_RIGHT: 'OMEGA.7 Right',
    DeviceType.OMEGA7_LEFT: 'OMEGA.7 Left',
    DeviceType.CONTROLLER: 'CONTROLLER',
    DeviceType.CONTROLLER_HR: 'CONTROLLER HR',
    DeviceType.CUSTOM: 'Custom',
    DeviceType.SIGMA3: 'SIGMA.3',
    DeviceType.SIGMA7_RIGHT: 'SIGMA.7 Right',
    DeviceType.SIGMA7_LEFT: 'SIGMA.7 Left',
    DeviceType.LAMBDA3: 'LAMBDA.3',
    DeviceType.LAMBDA7_RIGHT: 'LAMBDA.7 Right',
    DeviceType.LAMBDA7_LEFT: 'LAMBDA.7 Left',
    DeviceType.FALCON: 'Novint Falcon',
}

_handedness = {
    DeviceType.OMEGA6_RIGHT: Handedness.RIGHT,
    DeviceType.OMEGA6_LEFT: Handedness.LEFT,
    DeviceType.OMEGA7_RIGHT: Handedness.RIGHT,
    DeviceType.OMEGA7_LEFT: Handedness.LEFT,
    DeviceType.SIGMA7_RIGHT: Handedness.RIGHT,
    DeviceType.SIGMA7_LEFT: Handedness.LEFT,
    DeviceType.LAMBDA7_RIGHT: Handedness.RIGHT,
    DeviceType.LAMBDA7_LEFT: Handedness.LEFT,
}

_handedness_str = ['None', 'Left', 'Right']

_estimator_mode_str = {
    VelocityEstimatorMode.WINDOWING: "Windowing"
}

_dof = {
    DeviceType.DELTA3: 3,
    DeviceType.OMEGA3: 3,
    DeviceType.OMEGA6_RIGHT: 6,
    DeviceType.OMEGA6_LEFT: 6,
    DeviceType.OMEGA7_RIGHT: 7,
    DeviceType.OMEGA7_LEFT: 6,
    DeviceType.SIGMA3: 3,
    DeviceType.SIGMA7_RIGHT: 7,
    DeviceType.SIGMA7_LEFT: 7,
    DeviceType.LAMBDA3: 3,
    DeviceType.LAMBDA7_RIGHT: 7,
    DeviceType.LAMBDA7_LEFT: 7,
    DeviceType.FALCON: 3,
}

def com_mode_str(com_mode: int) -> ComModeStr:
    return typing.cast(ComModeStr, _com_mode_strs[com_mode])


def com_mode_from_str(com_mode_str: ComModeStr) -> ComMode:
    return _com_modes[com_mode_str]


def num_dof(devtype: DeviceType) -> int:
    if devtype not in _dof:
        return 0

    return _dof[devtype]

def devtype_str(devtype: DeviceType) -> str:
    return _devtype_strs[devtype]


def handedness(devtype: DeviceType) -> Handedness:
    if devtype in _handedness:
        return _handedness[devtype]

    return Handedness.NONE

def handedness_str(handedness: Handedness) -> str:
    return _handedness_str[handedness]


def velocity_estimator_mode_str(mode: VelocityEstimatorMode):
    return _estimator_mode_str[mode]



def errno_to_exception(errno: int):
    """
    Convert a DHD error number to an exception.
    """
    return _error[errno]
