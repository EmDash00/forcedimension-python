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
    def __init__(self, msg="Undocumented error.", **kwargs):
        return super().__init__(msg)


class DHDErrorCom(IOError):
    def __init__(self, **kwargs):
        return super().__init__("Communication error between the HapticDevice "
                                "and the host computer.")


class DHDErrorDHCBusy(IOError):
    def __init__(self, **kwargs):
        return super().__init__("The device controller is busy and cannot "
                                "perform the required task")


class DHDErrorNoDriverFound(IOError):
    def __init__(self, ID: Optional[int] = None, **kwargs):
        if (ID is not None):
            specification = " for device ID {} ".format(ID)
        else:
            specification = " "

        return super().__init__("A required device driver{}is not installed, "
                                "please refer to your user manual's "
                                "instalation section".format(specification))


class DHDErrorNoDeviceFound(IOError):
    def __init__(self, **kwargs):
        return super().__init__("No compatible force dimension device was "
                                "found. ")


class DHDErrorNotAvailable(Exception):
    def __init__(
        self,
        feature: Optional[Callable[[Any], Any]],
        ID: Optional[int] = None,
        **kwargs
    ):

        if feature is not None:
            feature_str = "{}".format(feature)
        else:
            feature_str = "The requested feature"

        if ID is not None:
            spec = "device ID {}".format(ID)
        else:
            spec = "the ."

        return super().__init__("{} is not avaiable for "
                                "{}.".format(feature_str, spec))


class DHDErrorTimeout(IOError):
    def __init__(
        self,
        operation: Optional[Callable[[Any], Any]] = None,
        ID: Optional[int] = None,
        **kwargs
    ):
        if (operation is not None):
            op_str = "{}".format(operation)
        else:
            op_str = "The operation"

        if (ID is not None):
            spec = " on device ID {}".format(ID)
        else:
            spec = ""

        return super().__init__("{} has timed out{}.".format(op_str, spec))


class DHDErrorGeometry(IOError):
    def __init__(self, ID: Optional[int] = None, **kwargs):

        if (ID is not None):
            spec = "device ID {}".format(ID)
        else:
            spec = "the device"

        return super().__init__("An error has occured within {} "
                                "geometric model".format(spec))


class DHDErrorExpertModeDisabled(Exception):
    def __init__(self,
                 feature: Optional[Callable[[Any], Any]] = None,
                 **kwargs):
        return super().__init__("{} is not available because expert mode is "
                                "disabled.".format(feature))


class DHDErrorDeviceNotReady(Exception):
    def __init__(
        self,
        cmd: Optional[Callable[[Any], Any]],
        ID: Optional[int] = None,
        **kwargs
    ):

        if cmd is not None:
            cmd_str = "The {} feature".format(cmd)
        else:
            cmd_str = "The requested command"

        if ID is not None:
            spec = "device ID {} ".format(ID)
        else:
            spec = "the device"

        return super().__init__("{} is not ready to process on "
                                "{}.".format(cmd_str, spec))


class DHDErrorConfiguration(IOError):
    def __init__(self, ID: Optional[int] = None, **kwargs):
        if ID is not None:
            spec = "device ID {}'s ".format(ID)
        else:
            spec = "the device"

        super().__init__("There was an error trying to read/write the "
                         "calibration data into {} memory".format(spec))


class DHDErrorRedundantFail(Exception):
    def __init__(self, ID: Optional[int] = None, **kwargs):
        if ID is not None:
            spec = " on device ID {}".format(ID)
        else:
            spec = ""

        super().__init__("The redundant encoder integrity "
                         "test failed{}.".format(spec))


class DHDErrorNotEnabled(Exception):
    def __init__(
        self,
        cmd: Optional[Callable[[Any], Any]],
        ID: Optional[int] = None,
        **kwargs
    ):

        if cmd is not None:
            cmd_str = "The {} feature".format(cmd)
        else:
            cmd_str = "A particular feature"

        if ID is not None:
            spec = "device ID {} ".format(ID)
        else:
            spec = "the device."

        return super().__init__("{} is not enabled on "
                                "{}.".format(cmd_str, spec))


class DHDErrorDeviceInUse(IOError):
    def __init__(self, ID: Optional[int] = None, **kwargs):
        if ID is not None:
            spec = "Device ID {}".format(ID)
        else:
            spec = "The device"

        super().__init__("{} is already in use".format(spec))


_error = [
    None,
    DHDError,
    DHDErrorCom,
    DHDErrorDHCBusy,
    DHDErrorNoDriverFound,
    DHDErrorNoDeviceFound,
    DHDErrorNotAvailable,
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
    DHDErrorNotEnabled,
    DHDErrorDeviceInUse
]


def errno_to_exception(errno: ErrorNum):
    return _error[errno]
