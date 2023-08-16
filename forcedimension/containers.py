import ctypes
import dataclasses
from array import array
from ctypes import c_double, c_int, c_ushort
from math import nan
from typing import Iterable, List, Literal, NamedTuple, Optional, Tuple

import pydantic

from forcedimension.dhd.constants import (
    DEFAULT_TIMEGUARD_US, DEFAULT_VELOCITY_WINDOW, MAX_DOF,
    VelocityEstimatorMode
)
from forcedimension.typing import (
    IntArray, SupportsPtr, SupportsPtrs3, c_double_ptr, c_int_ptr, c_ushort_ptr
)


class VersionTuple(NamedTuple):
    """
    Adapts the four seperate number return into a single grouped
    NamedTuple.
    """
    major: int
    minor: int
    release: int
    revision: int

    def __str__(self):
        return f"{self.major}.{self.minor}.{self.release}-{self.revision}"


class GripperUpdateOpts(NamedTuple):
    """
    Deprecated. Will be removed in v1.0.0
    """
    enc: float = 1000
    thumb_pos: float = 1000
    finger_pos: float = 1000
    v: float = 4000
    w: float = 4000


class UpdateOpts(NamedTuple):
    """
    Deprecated. Will be removed in v1.0.0
    """
    enc: Optional[float] = 1000
    v: Optional[float] = 4000
    w: Optional[float] = None
    buttons: Optional[float] = 100
    ft: Optional[float] = 4000
    req: Optional[float] = 4000
    gripper: Optional[GripperUpdateOpts] = None


class TrajectoryGenParam(pydantic.BaseModel):
    vmax: float = nan
    amax: float = nan
    jerk: float = nan

    def __iter__(self):
        yield self.vmax
        yield self.amax
        yield self.jerk

    @pydantic.field_validator('vmax')
    @classmethod
    def validate_vmax(cls, val: Optional[float]):
        if val is None:
            return

        if val < 0:
            raise ValueError("vmax must be greater than 0")


    @pydantic.field_validator('amax')
    @classmethod
    def validate_amax(cls, val: Optional[float]):
        if val is None:
            return

        if val < 0:
            raise ValueError("amax must be greater than 0")


    @pydantic.field_validator('jerk')
    @classmethod
    def validate_jerk(cls, val: Optional[float]):
        if val is None:
            return

        if val < 0:
            raise ValueError("jerk must be greater than 0")


class VelocityConfig(pydantic.BaseModel):
    window: int = DEFAULT_VELOCITY_WINDOW
    mode: VelocityEstimatorMode = VelocityEstimatorMode.WINDOWING

    @pydantic.field_validator('window')
    @classmethod
    def validate_window(cls, val: Optional[int]):
        if val is None:
            return

        if val < 0:
            raise ValueError("window must be greater than 0")


class HapticDeviceInfo(pydantic.BaseModel):
    serial_number: Optional[int] = None
    has_base: bool
    has_wrist: bool
    has_active_wrist: bool
    has_gripper: bool
    has_active_gripper: bool
    is_left_handed: bool

    @pydantic.field_validator('serial_number')
    @classmethod
    def validate_serial_number(cls, val: Optional[int]):
        if val is None:
            return

        if val < 0:
            raise ValueError("serial_number must be greater than 0")


class HapticDeviceConfig(pydantic.BaseModel):
    class GripperConfig(pydantic.BaseModel):
        max_force: Optional[float] = None
        velocity_estimator: VelocityConfig = dataclasses.field(
            default_factory=VelocityConfig
        )

        @pydantic.field_validator('max_force')
        @classmethod
        def validate_max_force(cls, val: Optional[float]):
            if val is None:
                return

            if val < 0:
                raise ValueError(
                    "max_force must be None or at least 0 [N]"
                )

    class RegulatorConfig(pydantic.BaseModel):
        is_pos_regulated: bool = False
        is_rot_regulated: bool = False
        is_grip_regulated: bool = False
        motor_ratio_max: float = 1.0
        enc_move_param: TrajectoryGenParam = dataclasses.field(
            default_factory=TrajectoryGenParam
        )
        enc_track_param: TrajectoryGenParam = dataclasses.field(
            default_factory=TrajectoryGenParam
        )
        pos_move_param: TrajectoryGenParam = dataclasses.field(
            default_factory=TrajectoryGenParam
        )
        pos_track_param: TrajectoryGenParam = dataclasses.field(
            default_factory=TrajectoryGenParam
        )
        rot_move_param: TrajectoryGenParam = dataclasses.field(
            default_factory=TrajectoryGenParam
        )
        rot_track_param: TrajectoryGenParam = dataclasses.field(
            default_factory=TrajectoryGenParam
        )
        grip_move_param: TrajectoryGenParam = dataclasses.field(
            default_factory=TrajectoryGenParam
        )
        grip_track_param: TrajectoryGenParam = dataclasses.field(
            default_factory=TrajectoryGenParam
        )

        @pydantic.field_validator('motor_ratio_max')
        @classmethod
        def validate_motor_ratio_max(cls, val: float):
            if val < 0.0 or val > 1.0:
                raise ValueError("motor_ratio_max must be between 0.0 and 1.0")

    is_simulator_enabled: bool = False
    is_force_enabled: bool = True
    is_brake_enabled: bool = False
    is_button_emulation_enabled: bool = True
    is_gravity_compensation_enabled: bool = True
    com_mode: Literal['async', 'sync', 'virtual', 'network'] = 'async'
    timeguard: int = DEFAULT_TIMEGUARD_US
    linear_velocity_estimator: VelocityConfig = dataclasses.field(
        default_factory=VelocityConfig
    )
    angular_velocity_estimator: VelocityConfig = dataclasses.field(
        default_factory=VelocityConfig
    )
    max_force: Optional[float] = None
    max_torque: Optional[float] = None
    standard_gravity: float = 9.81

    gripper: Optional[GripperConfig] = dataclasses.field(
        default_factory=GripperConfig
    )

    regulator: Optional[RegulatorConfig] = dataclasses.field(
        default_factory=RegulatorConfig
    )

    @pydantic.field_validator('timeguard')
    @classmethod
    def validate_timeguard(cls, val: Optional[float]):
        if val is None:
            return

        if val < 0 and val != -1:
            raise ValueError(
                "timeguard must be None, at least 0 us, or -1 "
                "(which sets it to its default value)."
            )

    @pydantic.field_validator('max_force')
    @classmethod
    def validate_max_force(cls, val: Optional[float]):
        if val is None:
            return

        if val < 0:
            raise ValueError(
                "max_force must be None or at least 0 [N]"
            )

    @pydantic.field_validator('max_torque')
    @classmethod
    def validate_max_torque(cls, val: Optional[float]):
        if val is None:
            return

        if val < 0:
            raise ValueError(
                "max_torque must be None or at least 0 [Nm]"
            )

    @pydantic.field_validator('standard_gravity')
    @classmethod
    def validate_standard_gravity(cls, val: float):

        if val < 0:
            raise ValueError(
                "standard_gravity must be at least 0 [m/s^2]"
            )


class Vector3(array, SupportsPtrs3[c_double], SupportsPtr[c_double_ptr]):
    """
    A List[float] providing convience "x", "y", "z" read-write accessor
    properties. This class subclasses List[float]; therefore, for all intents
    and purposes, you can treat it as a list. This allows for it to be
    compatible with Python's standard library functions.
    """
    def __new__(
        cls, initializer: Iterable[float] = (0., 0., 0.)
    ):
        if isinstance(initializer, array):
            return initializer

        arr = super(Vector3, cls).__new__(
            cls, 'd', initializer  # type: ignore
        )

        if len(arr) != 3:
            raise ValueError()

        return arr

    def __init__(self, *args, **kwargs):
        ptr = self.buffer_info()[0]
        self._ptrs = (
            ctypes.cast(ptr, c_double_ptr),
            ctypes.cast(ptr + self.itemsize, c_double_ptr),
            ctypes.cast(ptr + 2 * self.itemsize, c_double_ptr),
        )

    @property
    def ptr(self) -> c_double_ptr:
        return self._ptrs[0]

    @property
    def ptrs(self) -> Tuple[c_double_ptr, c_double_ptr, c_double_ptr]:
        return self._ptrs

    @property
    def x(self) -> float:
        return self[0]

    @x.setter
    def x(self, value: float):
        self[0] = value

    @property
    def y(self) -> float:
        return self[1]

    @y.setter
    def y(self, value: float):
        self[1] = value

    @property
    def z(self) -> float:
        return self[2]

    @z.setter
    def z(self, value: float):
        self[2] = value


class Enc3(array, SupportsPtrs3[c_int], SupportsPtr[c_int]):
    def __new__(
        cls, initializer: Iterable[float] = (0., 0., 0.)
    ):
        if isinstance(initializer, array):
            return initializer

        arr = super(Enc3, cls).__new__(
            cls, 'd', initializer  # type: ignore
        )

        if len(arr) != 3:
            raise ValueError()

        return arr

    def __init__(self, *args, **kwargs):
        ptr = self.buffer_info()[0]
        self._ptrs = (
            ctypes.cast(ptr, c_int_ptr),
            ctypes.cast(ptr + self.itemsize, c_int_ptr),
            ctypes.cast(ptr + 2 * self.itemsize, c_int_ptr),
        )

    @property
    def ptr(self) -> c_int_ptr:
        return self._ptrs[0]

    @property
    def ptrs(self) -> Tuple[c_int_ptr, c_int_ptr, c_int_ptr]:
        return self._ptrs


class Enc4(array, SupportsPtr[c_int]):
    def __new__(
        cls, initializer: Iterable[float] = (0., 0., 0., 0.)
    ):
        if isinstance(initializer, array):
            return initializer

        arr = super(Enc4, cls).__new__(
            cls, 'd', initializer  # type: ignore
        )

        if len(arr) != 4:
            raise ValueError()

        return arr

    def __init__(self, *args, **kwargs):
        self._ptr = ctypes.cast(self.buffer_info()[0], c_int_ptr)

    @property
    def ptr(self) -> c_int_ptr:
        return self._ptr


class DOFEncs(array, SupportsPtr[c_int]):
    """
    A List[float] providing convience "x", "y", "z" read-write accessor
    properties. This class subclasses List[float]; therefore, for all intents
    and purposes, you can treat it as a list. This allows for it to be
    compatible with Python's standard library functions.
    """
    def __new__(
        cls, initializer: Iterable[int] = (0 for _ in range(MAX_DOF))
    ):
        arr = super(DOFEncs, cls).__new__(
            cls, 'i', initializer  # type: ignore
        )

        if len(arr) != MAX_DOF:
            raise ValueError()

        return arr

    def __init__(self):
        self._ptr = ctypes.cast(self.buffer_info()[0], c_int_ptr)
        self._delta = Enc3(self[:3])
        self._wrist = Enc3(self[3:6])
        self._wrist_grip = Enc4(self[3:7])
        self._gripper = ctypes.cast(
            self.buffer_info()[0] + self.itemsize * (MAX_DOF - 1), c_int_ptr
        ).contents

    @property
    def ptr(self) -> c_int_ptr:
        return self._ptr

    @property
    def delta(self) -> Enc3:
        return self._delta

    @property
    def wrist(self) -> Enc3:
        return self._wrist

    @property
    def wrist_grip(self) -> Enc4:
        return self._wrist_grip

    @property
    def gripper(self) -> c_int:
        return self._gripper


class DOFMotorArray(array, SupportsPtr[c_ushort]):
    """
    A List[float] providing convience "x", "y", "z" read-write accessor
    properties. This class subclasses List[float]; therefore, for all intents
    and purposes, you can treat it as a list. This allows for it to be
    compatible with Python's standard library functions.
    """
    def __new__(
        cls, initializer: Iterable[int] = (0 for _ in range(MAX_DOF))
    ):
        arr = super(DOFMotorArray, cls).__new__(
            cls, 'H', initializer  # type: ignore
        )

        if len(arr) != MAX_DOF:
            raise ValueError()

        return arr

    def __init__(self):
        self._ptr = ctypes.cast(self.buffer_info()[0], c_ushort_ptr)

    @property
    def ptr(self) -> c_ushort_ptr:
        return self._ptr


class DOFJointAngles(array, SupportsPtr[c_double]):
    """
    A List[float] providing convience "x", "y", "z" read-write accessor
    properties. This class subclasses List[float]; therefore, for all intents
    and purposes, you can treat it as a list. This allows for it to be
    compatible with Python's standard library functions.
    """
    def __new__(
        cls, initializer: Iterable[float] = (0 for _ in range(MAX_DOF))
    ):
        arr = super(DOFJointAngles, cls).__new__(
            cls, 'd', initializer  # type: ignore
        )

        if len(arr) != MAX_DOF:
            raise ValueError()

        return arr

    def __init__(self):
        self._ptr = ctypes.cast(self.buffer_info()[0], c_double_ptr)

        self._delta = Vector3(self[:3])
        self._wrist = Vector3(self[3:6])
        self._gripper = self._gripper = ctypes.cast(
            self.buffer_info()[0] + 7 * self.itemsize, c_double_ptr
        ).contents

    @property
    def ptr(self) -> c_double_ptr:
        return self._ptr

    @property
    def delta(self) -> Vector3:
        return self._delta

    @property
    def wrist(self) -> Vector3:
        return self._wrist

    @property
    def gripper(self) -> c_double:
        return self._gripper


class Mat3x3(List[array], SupportsPtr[c_double]):
    """
    Represents the type of a coordinate frame matrix.
    """

    def __init__(
        self, iterable: Iterable[float] = (0. for _ in range(MAX_DOF))
    ):
        self._arr = array('d', iterable)
        if len(self._arr) != 9:
            raise ValueError()

        self._ptr = ctypes.cast(self._arr.buffer_info()[0], c_double_ptr)
        super().__init__(
            self._arr[i:i + 3] for i in range(3)
        )

    @property
    def ptr(self) -> c_double_ptr:
        return self._ptr


class Mat6x6(List[array], SupportsPtr[c_double]):
    """
    Represents the type of a coordinate frame matrix.
    """

    def __init__(
        self, iterable: Iterable[float] = (0. for _ in range(36))
    ):
        self._arr = array('d', iterable)
        if len(self._arr) != 36:
            raise ValueError()

        self._ptr = ctypes.cast(self._arr.buffer_info()[0], c_double_ptr)
        super().__init__(
            self._arr[i:i + 6] for i in range(6)
        )

    @property
    def ptr(self) -> c_double_ptr:
        return self._ptr


DefaultVecType = Vector3
DefaultEnc3Type = Enc3
DefaultEnc4Type = Enc4
DefaultDOFEncsType = DOFEncs
DefaultDOFJointAnglesType = DOFJointAngles
DefaultMat3x3Type = Mat3x3
DefaultMat6x6Type = Mat6x6


try:
    import numpy as np
    import numpy.typing as npt

    from forcedimension.typing import FloatArray

    class NumpyVector3(
        np.ndarray, SupportsPtrs3[c_double], SupportsPtr[c_double]
    ):
        """
        A view over a numpy ndarry, which provides convenience "x", "y", and "z"
        read-write accessor properties. This class subclasses ndarray;
        therefore, for all intents and purposes you can treat it as an ndarray.
        This allows you to simply pass in this class to any and all numpy
        methods.
        """
        def __new__(cls, data: npt.ArrayLike = (0., 0., 0.)):
            arr = np.ascontiguousarray(data, dtype=c_double).view(cls)

            if len(arr) != 3:
                raise ValueError

            return arr

        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            self._ptrs = (
                ctypes.cast(self.ctypes.data, c_double_ptr),
                ctypes.cast(self.ctypes.data + self.itemsize, c_double_ptr),
                ctypes.cast(self.ctypes.data + 2 * self.itemsize, c_double_ptr)
            )

        @property
        def ptr(self) -> c_double_ptr:
            return self._ptrs[0]

        @property
        def ptrs(self) -> Tuple[c_double_ptr, c_double_ptr, c_double_ptr]:
            return self._ptrs

        @property
        def x(self) -> float:
            return self[0]

        @x.setter
        def x(self, value: float):
            self[0] = value

        @property
        def y(self) -> c_double:
            return self[1]

        @y.setter
        def y(self, value: float):
            self[1] = value

        @property
        def z(self) -> c_double:
            return self[2]

        @z.setter
        def z(self, value: float):
            self[2] = value

    class NumpyEnc3(np.ndarray, SupportsPtr[c_int], SupportsPtrs3[c_int]):
        """
        A view over a numpy ndarry, which provides convenience "x", "y", and "z"
        read-write accessor properties. This class subclasses ndarray;
        therefore, for all intents and purposes you can treat it as an ndarray.
        This allows you to simply pass in this class to any and all numpy
        methods.
        """
        def __new__(cls, data: npt.ArrayLike = (0., 0., 0.)):
            arr = np.ascontiguousarray(data, dtype=c_int).view(cls)

            if len(arr) != 3:
                raise ValueError

            return arr

        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            self._ptrs = (
                ctypes.cast(self.ctypes.data, c_int_ptr),
                ctypes.cast(self.ctypes.data + self.itemsize, c_int_ptr),
                ctypes.cast(self.ctypes.data + 2 * self.itemsize, c_int_ptr)
            )

        @property
        def ptr(self) -> c_int_ptr:
            return self._ptrs[0]

        @property
        def ptrs(self) -> Tuple[c_int_ptr, c_int_ptr, c_int_ptr]:
            return self._ptrs

    class NumpyEnc4(np.ndarray, SupportsPtr[c_int]):
        """
        A view over a numpy ndarry, which provides convenience "x", "y", and "z"
        read-write accessor properties. This class subclasses ndarray;
        therefore, for all intents and purposes you can treat it as an ndarray.
        This allows you to simply pass in this class to any and all numpy
        methods.
        """
        def __new__(cls, data: npt.ArrayLike = (0., 0., 0., 0.)):
            arr = np.ascontiguousarray(data, dtype=c_int).view(cls)

            if len(arr) != 4:
                raise ValueError

            return arr

        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            self._ptr = ctypes.cast(self.ctypes.data, c_int_ptr)

        @property
        def ptr(self) -> c_int_ptr:
            return self._ptr

    class NumpyDOFEncs(np.ndarray, SupportsPtr[c_int]):
        def __new__(cls, data: IntArray = tuple(0 for _ in range(MAX_DOF))):
            arr = np.ascontiguousarray(data, dtype=c_int).view(cls)

            if len(arr) != MAX_DOF:
                raise ValueError()

            return arr

        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)

            self._ptr = ctypes.cast(self.ctypes.data, c_int_ptr)
            self._delta = NumpyEnc3(self[:3])
            self._wrist = NumpyEnc3(self[3:6])
            self._wrist_grip = NumpyEnc4(self[3:7])

            self._gripper = ctypes.cast(
                self.ctypes.data + 7 * self.itemsize, c_int_ptr
            ).contents

        @property
        def ptr(self) -> c_int_ptr:
            return self._ptr

        @property
        def delta(self) -> NumpyEnc3:
            return self._delta

        @property
        def wrist(self) -> NumpyEnc3:
            return self._wrist

        @property
        def wrist_grip(self) -> NumpyEnc4:
            return self._wrist_grip

        @property
        def gripper(self) -> c_int:
            return self._gripper

    class NumpyDOFMotorArray(np.ndarray, SupportsPtr[c_ushort]):
        def __new__(cls, data: IntArray = tuple(0 for _ in range(MAX_DOF))):
            arr = np.ascontiguousarray(data, dtype=c_ushort).view(cls)

            if len(arr) != MAX_DOF:
                raise ValueError()

            return arr

        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            self._ptr = ctypes.cast(self.ctypes.data, c_ushort_ptr)

        @property
        def ptr(self) -> c_ushort_ptr:
            return self._ptr

    class NumpyDOFJointAngles(np.ndarray, SupportsPtr[c_double]):
        def __new__(
            cls, data: npt.ArrayLike = tuple(0. for _ in range(MAX_DOF))
        ):
            arr = np.ascontiguousarray(data, dtype=c_double).view(cls)

            if len(arr) != MAX_DOF:
                raise ValueError()

            return arr

        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            self._ptr = ctypes.cast(self.ctypes.data, c_double_ptr)

            self._delta = NumpyVector3(self[:3])
            self._wrist = NumpyVector3(self[3:6])
            self._gripper = ctypes.cast(
                self.ctypes.data + 7 * self.itemsize, c_double_ptr
            ).contents

        @property
        def ptr(self) -> c_double_ptr:
            return self._ptr

        @property
        def delta(self) -> NumpyVector3:
            return self._delta

        @property
        def wrist(self) -> NumpyVector3:
            return self._wrist

        @property
        def gripper(self) -> c_double:
            return self._gripper

    class NumpyMat3x3(np.ndarray, SupportsPtr[c_double]):
        """
        A view over a numpy ndarry, which provides convenience "x", "y", and "z"
        read-write accessor properties. This class subclasses ndarray;
        therefore, for all intents and purposes you can treat it as an ndarray.
        This allows you to simply pass in this class to any and all numpy
        methods.
        """
        def __new__(cls, data: npt.ArrayLike = tuple(0. for _ in range(9))):
            arr = np.ascontiguousarray(data, dtype=c_double)

            if len(arr) != 9:
                raise ValueError()

            return arr.reshape((3, 3)).view(cls)

        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            self._ptr = ctypes.cast(self.ctypes.data, c_double_ptr)

        @property
        def ptr(self) -> c_double_ptr:
            return self._ptr

    class NumpyMat6x6(np.ndarray, SupportsPtr[c_double]):
        """
        A view over a numpy ndarry, which provides convenience "x", "y", and "z"
        read-write accessor properties. This class subclasses ndarray;
        therefore, for all intents and purposes you can treat it as an ndarray.
        This allows you to simply pass in this class to any and all numpy
        methods.
        """
        def __new__(cls, data: npt.ArrayLike = tuple(0. for _ in range(36))):
            arr = np.ascontiguousarray(data, dtype=c_double)

            if len(arr) != 36:
                raise ValueError()

            return arr.reshape((6, 6)).view(cls)

        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            self._ptr = ctypes.cast(self.ctypes.data, c_double_ptr)

        @property
        def ptr(self) -> c_double_ptr:
            return self._ptr

    DefaultVecType = NumpyVector3
    DefaultEnc3Type = NumpyEnc3
    DefaultEnc4Type = NumpyEnc4
    DefaultDOFEncsType = NumpyDOFEncs
    DefaultDOFJointAnglesType = NumpyDOFJointAngles
    DefaultMat3x3Type = NumpyMat3x3
    DefaultMat6x6Type = NumpyMat6x6
except ImportError:
    pass
