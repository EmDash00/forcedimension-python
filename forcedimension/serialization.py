from math import nan
from typing import Any, Literal, Optional

import pydantic

from forcedimension.containers import DefaultVecType
from forcedimension.dhd import Handedness, VelocityEstimatorConfig
from forcedimension.dhd.constants import (
    DEFAULT_TIMEGUARD_US, DEFAULT_VELOCITY_WINDOW, DeviceType,
    VelocityEstimatorMode
)

from forcedimension.drd import (
    DEFAULT_ENC_MOVE_PARAMS,
    DEFAULT_ENC_TRACK_PARAMS,
    DEFAULT_POS_MOVE_PARAMS,
    DEFAULT_POS_TRACK_PARAMS,
    DEFAULT_ROT_MOVE_PARAMS,
    DEFAULT_ROT_TRACK_PARAMS,
    DEFAULT_GRIP_MOVE_PARAMS,
    DEFAULT_GRIP_TRACK_PARAMS,
    TrajectoryGenParams
)

class HapticDeviceSpecs(pydantic.BaseModel):
    name: Optional[str]
    devtype: DeviceType
    serial_number: int = 0
    has_base: bool
    has_wrist: bool
    has_active_wrist: bool
    has_gripper: bool
    has_active_gripper: bool
    num_dof: int
    handedness: Handedness
    is_drd_supported: bool = False

    @pydantic.field_validator('serial_number')
    @classmethod
    def validate_serial_number(cls, val: Optional[int]):
        if val is None:
            return

        if val < 0 and val != -1:
            raise ValueError("serial_number must be greater than 0")


class HapticDeviceConfig(pydantic.BaseModel):
    class GripperConfig(pydantic.BaseModel):
        max_force: Optional[float] = None
        velocity_estimator: VelocityEstimatorConfig = pydantic.Field(
            default_factory=VelocityEstimatorConfig
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
        is_filtering_enabled: bool = True
        is_pos_regulated: bool = True
        is_rot_regulated: bool = True
        is_grip_regulated: bool = True
        max_motor_ratio: float = 1.0
        enc_move_param: TrajectoryGenParams = pydantic.Field(
            default_factory = lambda: TrajectoryGenParams(
                vmax=2e4, amax=2e4, jerk=2e4
            )
        )
        enc_track_param: TrajectoryGenParams = pydantic.Field(
            default_factory = lambda: TrajectoryGenParams(
                vmax=1.0, amax=1.0, jerk=1.0
            )
        )
        pos_move_param: TrajectoryGenParams = pydantic.Field(
            default_factory = lambda: TrajectoryGenParams(
                vmax=1.0, amax=1.0, jerk=1.0
            )
        )
        pos_track_param: TrajectoryGenParams = pydantic.Field(
            default_factory = lambda: TrajectoryGenParams(
                vmax=1.0, amax=1.0, jerk=1.0
            )
        )
        rot_move_param: TrajectoryGenParams = pydantic.Field(
            default_factory = lambda: TrajectoryGenParams(
                vmax=30.0, amax=30.0, jerk=30.0
            )
        )
        rot_track_param: TrajectoryGenParams = pydantic.Field(
            default_factory = lambda: TrajectoryGenParams(
                vmax=30.0, amax=30.0, jerk=30.0
            )
        )
        grip_move_param: TrajectoryGenParams = pydantic.Field(
            default_factory = lambda: TrajectoryGenParams(
                vmax=1.0, amax=1.0, jerk=1.0
            )

        )
        grip_track_param: TrajectoryGenParams = pydantic.Field(
            default_factory = lambda: TrajectoryGenParams(
                vmax=1.0, amax=1.0, jerk=1.0
            )
        )

        @pydantic.field_validator('max_motor_ratio')
        @classmethod
        def validate_motor_ratio_max(cls, val: float):
            if val < 0.0 or val > 1.0:
                raise ValueError("motor_ratio_max must be between 0.0 and 1.0")

    mass: float = 0.0

    is_force_enabled: bool = True
    is_brake_enabled: bool = True
    is_button_emulation_enabled: bool = False
    is_gravity_compensation_enabled: bool = False

    base_angles: DefaultVecType = pydantic.Field(  # type: ignore
        default_factory=DefaultVecType
    )

    com_mode: Literal['async', 'sync', 'virtual', 'network'] = 'async'
    timeguard: int = DEFAULT_TIMEGUARD_US

    linear_velocity_estimator: VelocityEstimatorConfig = pydantic.Field(
        default_factory=VelocityEstimatorConfig
    )
    angular_velocity_estimator: VelocityEstimatorConfig = pydantic.Field(
        default_factory=VelocityEstimatorConfig
    )
    max_force: Optional[float] = None
    max_torque: Optional[float] = None
    standard_gravity: float = 9.81

    gripper: GripperConfig = pydantic.Field(
        default_factory=GripperConfig
    )

    regulator: RegulatorConfig = pydantic.Field(
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
