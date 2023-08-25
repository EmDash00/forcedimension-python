from math import nan
from typing import Literal, Optional

import pydantic

class TrajectoryGenParams(pydantic.BaseModel):
    vmax: float = nan
    amax: float = nan
    jerk: float = nan

    def __iter__(self):
        yield self.vmax
        yield self.amax
        yield self.jerk

    def pretty_str(
        self, default_vmax: float, default_amax: float, default_jerk: float
    ):
        return (
            f"vmax={self.vmax}"
            f"{' (default)' if self.vmax == default_vmax else ''} "
            f"amax={self.amax}"
            f"{' (default)' if self.amax == default_amax else ''} "
            f"jerk={self.jerk}"
            f"{' (default)' if self.jerk == default_jerk else ''} "
        )

    @pydantic.field_validator('vmax')
    @classmethod
    def validate_vmax(cls, val: Optional[float]):
        if val is None:
            return

        if val < 0:
            raise ValueError("vmax must be greater than or equal to 0")

        return val

    @pydantic.field_validator('amax')
    @classmethod
    def validate_amax(cls, val: Optional[float]):
        if val is None:
            return

        if val < 0:
            raise ValueError("amax must be greater than 0")

        return val

    @pydantic.field_validator('jerk')
    @classmethod
    def validate_jerk(cls, val: Optional[float]):
        if val is None:
            return

        if val < 0:
            raise ValueError("jerk must be greater than 0")

        return val


DEFAULT_ENC_TRACK_PARAMS = TrajectoryGenParams(vmax=1., amax=1., jerk=1.)
DEFAULT_ENC_MOVE_PARAMS = TrajectoryGenParams(vmax=2e4, amax=2e4, jerk=2e4)

DEFAULT_POS_TRACK_PARAMS = TrajectoryGenParams(vmax=1., amax=1., jerk=1.)
DEFAULT_POS_MOVE_PARAMS = TrajectoryGenParams(vmax=1., amax=1., jerk=1.)

DEFAULT_ROT_TRACK_PARAMS = TrajectoryGenParams(vmax=30., amax=30., jerk=30.)
DEFAULT_ROT_MOVE_PARAMS = TrajectoryGenParams(vmax=30., amax=30., jerk=30.)

DEFAULT_GRIP_TRACK_PARAMS = TrajectoryGenParams(vmax=30., amax=30., jerk=30.)
DEFAULT_GRIP_MOVE_PARAMS = TrajectoryGenParams(vmax=30., amax=30., jerk=30.)
