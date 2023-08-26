__version__ = '0.2.0'

import ctypes as ct
from enum import Enum
import math
import time
import json
from math import nan
import textwrap
from threading import Condition, Lock, Thread, Event
from typing import Callable, Generic, List, Optional, Type, TypeVar
from typing import cast as _cast
import warnings
from copy import copy
import yaml

import forcedimension.containers as containers
import forcedimension.dhd as dhd
from forcedimension.dhd.adaptors import Handedness
import forcedimension.drd as drd
import forcedimension.util as __util
from forcedimension.containers import (
    DefaultDOFEncsType, DefaultDOFJointAnglesType, DefaultEnc3Type,
    DefaultMat3x3Type, DefaultMat6x6Type, DefaultVecType,
    GripperUpdateOpts, UpdateOpts, _HapticPollerOptions
)
from forcedimension.serialization import (
    HapticDeviceSpecs, HapticDeviceConfig, TrajectoryGenParams
)
from forcedimension.dhd import ErrorNum, Status
from forcedimension.typing import FloatVectorLike, IntVectorLike
from forcedimension.util import ImmutableWrapper


dhd.expert.enableExpertMode()

class HapticDevice:
    """
    A HapticDevice is a high-level wrapper for any compatible Force Dimension
    device.
    """


    class Gripper:
        """
        A high-level wrapper for methods and kinematic data of a Gripper on a
        HapticDevice.

        Certain kinds of HapticDevices opened will have grippers. If that is the
        case, a Gripper object will be instantiated as well containing methods to
        get kinematic information about the Gripper.
        """

        def __init__(
            self,
            parent,
            config_file_data: Optional[Dict[str, Any]],
            config_data: Optional[Dict[str, Any]],
            restore: bool
        ):
            self._parent: HapticDevice = parent
            self._config = self._parent._config.gripper
            self._init_config(config_file_data, config_data, restore)

            self._id = parent.ID
            self._enc = ct.c_int()
            self._angle = ct.c_double()
            self._gap = ct.c_double()
            self._v = ct.c_double()
            self._w = ct.c_double()
            self._fg = ct.c_double()

            self._thumb_pos = containers.Vector3()
            self._finger_pos = containers.Vector3()

            self._thumb_pos_view = ImmutableWrapper(self._thumb_pos)
            self._finger_pos_view = ImmutableWrapper(self._finger_pos)

            self._fg_req: float = 0.0

            self._init_config(config_file_data, config_data, restore)

        def _init_velocity_estimator(self, restore: bool):
            if restore:
                self.config_velocity()

        def _init_max_gripper_force(self, restore: bool):
            if restore:
                self.set_max_force(None)
                return

            limit = dhd.getMaxGripperForce(self._id)

            if limit < 0:
                limit = None

            self._config.max_force = limit

        def _init_config(
            self,
            config_file_data: Optional[Dict[str, Any]],
            config_data: Optional[Dict[str, Any]],
            restore: bool
        ):

            unset_configs = {
               'velocity_estimator'
            }

            if config_file_data is not None and config_data is not None:
                preserved_keys = set(
                    set(config_file_data.keys()) ^ set(config_data.keys())
                )
                overwritten_keys = set(
                    set(config_file_data.keys()) & set(config_data.keys())
                )

                unset_configs -= preserved_keys
                unset_configs -= overwritten_keys

                for key in preserved_keys:
                    HapticDevice.Gripper._config_setters[key](
                        self, config_file_data[key]
                    )

                for key in overwritten_keys:
                    HapticDevice.Gripper._config_setters[key](
                        self, config_data[key]
                    )
            elif config_file_data is not None and config_data is None:
                for key, value in config_file_data:
                    HapticDevice.Gripper._config_setters[key](value)
            elif config_file_data is None and config_data is not None:
                for key, value in config_data:
                    HapticDevice.Gripper._config_setters[key](value)

            # Default initialize all unset configs.

            for config in unset_configs:
                HapticDevice.Gripper._config_initializers[config](
                    self, restore
                )

        def req(self, fg: float):
            self._fg_req = fg

        def config_velocity(
            self,
            window_size: int = dhd.DEFAULT_VELOCITY_WINDOW,
            mode: dhd.VelocityEstimatorMode = dhd.VelocityEstimatorMode.WINDOWING
        ):
            """
            Configures the internal shared linear and angular velocity
            estimator used by the Force Dimension SDK for the force gripper.
            Calling this without parameters resets the linear velocity
            estimator to its default settings.

            :param window_size int:
                Time interval to use for computing linear velocity (in [ms]).

            :param VelocityEstimatorMode mode:
                Velocity estimator mode. Currently only
                :data:`forcedimension.dhd.VelocityEstimatorMode.WINDOWING` is
            """

            if dhd.configGripperVelocity(window_size, mode, self._id):
                raise dhd.errno_to_exception(dhd.errorGetLast())(
                    op='forcedimension.dhd.configGripperVelocity()',
                    ID=self._id
                )

            self._config.velocity_estimator.window_size = window_size
            self._config.velocity_estimator.mode = mode

        def set_max_force(self, limit: Optional[float]):
            """
            Define or disable a limit (in [N]) to the force magnitude that can be
            applied by the haptic device.

            :param Optional[float] limit:
                The desired limit (in [N]) to the force magnitude that can be
                applied. If the limit is `None`, the force limit is disabled.
            """

            if limit is not None:
                if limit < 0:
                    raise ValueError("limit must be greater than 0 or None.")

            if limit is None:
                limit = -1.0

            if dhd.setMaxGripperForce(limit, self._id):
                raise dhd.errno_to_exception(dhd.errorGetLast())()

            self._config.max_force = limit

        @property
        def thumb_pos(self) -> containers.Vector3:
            """
            Provides a read-only reference to the last-known position of the
            thumb rest position (in [m]) of the gripper about the X, Y, and Z
            axes.

            :raises DHDError:
                If an error has occured with the device, invalidating the
                device state.
            """

            self.check_exception()
            return _cast(containers.Vector3, self._thumb_pos_view)

        @property
        def finger_pos(self) -> containers.Vector3:
            """
            Provides a read-only reference to the last-known position of the
            forefinger rest position of the gripper (in [m]) about the X, Y,
            and Z axes.

            :raises DHDError:
                If an error has occured with the device, invalidating the
                device state.
            """

            self.check_exception()
            return _cast(containers.Vector3, self._finger_pos_view)

        @property
        def gap(self) -> float:
            """
            Provides a read-only copy of the last-known gripper opening distance
            (in [m]).

            :raises DHDError:
                If an error has occured with the device, invalidating the
                device state.
            """

            self.check_exception()
            return self._gap.value

        @property
        def angle(self) -> float:
            """
            Provides a read-only copy of the last-known gripper opening angle
            (in [rad]).

            :raises DHDError:
                If an error has occured with the device, invalidating the
                device state.
            """

            self.check_exception()
            return self._angle.value

        @property
        def v(self) -> float:
            """
            Provides a read-only copy of the last-known linear velocity of the
            gripper (in [m/s]).

            :raises DHDError:
                If an error has occured with the device, invalidating the
                device state.
            """

            self.check_exception()
            return self._v.value

        @property
        def w(self) -> float:
            """
            Provides a read-only copy of the last-known angular velocity of the
            gripper (in [rad/s]).

            :raises DHDError:
                If an error has occured with the device, invalidating the
                device state.
            """

            self.check_exception()
            return self._w.value

        @property
        def fg(self) -> float:
            """
            Provides a read-only copy of the last-known force applied by the
            gripper (in [N]).

            :raises DHDError:
                If an error has occured with the device, invalidating the
                device state.
            """

            self.check_exception()
            return self._fg.value

        def calculate_gap(self):
            """
            Calculate the value of the gripper opening (in [m]) from the current
            value of the gripper encoder and store it in an internal buffer.
            """

            err = dhd.expert.gripperEncoderToGap(
                self._enc.value, self._gap, self._id
            )

            if err:
                raise dhd.errno_to_exception(dhd.errorGetLast())()

        def calculate_angle(self):
            """
            Calculate the value of the gripper opening angle (in [rad]) from the
            current value of the gripper encoder and store it in an internal
            buffer.
            """

            dhd.expert.gripperEncoderToAngleRad(
                self._enc.value, self._angle, self._id
            )

        def update_enc(self):
            """
            Performs a blocking read to the HapticDevice, requesting the value
            of the gripper encoder.

            :raises DHDError:
                If an error has occured with the device, invalidating the
                device state.
            """

            err = dhd.expert.getGripperEncoder(self._enc, self._id)

            if err == -1:
                raise dhd.errno_to_exception(dhd.errorGetLast())()

            return self

        def update_enc_and_calculate(self):
            """
            Update the value of the gripper encoders and calculate the value of
            the gripper opening (in [m]) and gripper opening angle (in [rad]).

            :raises DHDError:
                If an error has occured with the device, invalidating the
                device state.
            """

            self.update_enc()
            self.calculate_angle()
            self.calculate_gap()

        def update_linear_velocity(self):
            """
            Computes the estimated instanteous linear velocity of the gripper
            (in [m/s]).

            :raises DHDError:
                If an error has occured with the device, invalidating the
                device state.
            """

            err = dhd.getGripperLinearVelocity(self._v, self._id)

            if err == -1:
                raise dhd.errno_to_exception(dhd.errorGetLast())()

        def update_angular_velocity(self):
            """
            Computes the estimated instanteous linear velocity of the gripper
            (in [rad/s]).

            :raises DHDError:
                If an error has occured with the device, invalidating the
                device state.
            """

            err = dhd.getGripperAngularVelocityRad(self._w, self._id)

            if err == -1:
                raise dhd.errno_to_exception(dhd.errorGetLast())()

        def update_angle(self):
            """
            Performs a blocking read to the HapticDevice, requesting the value
            of the gripper opening angle (in [rad]).

            :raises DHDError:
                If an error has occured with the device, invalidating the
                device state.
            """

            err = dhd.getGripperAngleRad(self._angle, self._id)

            if err == -1:
                raise dhd.errno_to_exception(dhd.errorGetLast())()

        def update_gap(self):
            """
            Performs a blocking read to the HapticDevice, requesting the value
            of the gripper opening (in [m]).

            :raises DHDError:
                If an error has occured with the device, invalidating the
                device state.
            """

            err = dhd.getGripperGap(self._gap, self._id)

            if err == -1:
                raise dhd.errno_to_exception(dhd.errorGetLast())()

        def update_thumb_pos(self):
            """
            Performs a blocking read to the HapticDevice, requesting the value
            of the gripper thumb rest position (in [m]) about the X, Y, and Z axes.

            :raises DHDError:
                If an error has occured with the device, invalidating the
                device state.
            """

            err = dhd.direct.getGripperThumbPos(self._thumb_pos, self._id)

            if err == -1:
                raise dhd.errno_to_exception(dhd.errorGetLast())()

        def update_finger_pos(self):
            """
            Performs a blocking read to the HapticDevice, requesting the value
            of the gripper forefinger rest position (in [m]) about the X, Y,
            and Z axes.

            :raises DHDError:
                If an error has occured with the device, invalidating the
                device state.
            """

            err = dhd.direct.getGripperFingerPos(self._finger_pos, self._id)

            if err == -1:
                raise dhd.errno_to_exception(dhd.errorGetLast())()

        def check_exception(self):
            self._parent.check_exception()

        _config_setters = {
            'velocity_estimator':
                lambda self, value: self.config_velocity(**value),
            'max_force': set_max_force
        }
        _config_initializers = {
            'velocity_estimator': _init_velocity_estimator
        }

    class Regulator:

        def __init__(
            self,
            parent,
            config_file_data: Optional[Dict[str, Any]],
            config_data: Optional[Dict[str, Any]],
            restore: bool
        ):
            self._parent: HapticDevice = parent
            self._haptic_daemon: Optional[HapticDaemon] = None
            self._stop_event = Event()
            self.pollers = {}

            self._info = self._parent._specs
            self._config = self._parent._config.regulator

            self._control_freq = nan
            self._is_drd_running = False
            self._info.is_drd_supported = drd.isSupported(self._parent._id)
            self._is_initialized = drd.isInitialized(self._parent._id)

            self._init_config(config_file_data, config_data, restore)

        def _init_motor_ratio_max(self, restore: bool):
            if restore:
                self.set_mot_ratio_max(1.0)
                return

            self._config.max_motor_ratio = drd.getMotRatioMax(self._parent._id)

        def _init_enc_move_param(self, restore: bool):
            if restore:
                self.set_enc_move_param(
                    drd.DEFAULT_ENC_MOVE_PARAMS.vmax,
                    drd.DEFAULT_ENC_MOVE_PARAMS.amax,
                    drd.DEFAULT_ENC_MOVE_PARAMS.jerk,
                )
                return

            enc_move_param, err = drd.getEncMoveParam()

            if err:
                raise dhd.errno_to_exception(dhd.errorGetLast())(
                    op='forcedimension.drd.getEncMoveParam()',
                    ID=self._parent._id
                )

            self._config.enc_move_param.vmax = enc_move_param[0]
            self._config.enc_move_param.amax = enc_move_param[1]
            self._config.enc_move_param.jerk = enc_move_param[2]

        def _init_enc_track_param(self, restore: bool):
            if restore:
                self.set_enc_track_param(
                    drd.DEFAULT_ENC_TRACK_PARAMS.vmax,
                    drd.DEFAULT_ENC_TRACK_PARAMS.amax,
                    drd.DEFAULT_ENC_TRACK_PARAMS.jerk,
                )
                return

            enc_track_param, err = drd.getEncTrackParam()

            if err:
                raise dhd.errno_to_exception(dhd.errorGetLast())(
                    op='forcedimension.drd.getEncTrackParam()',
                    ID=self._parent._id
                )

            self._config.enc_track_param.vmax = enc_track_param[0]
            self._config.enc_track_param.amax = enc_track_param[1]
            self._config.enc_track_param.jerk = enc_track_param[2]

        def _init_pos_move_param(self, restore: bool):
            if restore:
                self.set_pos_move_param(
                    drd.DEFAULT_POS_MOVE_PARAMS.vmax,
                    drd.DEFAULT_POS_MOVE_PARAMS.amax,
                    drd.DEFAULT_POS_MOVE_PARAMS.jerk,
                )
                return

            pos_move_param, err = drd.getPosMoveParam()

            if err:
                raise dhd.errno_to_exception(dhd.errorGetLast())(
                    op='forcedimension.drd.getPosMoveParam()',
                    ID=self._parent._id
                )

            self._config.pos_move_param.vmax = pos_move_param[0]
            self._config.pos_move_param.amax = pos_move_param[1]
            self._config.pos_move_param.jerk = pos_move_param[2]

        def _init_pos_track_param(self, restore: bool):
            if restore:
                self.set_pos_track_param(
                    drd.DEFAULT_POS_TRACK_PARAMS.vmax,
                    drd.DEFAULT_POS_TRACK_PARAMS.amax,
                    drd.DEFAULT_POS_TRACK_PARAMS.jerk,
                )
                return

            pos_track_param, err = drd.getPosTrackParam()

            if err:
                raise dhd.errno_to_exception(dhd.errorGetLast())(
                    op='forcedimension.drd.getPosTrackParam()',
                    ID=self._parent._id
                )

            self._config.pos_track_param.vmax = pos_track_param[0]
            self._config.pos_track_param.amax = pos_track_param[1]
            self._config.pos_track_param.jerk = pos_track_param[2]

        def _init_rot_move_param(self, restore: bool):
            if restore:
                self.set_rot_move_param(
                    drd.DEFAULT_ROT_MOVE_PARAMS.vmax,
                    drd.DEFAULT_ROT_MOVE_PARAMS.amax,
                    drd.DEFAULT_ROT_MOVE_PARAMS.jerk,
                )
                return

            rot_move_param, err = drd.getRotMoveParam()

            if err:
                raise dhd.errno_to_exception(dhd.errorGetLast())(
                    op='forcedimension.drd.getRotMoveParam()',
                    ID=self._parent._id
                )

            self._config.rot_move_param.vmax = rot_move_param[0]
            self._config.rot_move_param.amax = rot_move_param[1]
            self._config.rot_move_param.jerk = rot_move_param[2]

        def _init_rot_track_param(self, restore: bool):
            if restore:
                self.set_rot_track_param(
                    drd.DEFAULT_ROT_TRACK_PARAMS.vmax,
                    drd.DEFAULT_ROT_TRACK_PARAMS.amax,
                    drd.DEFAULT_ROT_TRACK_PARAMS.jerk,
                )
                return

            rot_track_param, err = drd.getRotTrackParam()

            if err:
                raise dhd.errno_to_exception(dhd.errorGetLast())(
                    op='forcedimension.drd.getRotTrackParam()',
                    ID=self._parent._id
                )

            self._config.rot_track_param.vmax = rot_track_param[0]
            self._config.rot_track_param.amax = rot_track_param[1]
            self._config.rot_track_param.jerk = rot_track_param[2]


        def _init_grip_move_param(self, restore: bool):
            if restore:
                self.set_grip_move_param(
                    drd.DEFAULT_GRIP_MOVE_PARAMS.vmax,
                    drd.DEFAULT_GRIP_MOVE_PARAMS.amax,
                    drd.DEFAULT_GRIP_MOVE_PARAMS.jerk,
                )
                return

            grip_move_param, err = drd.getGripMoveParam()

            if err:
                raise dhd.errno_to_exception(dhd.errorGetLast())(
                    op='forcedimension.drd.getGripMoveParam()',
                    ID=self._parent._id
                )

            self._config.grip_move_param.vmax = grip_move_param[0]
            self._config.grip_move_param.amax = grip_move_param[1]
            self._config.grip_move_param.jerk = grip_move_param[2]

        def _init_grip_track_param(self, restore: bool):
            if restore:
                self.set_grip_track_param(
                    drd.DEFAULT_GRIP_TRACK_PARAMS.vmax,
                    drd.DEFAULT_GRIP_TRACK_PARAMS.amax,
                    drd.DEFAULT_GRIP_TRACK_PARAMS.jerk,
                )
                return

            grip_track_param, err = drd.getGripTrackParam()

            if err:
                raise dhd.errno_to_exception(dhd.errorGetLast())(
                    op='forcedimension.drd.getGripTrackParam()',
                    ID=self._parent._id
                )

            self._config.grip_track_param.vmax = grip_track_param[0]
            self._config.grip_track_param.amax = grip_track_param[1]
            self._config.grip_track_param.jerk = grip_track_param[2]

        def _init_config(
            self,
            config_file_data: Optional[Dict[str, Any]],
            config_data: Optional[Dict[str, Any]],
            restore: bool
        ):
            unset_configs = {
               'motor_ratio_max',
               'enc_move_param',
               'enc_track_param',
               'pos_move_param',
               'pos_track_param',
               'rot_move_param',
               'rot_track_param',
               'grip_move_param',
               'grip_track_param',
            }

            if config_file_data is not None and config_data is not None:
                preserved_keys = set(
                    set(config_file_data.keys()) ^ set(config_data.keys())
                )
                overwritten_keys = set(
                    set(config_file_data.keys()) & set(config_data.keys())
                )

                unset_configs -= preserved_keys
                unset_configs -= overwritten_keys

                for key in preserved_keys:
                    HapticDevice.Regulator._config_setters[key](
                        self, config_file_data[key]
                    )

                for key in overwritten_keys:
                    HapticDevice.Regulator._config_setters[key](
                        self, config_data[key]
                    )
            elif config_file_data is not None and config_data is None:
                for key, value in config_file_data:
                    HapticDevice.Regulator._config_setters[key](value)
            elif config_file_data is None and config_data is not None:
                for key, value in config_data:
                    HapticDevice.Regulator._config_setters[key](value)

            # Default initialize all unset configs.

            for config in unset_configs:
                HapticDevice.Regulator._config_initializers[config](
                    self, restore
                )

        @property
        def is_drd_supported(self) -> bool:
            return self._info.is_drd_supported

        @property
        def is_initialized(self) -> bool:
            return self._is_initialized

        @property
        def is_drd_running(self) -> bool:
            return self._is_drd_running

        @property
        def control_freq(self) -> float:
            return self._control_freq

        @property
        def is_moving(self) -> bool:
            return drd.isMoving(self._parent._id)

        @property
        def motor_ratio_max(self) -> float:
            return self._config.max_motor_ratio

        @property
        def enc_move_param(self) -> TrajectoryGenParams:
            return self._config.enc_move_param

        @property
        def enc_track_param(self) -> TrajectoryGenParams:
            return self._config.enc_track_param

        @property
        def pos_move_param(self) -> TrajectoryGenParams:
            return self._config.pos_move_param

        @property
        def pos_track_param(self) -> TrajectoryGenParams:
            return self._config.pos_track_param

        @property
        def rot_move_param(self) -> TrajectoryGenParams:
            return self._config.rot_move_param

        @property
        def rot_track_param(self) -> TrajectoryGenParams:
            return self._config.rot_track_param

        @property
        def grip_move_param(self) -> TrajectoryGenParams:
            return self._config.grip_move_param

        @property
        def grip_track_param(self) -> TrajectoryGenParams:
            return self._config.grip_track_param

        def is_filtering(self) -> bool:
            return drd.isFiltering(self._parent._id)

        def update_control_freq(self):
            if (control_freq := drd.getCtrlFreq(self._parent._id)) < 0:
                dhd.errno_to_exception(dhd.errorGetLast())(
                    op='forcedimension.drd.getCtrlFreq()',
                    ID=self._parent._id
                )

            self._control_freq = control_freq

        def initialize(self, redo=False):
            if redo:
                self._initialized = False

                if drd.autoInit(self._parent._id):
                    raise dhd.errno_to_exception(dhd.errorGetLast())(
                        op='forcedimension.drd.autoInit()',
                        ID=self._parent._id
                    )

                self._initialized = True

            if self._initialized:
                return

            if drd.checkInit(self._parent._id):
                raise dhd.errno_to_exception(dhd.errorGetLast())(
                    op='forcedimension.drd.checkInit()',
                    ID=self._parent._id
                )

            self._initialized = True

        def precision_initialize(self):
            if drd.precisionInit(self._parent._id):
                raise dhd.errno_to_exception(dhd.errorGetLast())(
                    op='forcedimension.drd.precisionInit()',
                    ID=self._parent._id
                )

            self._initialized = True

        def regulate(self, enabled: bool = True):
            self.regulate_pos(enabled)
            self.regulate_rot(enabled)
            self.regulate_grip(enabled)

        def regulate_pos(self, enabled: bool = True):
            self._is_pos_regulated = enabled

            if drd.regulatePos(enabled, self._parent._id):
                raise dhd.errno_to_exception(dhd.errorGetLast())(
                    op='forcedimension.drd.regulatePos()',
                    ID=self._parent._id
                )

            return self

        def regulate_rot(self, enabled: bool = True):
            self._is_rot_regulated = enabled

            if drd.regulateRot(enabled, self._parent._id):
                raise dhd.errno_to_exception(dhd.errorGetLast())(
                    op='forcedimension.drd.regulateRot()',
                    ID=self._parent._id
                )

            return self

        def regulate_grip(self, enabled: bool = True):
            self._is_grip_regulated = enabled

            if drd.regulateGrip(enabled, self._parent._id):
                raise dhd.errno_to_exception(dhd.errorGetLast())(
                    op='forcedimension.drd.regulateGrip()',
                    ID=self._parent._id
                )

            return self

        def enable_filter(self, enabled: bool = True):
            self._is_filter_enabled = enabled

            if drd.enableFilter(enabled, self._parent._id):
                raise dhd.errno_to_exception(dhd.errorGetLast())(
                    op='forcedimension.drd.enableFilter()',
                    ID=self._parent._id
                )

        def start_drd(self):
            if not self._is_drd_running:
                if drd.start(self._parent._id):
                    raise dhd.errno_to_exception(dhd.errorGetLast())(
                        op='forcedimension.drd.start()',
                        ID=self._parent._id
                    )

                self._is_drd_running = True

        def stop_drd(self):
            if self._is_drd_running:
                if drd.stop(self._parent._id):
                    raise dhd.errno_to_exception(dhd.errorGetLast())(
                        op='forcedimension.drd.start()',
                        ID=self._parent._id
                    )

                self._is_drd_running = False


            if self._haptic_daemon is not None:
                raise RuntimeError(
                    "Mixed use of HapticDaemon and HapticDevice.Regulator is "
                    "not supported."
                )

        def stop(self):
            if self._haptic_daemon is not None:
                self._haptic_daemon.stop()
                return


        def update(self):
            self.update_position_and_orientation()
            self.update_velocity()

        def update_position_and_orientation(self):
            if not self._is_drd_running:
                raise RuntimeError("DRD is not running.")

            err = drd.direct.getPositionAndOrientation(
                self._parent._pos,
                self._parent._orientation_angles,
                self._parent._mock_gripper._gap,
                self._parent._frame,
                self._parent._id
            )

            if err:
                raise dhd.errno_to_exception(dhd.errorGetLast())(
                    op='forcedimension.drd.getPositionAndOrientation()',
                    ID=self._parent._id
                )

        def update_velocity(self):
            if not self._is_drd_running:
                raise RuntimeError("DRD is not running.")

            err = drd.direct.getVelocity(
                self._parent._v, self._parent._w, self._parent._id
            )

            if err:
                raise dhd.errno_to_exception(dhd.errorGetLast())(
                    op='forcedimension.drd.getVelocity()',
                    ID=self._parent._id
                )

        def set_mot_ratio_max(self, scale: float):
            """
            Sets the maximum joint torque applied to all regulated joints
            expressed as a fraction of the maximum torque available for each
            joint.

            In practice, this limits the maximum regulation torque (in joint
            space), making it potentially safer to operate in environments
            where humans or delicate obstacles are present.
            """

            if drd.setMotRatioMax(scale, self._parent._id):
                raise dhd.errno_to_exception(dhd.errorGetLast())(
                    op='forcedimension.drd.setMotRatioMax()',
                    ID=self._parent._id
                )

            self._config.max_motor_ratio = scale

            return self

        def set_enc_move_param(
            self,
            vmax: Optional[float] = None,
            amax: Optional[float] = None,
            jerk: Optional[float] = None
        ):
            """
            Sets the trajectory generation parameters for
            :func:`HapticDevice.Regulator.move_to_enc()`
            :func:`HapticDevice.Regulator.move_to_all_enc()`. Unset parameters
            will not be changed.

            :param Optional[float] vmax:
                Maximum allowable velocity during a generated trajectory
                (in [inc/s]).

            :param Optional[float] amax:
                Maximum allowable acceleration during a generated trajectory
                (in [inc/s^2]).

            :param Optional[float] jerk:
                Maximum allowable jerk during the generated trajectory
                (in [inc/s^3])
            """

            if vmax is None:
                vmax = self._config.enc_move_param.vmax

            if amax is None:
                amax = self._config.enc_move_param.amax

            if jerk is None:
                jerk = self._config.enc_move_param.jerk

            if drd.setEncMoveParam(vmax, amax, jerk, self._parent._id):
                raise dhd.errno_to_exception(dhd.errorGetLast())(
                    op='forcedimension.drd.setEncMoveParam()',
                    ID=self._parent._id
                )

            self._config.enc_move_param.vmax = vmax
            self._config.enc_move_param.amax = amax
            self._config.enc_move_param.jerk = jerk

            return self


        def set_enc_track_param(
            self,
            vmax: Optional[float] = None,
            amax: Optional[float] = None,
            jerk: Optional[float] = None
        ):
            """
            Sets the trajectory generation parameters for
            :func:`HapticDevice.Regulator.track_enc()` and
            :func:`HapticDevice.Regulator.track_all_enc()`. Unset parameters
            will not be changed.

            :param Optional[float] vmax:
                Maximum allowable velocity during a generated trajectory
                (in [inc/s]).

            :param Optional[float] amax:
                Maximum allowable acceleration during a generated trajectory
                (in [inc/s^2]).

            :param Optional[float] jerk:
                Maximum allowable jerk during the generated trajectory
                (in [inc/s^3])
            """

            if vmax is None:
                vmax = self._config.enc_track_param.vmax

            if amax is None:
                amax = self._config.enc_track_param.amax

            if jerk is None:
                jerk = self._config.enc_track_param.jerk

            if drd.setEncMoveParam(vmax, amax, jerk, self._parent._id):
                raise dhd.errno_to_exception(dhd.errorGetLast())(
                    op='forcedimension.drd.setEncTrackParam()',
                    ID=self._parent._id
                )

            self._config.enc_track_param.vmax = vmax
            self._config.enc_track_param.amax = amax
            self._config.enc_track_param.jerk = jerk

            return self

        def set_pos_move_param(
            self,
            vmax: Optional[float] = None,
            amax: Optional[float] = None,
            jerk: Optional[float] = None
        ):
            """
            Sets the trajectory generation parameters for
            :func:`HapticDevice.Regulator.move_to_pos()` and
            :func:`HapticDevice.Regulator.move_to()` (on DOFs 1-3). Unset
            parameters will not be changed.

            :param Optional[float] vmax:
                Maximum allowable velocity during a generated trajectory
                (in [m/s]).

            :param Optional[float] amax:
                Maximum allowable acceleration during a generated trajectory
                (in [m/s^2]).

            :param Optional[float] jerk:
                Maximum allowable jerk during the generated trajectory
                (in [m/s^3])
            """

            if vmax is None:
                vmax = self._config.pos_move_param.vmax

            if amax is None:
                amax = self._config.pos_move_param.amax

            if jerk is None:
                jerk = self._config.pos_move_param.jerk

            if drd.setPosMoveParam(vmax, amax, jerk, self._parent._id):
                raise dhd.errno_to_exception(dhd.errorGetLast())(
                    op='forcedimension.drd.setPosMoveParam()',
                    ID=self._parent._id
                )

            self._config.pos_move_param.vmax = vmax
            self._config.pos_move_param.amax = amax
            self._config.pos_move_param.jerk = jerk

            return self


        def set_pos_track_param(
            self,
            vmax: Optional[float] = None,
            amax: Optional[float] = None,
            jerk: Optional[float] = None
        ):
            """
            Sets the trajectory generation parameters for
            :func:`HapticDevice.Regulator.track_pos()` and
            :func:`HapticDevice.Regulator.track()` (on DOFs 1-3).
            Unset parameters will not be changed.

            :param Optional[float] vmax:
                Maximum allowable velocity during a generated trajectory
                (in [m/s]).

            :param Optional[float] amax:
                Maximum allowable acceleration during a generated trajectory
                (in [m/s^2]).

            :param Optional[float] jerk:
                Maximum allowable jerk during the generated trajectory
                (in [m/s^3])
            """

            if vmax is None:
                vmax = self._config.pos_track_param.vmax

            if amax is None:
                amax = self._config.pos_track_param.amax

            if jerk is None:
                jerk = self._config.pos_track_param.jerk

            if drd.setPosTrackParam(vmax, amax, jerk, self._parent._id):
                raise dhd.errno_to_exception(dhd.errorGetLast())(
                    op='forcedimension.drd.setPosTrackParam()',
                    ID=self._parent._id
                )

            self._config.pos_track_param.vmax = vmax
            self._config.pos_track_param.amax = amax
            self._config.pos_track_param.jerk = jerk

            return self


        def set_rot_move_param(
            self,
            vmax: Optional[float] = None,
            amax: Optional[float] = None,
            jerk: Optional[float] = None
        ):
            """
            Sets the trajectory generation parameters for
            :func:`HapticDevice.Regulator.move_to_rot()` and
            :func:`HapticDevice.Regulator.move_to()` (on DOFs 4-6).
            Unset parameters will not be changed.

            :param Optional[float] vmax:
                Maximum allowable velocity during a generated trajectory
                (in [rad/s]).

            :param Optional[float] amax:
                Maximum allowable acceleration during a generated trajectory
                (in [rad/s^2]).

            :param Optional[float] jerk:
                Maximum allowable jerk during the generated trajectory
                (in [rad/s^3])
            """

            if vmax is None:
                vmax = self._config.rot_move_param.vmax

            if amax is None:
                amax = self._config.rot_move_param.amax

            if jerk is None:
                jerk = self._config.rot_move_param.jerk

            if drd.setRotMoveParam(vmax, amax, jerk, self._parent._id):
                raise dhd.errno_to_exception(dhd.errorGetLast())(
                    op='forcedimension.drd.setRotMoveParam()',
                    ID=self._parent._id
                )

            self._config.rot_move_param.vmax = vmax
            self._config.rot_move_param.amax = amax
            self._config.rot_move_param.jerk = jerk

            return self


        def set_rot_track_param(
            self,
            vmax: Optional[float] = None,
            amax: Optional[float] = None,
            jerk: Optional[float] = None
        ):
            """
            Sets the trajectory generation parameters for
            :func:`HapticDevice.Regulator.track_rot()` and
            :func:`HapticDevice.Regulator.track()` (on DOFs 4-6).
            Unset parameters will not be changed.

            :param Optional[float] vmax:
                Maximum allowable velocity during a generated trajectory
                (in [rad/s]).

            :param Optional[float] amax:
                Maximum allowable acceleration during a generated trajectory
                (in [rad/s^2]).

            :param Optional[float] jerk:
                Maximum allowable jerk during the generated trajectory
                (in [rad/s^3])
            """

            if vmax is None:
                vmax = self._config.rot_track_param.vmax

            if amax is None:
                amax = self._config.rot_track_param.amax

            if jerk is None:
                jerk = self._config.rot_track_param.jerk

            if drd.setRotTrackParam(vmax, amax, jerk, self._parent._id):
                raise dhd.errno_to_exception(dhd.errorGetLast())(
                    op='forcedimension.drd.setRotTrackParam()',
                    ID=self._parent._id
                )

            self._config.rot_track_param.vmax = vmax
            self._config.rot_track_param.amax = amax
            self._config.rot_track_param.jerk = jerk

            return self

        def set_grip_move_param(
            self,
            vmax: Optional[float] = None,
            amax: Optional[float] = None,
            jerk: Optional[float] = None
        ):
            """
            Sets the trajectory generation parameters for
            :func:`HapticDevice.Regulator.move_to_grip()` and
            :func:`HapticDevice.Regulator.move_to()` (DOF 7).
            Unset parameters will not be changed.

            :param Optional[float] vmax:
                Maximum allowable velocity during a generated trajectory
                (in [m/s]).

            :param Optional[float] amax:
                Maximum allowable acceleration during a generated trajectory
                (in [m/s^2]).

            :param Optional[float] jerk:
                Maximum allowable jerk during the generated trajectory
                (in [m/s^3])
            """

            if vmax is None:
                vmax = self._config.grip_move_param.vmax

            if amax is None:
                amax = self._config.grip_move_param.amax

            if jerk is None:
                jerk = self._config.grip_move_param.jerk

            if drd.setGripMoveParam(vmax, amax, jerk, self._parent._id):
                raise dhd.errno_to_exception(dhd.errorGetLast())(
                    op='forcedimension.drd.setGripMoveParam()',
                    ID=self._parent._id
                )

            self._config.grip_move_param.vmax = vmax
            self._config.grip_move_param.amax = amax
            self._config.grip_move_param.jerk = jerk

            return self


        def set_grip_track_param(
            self,
            vmax: Optional[float] = None,
            amax: Optional[float] = None,
            jerk: Optional[float] = None
        ):
            """
            Sets the trajectory generation parameters for
            :func:`HapticDevice.Regulator.track_grip()` and
            :func:`HapticDevice.Regulator.track()` (DOF 7).
            Unset parameters will not be changed.

            :param Optional[float] vmax:
                Maximum allowable velocity during a generated trajectory
                (in [m/s]).

            :param Optional[float] amax:
                Maximum allowable acceleration during a generated trajectory
                (in [m/s^2]).

            :param Optional[float] jerk:
                Maximum allowable jerk during the generated trajectory
                (in [m/s^3])
            """

            if vmax is None:
                vmax = self._config.grip_track_param.vmax

            if amax is None:
                amax = self._config.grip_track_param.amax

            if jerk is None:
                jerk = self._config.grip_track_param.jerk

            if drd.setGripTrackParam(vmax, amax, jerk, self._parent._id):
                raise dhd.errno_to_exception(dhd.errorGetLast())(
                    op='forcedimension.drd.setGripTrackParam()',
                    ID=self._parent._id
                )

            self._config.grip_track_param.vmax = vmax
            self._config.grip_track_param.amax = amax
            self._config.grip_track_param.jerk = jerk

            return self

        def zero(self, block: bool = True):
            """
            Simultaneously sends the device end-effector position and
            orientation to zero. The motion uses smooth
            acceleration/deceleration and follows a straight line in space and
            a straight curve for the rotation. The acceleration and velocity
            profiles can be controlled by adjusting the trajectory generation
            parameters.
            """

            gap = 0

            if self._parent._specs.has_gripper:
                gap = self._parent._mock_gripper.gap

            self.move_to((0, 0, 0, 0, 0, 0, gap), block=block)

            return self

        def zero_pos(self, block: bool = True):
            """
            Sends the device end-effector position to the origin.
            The motion uses smooth acceleration/deceleration and follows a
            straight line in space. The acceleration and velocity profiles can
            be controlled by adjusting the trajectory generation parameters.
            """

            return self.move_to_pos((0., 0., 0.), block=block)

        def zero_rot(self, block: bool = True):
            """
            Sends the device end-effector to zero rotation about the first,
            second, and third joint angles The motion uses smooth
            acceleration/deceleration and follows a straight curve. The
            acceleration and velocity profiles can be controlled by adjusting
            the trajectory generation parameters.
            """
            return self.move_to_rot((0., 0., 0.), block=block)

        def move_to_enc(
            self,
            *cmds: Union[IntVectorLike, Iterable[IntVectorLike]],
            block: bool = True
        ):
            """
            Sequentially sends the device end-effector to a sequence desired
            delta encoder configurations. The motion uses smooth
            acceleration/deceleration. The acceleration and velocity profiles
            can be controlled by adjusting the trajectory generation
            parameters.

            :param Union[IntVectorLike, Iterable[IntVectorLike]] *cmds:
                Encoder values (or an iterable of encoder values) for each
                delta encoder axis.

            :param bool block:
                If `True` the call will block until the command is completed.
                If `False` the call will return immediately. `False` values are
                only supported for calls with a single target (non-iterable).


            Warning
            -------
            Paths are NOT guarunteed to be continuous if a new command is
            given before the previous command has ended. This can happen if
            `block` is `False` and multiple commands are provided without
            proper synchronization. Setting `block` to `False` should be
            reserved for use cases for iteratables which continously feed
            inputs spaced over time (i.e. reading from some stream which
            periodically sends commands).


            See Also
            --------
            :func:`forcedimension.Regulator.track_enc()`
            """

            for cmd in cmds:
                if isinstance(cmd, Iterable):
                    for point in cmd:
                        if drd.moveToEnc(point, block, self._parent._id):
                            dhd.errno_to_exception(dhd.errorGetLast())(
                                op='forcedimension.drd.moveToEnc()',
                                ID=self._parent._id
                            )
                else:
                    if drd.moveToEnc(cmd, block, self._parent._id):
                        dhd.errno_to_exception(dhd.errorGetLast())(
                            op='forcedimension.drd.moveToEnc()',
                            ID=self._parent._id
                        )

            return self

        def track_enc(self,
            *cmds: Union[IntVectorLike, Iterable[IntVectorLike]]
        ):
            """
            Sequentially sends the device end-effector to a sequence of
            delta encoder configurations. This motion is guaunteed to be
            continuous. If motion filters are enabled, the
            motion follows a smooth acceleration/deceleration constraint on
            each encoder axis. The acceleration and velocity profiles can be
            controlled by adjusting the trajectory generation parameters.

            :param Union[IntVectorLike, Iterable[IntVectorLike]] *cmds:
                A sequence of end-effector encoder values.
            """
            for cmd in cmds:
                if isinstance(cmd, Iterable):
                    for point in cmd:
                        if drd.trackEnc(point, self._parent._id):
                            dhd.errno_to_exception(dhd.errorGetLast())(
                                op='forcedimension.drd.trackEnc()',
                                ID=self._parent._id
                            )
                else:
                    if drd.trackEnc(cmd, self._parent._id):
                        dhd.errno_to_exception(dhd.errorGetLast())(
                            op='forcedimension.drd.trackEnc()',
                            ID=self._parent._id
                        )

            return self

        def move_to(
            self,
            *cmds: Union[FloatVectorLike, Iterable[FloatVectorLike]],
            block: bool = True
        ):
            """
            Sends the device end-effector to a set of desired
            7-DOF configurations.  The motion uses smooth
            acceleration/deceleration and will follow straight lines in
            position space as well as straight curves in orientation space. The
            acceleration and velocity profiles can be controlled by adjusting
            the trajectory generation parameters.

            :param Union[IntVectorLike, Iterable[IntVectorLike]] *cmds:
                An iterable where each element is a target 7 DOF configuration
                or an iterable of 7 DOF configurations to sequentially
                follow. DOFs 1-3 represent the position about the X,
                Y, and Z axes (in [m]). DOFs 4-6 represent the orientation about
                the first, second, and third wrist joints (in [rad]). DOF 7
                represents the gripper opening gap (in [m]).

            :param bool block:
                If `True` the call will block until the command is completed.
                If `False` the call will return immediately. `False` values are
                only supported for calls with a single target (non-iterable).


            Warning
            -------
            Paths are NOT guarunteed to be continuous if a new command is
            given before the previous command has ended. This can happen if
            `block` is `False` and multiple commands are provided without
            proper synchronization. Setting `block` to `False` should be
            reserved for use cases for iteratables which continously feed
            inputs spaced over time (i.e. reading from some stream which
            periodically sends commands).


            See Also
            --------
            :func:`forcedimension.Regulator.track_pos()`
            """

            for cmd in cmds:
                if isinstance(cmd, Iterable):
                    for conf in cmd:
                        if drd.moveTo(conf, block, self._parent._id):
                            dhd.errno_to_exception(dhd.errorGetLast())(
                                op='forcedimension.drd.moveTo()',
                                ID=self._parent._id
                            )
                else:
                    if drd.moveTo(cmd, block, self._parent._id):
                        dhd.errno_to_exception(dhd.errorGetLast())(
                            op='forcedimension.drd.moveTo()',
                            ID=self._parent._id
                        )

            return self

        def track(
            self,
            *cmds: Union[FloatVectorLike, Iterable[FloatVectorLike]],
        ):
            """
            This function sends the device end-effector to a desired Cartesian
            7-DOF configuration. If motion filters are enabled, the motion
            follows a smooth acceleration/deceleration constraint on each
            Cartesian axis. The acceleration and velocity profiles can be
            controlled by adjusting the trajectory generation parameters.

            :param Union[FloatVectorLike, Iterable[FloatVectorLike]] *cmds:
                An iterable where each element is a target 7 DOF configuration
                or an iterable of 7 DOF configurations to sequentially
                follow. DOFs 1-3 represent the position about the X,
                Y, and Z axes, respectively (in [m]).
                DOFs 4-6 represent the orientation about the first, second, and
                third wrist joints, respectively (in [rad]). DOF 7
                represents the gripper opening gap (in [m]).
            """
            for cmd in cmds:
                if isinstance(cmd, Iterable):
                    for conf in cmd:
                        if drd.track(conf, self._parent._id):
                            dhd.errno_to_exception(dhd.errorGetLast())(
                                op='forcedimension.drd.track()', ID=self._parent._id
                            )
                else:
                    if drd.track(cmd, self._parent._id):
                        dhd.errno_to_exception(dhd.errorGetLast())(
                            op='forcedimension.drd.track()', ID=self._parent._id
                        )
            return self


        def move_to_all_enc(
            self,
            *cmds: Union[IntVectorLike, Iterable[IntVectorLike]],
            block: bool = True
        ):
            """
            Sends the device end-effector to a set of desired encoder
            positions in each degree-of-freedom. The motion follows a straight
            line in the encoder space, with smooth acceleration/deceleration.
            The acceleration and velocity profiles can be controlled by
            adjusting the trajectory generation parameters.

            :param Union[IntVectorLike, Iterable[IntVectorLike]] *cmds:
                An iterable where each element is a target encoder position
                or an iterable of target encoder positions to sequentially
                follow. Each position specifies a target for each
                degree-of-freedom.

            :param bool block:
                If `True` the call will block until the command is completed.
                If `False` the call will return immediately. `False` values are
                only supported for calls with a single target (non-iterable).

            Warning
            -------
            Paths are NOT guarunteed to be continuous if a new command is
            given before the previous command has ended. This can happen if
            `block` is `False` and multiple commands are provided without
            proper synchronization. Setting `block` to `False` should be
            reserved for use cases for iteratables which continously feed
            inputs spaced over time (i.e. reading from some stream which
            periodically sends commands).


            See Also
            --------
            :func:`forcedimension.Regulator.track_all_enc()`
            """

            if block:
                if len(cmds) > 1:
                    raise ValueError(
                        "Blocking is only supported for single position, "
                        "non-iterable, commands."
                    )

                if isinstance(cmds, Iterable):
                    raise ValueError(
                        "Blocking is only supported for single position, "
                        "non-iterable, commands."
                    )

            for cmd in cmds:
                if isinstance(cmd, Iterable):
                    for point in cmd:
                        if drd.moveToAllEnc(point, block, self._parent._id):
                            dhd.errno_to_exception(dhd.errorGetLast())(
                                op='forcedimension.drd.moveToAllEnc()',
                                ID=self._parent._id
                            )
                else:
                    if drd.moveToAllEnc(cmd, block, self._parent._id):
                        dhd.errno_to_exception(dhd.errorGetLast())(
                            op='forcedimension.drd.moveToAllEnc()',
                            ID=self._parent._id
                        )
            return self

        def track_all_enc(
            self,
            *cmds: Union[IntVectorLike, Iterable[IntVectorLike]]
        ):
            """
            Sequentially sends the device end-effector to a set
            of desired encoder positions in each degree of freedom. If motion
            filters are enabled, the motion follows a smooth
            acceleration/deceleration constraint on each encoder axis. The
            acceleration and velocity profiles can be controlled by adjusting
            the trajectory generation parameters.

            :param Union[IntVectorLike, Iterable[IntVectorLike]] *cmds:
                An iterable where each element is a target encoder position
                or an iterable of target encoder positions to sequentially
                follow. Each position specifies a target for each
                degree-of-freedom.

            """

            for cmd in cmds:
                if isinstance(cmd, Iterable):
                    for point in cmd:
                        if drd.trackAllEnc(point, self._parent._id):
                            dhd.errno_to_exception(dhd.errorGetLast())(
                                op='forcedimension.drd.trackAllEnc()',
                                ID=self._parent._id
                            )
                else:
                    if drd.trackAllEnc(cmd, self._parent._id):
                        dhd.errno_to_exception(dhd.errorGetLast())(
                            op='forcedimension.drd.trackAllEnc()',
                            ID=self._parent._id
                        )
            return self

        def move_to_pos(
            self,
            *cmds: Union[FloatVectorLike, Iterable[FloatVectorLike]],
            block: bool = True
        ):
            """
            Sends the device end-effector to a set of desired
            positions. The motion follows a straight line in position space,
            with smooth acceleration/deceleration. The acceleration and
            velocity profiles can be controlled by adjusting the trajectory
            generation parameters.

            :param Union[FloatVectorLike, Iterable[FloatVectorLike]] *cmds:
                An iterable where each element is a target position
                or an iterable of target positions to sequentially
                follow. Positions are about the X, Y, and Z axes (in [m]).

            :param bool block:
                If `True` the call will block until the command is completed.
                If `False` the call will return immediately. `False` values are
                only supported for calls with a single target (non-iterable).

            :raises ValueError:
                If more than one command (or an iterable of commands) was
                requested when `block` is `False`.

            Warning
            -------
            Paths are NOT guarunteed to be continuous if a new command is
            given before the previous command has ended. This can happen if
            `block` is `False` and multiple commands are provided without
            proper synchronization. Setting `block` to `False` should be
            reserved for use cases for iteratables which continously feed
            inputs spaced over time (i.e. reading from some stream which
            periodically sends commands).


            See Also
            --------
            :func:`forcedimension.Regulator.track_pos()`
            """

            for cmd in cmds:
                if isinstance(cmd, Iterable):
                    for point in cmd:
                        if drd.moveToPos(point, block, self._parent._id):
                            dhd.errno_to_exception(dhd.errorGetLast())(
                                op='forcedimension.drd.moveToPos()',
                                ID=self._parent._id
                            )
                else:
                    if drd.moveToPos(cmd, block, self._parent._id):
                        dhd.errno_to_exception(dhd.errorGetLast())(
                            op='forcedimension.drd.moveToPos()',
                            ID=self._parent._id
                        )

            return self


        def track_pos(self,
            *cmds: Union[FloatVectorLike, Iterable[FloatVectorLike]]
        ):
            """
            Sends the device end-effector to a set of desired
            positions. The motion follows a straight line in position space,
            with smooth acceleration/deceleration. The acceleration and
            velocity profiles can be controlled by adjusting the trajectory
            generation parameters.

            :param Union[FloatVectorLike, Iterable[FloatVectorLike]] *cmds:
                An iterable where each element is a target position
                or an iterable of target positions to sequentially
                follow. Positions are about the X, Y, and Z axes (in [m]).

            :param bool block:
                If `True` the call will block until the command is completed.
                If `False` the call will return immediately. `False` values are
                only supported for calls with a single target (non-iterable).
            """
            for cmd in cmds:
                if isinstance(cmd, Iterable):
                    for point in cmd:
                        if drd.trackPos(point, self._parent._id):
                            dhd.errno_to_exception(dhd.errorGetLast())(
                                op='forcedimension.drd.trackPos()',
                                ID=self._parent._id
                            )
                else:
                    if drd.trackPos(cmd, self._parent._id):
                        dhd.errno_to_exception(dhd.errorGetLast())(
                            op='forcedimension.drd.trackPos()',
                            ID=self._parent._id
                        )

            return self

        def move_to_rot(
            self,
            *cmds: Union[FloatVectorLike, Iterable[FloatVectorLike]],
            block: bool = True
        ):
            """
            Sequentially sends the device end-effector to a set of desired
            Cartesian rotations. The motion follows a straight curve between
            each command, with smooth acceleration/deceleration. The
            acceleration and velocity profiles can be controlled by adjusting
            the trajectory generation parameters.

            :param Union[FloatVectorLike, Iterable[FloatVectorLike]] *cmds:
                An iterable where each element is a target orientation
                or an iterable of target orientations to sequentially
                follow. Orientations are about the first, second, and third
                joint angles (in [rad]).

            :param bool block:
                If `True` the call will block until the command is completed.
                If `False` the call will return immediately. `False` values are
                only supported for calls with a single target (non-iterable).

            Info
            ----
            Paths are NOT guarunteed to be continuous a sequence of calls where
            `block` is `False`. Use
            :func:`forcedimension.HapticDevice.Regulator.track_rot()` for
            that instead.

            :raises ValueError:
                If more than one command (or an iterable of commands) was
                requested when `block` is `False`.
            """
            for cmd in cmds:
                if isinstance(cmd, Iterable):
                    for point in cmd:
                        if drd.moveToRot(point, block, self._parent._id):
                            dhd.errno_to_exception(dhd.errorGetLast())(
                                op='forcedimension.drd.moveToRot()',
                                ID=self._parent._id
                            )
                else:
                    if drd.moveToRot(cmd, block, self._parent._id):
                        dhd.errno_to_exception(dhd.errorGetLast())(
                            op='forcedimension.drd.moveToRot()',
                            ID=self._parent._id
                        )

            return self

        def track_rot(self,
            *cmds: Union[FloatVectorLike, Iterable[FloatVectorLike]]
        ):
            """
            Sequentially sends the device end-effector to a set of
            desired Cartesian orientation. If motion filters are enabled, the
            motion follows a smooth acceleration/deceleration curve along each
            Cartesian axis. The acceleration and velocity profiles can be
            controlled by adjusting the trajectory generation parameters.

            :param Union[FloatVectorLike, Iterable[FloatVectorLike]] *cmds:
                An iterable where each element is a target orientation
                or an iterable of target orientations to sequentially
                follow. Orientations are about the first, second, and third
                joint angles (in [rad]).

            :param bool block:
                If `True` the call will block until the command is completed.
                If `False` the call will return immediately. `False` values are
                only supported for calls with a single target (non-iterable).

            :raises ValueError:
                If more than one command (or an iterable of commands) was
                requested when `block` is `False`.
            """
            for cmd in cmds:
                if isinstance(cmd, Iterable):
                    for point in cmd:
                        if drd.trackRot(point, self._parent._id):
                            dhd.errno_to_exception(dhd.errorGetLast())(
                                op='forcedimension.drd.trackRot()',
                                ID=self._parent._id
                            )
                else:
                    if drd.trackRot(cmd, self._parent._id):
                        dhd.errno_to_exception(dhd.errorGetLast())(
                            op='forcedimension.drd.trackRot()',
                            ID=self._parent._id
                        )

            return self

        def move_to_grip(
            self,
            *cmds: Union[float, Iterable[float]],
            block: bool = True
        ):
            """
            Sequentially sends the device gripper to a set of desired opening
            distances. The motion is executed with smooth
            acceleration/deceleration. The acceleration and velocity profiles
            can be controlled by adjusting the trajectory generation
            parameters.

            :param Union[float, Iterable[float]] *cmds:
                An iterable where each element is a target gripper opening
                distance or an iterable of target gripper opening distances.
                Gripper opening distances are in [m].

            :param bool block:
                If `True` the call will block until the command is completed.
                If `False` the call will return immediately. `False` values are
                only supported for calls with a single target (non-iterable).

            Info
            ----
            Paths are NOT guarunteed to be continuous a sequence of calls where
            `block` is `False`. Use
            :func:`forcedimension.Regulator.HapticDevice.track_grip()` for that
            instead.

            :raises ValueError:
                If more than one command (or an iterable of commands) was
                requested when `block` is `False`.
            """

            for cmd in cmds:
                if isinstance(cmd, Iterable):
                    for point in cmd:
                        if drd.moveToGrip(point, block, self._parent._id):
                            dhd.errno_to_exception(dhd.errorGetLast())(
                                op='forcedimension.drd.moveToGrip()',
                                ID=self._parent._id
                            )
                else:
                    if drd.moveToGrip(cmd, block, self._parent._id):
                        dhd.errno_to_exception(dhd.errorGetLast())(
                            op='forcedimension.drd.moveToGrip()',
                            ID=self._parent._id
                        )

            return self

        def track_grip(self,
            *cmds: Union[float, Iterable[float]]
        ):
            """
            This function sequentially sends the device gripper to a set of
            desired opening distances (in [m]). If motion filters are enabled,
            the motion follows a smooth acceleration/deceleration. The
            acceleration and velocity profiles can be controlled by adjusting
            the trajectory generation parameters.

            :param Union[float, Iterable[float]] *cmds:
                An iterable where each element is a target gripper opening
                distance or an iterable of target gripper opening distances.
                Gripper opening distances are in [m].
            """

            for cmd in cmds:
                if isinstance(cmd, Iterable):
                    for point in cmd:
                        if drd.trackGrip(point, self._parent._id):
                            dhd.errno_to_exception(dhd.errorGetLast())(
                                op='forcedimension.drd.trackGrip()',
                                ID=self._parent._id
                            )
                else:
                    if drd.trackGrip(cmd, self._parent._id):
                        dhd.errno_to_exception(dhd.errorGetLast())(
                            op='forcedimension.drd.trackGrip()',
                            ID=self._parent._id
                        )

            return self

        def hold(self):
            """
            This function immediately makes the device hold its current
            position. All motion commands are abandoned.
            """

            if drd.hold(self._parent._id):
                raise dhd.errno_to_exception(dhd.errorGetLast())(
                    op='forcedimension.drd.hold()', ID=self._parent._id
                )

            return self

        def lock(self, enabled: bool = True):
            """
                If `enabled` is `True`, the device moves to its park position
                engages the mechanical locks. If `enabled` is `False`, the
                mechanical locks are disengaged. It is recommended to follow
                engaging locks with
                `:func:forcedimension.HapticDevice.Regulator.stop()`.

                :param bool enabled:
                    `True` to park and engage mechanical locks, `False` to
                    disengage locks.

                :raises ArgumentError:
                    If enabled is not implicitly converitble to C uchar.

                :raises DHDErrorNotAvailable:
                    If the device is not equipped with mechanical locks.
            """

            if drd.lock(enabled, False, self._parent._id):
                raise dhd.errno_to_exception(dhd.errorGetLast())(
                    op='forcedimension.drd.hold()', ID=self._parent._id
                )

            return self

        _config_setters: Dict[str, Callable[..., Any]] = {
            'is_pos_regulated': regulate_pos,

            'is_rot_regulated': regulate_rot,

            'is_grip_regulated': regulate_grip,

            'motor_ratio_max': set_mot_ratio_max,

            'enc_move_param':
                lambda self, value: self.set_enc_move_param(**value),

            'enc_track_param':
                lambda self, value: self.set_enc_track_param(**value),

            'pos_move_param':
                lambda self, value: self.set_pos_move_param(**value),

            'pos_track_param':
                lambda self, value: self.set_pos_track_param(**value),

            'rot_move_param':
                lambda self, value: self.set_rot_move_param(**value),

            'rot_track_param':
                lambda self, value: self.set_rot_track_param(**value),

            'grip_move_param':
                lambda self, value: self.set_grip_move_param(**value),

            'grip_track_param':
                lambda self, value: self.set_grip_track_param(**value)
        }

        _config_initializers = {
            'motor_ratio_max': _init_motor_ratio_max,
            'enc_move_param': _init_enc_move_param,
            'enc_track_param': _init_enc_track_param,
            'pos_move_param': _init_pos_move_param,
            'pos_track_param': _init_pos_track_param,
            'rot_move_param': _init_rot_move_param,
            'rot_track_param': _init_rot_track_param,
            'grip_move_param': _init_grip_move_param,
            'grip_track_param': _init_grip_track_param
        }

    class _Expert:
        def __init__(self, parent):
            self._parent: HapticDevice = parent

        def set_timeguard(self, interval: int = dhd.DEFAULT_TIMEGUARD_US):
            if dhd.expert.setTimeGuard(interval, self._parent._id):
                raise dhd.errno_to_exception(dhd.errorGetLast())(
                    op='forcedimension.dhd.expert.setTimeGuard()',
                    ID=self._parent._id
                )

        def set_com_mode(self, mode: dhd.ComMode = dhd.ComMode.ASYNC):
            if dhd.expert.setComMode(mode, self._parent._id):
                raise dhd.errno_to_exception(dhd.errorGetLast())(
                    op='forcedimension.dhd.expert.setComMode()',
                    ID=self._parent._id
                )

            self._parent._config.com_mode = dhd.com_mode_str(mode)

    def __init__(
            self,
            *,
            ID: Optional[int] = None,
            devtype: Optional[dhd.DeviceType] = None,
            serial_number: Optional[int] = None,
            ensure_memory: bool = False,
            config: Optional[HapticDeviceConfig] = None,
            config_file: Optional[str] = None,
            restore_defaults: bool = False,
            name: Optional[str] = None,
            **kwargs
    ):
        """
        Create a handle to a Force Dimension haptic device. You may specify
        an ID, device type, or serial number to more precisely control which
        device is opened. If none are specified, the first device found is
        opened. It is recommended to use context management
        (i.e. using a `with` statement) to ensure the device is properly
        closed. Once opened, properties of the device are stored as fields
        in the class. Functions that update internal state of the Haptic Device
        do not allocate memory.

        :param Optional[int] ID:
            If specified, will open the device of the given ID.

        :param Optional[DeviceType] devtype:
            If specified, will open the first device of the give type.

        :param Optional[int] serial_number:
            If specified, will open the device of the given serial number.
            This feature is only available on newer devices.

        :param int wait_for_reset:
            If specified, will call
            :func:`forcedimension.HapticDevice.wait_for_reset()` with
            the value given. If None is given, will wait indefinetely. This
            ensures the device is calibrated before use.

        :param bool ensure_memory:
            If `True`, the device will fail to instantiate if a firmware or
            internal configuration health check fails.

        :param HapticDeviceConfig config:
            Initial config settings for the Haptic Device. Overrides settings
            set by `config_file`. Settings unset by both `config` and
            `config_file` are unchanged or, if
            `restore_defaults` is `True`, reset to their defaults.

        :param HapticDeviceConfig config_file:
            Initial config settings for the Haptic Device in a `.json` or
            `.yml` file. Is overridden by settings set in `config`.
            Settings unset by both `config` and
            `config_file` are unchanged or, if
            `restore_defaults` is `True`, reset to their defaults.

        :param bool restore_defaults:
            If `True` will restore all config settings not set by either
            `config` or `config_file`

        :raises ValueError:
            If more than one of `ID`, `devtype`, or `serial_number` are
            specified.

        :raises ValueError:
            If ID is specified and is less than 0.

        :raises DHDErrorNoDeviceFound:
            If no devices were found.

        :raises DHDIOError:
            If communications were interrupted while opening the device.
        """

        if [ID, devtype, serial_number].count(None) < 2:
            raise ValueError(
                "At most 1 of ID, devtype, or serial_number may be specified."
            )

        if ID is not None:
            if ID < 0:
                raise ValueError("ID must be greater than 0.")

        if config_file is not None:
            is_yaml = config_file.endswith(".yml")
            is_json = config_file.endswith(".json")

            if not (is_yaml or is_json):
                raise ValueError(
                    "The config file must be a .yml or .json file"
                )

        self._id = 0
        self._serial_number = -1
        self._devtype = dhd.DeviceType.NONE

        self._exception: Optional[dhd.DHDIOError] = None

        self._expert = HapticDevice._Expert(self)

        if (dhd.getDeviceCount() <= 0):
            raise dhd.DHDErrorNoDeviceFound()

        if (ID is None) and (devtype is None) and (serial_number is None):
            # Open the first device we find.

            if (id := drd.open()) == -1:
                raise dhd.errno_to_exception(dhd.errorGetLast())()

            self._id = id

            if (devtype_opened := dhd.getSystemType()) == -1:
                raise dhd.errno_to_exception(dhd.errorGetLast())(
                    ID=self._id,
                    op='forcedimension.dhd.getSystemType()'
                )

            self._devtype = devtype_opened
        elif ID is not None:
            # Open device of given ID.

            if (id := drd.openID(ID)) == -1:
                raise dhd.errno_to_exception(dhd.errorGetLast())()

            self._id = id

            if (devtype_opened := dhd.getSystemType()) == -1:
                raise dhd.errno_to_exception(dhd.errorGetLast())(
                    ID=self._id,
                    op='forcedimension.dhd.getSystemType()'
                )

            self._devtype = devtype_opened
        elif devtype is not None:
            # Open first device of given devtype.

            if (id := dhd.openType(devtype)) == -1:
                raise dhd.errno_to_exception(dhd.errorGetLast())()

            self._id = id

            if dhd.close(self._id) == -1:
                raise dhd.errno_to_exception(dhd.errorGetLast())(
                    ID=self._id,
                    op='forcedimension.dhd.close()'
                )

            if drd.openID(self._id) == -1:
                raise dhd.errno_to_exception(dhd.errorGetLast())()

            self._devtype = devtype
        elif serial_number is not None:
            if (id := dhd.openSerial(serial_number)) == -1:
                raise dhd.errno_to_exception(dhd.errorGetLast())()

            if dhd.close(self._id) == -1:
                raise dhd.errno_to_exception(dhd.errorGetLast())(
                    ID=self._id,
                    op='forcedimension.dhd.close()'
                )

            if drd.openID(self._id) == -1:
                raise dhd.errno_to_exception(dhd.errorGetLast())()

            if (devtype_opened := dhd.getSystemType()) == -1:
                raise dhd.errno_to_exception(dhd.errorGetLast())(
                    ID=self._id,
                    op='forcedimension.dhd.getSystemType()'
                )

            self._devtype = devtype_opened

        if ensure_memory:
            if not self.check_controller_memory():
                raise dhd.DHDErrorConfiguration(ID=self._id)

        if 'wait_for_reset' in kwargs:
            self.wait_for_reset(kwargs['wait_for_reset'])

        self._config = HapticDeviceConfig()

        config_file_data = None
        config_data = None

        # Load in the config file
        if config_file is not None:
            with open(config_file, 'r') as f:
                if is_yaml:  # type: ignore
                    config_file_data = yaml.safe_load(f)
                elif is_json:   # type: ignore
                    config_file_data = json.load(f)

            self._config.model_validate(config_file_data)  # type: ignore
            self._config.model_construct(config_file_data)  # type: ignore
            config_data = self._config.model_dump(exclude_defaults=True)

        # Load in the provided config
        if config is not None:
            config_data =  config.model_dump(exclude_defaults=True)

        self._init_config(config_file_data, config_data, restore_defaults)

        self._mass = nan
        self._gripper = None

        self._encs = containers.DOFIntArray()
        self._encs_v = containers.DOFFloatArray()

        self._joint_angles = containers.DOFFloatArray()
        self._joint_v = containers.DOFFloatArray()

        self._pos = containers.Vector3()
        self._v = containers.Vector3()

        self._orientation_angles = containers.Vector3()

        self._w = containers.Vector3()

        self._f = containers.Vector3()
        self._t = containers.Vector3()

        self._delta_jacobian = containers.Mat3x3()
        self._wrist_jacobian = containers.Mat3x3()
        self._frame = containers.Mat3x3()

        self._inertia_matrix = containers.Mat6x6()

        self._status = Status()

        self._f_req = containers.Vector3()
        self._t_req = containers.Vector3()
        self._vibration_req: List[float] = [0.] * 2
        self._buttons = 0

        self._delta_joint_angles_view = ImmutableWrapper(
            self._joint_angles.delta
        )

        self._wrist_joint_angles_view = ImmutableWrapper(
            self._joint_angles.wrist
        )

        self._base_angles_view = ImmutableWrapper(self._config.base_angles)

        self._pos_view = ImmutableWrapper(self._pos)
        self._w_view = ImmutableWrapper(self._w)
        self._v_view = ImmutableWrapper(self._v)
        self._f_view = ImmutableWrapper(self._f)
        self._t_view = ImmutableWrapper(self._t)

        self._delta_jacobian_view = ImmutableWrapper(self._delta_jacobian)
        self._wrist_jacobian_view = ImmutableWrapper(self._wrist_jacobian)
        self._frame_view = ImmutableWrapper(self._frame)

        self._inertia_matrix_view = ImmutableWrapper(self._inertia_matrix)

        self._status_view = ImmutableWrapper(self._status)

        self._is_neutral = False
        self._is_stopped = False

        # Get all device specs.

        handedness = dhd.handedness(self._devtype)
        has_base = dhd.hasBase(self._id)
        has_wrist = dhd.hasWrist(self._id)
        has_active_wrist = dhd.hasActiveWrist(self._id)
        has_gripper = True if self._gripper is not None else False
        has_active_gripper = dhd.hasActiveGripper(self._id)
        num_dof = dhd.num_dof(self._devtype)

        serial_number, err = dhd.getSerialNumber(self._id)

        if err:
            if (errno := dhd.errorGetLast()) != ErrorNum.NOT_AVAILABLE:
                raise dhd.errno_to_exception(errno)(
                    op='forcedimension.dhd.getSerialNumber()',
                    ID=self._id
                )
            else:
                serial_number = 0

        self._specs = HapticDeviceSpecs(
            name=name,
            devtype=self._devtype,
            serial_number=self._serial_number,
            has_base=has_base,
            has_wrist=has_wrist,
            has_active_wrist=has_active_wrist,
            has_gripper=has_gripper,
            has_active_gripper=has_active_gripper,
            num_dof=num_dof,
            handedness=handedness
        )

        self._regulator = HapticDevice.Regulator(
            self, config_file_data, config_data, restore_defaults
        )

        self._mock_gripper = HapticDevice.Gripper(
            self, config_file_data, config_data, restore_defaults
        )
        if dhd.hasGripper(self._id):
            self._gripper = self._mock_gripper

        # Pre-compute and cache the spec string since it doesn't change
        self._spec_str = textwrap.dedent(
            f"""\
            Name: {self._specs.name if self._specs.name is not None else ""}
            Device Model: {
                dhd.devtype_str(self._specs.devtype)
            }
            Serial Number: {
                self._specs.serial_number
            }
            Has Base: {
                "Yes" if self._specs.has_base else "No"
            }
            Has Wrist: {
                "Yes" if self._specs.has_wrist else "No"
            }
            Has Active Wrist: {
                "Yes" if self._specs.has_active_wrist else "No"
            }
            Has Gripper: {
                "Yes" if self._specs.has_gripper else "No"
            }
            Has Active Gripper: {
                "Yes" if self._specs.has_active_wrist else "No"
            }
            Degrees of Freedom: {self._specs.num_dof}
            Handedness: {dhd.handedness_str(self._specs.handedness)}
            Supports DRD: {"Yes" if self._specs.is_drd_supported else "No"}\
            """
        )

        self.update_delta_encs_and_calculate()
        self.update_force_and_torque_and_gripper_force()



        if self._specs.has_wrist:
            pos_updater = self.update_position_and_orientation
        else:
            pos_updater = self.update_position

        self._update_list = [
            pos_updater,
            self.update_velocity,
            self.update_angular_velocity,
            self.update_buttons,
            self.update_force_and_torque_and_gripper_force
        ]

    def _init_force_enabled(self, restore: bool = False):
        if restore:
            self.enable_force()

    def _init_brake_enabled(self, restore: bool = False):
        if restore:
            self.enable_brakes()

    def _init_linear_velocity_estimator(self, restore: bool = False):
        if restore:
            self.config_linear_velocity()

    def _init_angular_velocity_estimator(self, restore: bool = False):
        if restore:
            self.config_angular_velocity()

    def _init_base_angles(self, restore: bool = False):
        if restore:
            self.set_base_angles(0., 0., 0.)
            return

        if dhd.getBaseAngleXRad(
            self._config.base_angles.ptrs[0].contents, self._id
        ):
            raise dhd.errno_to_exception(dhd.errorGetLast())(
                op='forcedimension.dhd.getBaseAngleXRad()',
                ID=self._id
            )

        if dhd.getDeviceAngleRad(
            self._config.base_angles.ptrs[1].contents, self._id
        ):
            raise dhd.errno_to_exception(dhd.errorGetLast())(
                op='forcedimension.dhd.getDeviceAngleRad()',
                ID=self._id
            )

        if dhd.getBaseAngleZRad(
            self._config.base_angles.ptrs[2].contents, self._id
        ):
            raise dhd.errno_to_exception(dhd.errorGetLast())(
                op='forcedimension.dhd.getBaseAngleZRad()',
                ID=self._id
            )

    def _init_com_mode(self, restore: bool = False):
        if restore:
            self._expert_set_com_mode()
            return

        if (com_mode := dhd.getComMode(self._id)) == -1:
            raise dhd.errno_to_exception(dhd.errorGetLast())(
                ID=self._id, op='forcedimension.dhd.getComMode()'
            )

        self._config.com_mode = dhd.com_mode_str(com_mode)

    def _init_max_force(self, restore: bool = False):
        if restore:
            self.set_max_force(None)
            return

        if (max_force := dhd.getMaxForce(self._id)) < 0:
            max_force = None

        self._config.max_force = max_force

    def _init_max_torque(self, restore: bool = False):
        if restore:
            self.set_max_torque(None)
            return

        if (max_torque := dhd.getMaxTorque(self._id)) < 0:
            max_torque = None

        self._config.max_torque = max_torque

    def _init_mass(self, restore: bool = False):
        self._config.mass, err = dhd.getEffectorMass(self._id)
        if err:
            raise dhd.errno_to_exception(dhd.errorGetLast())()

    def _init_config(
        self,
        config_file_data: Optional[Dict[str, Any]],
        config_data: Optional[Dict[str, Any]],
        restore: bool
    ):
        unset_configs = {
            'is_force_enabled',
            'is_brake_enabled',
            'linear_velocity_estimator',
            'angular_velocity_estimator',
            'base_angles',
            'com_mode',
            'max_force',
            'max_torque',
            'mass'
        }

        if config_file_data is not None and config_data is not None:
            preserved_keys = set(
                set(config_file_data.keys()) ^ set(config_data.keys())
            )
            overwritten_keys = set(
                set(config_file_data.keys()) & set(config_data.keys())
            )

            preserved_keys -= {'regulator', 'gripper'}
            overwritten_keys -= {'regulator', 'gripper'}

            for key in preserved_keys:
                HapticDevice._config_setters[key](self, config_file_data[key])

            for key in overwritten_keys:
                HapticDevice._config_setters[key](self, config_data[key])

            unset_configs -= preserved_keys
            unset_configs -= overwritten_keys

        elif config_file_data is not None and config_data is None:
            self.set_config(config_file_data)
            unset_configs -= set(config_file_data.keys())

        elif config_file_data is None and config_data is not None:
            self.set_config(config_data)
            unset_configs -= set(config_data.keys())

        # Get all remaining config information
        for config in unset_configs:
            HapticDevice._config_initializer[config](self, restore)

    # def set_config(self, config: HapticDeviceConfig):
        # ...

    def set_config(self, config_data: Dict[str, Any]):
        for key, value in config_data:
            HapticDevice._config_setters[key](self, value)

    def check_exception(self):
        """
        Checks if an exception has occured in an update function, invalidating
        the device's internal state.
        """
        if self._exception is not None:
            raise self._exception

    def dump_specs(self):
        """
        Dumps the specifictions of the Force Dimension Haptic Device to a
        serializable Python dictionary.
        """

        return self._specs.model_dump()

    def dump_config(self, exclude_defaults: bool = False):
        """
        Dumps the current configuration as a serializable Python dictionary.
        """

        return self._config.model_dump(exclude_defaults=exclude_defaults)

    @property
    def spec_str(self):
        """
        A human-readable string that summarizes the specifications of the
        HapticDevice.
        """

        return self._spec_str

    def get_config_str(self):
        """
        A human-readable string that summarizes the current configuration of
        the HapticDevice.
        """

        base_angles = copy(self._config.base_angles)
        base_angles[0] /= 2 * math.pi
        base_angles[1] /= 2 * math.pi
        base_angles[2] /= 2 * math.pi

        return (textwrap.dedent(f"""\
            Force Enabled: {
                "Yes (default)" if self._config.is_force_enabled else "No"
            }
            Brakes Enabled: {
                "Yes (default)" if self._config.is_brake_enabled else "No"
            }
            Gravity Compensation Enabled: {
                "Yes"
                if self._config.is_gravity_compensation_enabled else
                "No (default)"
            }
            Button Emulation Enabled: {
                "Yes" if self._config.is_button_emulation_enabled
                else "No (default)"
            }
            Base Angles (rev): {base_angles}
            Communication Mode: {
                "async (default)" if self._config.com_mode == 'async' else
                self._config.com_mode
            }
            Timeguard: {
                'default' if self._config.timeguard == dhd.DEFAULT_TIMEGUARD_US
                else (
                    'None' if self._config.timeguard == 0 else
                    self._config.timeguard
                )
            }
            Linear Velocity Estimator:
                mode: {
                    dhd.velocity_estimator_mode_str(
                        self._config.linear_velocity_estimator.mode
                    )
                }{
                    " (default)"
                    if self._config.linear_velocity_estimator.mode ==
                    dhd.VelocityEstimatorMode.WINDOWING
                    else ""
                }
                window size: {
                    self._config.linear_velocity_estimator.window_size
                } us{
                    " (default)"
                    if self._config.linear_velocity_estimator.window_size ==
                    dhd.DEFAULT_VELOCITY_WINDOW else ""
                }
            Angular Velocity Estimator:
                mode: {
                    dhd.velocity_estimator_mode_str(
                        self._config.angular_velocity_estimator.mode
                    )
                }{
                    " (default)"
                    if self._config.angular_velocity_estimator.mode ==
                    dhd.VelocityEstimatorMode.WINDOWING
                    else ""
                }
                window size: {
                    self._config.angular_velocity_estimator.window_size
                } us{
                    " (default)"
                    if self._config.angular_velocity_estimator.window_size ==
                    dhd.DEFAULT_VELOCITY_WINDOW else ""
                }
            Max Force: {self._config.max_force}{
                " N" if self._config.max_force is not None else ""
            }{
                " (default)" if self._config.max_force is None else ""
            }
            Max Torque: {self._config.max_torque}{
                " Nm" if self._config.max_torque is not None else ""
            }{
                " (default)" if self._config.max_torque is None else ""
            }
            Standard Gravity: {self._config.standard_gravity} m/s^2{
                " (default)" if self._config.standard_gravity == 9.81 else ""
            }

            Gripper:
                Max Force: {self._config.max_force}{
                    " N" if self._config.gripper.max_force is not None else ""
                }{
                " (default)" if self._config.gripper.max_force is None else ""
            }
                Velocity Estimator:
                    mode: {
                        dhd.velocity_estimator_mode_str(
                            self._config.gripper.velocity_estimator.mode
                        )
                    }{
                        " (default)" if
                        self._config.gripper.velocity_estimator.mode ==
                        dhd.VelocityEstimatorMode.WINDOWING else ""
                    }
                    window size: {
                        self._config.gripper.velocity_estimator.window_size
                    } us{
                    " (default)"
                    if self._config.gripper.velocity_estimator.window_size ==
                    dhd.DEFAULT_VELOCITY_WINDOW else ""
                }

            Regulator:
                Position Regulation: {
                    "enabled (default)"
                    if self._config.regulator.is_pos_regulated else
                    "disabled"
                  }
                Rotation Regulation: {
                    "enabled (default)"
                    if self._config.regulator.is_rot_regulated else
                    "disabled"
                  }
                Gripper Regulation: {
                    "enabled (default)"
                    if self._config.regulator.is_grip_regulated else
                    "disabled"
                  }
                Max Motor Ratio: {self._config.regulator.max_motor_ratio}{
                    " (default)" if self._config.regulator.max_motor_ratio == 1
                    else ""
                }

                Trajectory Generation Parameters:
                    Encoder Track (inc): {
                        self._config.regulator.enc_track_param.pretty_str(
                            drd.DEFAULT_ENC_TRACK_PARAMS.vmax,
                            drd.DEFAULT_ENC_TRACK_PARAMS.amax,
                            drd.DEFAULT_ENC_TRACK_PARAMS.jerk
                        )
                    }
                    Encoder Move (inc): {
                        self._config.regulator.enc_move_param.pretty_str(
                            drd.DEFAULT_ENC_MOVE_PARAMS.vmax,
                            drd.DEFAULT_ENC_MOVE_PARAMS.amax,
                            drd.DEFAULT_ENC_MOVE_PARAMS.jerk
                        )
                    }

                    Position Track (m): {
                        self._config.regulator.pos_track_param.pretty_str(
                            drd.DEFAULT_POS_TRACK_PARAMS.vmax,
                            drd.DEFAULT_POS_TRACK_PARAMS.amax,
                            drd.DEFAULT_POS_TRACK_PARAMS.jerk
                        )
                    }
                    Position Move (m): {
                        self._config.regulator.pos_move_param.pretty_str(
                            drd.DEFAULT_POS_MOVE_PARAMS.vmax,
                            drd.DEFAULT_POS_MOVE_PARAMS.amax,
                            drd.DEFAULT_POS_MOVE_PARAMS.jerk
                        )
                    }

                    Rotation Track (rad): {
                        self._config.regulator.rot_track_param.pretty_str(
                            drd.DEFAULT_ROT_TRACK_PARAMS.vmax,
                            drd.DEFAULT_ROT_TRACK_PARAMS.amax,
                            drd.DEFAULT_ROT_TRACK_PARAMS.jerk
                        )
                    }
                    Rotation Move (rad): {
                        self._config.regulator.rot_move_param.pretty_str(
                            drd.DEFAULT_ROT_MOVE_PARAMS.vmax,
                            drd.DEFAULT_ROT_MOVE_PARAMS.amax,
                            drd.DEFAULT_ROT_MOVE_PARAMS.jerk
                        )
                    }

                    Gripper Track (m): {
                        self._config.regulator.grip_track_param.pretty_str(
                            drd.DEFAULT_GRIP_TRACK_PARAMS.vmax,
                            drd.DEFAULT_GRIP_TRACK_PARAMS.amax,
                            drd.DEFAULT_GRIP_TRACK_PARAMS.jerk
                        )
                    }
                    Gripper Move (m): {
                        self._config.regulator.grip_move_param.pretty_str(
                            drd.DEFAULT_GRIP_MOVE_PARAMS.vmax,
                            drd.DEFAULT_GRIP_MOVE_PARAMS.amax,
                            drd.DEFAULT_GRIP_MOVE_PARAMS.jerk
                        )
                    }
            """

        ))

    def get_summary_str(self) -> str:
        """
        Constructs a human-readable string that summarizes the HapticDevice
        (its specifications and configuration)
        """

        return (
            textwrap.dedent(
                """\
                Specifications
                --------------
                """
            ) +
            self._spec_str +
            textwrap.dedent(
                """


                Configuration
                -------------
                """
            ) +
            self.get_config_str()
        )

    def print_summary(self):
        """
        Prints a human-readable string that summarizes the HapticDevice (its
        specifications and configuration)
        """

        print(self.get_summary_str())


    @property
    def expert(self) -> _Expert:
        """
        A wrapper for the expert level functionality. Unlike the C++ API, the
        Python bindings enable expert mode by default and simply obscures them
        via scoping.
        """
        return self._expert

    @property
    def gripper(self) -> Optional[Gripper]:
        return self._gripper

    @property
    def regulator(self) -> Regulator:
        return self._regulator

    @property
    def ID(self) -> int:
        """
        Provides a read-only copy of the ID of the HapticDevice.
        Thread-safe.
        """

        self.check_exception()
        return self._id

    @property
    def devtype(self) -> dhd.DeviceType:
        return self._devtype

    @property
    def timeguard(self) -> int:
        return self._config.timeguard

    @property
    def mass(self) -> float:
        """
        Get the mass of the end-effector used for gravity compensation
        (in [kg]). Thread-safe.

        :raises DHDError:
            If an error has occured with the device, invalidating the
            device state.

        :returns: the set mass of the end-effector (in [kg])
        """

        return self._mass

    def set_mass(self, m: float):
        """
        Sets the mass of the end-effector used for gravity compensation
        (in [kg]). Thread-safe.

        :raises DHDError:
            If the operation failed.
        """
        if dhd.setEffectorMass(m, ID=self._id):
            raise dhd.errno_to_exception(dhd.errorGetLast())()

    @property
    def standard_gravity(self) -> float:
        return self._standard_gravity

    @property
    def left_handed(self) -> bool:
        return self._specs.handedness == Handedness.LEFT

    @property
    def handedness(self) -> Handedness:
        return self._specs.handedness

    @property
    def has_base(self) -> bool:
        return self._specs.has_base

    @property
    def has_wrist(self) -> bool:
        return self._specs.has_wrist

    @property
    def has_active_wrist(self) -> bool:
        return self._specs.has_active_wrist

    @property
    def has_gripper(self) -> bool:
        return self._specs.has_gripper

    @property
    def has_active_gripper(self) -> bool:
        return self._specs.has_active_gripper

    def _expert_set_com_mode(self, mode: dhd.ComMode = dhd.ComMode.ASYNC):
        self._expert.set_com_mode(mode)

    @property
    def com_mode(self) -> dhd.ComMode:
        return dhd.com_mode_from_str(self._config.com_mode)

    @property
    def is_neutral(self) -> bool:
        return self._is_neutral

    @property
    def is_stopped(self) -> bool:
        return self._is_stopped

    @property
    def button_emulation_enabled(self) -> bool:
        return self._button_emulation_enabled

    @property
    def status(self) -> Status:
        """
        Provides a read-only reference to the last-known status of the device.
        Thread-safe.

        :returns:
            A Status object representing the last-known status of the device.
        """
        return _cast(Status, self._status_view)

    @property
    def base_angles(self) -> containers.Vector3:
        """
        Provides a read-only reference of the device base plate angle
        (in [rad]) about the X, Y, and Z axes.
        """

        return _cast(containers.Vector3, self._base_angles_view)

    def set_base_angles(
        self,
        x: Optional[float] = None,
        y: Optional[float] = None,
        z: Optional[float] = None
    ):
        """
        Sets the device base plate angles (in [rad]) about the X, Y, and Z
        axes. If an angle is not specified, it is not set.

        :param Optional[float] x:
            Angle (in [rad]) to set the device plate angle about the X axis to.

        :param Optional[float] y:
            Angle (in [rad]) to set the device plate angle about the Y axis to.

        :param Optional[float] z:
            Angle (in [rad]) to set the device plate angle about the Z axis to.
        """
        if x is not None:
            self._config.base_angles[0] = x

            if dhd.setBaseAngleXRad(x):
                raise dhd.errno_to_exception(dhd.errorGetLast())(
                    op='forcedimension.dhd.setBaseAngleXRad()',
                    ID=self._id
                )

        if y is not None:
            self._config.base_angles[1] = y

            if dhd.setDeviceAngleRad(y):
                raise dhd.errno_to_exception(dhd.errorGetLast())(
                    op='forcedimension.dhd.setDeviceAngleRad()',
                    ID=self._id
                )

        if z is not None:
            self._config.base_angles[2] = z

            if dhd.setBaseAngleZRad(z):
                raise dhd.errno_to_exception(dhd.errorGetLast())(
                    op='forcedimension.dhd.setBaseAngleZRad()',
                    ID=self._id
                )

    def set_output_bits(self, mask: int):
        """
        Sets the user programmable output bbits on devices that support it.

        :param int mask:
            Bitwise mask that toggles the progammable output bits.

        :raises ArgumentError:
            If `mask` is not implicitly convertible to a C uint.
        """

        if dhd.setOutput(mask, self._id):
            raise dhd.errno_to_exception(dhd.errorGetLast())

    def set_standard_gravity(self, g: float = 9.81):
        """
        Sets the standard gravity constant (in [m/s^2]) used in gravity compensation.
        By default, the constant is set to 9.81 m/s^2

        :param float g:
            Standard gravity constant to set (in [m/s^2])

        :raises ArgumentError:
            If `g` is not implicitly convertible to C double.
        """

        if dhd.setStandardGravity(g, self._id):
            raise dhd.errno_to_exception(dhd.errorGetLast())

        self._standard_gravity = g

    def enable_button_emulation(self, enabled: bool = True):
        """
        Enables the button behavior emulation in devices that feature a
        gripper.

        :param bool enabled:
            `True` to enable button emulation, `False` to disable.

        Info
        ----
        Omega.7 devices with firmware version 2.x need to be enabled for the
        button emulation to report the emulated button status.
        """

        self._button_emulation_enabled = enabled

        if dhd.emulateButton(enabled, self._id):
            raise dhd.errno_to_exception(dhd.errorGetLast())


    def enable_force(self, enabled: bool = True):
        """
        Enable/disable force on the end-effector.

        :param bool enabled: `True` to enable, `False` to disable
        """

        dhd.enableForce(enabled, ID=self._id)

    def enable_brakes(self, enabled: bool = True):
        """
        Enable electromagnetic braking on the device. If the brakes are
        disabled, the device is forced into IDLE mode. No forces will be
        applied in that mode.

        :param enabled bool:
            `True` to enable electromagnetic braking, `False` to disable.
        """

        if dhd.setBrakes(enabled):
            raise dhd.errno_to_exception(dhd.errorGetLast())(
                ID=self._id,
                op='forcedimension.dhd.setBrakes()'
            )

    def enable_gravity_compensation(self, enabled: bool = True):
        """
        Enable built-in gravity compensation for the end-effector that will be
        added on top of the force request.

        :param bool enabled:
            `True` to enable, `False` to disable.

        See Also
        --------
        :func:`forcedimension.HapticDevice.set_mass`
        """

        dhd.setGravityCompensation(enabled, ID=self._id)

        self._f_req[0] = 0.
        self._f_req[1] = 0.
        self._f_req[2] = 0.

        self._t_req[0] = 0.
        self._t_req[1] = 0.
        self._t_req[2] = 0.

        dhd.stop(self._id)

    def config_linear_velocity(
        self,
        window_size: int = dhd.DEFAULT_VELOCITY_WINDOW,
        mode: dhd.VelocityEstimatorMode = dhd.VelocityEstimatorMode.WINDOWING
    ):
        """
        Configures the internal linear velocity estimator used by
        the Force Dimension SDK for the end-effector. Calling this without
        parameters resets the linear velocity estimator to its default
        settings.

        :param window_size int:
            Time interval to use for computing linear velocity (in [ms]).

        :param VelocityEstimatorMode mode:
            Velocity estimator mode. Currently only
            :data:`forcedimension.dhd.VelocityEstimatorMode.WINDOWING` is
            supported by the Force Dimension SDK.
        """

        if dhd.configLinearVelocity(window_size, mode, self._id):
            raise dhd.errno_to_exception(dhd.errorGetLast())(
                op='forcedimension.dhd.configLinearVelocity()', ID=self._id
            )

        self._config.linear_velocity_estimator.window_size = window_size
        self._config.linear_velocity_estimator.mode = mode


    def config_angular_velocity(
        self,
        window_size: int = dhd.DEFAULT_VELOCITY_WINDOW,
        mode: dhd.VelocityEstimatorMode = dhd.VelocityEstimatorMode.WINDOWING
    ):
        """
        Configures the internal angular velocity estimator used by
        the Force Dimension SDK for the end-effector. Calling this without
        parameters resets the angular velocity estimator to its default
        settings.

        :param window_size int:
            Time interval to use for computing linear velocity (in [ms]).

        :param VelocityEstimatorMode mode:
            Velocity estimator mode. Currently only
            :data:`forcedimension.dhd.VelocityEstimatorMode.WINDOWING` is
            supported by the Force Dimension SDK.
        """

        if dhd.configAngularVelocity(window_size, mode, self._id):
            raise dhd.errno_to_exception(dhd.errorGetLast())(
                op='forcedimension.dhd.configAngularVelocity()', ID=self._id
            )

        self._config.angular_velocity_estimator.window_size = window_size
        self._config.angular_velocity_estimator.mode = mode

    @property
    def delta_joint_angles(self) -> containers.Vector3:
        self.check_exception()
        return _cast(containers.Vector3, self._delta_joint_angles_view)

    @property
    def wrist_joint_angles(self) -> containers.Vector3:
        self.check_exception()
        return _cast(containers.Vector3, self._wrist_joint_angles_view)

    @property
    def pos(self) -> containers.Vector3:
        """
        Provides a read-only reference to the last-known position of the
        HapticDevice's end-effector (in [m]) about the X, Y, and Z axes.
        Thread-safe.

        :raises DHDError:
            If an error has occured with the device, invalidating the
            device state.
        """

        self.check_exception()
        return _cast(containers.Vector3, self._pos_view)

    @property
    def v(self) -> containers.Vector3:
        """
        Provides a read-only reference to the last-known linear velocity of the
        HapticDevice's end-effector. Thread-safe.

        :raises DHDError:
            If an error has occured with the device, invalidating the
            device state.
        """

        self.check_exception()
        return _cast(containers.Vector3, self._v_view)

    @property
    def w(self) -> containers.Vector3:
        """
        Provides a read-only reference to the last-known angular velocity of the
        HapticDevice's end-effector. Thread-safe.

        :raises DHDError:
            If an error has occured with the device, invalidating the
            device state.
        """

        self.check_exception()
        return _cast(containers.Vector3, self._w_view)

    @property
    def t(self) -> containers.Vector3:
        """
        Provides a read-only reference to the last-known applied torque of the
        HapticDevice's end-effector. Thread-safe.

        :raises DHDError:
            If an error has occured with the device, invalidating the
            device state.
        """

        self.check_exception()
        return _cast(containers.Vector3, self._t_view)

    @property
    def f(self) -> containers.Vector3:
        """
        Provides a read-only reference to the last-known applied force of the
        HapticDevice's end-effector. Thread-safe.

        :raises DHDError:
            If an error has occured with the device, invalidating the
            device state.
        """

        self.check_exception()
        return _cast(containers.Vector3, self._f_view)

    @property
    def delta_jacobian(self) -> containers.Mat3x3:
        self.check_exception()
        return _cast(containers.Mat3x3, self._delta_jacobian_view)

    @property
    def wrist_jacobian(self) -> containers.Mat3x3:
        self.check_exception()
        return _cast(containers.Mat3x3, self._wrist_jacobian_view)

    @property
    def frame(self) -> containers.Mat3x3:
        self.check_exception()
        return _cast(containers.Mat3x3, self._frame_view)

    @property
    def inertia_matrix(self) -> containers.Mat6x6:
        self.check_exception()
        return _cast(containers.Mat6x6, self._inertia_matrix_view)

    def set_update_list(self, lst: List[Callable[..., Any]]):
        self._update_list = lst

    def update(self):
        for updater in self._update_list:
            updater()

        return self

    def calculate_pos(self):
        """
        Calculates and stores the position of the device given the current
        value of the delta encoders in the internal buffer.
        """

        dhd.expert.direct.deltaEncoderToPosition(
            self._encs.delta, self._pos, self._id
        )

        return self

    def calculate_delta_joint_angles(self):
        """
        Calculates and stores the joint angles of the DELTA structure given
        the current end-effector encoder readings in the internal buffer.
        """

        dhd.expert.direct.deltaEncodersToJointAngles(
            self._encs.wrist, self._joint_angles.delta, self._id
        )

        return self

    def calculate_delta_jacobian(self):
        """
        Calculates and stores the Jacobian matrix of the DELTA structure given
        current joint angle configuration in the internal buffer.
        """

        self.calculate_delta_joint_angles()

        dhd.expert.direct.deltaJointAnglesToJacobian(
            self._joint_angles.delta, self._delta_jacobian, self._id
        )

        return self

    def calculate_wrist_joint_angles(self):
        """
        Calculates and stores the joint angles of the WRIST structure given
        the current end-effector encoder values in the internal buffer.
        """

        dhd.expert.direct.wristEncodersToJointAngles(
            self._encs.wrist, self._joint_angles.wrist, self._id
        )

        return self

    def calculate_wrist_jacobian(self):
        """
        Calculates and stores the Jacobian matrix of the WRIST structure given
        current joint angle configuration in the internal buffer.
        """

        dhd.expert.direct.wristJointAnglesToJacobian(
            self._joint_angles.wrist, self._wrist_jacobian, self._id
        )

        return self

    def calculate_inertia_matrix(self):
        """
        Calculates the 6x6 inertia matrix (with respect to the X, Y, and Z
        axes) given the current joint angles configuration in the internal
        buffer.
        """

        dhd.expert.direct.jointAnglesToIntertiaMatrix(
            self._joint_angles, self._inertia_matrix, self._id
        )

        return self

    def update_encs_and_calculate(self):
        self.update_encs()
        self.calculate_pos()
        self.calculate_wrist_joint_angles()
        self.calculate_delta_jacobian()
        self.calculate_wrist_jacobian()

    def update_enc_velocities_and_calculate(self):
        self.update_enc_velocities()


    def update_delta_encs_and_calculate(self):
        """
        Updates the DELTA encoders and given those values, calculates the
        position of the end-effector, DELTA joint angles, and the DELTA
        jacobian.
        """

        self.update_delta_encs()
        self.calculate_pos()
        self.calculate_delta_joint_angles()
        self.calculate_delta_jacobian()

        return self

    def update_wrist_encs_and_calculate(self):
        """
        Updates the WRIST encoders and given those values, calculates the wrist
        joint angles and the WRIST jacobian.
        """

        self.update_wrist_encs()
        self.calculate_wrist_joint_angles()
        self.calculate_wrist_jacobian()

        return self

    def update_encs(self):
        """
        Performs a blocking read to the HapticDevice, requesting the current
        encoder readers of the DELTA structure that controls the end-effector
        and updates the last-known position with the response. The requested
        values are then loaded into an internal buffer.

        :raises DHDError:
            If an error has occured with the device.
        """

        err = dhd.expert.direct.getEnc(self._encs, 0xff, self._id)

        if err == -1:
            raise dhd.errno_to_exception(
                dhd.errorGetLast())(
                    ID=self._id,
                    op='forcedimension.dhd.expert.getEnc()'
            )

        return self


    def update_delta_encs(self):
        """
        Performs a blocking read to the HapticDevice, requesting the current
        encoder readers of the DELTA structure that controls the end-effector
        and updates the last-known position with the response. The requested
        values are then loaded into an internal buffer.

        :raises DHDError:
            If an error has occured with the device.
        """

        err = dhd.expert.direct.getDeltaEncoders(self._encs.delta, self._id)

        if err == -1:
            raise dhd.errno_to_exception(
                dhd.errorGetLast())(
                    ID=self._id,
                    op='forcedimension.dhd.expert.getDeltaEncoders()'
            )

        return self

    def update_wrist_encs(self):
        """
        Performs a blocking read to the HapticDevice, requesting the current
        encoder readers of the DELTA structure that controls the end-effector
        and updates the last-known position with the response. The requested
        values are then loaded into an internal buffer.

        :raises DHDError:
            If an error has occured with the device.
        """

        err = dhd.expert.direct.getWristEncoders(self._encs.wrist, self._id)

        if err == -1:
            raise dhd.errno_to_exception(
                dhd.errorGetLast())(
                    ID=self._id,
                    op='forcedimension.dhd.expert.getWristEncoders()'
            )

        return self

    def update_enc_velocities(self):
        """
        Performs a blocking read to the HapticDevice, requesting the current
        encoder readers of the DELTA structure that controls the end-effector
        and updates the last-known position with the response. The requested
        values are then loaded into an internal buffer.

        :raises DHDError:
            If an error has occured with the device.
        """

        err = dhd.expert.direct.getEncVelocities(self._encs_v, self._id)

        if err == -1:
            raise dhd.errno_to_exception(
                dhd.errorGetLast())(
                    ID=self._id,
                    op='forcedimension.dhd.expert.getEncVelocities()'
            )

        return self

    def update_joint_angles(self):
        err = dhd.expert.direct.getJointAngles(self._joint_angles, self._id)

        if err == -1:
            raise dhd.errno_to_exception(
                dhd.errorGetLast())(
                    ID=self._id,
                    op='forcedimension.dhd.expert.getJointAngles()'
            )

        return self

    def update_joint_angle_velocities(self):
        err = dhd.expert.direct.getJointVelocities(
            self._joint_v, self._id
        )

        if err == -1:
            raise dhd.errno_to_exception(
                dhd.errorGetLast())(
                    ID=self._id,
                    op='forcedimension.dhd.expert.getJointVelocities()'
            )

        return self

    def update_position(self):
        """
        Performs a blocking read to the HapticDevice, requesting the current
        position of the end-effector and updates the last-known position with
        the response. The requested values are then loaded into an internal
        buffer.

        :raises DHDError:
            If an error has occured with the device.
        """

        err = dhd.direct.getPosition(self._pos, self._id)

        if err == -1:
            raise dhd.errno_to_exception(dhd.errorGetLast())(
                ID=self._id,
                op='forcedimension.dhd.getPosition()'
            )

        return self

    def update_velocity(self):
        """
        Performs a blocking read to the HapticDevice, requesting the current
        linear velocity of the end-effector and updates the last-known linear
        velocity with the response. The requested values are then loaded into
        an internal buffer.

        :raises DHDError:
            If an error has occured with the device.
        """

        err = dhd.direct.getLinearVelocity(self._v, self._id)

        if err == -1:
            if dhd.errorGetLast() != ErrorNum.TIMEOUT:
                raise dhd.errno_to_exception(dhd.errorGetLast())(
                    ID=self._id,
                    op='forcedimension.dhd.getLinearVelocity()'
                )
            else:
                self._v[0] = nan
                self._v[1] = nan
                self._v[2] = nan

        return self

    def update_angular_velocity(self):
        """
        Performs a blocking read to the HapticDevice, requesting the current
        angular velocity of the end-effector and updates the last-known
        angular velocity with the response. The requested values are then
        loaded into an internal buffer.

        :raises DHDError:
            If an error has occured with the device.
        """

        err = dhd.direct.getAngularVelocityRad(self._w, self._id)

        if err:
            if dhd.errorGetLast() != ErrorNum.TIMEOUT:
                raise dhd.errno_to_exception(dhd.errorGetLast())(
                    ID=self._id,
                    op='forcedimension.dhd.getAngularVelocityRad()'
                )
            else:
                self._w[0] = nan
                self._w[1] = nan
                self._w[2] = nan

        return self

    def update_force(self):
        """
        Performs a blocking read to the HapticDevice, requesting the current
        force the end end-effector is experiencing and updates the last-known
        force with the response. The requested values are then loaded into an
        internal buffer.

        :raises DHDError:
            If an error has occured with the device.
        """

        err = dhd.getForce(self._f, self._id)

        if err == -1:
            raise dhd.errno_to_exception(dhd.errorGetLast())(
                ID=self._id,
                op='forcedimension.dhd.getForce()'
            )

        return self

    def update_force_and_torque(self):
        """
        Performs a blocking read to the HapticDevice, requesting in parallel
        the current force and torque applied to the end-effector and
        updates the last-known force and torque with the response. The
        requested values are then loaded into internal buffers.


        :raises DHDError:
            If an error has occured with the device.
        """

        if dhd.direct.getForceAndTorque(self._f, self._t, self._id):
            raise dhd.errno_to_exception(dhd.errorGetLast())(
                ID=self._id,
                op='forcedimension.dhd.getForceAndTorque()'
            )

        return self

    def update_force_and_torque_and_gripper_force(self):
        """
        Performs a blocking read to the HapticDevice, requesting in parallel
        the force and torque applied to the end-effector as well as the force
        applied to the gripper. The requested values are then
        loaded into internal buffers. This is equivalent to
        :func:`forcedimension.HapticDevice.update_force_and_torque()` if the
        device does not have a gripper.

        :raises DHDError:
            If an error has occured with the device.
        """

        err = dhd.direct.getForceAndTorqueAndGripperForce(
            self._f, self._t, self._mock_gripper._fg, self._id
        )

        if err:
            raise dhd.errno_to_exception(dhd.errorGetLast())(
                ID=self._id,
                op='forcedimension.dhd.getForceAndTorqueAndGripperForce()'
            )

        return self

    def update_orientation(self):
        """
        Performs a blocking read to the HapticDevice,
        its orientation frame matrix and updates an internal buffer with those
        values.
        """

        if dhd.direct.getOrientationFrame(self._frame, self._id):
            raise dhd.errno_to_exception(dhd.errorGetLast())(
                ID=self._id,
                op='forcedimension.dhd.getOrientationFrame()'
            )

        return self


    def update_position_and_orientation(self):
        """
        Performs a blocking read to the HapticDevice, requesting in parallel
        the position of the end effector (in [m]) about the X, Y, and Z axes
        as well as its orientation frame matrix. The requested values are then
        loaded into internal buffers.
        """

        err = dhd.direct.getPositionAndOrientationFrame(
            self._pos, self._frame, self._id
        )

        if err:
            raise dhd.errno_to_exception(dhd.errorGetLast())(
                ID=self._id,
                op='forcedimension.dhd.getPositionAndOrientationFrame()'
            )

        return self

    def update_buttons(self):
        """
        Performs a blocking read and gets the state of all buttons on the
        device into a int bit vector buffer.

        See Also
        --------
        :func:`HapticDevice.get_button`
        """

        self._buttons = dhd.getButtonMask(ID=self._id)

        return self

    def update_status(self):
        """
        Perform a blocking read to the HapticDevice, requesting all pertinent
        status information.

        :raises DHDError:
            If an error has occured with the device.

        :returns:
            :class:`forcedimension.dhd.adaptors.Status` object containing all
            status information.
        """

        if dhd.getStatus(self._status, ID=self._id):
            raise dhd.errno_to_exception(dhd.errorGetLast())()

        return self


    def submit(self, respect_neutral_stop: bool = True):
        """
        Push the requested forces and torques to the device in a blocking send.

        See Also
        --------
        :func:`HapticDevice.req`
        """
        if respect_neutral_stop and (self._is_neutral or self._is_stopped):
            return

        if (not respect_neutral_stop) and self._is_neutral:
            self._is_neutral = False
            self.enable_brakes()

        if (not respect_neutral_stop) and self._is_stopped:
            self._is_neutral = False
            self.enable_force()

        err = dhd.setForceAndTorqueAndGripperForce(
            self._f_req, self._t_req, self._mock_gripper._fg_req, self._id
        )

        if err == -1:
            raise dhd.errno_to_exception(dhd.errorGetLast())(
                ID=self._id,
                op='forcedimension.dhd.setForceAndTorqueAndGripperForce()'
            )

        return self

    def req(self, f: IntVectorLike, t: IntVectorLike = (0, 0, 0)):
        """
        Load the request force and request torque buffer for this device.
        This won't send the request to the device. This is used by the
        HapticDaemon.

        :param IntVectorLike f:
            The force (in [N]) to apply to the end effector about
            the X, Y, and Z axes.

        :param IntVectorLike t:
            The torque (in [Nm]) to apply to the end effector about the
            X, Y, and Z axes.

        See Also
        --------
        :func:`HapticDevice.submit`
        """
        if (self._is_neutral):
            self._is_neutral = False
            self.enable_brakes()

        self._f_req[0] = f[0]
        self._f_req[1] = f[1]
        self._f_req[2] = f[2]

        self._t_req[0] = t[0]
        self._t_req[1] = t[1]
        self._t_req[2] = t[2]

        return self

    def req_vibration(self, freq: float, amplitude: float):
        """
        Load the requested vibration into the vibration buffer. This won't
        send the request to the device. This is used by the HapticDaemon. This
        vibration will be added on top of the force requested to the device.

        :param float freq:
            Frequency of the vibration (in [Hz]).

        :param float amplitude:
            Amplitude of the vibration.

        See Also
        --------
        :func:`HapticDevice.submit_vibration`
        """

        self._vibration_req[0] = freq
        self._vibration_req[1] = amplitude

        return self

    def submit_vibration(self):
        err = dhd.setVibration(
            self._vibration_req[0], self._vibration_req[1], 0, self._id
        )

        if err == -1:
            raise dhd.errno_to_exception(dhd.errorGetLast())(
                ID=self._id,
                op='forcedimension.dhd.setVibration()'
            )

        return self

    def neutral(self):
        """
        Disable electromagnetic braking and put the device in IDLE mode,
        consequently disabling forces. A call to
        :func:`forcedimension.HapticDevice.req` or
        :func:`forcedimension.HapticDevice.submit` with
        `respect_neutral_stop=False` will re-enable forces.
        """

        self._is_neutral = True
        self.enable_brakes(enabled=False)

    def stop(self):
        """
        Disable force and put the device in BRAKE mode. You may feel a viscous
        force that keeps that device from moving too quickly in this mode if
        the electromagnetic brakes are enabled. A call to
        :func:`forcedimension.HapticDevice.req` or
        :func:`forcedimension.HapticDevice.submit` with
        `respect_neutral_stop=False` will re-enable forces.
        """

        if dhd.stop(self._id):
            raise dhd.errno_to_exception(dhd.errorGetLast())(
                ID=self._id,
                op='forcedimension.dhd.stop()'
            )

    def wait_for_reset(self, timeout: Optional[int] = None):
        """
        Puts the device in RESET mode and waits
        for the user to calibrate the device. To calibrate the device, the user
        must put the device  end-effector at its rest position.

        :param Optional[int] timeout:
            Maximum amount of time to wait for the calibration (in [ms]). If it
            is not specified, will wait indefinetely.

        :raises DHDErrorTimeout:
            If the timeout was reached.

        """
        if dhd.waitForReset(timeout, self._id):
            raise dhd.DHDErrorTimeout(
                op='forcedimension.dhd.waitForReset()',
                ID=self._id
            )

    def check_controller_memory(self) -> bool:
        """
        Evalutes the integrity of the device controller firmware and internal
        configuration on supported device types.

        :returns:
            `True` if the check succeeded, and `False` if the firmware of
            internal heatlh configuration health check failed.
        """
        return not bool(dhd.checkControllerMemory(self._id))

    @property
    def max_force(self) -> Optional[float]:
        """
        Retrieve the current limit (in [N]) to the force magnitude that can be
        applied by the haptic device. The limit is `None` if there is no limit.
        """

        return self._max_force

    def set_max_force(self, limit: Optional[float]):
        """
        Define or disable a limit (in [N]) to the force magnitude that can be
        applied by the haptic device.

        :param Optional[float] limit:
            The desired limit (in [N]) to the force magnitude that can be
            applied. If the limit is None, the force limit is disabled.

        :raises DHDError:
            If an error has occured with the device.
        """

        if limit is not None:
            if limit < 0:
                raise ValueError("limit must be greater than 0 or None.")

        self._max_force = limit

        if limit is None:
            limit = -1.0

        if dhd.setMaxForce(limit, self._id):
            raise dhd.errno_to_exception(dhd.errorGetLast())

        return self

    @property
    def max_torque(self) -> Optional[float]:
        """
        Retrieve the current limit (in [Nm]) to the torque magnitude that can be
        applied by the haptic device. The limit is `None` if there is no limit.
        """

        return self._max_torque


    def set_max_torque(self, limit: Optional[float]):
        """
        Define or disable a limit (in [N]) to the force magnitude that can be
        applied by the haptic device.

        :raises DHDError:
            If an error has occured with the device.

        :param Optional[float] limit:
            The desired limit (in [N]) to the force magnitude that can be
            applied. If the limit is None, the force limit is disabled.
        """

        if limit is not None:
            if limit < 0:
                raise ValueError("limit must be greater than 0 or None.")

        self._max_torque = limit

        if limit is None:
            limit = -1.0

        if dhd.setMaxTorque(limit, self._id):
            raise dhd.errno_to_exception(dhd.errorGetLast())()

        return self


    def get_button(self, button_id: int = 0) -> bool:
        """
        See if the button on the device is being pressed.

        See Also
        --------
        :class:`forcedimension.dhd.constants.NovintButtonID`


        :param int button_id:
            The button to check

        :returns:
            `True` if the button is being pressed, `False` otherwise.
        """

        return bool(self._buttons & _cast(int, 1 << button_id))

    def close(self):
        """
        Shuts down any polling being done on the device and then closes the
        handle to the device.
        """

        self._regulator.stop()
        drd.close(self._id)

    def __enter__(self):
        return self

    def __exit__(self, t, value, traceback):
        self.close()

    _summary_switch = {
        'all': (True, True), 'info': (True, False), 'config': (False, True)
    }

    _config_setters: Dict[str, Callable[..., Any]] = {
        'mass': set_mass,

        'is_force_enabled': enable_force,

        'is_brake_enabled': enable_brakes,

        'is_button_emulation_enabled': enable_button_emulation,

        'base_angles':
            lambda self, value: self.set_base_angles(
                value[0], value[1], value[2]
            ),

        'com_mode': _expert_set_com_mode,

        'linear_velocity_estimator':
            lambda self, value: self.config_linear_velocity(**value),

        'angular_velocity_estimator':
            lambda self, value: self.config_linear_velocity(**value),

        'max_force': set_max_force,

        'max_torque': set_max_torque,

        'standard_gravity': set_standard_gravity,
    }

    _config_initializer = {
        'com_mode': _init_com_mode,
        'is_force_enabled': _init_force_enabled,
        'is_brake_enabled': _init_brake_enabled,
        'linear_velocity_estimator': _init_linear_velocity_estimator,
        'angular_velocity_estimator': _init_angular_velocity_estimator,
        'mass': _init_mass,
        'base_angles': _init_base_angles,
        'max_force': _init_max_force,
        'max_torque': _init_max_torque
    }


class _Poller(Thread):
    def __init__(
        self,
        f: Callable[[], None],
        min_period: Optional[float] = None,
        paused=False
    ):
        self._min_period = min_period
        self.ex = None

        self._stopped = False

        self._paused = False

        self._pause_cond = Condition(Lock())
        self._f = f

        super().__init__(daemon=True)

        if paused:
            self.pause()

    def stop(self):
        if not self._stopped:
            self._stopped = True
            self.resume()

            if self.ident is not None:
                self.join()

    def pause(self):
        if not self._paused:
            self._paused = True
            self._pause_cond.acquire()

    def resume(self):
        if self._paused:
            self._paused = False
            self._pause_cond.notify()
            self._pause_cond.release()

    def run(self):
        try:
            while not self._stopped:
                with self._pause_cond:
                    if self._paused:
                        self._pause_cond.wait()

                while not self._paused and not self._stopped:
                    t = time.monotonic()

                    self._f()

                    if self._min_period is not None:
                        time.sleep(self._min_period * 0.9)
                        while (time.monotonic() - t) < self._min_period:
                            pass

                while self._paused and not self._stopped:
                    if self._min_period is not None:
                        time.sleep(self._min_period)

        except dhd.DHDIOError as ex:
            self.ex = ex
            self._paused = True


class HapticPoller(Thread):
    def __init__(
        self,  h: HapticDevice, interval: float, *args,
        high_prec: bool = False, **kwargs
    ):
        super().__init__(*args, **kwargs)

        self._h = h

        self._interval = int(interval * 1E9)

        self._stop_event = Event()
        self._pause_event = Event()
        self._pause_sync = Event()

        self._target_args = kwargs.get('args', ())
        self._target_kwargs = kwargs.get('kwargs', {})
        self._target = kwargs.get('target', None)

        self._high_prec = high_prec

    def stop(self):
        self._stop_event.set()
        self.join()

    def pause(self):
        self._pause_event.set()
        self._pause_sync.wait()

    def unpause(self):
        self._pause_event.clear()
        self._pause_sync.clear()

    def _execute_target(self):
        if self._target:
            self._target(*self._target_args, **self._target_kwargs)

    def _run_zero_interval(self):
        while not self._stop_event.is_set():
            if not self._pause_event.is_set():
                self._execute_target()
            else:
                self._pause_sync.set()
                time.sleep(0.001)

    def _run_low_prec(self):
        while not self._stop_event.wait(self._interval):

            if not self._pause_event.is_set():
                self._pause_sync.set()
                time.sleep(0.001)
                continue

            self._execute_target()

    def _run_high_prec(self):
        t0 = time.perf_counter()

        wait_period = self._interval
        if self._interval > 0.001:
            sleep_period = self._interval - 0.001
        else:
            sleep_period = 0

        if not self._stop_event.is_set():
            self._execute_target()

        t_sleep = time.perf_counter()
        while not self._stop_event.wait(sleep_period):
            spin(wait_period - (time.perf_counter() - t_sleep))

            t0 = time.perf_counter()
            if not self._pause_event.is_set():
                self._execute_target()
            else:
                self._pause_sync.set()
                sleep_period = 0.001
                wait_period = 0.001
                continue

            wait_period = max(self._interval - (time.perf_counter() - t0), 0)

            if wait_period > 0.001:
                sleep_period = wait_period - 0.001
            else:
                sleep_period = 0

            t_sleep = time.perf_counter()

    def run(self):
        try:
            if self._interval == 0:
                self._run_zero_interval()
                return

            if self._high_prec:
                self._run_high_prec()
            else:
                self._run_low_prec()

        finally:
            del self._target


class HapticDaemon(Thread):
    def __init__(
        self,
        dev: HapticDevice,
        update_list: UpdateOpts = UpdateOpts(),
        forceon=False
    ):
        warnings.warn(
            "HapticDaemon is deprecated as of Force Dimension Bindings "
            "v0.2.0. Use HapticDevice.Regulator.poll() instead.",
            DeprecationWarning, stacklevel=2
        )

        if not isinstance(dev, HapticDevice):
            raise TypeError("Daemon needs an instance of HapticDevice")

        self._paused = False
        self._dev = dev
        self._dev._regulator._haptic_daemon = self
        self._forceon = forceon
        self._pollers = []

        self._set_pollers(update_list)
        self._req_func = self._dev.submit

        super().__init__(daemon=True)

    def _set_pollers(self, update_list):
        if update_list is not None:
            funcs = (
                self._dev.update_delta_enc_and_calculate,
                self._dev.update_velocity,
                self._dev.update_angular_velocity,
                self._dev.update_buttons
            )

            self._pollers.extend(
                _Poller(update, 1/freq)
                for freq, update in zip(update_list[:-2], funcs)
                if freq is not None
            )

            if update_list.ft is not None:
                if (
                    update_list.gripper is not None and
                    self._dev._gripper is not None
                ):
                    self._pollers.append(
                        _Poller(
                            self._dev.update_force_and_torque,
                            1/update_list.ft
                        )
                    )
                else:
                    f = self._dev.update_force_and_torque_and_gripper_force
                    self._pollers.append(
                        _Poller(f, 1/update_list.ft)
                    )

        if update_list.gripper is not None and self._dev._gripper is not None:
            funcs = (
                self._dev._gripper.update_enc_and_calculate,
                self._dev._gripper.update_finger_pos,
                self._dev._gripper.update_thumb_pos,
                self._dev._gripper.update_linear_velocity,
                self._dev._gripper.update_angular_velocity
            )

            self._pollers.extend(
                _Poller(update, 1/freq)
                for freq, update in zip(update_list.gripper, funcs)
                if freq is not None
            )


        self._force_poller = (
            _Poller(lambda: self._dev.submit(True), 1/update_list.req, self._forceon)
        )

        self._paused = False
        self._stopped = False
        self._pause_cond = Condition(Lock())

    def stop(self):
        self._stopped = True

        self.resume()

        if self._pollers is not None:
            for poller in self._pollers:
                poller.stop()

        if self._force_poller is not None:
            self._force_poller.stop()

        if self.ident is not None:
            self.join()

    def pause(self):
        if not self._paused:
            self._paused = True

            for poller in self._pollers:
                poller.pause()

            self._pause_cond.acquire()

    def resume(self):
        if self._paused:
            self._paused = False

            self._pause_cond.notify()
            self._pause_cond.release()

            for poller in self._pollers:
                poller.resume()

    def forcepoll(self, poll=True):
        if poll:
            self._force_poller.resume()
        else:
            self._force_poller.pause()

    def run(self):
        try:
            for poller in self._pollers:
                poller.start()

            self._force_poller.start()

            while not self._stopped:
                with self._pause_cond:
                    if self._paused:
                        self._pause_cond.wait()

                for poller in self._pollers:
                    if poller.ex is not None:
                        raise poller.ex

                if self._force_poller.ex is not None:
                    raise self._force_poller.ex

                time.sleep(0.01)

        except dhd.DHDIOError as ex:
            self._paused = True

            for poller in self._pollers:
                poller.stop()
            self._dev._exception = ex


