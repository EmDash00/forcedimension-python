from __future__ import annotations

__version__ = '0.2.0b1'

import ctypes as ct
import json
import math
import textwrap
import time
from copy import copy
from math import nan
from threading import Condition, Event, Lock, Thread
from typing import Any, Callable, Dict, List, Optional, Tuple, Union
from typing import cast as _cast

import forcedimension_core as fdsdk
import forcedimension_core.constants as constants
import forcedimension_core.dhd as dhd
import forcedimension_core.drd as drd
import typing_extensions
import yaml
from forcedimension_core.constants import (
    ErrorNum, Handedness, VelocityEstimatorMode
)
from forcedimension_core.typing import Array
from typing_extensions import Self

import forcedimension.containers as containers
import forcedimension.util as util
from forcedimension.containers import GripperUpdateOpts, UpdateOpts
from forcedimension.serialization import (
    HapticDeviceConfig, HapticDeviceSpecs, TrajectoryGenParams
)
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

        Certain kinds of HapticDevices opened will have grippers. If that is
        the case, a Gripper object will be instantiated as well containing
        methods to get kinematic information about the Gripper.
        """

        def __init__(
            self,
            parent: HapticDevice,
            config_file_data: Optional[Dict[str, Any]],
            config_data: Optional[Dict[str, Any]],
            restore: bool
        ):
            self._parent = parent
            self._config = self._parent._config.gripper
            self._init_config(config_file_data, config_data, restore)

            self._id = parent.ID
            self._enc = ct.c_int()

            self._state = self._parent.state.gripper
            self._gap = self._parent._state._gripper_pos
            self._v = self._parent._state._gripper_v

            self._angle = ct.c_double()
            self._w = ct.c_double()

            self._fg = ct.c_double()

            self._thumb_pos = containers.Vec3()
            self._finger_pos = containers.Vec3()

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

        def zero(self) -> Self:
            self._fg_req = 0.

            return self

        def req(self, fg: float) -> Self:
            """
            Loads the requested gripper force into the request buffer.
            """

            self._fg_req = fg

            return self

        def config_velocity(
            self,
            window_size: int = constants.DEFAULT_VELOCITY_WINDOW,
            mode: constants.VelocityEstimatorMode = constants.VelocityEstimatorMode.WINDOWING
        ) -> Self:
            """
            Configures the internal shared linear and angular velocity
            estimator used by the Force Dimension SDK for the force gripper.
            Calling this without parameters resets the linear velocity
            estimator to its default settings.

            :param window_size int:
                Time interval to use for computing linear velocity (in [ms]).

            :param VelocityEstimatorMode mode:
                Velocity estimator mode. Currently only
                :data:`forcedimension.VelocityEsimatorMode.WINDOWING` is
            """

            if dhd.configGripperVelocity(window_size, mode, self._id):
                raise fdsdk.util.errno_to_exception(dhd.errorGetLast())(
                    op='forcedimension.dhd.configGripperVelocity()',
                    ID=self._id
                )

            self._config.velocity_estimator.window_size = window_size
            self._config.velocity_estimator.mode = mode

            return self

        def set_max_force(self, limit: Optional[float]) -> Self:
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
                raise fdsdk.util.errno_to_exception(dhd.errorGetLast())()

            self._config.max_force = limit

            return self

        @property
        def thumb_pos(self) -> containers.Vec3:
            """
            A read-only reference to the position of the thumb rest position
            (in [m]) of the gripper about the X, Y, and Z axes at the time of
            the last update.

            :raises DHDError:
                If an error has occured with the device, invalidating the
                device state.
            """

            self.check_exception()
            return _cast(containers.Vec3, self._thumb_pos_view)

        @property
        def finger_pos(self) -> containers.Vec3:
            """
            A read-only reference to the position of the
            forefinger rest position of the gripper (in [m]) about the X, Y,
            and Z axes at the time of the last update.

            :raises DHDError:
                If an error has occured with the device, invalidating the
                device state.
            """

            self.check_exception()
            return _cast(containers.Vec3, self._finger_pos_view)

        @property
        def gap(self) -> float:
            """
            A read-only copy of the gripper opening distance (in [m]) at the
            time of the last update.

            :raises DHDError:
                If an error has occured with the device, invalidating the
                device state.
            """

            self.check_exception()
            return self._gap.value

        @property
        def angle(self) -> float:
            """
            A read-only copy of the gripper opening angle (in [rad]) at the
            time of the last update.

            :raises DHDError:
                If an error has occured with the device, invalidating the
                device state.
            """

            self.check_exception()
            return self._angle.value

        @property
        def v(self) -> float:
            """
            A read-only copy of the linear velocity of the gripper (in [m/s])
            at the time of the last update.

            :raises DHDError:
                If an error has occured with the device, invalidating the
                device state.
            """

            self.check_exception()
            return self._v.value

        @property
        def w(self) -> float:
            """
            A read-only copy of the last-known angular velocity of the
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
            A read-only copy of the last-known force applied by the
            gripper (in [N]).

            :raises DHDError:
                If an error has occured with the device, invalidating the
                device state.
            """

            self.check_exception()
            return self._fg.value

        def calculate_gap(self) -> Self:
            """
            Calculate the value of the gripper opening (in [m]) from the current
            value of the gripper encoder and store it in an internal buffer.
            """

            err = dhd.expert.gripperEncoderToGap(
                self._enc.value, self._gap, self._id
            )

            if err:
                raise fdsdk.util.errno_to_exception(dhd.errorGetLast())()

            return self

        def calculate_angle(self) -> Self:
            """
            Calculate the value of the gripper opening angle (in [rad]) from the
            current value of the gripper encoder and store it in an internal
            buffer.
            """

            dhd.expert.gripperEncoderToAngleRad(
                self._enc.value, self._angle, self._id
            )

            return self

        def update_enc(self) -> Self:
            """
            Performs a blocking read to the HapticDevice, requesting the value
            of the gripper encoder.

            :raises DHDErrorCom:
                If a communication error has occured with the device.
            """

            err = dhd.expert.getGripperEncoder(self._enc, self._id)

            if err == -1:
                raise fdsdk.util.errno_to_exception(dhd.errorGetLast())()

            return self

        def update_enc_and_calculate(self) -> Self:
            """
            Update the value of the gripper encoders and calculate the value of
            the gripper opening (in [m]) and gripper opening angle (in [rad]).

            :raises DHDErrorCom:
                If an communication error has occured with the device.
            """

            self.update_enc()
            self.calculate_angle()
            self.calculate_gap()

            return self

        def update_linear_velocity(self) -> Self:
            """
            Computes the estimated instanteous linear velocity of the gripper
            (in [m/s]).

            :raises DHDErrorCom:
                If an communication error has occured with the device.
            """

            err = dhd.getGripperLinearVelocity(self._v, self._id)

            if err == -1:
                raise fdsdk.util.errno_to_exception(dhd.errorGetLast())()

            return self

        def update_angular_velocity(self) -> Self:
            """
            Computes the estimated instanteous linear velocity of the gripper
            (in [rad/s]).

            :raises DHDErrorCom:
                If an communication error has occured with the device.
            """

            err = dhd.getGripperAngularVelocityRad(self._w, self._id)

            if err == -1:
                raise fdsdk.util.errno_to_exception(dhd.errorGetLast())()

            return self

        def update_angle(self) -> Self:
            """
            Performs a blocking read to the HapticDevice, requesting the value
            of the gripper opening angle (in [rad]).

            :raises DHDErrorCom:
                If an communication error has occured with the device.
            """

            err = dhd.getGripperAngleRad(self._angle, self._id)

            if err == -1:
                raise fdsdk.util.errno_to_exception(dhd.errorGetLast())()

            return self

        def update_gap(self) -> Self:
            """
            Performs a blocking read to the HapticDevice, requesting the value
            of the gripper opening (in [m]).

            :raises DHDErrorCom:
                If an communication error has occured with the device.
            """

            err = dhd.getGripperGap(self._gap, self._id)

            if err == -1:
                raise fdsdk.util.errno_to_exception(dhd.errorGetLast())()

            return self

        def update_thumb_pos(self) -> Self:
            """
            Performs a blocking read to the HapticDevice, requesting the value
            of the gripper thumb rest position (in [m]) about the X, Y, and Z axes.

            :raises DHDErrorCom:
                If an communication error has occured with the device.
            """

            err = dhd.direct.getGripperThumbPos(self._thumb_pos, self._id)

            if err == -1:
                raise fdsdk.util.errno_to_exception(dhd.errorGetLast())()

            return self

        def update_finger_pos(self) -> Self:
            """
            Performs a blocking read to the HapticDevice, requesting the value
            of the gripper forefinger rest position (in [m]) about the X, Y,
            and Z axes.

            :raises DHDErrorCom:
                If an communication error has occured with the device.
            """

            err = dhd.direct.getGripperFingerPos(self._finger_pos, self._id)

            if err == -1:
                raise fdsdk.util.errno_to_exception(dhd.errorGetLast())()

            return self

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
            parent: HapticDevice,
            config_file_data: Optional[Dict[str, Any]],
            config_data: Optional[Dict[str, Any]],
            restore: bool
        ):
            self._parent = parent
            self._haptic_daemon: Optional[HapticDaemon] = None
            self._stop_event = Event()

            self._specs = self._parent._specs
            self._config = self._parent._config.regulator

            self._control_freq = nan
            self._is_drd_running = False
            self._specs.is_drd_supported = drd.isSupported(self._parent._id)
            self._is_initialized = drd.isInitialized(self._parent._id)
            self._end_event = Event()

            self._default_poller = containers.PollGroup(
                [self.update_position_and_orientation, self.update_velocity],
                self.wait_for_tick
            )

            self._polled_functions: Dict[Callable[[], Any], HapticPoller] = {}
            self._polled_targets: Dict[Callable[[], Any], HapticPoller] = {}
            self._poll_groups_dct: Dict[str, HapticPoller] = {}

            self._polled_funcs_view = ImmutableWrapper(self._polled_functions)
            self._polled_targets_view = ImmutableWrapper(self._polled_targets)
            self._poll_groups_dct_view = ImmutableWrapper(
                self._poll_groups_dct
            )

            self._updaters = set([
                self.update,
                self.update_position_and_orientation,
                self.update_velocity
            ])

            self._default_regulator_poll_groups = (
                containers.PollGroup(
                    targets=[
                        self.update_position_and_orientation,
                        self.update_velocity,
                        self._parent.submit
                    ],
                    wait_for=self.wait_for_tick,
                    high_precision=True,
                    name="DEFAULT"
                ),
                containers.PollGroup(
                    targets=[self._parent.update_buttons],
                    wait_for=1e-2,
                    high_precision=False,
                    name="DEFAULT BUTTON"
                )
            )

            self._default_poll_groups = (
                containers.PollGroup(
                    targets=[
                        self._parent.update_position_and_orientation,
                        self._parent.update_velocity,
                        self._parent.update_buttons,
                        self._parent.submit
                    ],
                    wait_for=1e-3,
                    name="DEFAULT"
                ),
            )

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

            vmax, amax, jerk, err = drd.getEncMoveParam()

            if err:
                raise fdsdk.util.errno_to_exception(dhd.errorGetLast())(
                    op='forcedimension.drd.getEncMoveParam()',
                    ID=self._parent._id
                )

            self._config.enc_move_param.vmax = vmax
            self._config.enc_move_param.amax = amax
            self._config.enc_move_param.jerk = jerk

        def _init_enc_track_param(self, restore: bool):
            if restore:
                self.set_enc_track_param(
                    drd.DEFAULT_ENC_TRACK_PARAMS.vmax,
                    drd.DEFAULT_ENC_TRACK_PARAMS.amax,
                    drd.DEFAULT_ENC_TRACK_PARAMS.jerk,
                )
                return

            vmax, amax, jerk, err = drd.getEncTrackParam()

            if err:
                raise fdsdk.util.errno_to_exception(dhd.errorGetLast())(
                    op='forcedimension.drd.getEncTrackParam()',
                    ID=self._parent._id
                )

            self._config.enc_track_param.vmax = vmax
            self._config.enc_track_param.amax = amax
            self._config.enc_track_param.jerk = jerk

        def _init_pos_move_param(self, restore: bool):
            if restore:
                self.set_pos_move_param(
                    drd.DEFAULT_POS_MOVE_PARAMS.vmax,
                    drd.DEFAULT_POS_MOVE_PARAMS.amax,
                    drd.DEFAULT_POS_MOVE_PARAMS.jerk,
                )
                return

            vmax, amax, jerk, err = drd.getPosMoveParam()

            if err:
                raise fdsdk.util.errno_to_exception(dhd.errorGetLast())(
                    op='forcedimension.drd.getPosMoveParam()',
                    ID=self._parent._id
                )

            self._config.pos_move_param.vmax = vmax
            self._config.pos_move_param.amax = amax
            self._config.pos_move_param.jerk = jerk

        def _init_pos_track_param(self, restore: bool):
            if restore:
                self.set_pos_track_param(
                    drd.DEFAULT_POS_TRACK_PARAMS.vmax,
                    drd.DEFAULT_POS_TRACK_PARAMS.amax,
                    drd.DEFAULT_POS_TRACK_PARAMS.jerk,
                )
                return

            vmax, amax, jerk, err = drd.getPosTrackParam()

            if err:
                raise fdsdk.util.errno_to_exception(dhd.errorGetLast())(
                    op='forcedimension.drd.getPosTrackParam()',
                    ID=self._parent._id
                )

            self._config.pos_track_param.vmax = vmax
            self._config.pos_track_param.amax = amax
            self._config.pos_track_param.jerk = jerk

        def _init_rot_move_param(self, restore: bool):
            if restore:
                self.set_rot_move_param(
                    drd.DEFAULT_ROT_MOVE_PARAMS.vmax,
                    drd.DEFAULT_ROT_MOVE_PARAMS.amax,
                    drd.DEFAULT_ROT_MOVE_PARAMS.jerk,
                )
                return

            vmax, amax, jerk, err = drd.getRotMoveParam()

            if err:
                raise fdsdk.util.errno_to_exception(dhd.errorGetLast())(
                    op='forcedimension.drd.getRotMoveParam()',
                    ID=self._parent._id
                )

            self._config.rot_move_param.vmax = vmax
            self._config.rot_move_param.amax = amax
            self._config.rot_move_param.jerk = jerk

        def _init_rot_track_param(self, restore: bool):
            if restore:
                self.set_rot_track_param(
                    drd.DEFAULT_ROT_TRACK_PARAMS.vmax,
                    drd.DEFAULT_ROT_TRACK_PARAMS.amax,
                    drd.DEFAULT_ROT_TRACK_PARAMS.jerk,
                )
                return

            vmax, amax, jerk, err = drd.getRotTrackParam()

            if err:
                raise fdsdk.util.errno_to_exception(dhd.errorGetLast())(
                    op='forcedimension.drd.getRotTrackParam()',
                    ID=self._parent._id
                )

            self._config.rot_track_param.vmax = vmax
            self._config.rot_track_param.amax = amax
            self._config.rot_track_param.jerk = jerk

        def _init_grip_move_param(self, restore: bool):
            if restore:
                self.set_grip_move_param(
                    drd.DEFAULT_GRIP_MOVE_PARAMS.vmax,
                    drd.DEFAULT_GRIP_MOVE_PARAMS.amax,
                    drd.DEFAULT_GRIP_MOVE_PARAMS.jerk,
                )
                return

            vmax, amax, jerk, err = drd.getGripMoveParam()

            if err:
                raise fdsdk.util.errno_to_exception(dhd.errorGetLast())(
                    op='forcedimension.drd.getGripMoveParam()',
                    ID=self._parent._id
                )

            self._config.grip_move_param.vmax = vmax
            self._config.grip_move_param.amax = amax
            self._config.grip_move_param.jerk = jerk

        def _init_grip_track_param(self, restore: bool):
            if restore:
                self.set_grip_track_param(
                    drd.DEFAULT_GRIP_TRACK_PARAMS.vmax,
                    drd.DEFAULT_GRIP_TRACK_PARAMS.amax,
                    drd.DEFAULT_GRIP_TRACK_PARAMS.jerk,
                )
                return

            vmax, amax, jerk, err = drd.getGripTrackParam()

            if err:
                raise fdsdk.util.errno_to_exception(dhd.errorGetLast())(
                    op='forcedimension.drd.getGripTrackParam()',
                    ID=self._parent._id
                )

            self._config.grip_track_param.vmax = vmax
            self._config.grip_track_param.amax = amax
            self._config.grip_track_param.jerk = jerk

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
        def DEFAULT_POLL_GROUPS(self) -> Tuple[containers.PollGroup]:
            """
            A tuple of default poll groups (without the use of DRD).
            """

            return self._default_poll_groups

        @property
        def DEFAULT_REGULATOR_POLL_GROUPS(self) -> Tuple[
            containers.PollGroup, containers.PollGroup
        ]:
            """
            A tuple of default poll groups (with the use of DRD).
            """

            return self._default_regulator_poll_groups

        @property
        def is_drd_supported(self) -> bool:
            return self._specs.is_drd_supported

        @property
        def is_initialized(self) -> bool:
            return self._is_initialized

        @property
        def is_drd_running(self) -> bool:
            return drd.isRunning(self._parent.ID)

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

        @property
        def is_filtering(self) -> bool:
            return drd.isFiltering(self._parent._id)

        def wait_end(self) -> Self:
            """
            Puts the current thread to sleep until the end Event has been
            signalled either gracefully or because exception has occured in a
            polling thread.
            """
            self._end_event.wait()

            if self._parent._exception is not None:
                raise self._parent._exception

            return self

        def notify_end(self) -> Self:
            self._end_event.set()

            return self

        def update_control_freq(self) -> Self:
            """
            Updates an internal buffer witht he average refresh rate of the
            control loop (in [kHz]) since this function was last called.
            """

            if (control_freq := drd.getCtrlFreq(self._parent._id)) < 0:
                fdsdk.util.errno_to_exception(dhd.errorGetLast())(
                    op='forcedimension.drd.getCtrlFreq()',
                    ID=self._parent._id
                )

            self._control_freq = control_freq

            return self

        def _manual_init(
            self, manual_init_wait: float = 5.
        ) -> Self:
            if drd.isInitialized(self._parent._id):
                return self

            print("Please manually initialize the device by moving the "
                  "end-effector back and forth.")

            while not drd.isInitialized(self._parent._id):
                time.sleep(0.001)

            print(
                "Manual initialization complete. Starting in "
                f"{manual_init_wait:.2f} seconds."
            )

            if manual_init_wait > 0:
                time.sleep(manual_init_wait)

            return self

        def initialize(
            self, redo: bool = False, manual_init_wait: float = 5.
        ) -> Self:
            """
            Initializes the haptic device. If the device supports
            automatic initialization, it robotically moves to a
            known position and resets encoder counters to their correct
            values. It subsequently ensures that the initialization was
            correctly performed by robotically sweeping the by robotically
            sweeping all endstops and comparing their joint space position to
            expected values (stored in each device internal memory).

            If the device does not support automatic initialization, you will
            be prompted to perform manual initializaiton.

            :param bool redo:
                ``True`` to reinitialize. If ``False`` and the device is
                already initialized, this function will do nothing. Devices
                that do not support automatic initialization cannot be
                reinitialized.

            :param float manual_init_wait:
                The amount of time to wait after manually initializing.

            :raises DHDErrorNotAvailable:
                If automatic initialization is not available on this device.
            """

            if manual_init_wait < 0:
                raise ValueError(
                    "'manual_init_wait' must be a greater than 0."
                )

            if redo:
                self._is_initialized = False

                if drd.autoInit(self._parent._id):
                    if dhd.errorGetLast() != ErrorNum.NOT_AVAILABLE:
                        raise fdsdk.util.errno_to_exception(dhd.errorGetLast())(
                            op='forcedimension.drd.autoInit()',
                            ID=self._parent._id
                        )

                    self._manual_init(manual_init_wait)

            if self._is_initialized:
                return self

            if drd.checkInit(self._parent._id):
                if dhd.errorGetLast() != ErrorNum.NOT_AVAILABLE:
                    raise fdsdk.util.errno_to_exception(dhd.errorGetLast())(
                        op='forcedimension.drd.checkInit()',
                        ID=self._parent._id
                    )

                self._manual_init(manual_init_wait)

            self._initialized = True

            return self

        def precision_initialize(self) -> Self:
            """
            Performs automatic initialization for supported devices by
            robotically moving each axis to a knokwn position and resetting its
            encoder counter to the correct values. The initialization is
            carreid out for each device axis in turn, and validates the
            initialization by asserting the position of a validation reference
            for each axis. This is preferred to
            :func:`HapticDevice.initialize()` for devices that support it.

            :raises DHDErrorNotAvailable:
                If the device does not support precision initialization.

            Info
            ----
            In order to make regulation as stable as possible, this function
            will automaticallyincrease the priority level of the regulation
            thread to a higher value than normal system process priority.

            See Also
            --------
            :func:`HapticDevice.Regulator.initialize()`
            :func:`HapticDevice.Regulator.check_initialization()`
            """

            if drd.precisionInit(self._parent._id):
                raise fdsdk.util.errno_to_exception(dhd.errorGetLast())(
                    op='forcedimension.drd.precisionInit()',
                    ID=self._parent._id
                )

            self._initialized = True

            return self

        def wait_for_tick(self):
            """
            Put the current thread to sleep until the next iteration of the
            robotic control loop begins.
            """
            if not self.is_drd_running:
                raise RuntimeError("DRD is not running.")

            drd.waitForTick(self._parent._id)

        def regulate(self, enabled: bool = True) -> Self:
            """
            Enable or disable regulation for position, rotation, and the force
            gripper. If regulation is disabled, the base
            can move freely and will display any force set using
            :func:`HapticDevice.req()` and :func:`HapticDevice.submit()`. If it
            is enabled, delta position, wrist rotation, and gripper opening is
            locked and can be controlled by calling all robotic functions
            (e.g. :func:`HapticDevice.Regulator.move_to()`). By default,
            regulation is enabled.

            :param bool enabled:
                ``True`` to enable ``False`` to disable.
            """

            self.regulate_pos(enabled)
            self.regulate_rot(enabled)
            self.regulate_grip(enabled)

            return self

        def regulate_pos(self, enabled: bool = True) -> Self:
            """
            This function toggles robotic regulation of the device delta base,
            which provides translations. If regulation is disabled, the base
            can move freely and will display any force set using
            :func:`HapticDevice.req()` and :func:`HapticDevice.submit()`. If it
            is enabled, base  position is locked and can be controlled by
            calling all robotic functions
            (e.g. :func:`HapticDevice.Regulator.move_to_pos()`). By default,
            delta base regulation is enabled.

            :param bool enabled:
                ``True`` to enable ``False`` to disable.
            """

            self._is_pos_regulated = enabled

            if drd.regulatePos(enabled, self._parent._id):
                raise fdsdk.util.errno_to_exception(dhd.errorGetLast())(
                    op='forcedimension.drd.regulatePos()',
                    ID=self._parent._id
                )

            return self

        def regulate_rot(self, enabled: bool = True) -> Self:
            """
            This function toggles robotic regulation of the device wrist. If
            regulation is disabled, the wrist can move freely and will display
            any torque set using :func:`HapticDevice.req()` and
            :func:`HapticDevice.submit()`. If it is enabled, wrist orientation
            is locked and can be controlled by calling all robotic functions
            (e.g. :func:`HapticDevice.Regulator.move_to_rot()). By default,
            wrist regulation is enabled.

            :param bool enabled:
                ``True`` to enable ``False`` to disable.
            """
            self._is_rot_regulated = enabled

            if drd.regulateRot(enabled, self._parent._id):
                raise fdsdk.util.errno_to_exception(dhd.errorGetLast())(
                    op='forcedimension.drd.regulateRot()',
                    ID=self._parent._id
                )

            return self

        def regulate_grip(self, enabled: bool = True) -> Self:
            """
            This function toggles robotic regulation of the device gripper.
            If regulation is disabled, the gripper can move freely and will
            display any force set using :func:`HapticDevice.req()` and
            :func:`HapticDevice.submit()`. If it is enabled, gripper
            orientation is locked and can be controlled by calling all robotic
            functions (e.g. :func:`HapticDevice.Regulator.move_to_grip()`). By
            default, gripper regulation is enabled.

            :param bool enabled:
                ``True`` to enable ``False`` to disable.
            """

            self._is_grip_regulated = enabled

            if drd.regulateGrip(enabled, self._parent._id):
                raise fdsdk.util.errno_to_exception(dhd.errorGetLast())(
                    op='forcedimension.drd.regulateGrip()',
                    ID=self._parent._id
                )

            return self

        def enable_filter(self, enabled: bool = True) -> Self:
            """
            This function controls the motion filtering for subsequent calls to
            :func:`HapticDevice.Regulator.track()`,
            :func:`HapticDevice.Regulator.track_all_enc()`,
            :func:`HapticDevice.Regulator.track_enc()`,
            :func:`HapticDevice.Regulator.track_pos()`
            :func:`HapticDevice.Regulator.track_rot()`
            :func:`HapticDevice.Regulator.track_grip()`. This guauntees
            acceleration contunity.
            """

            self._is_filter_enabled = enabled

            if drd.enableFilter(enabled, self._parent._id):
                raise fdsdk.util.errno_to_exception(dhd.errorGetLast())(
                    op='forcedimension.drd.enableFilter()',
                    ID=self._parent._id
                )

            return self

        def start_drd(self) -> Self:
            """
            This function starts the robotic control loop for the given device.
            The device must be initialized either manually (by moving around
            the device end-effector) with
            :func:`HapticDevice.Regulator.initialize()` or
            :func:`HapticDevice.Regulator.precision_initialize()`
            before this function can be called successfully.
            """

            if not self._is_drd_running:
                if drd.start(self._parent._id):
                    raise fdsdk.util.errno_to_exception(dhd.errorGetLast())(
                        op='forcedimension.drd.start()',
                        ID=self._parent._id
                    )

                self._is_drd_running = True

            return self

        def stop_drd(self) -> Self:
            """
            This function stops the robotic control loop for the given device.
            """

            if self._is_drd_running:
                if drd.stop(True, self._parent._id):
                    raise fdsdk.util.errno_to_exception(dhd.errorGetLast())(
                        op='forcedimension.drd.start()',
                        ID=self._parent._id
                    )

                self._is_drd_running = False

            return self

        def _register_polled_funcs(
            self, *targets: Callable[[], Any], poller: HapticPoller
        ):
            for target in targets:
                if target in self._updaters and not self.is_drd_running:
                    raise ValueError(
                        "Regulator updaters need DRD to be started."
                    )

                if target in self._parent._updater_dct:
                    for func in self._parent._updater_dct[target]:
                        if func in self._polled_functions:
                            raise ValueError(
                                f"{target.__name__} is already being polled"
                                f" by {self._polled_functions[target].name}"
                            )

                        self._polled_functions[func] = poller

                    self._polled_targets[target] = poller

                    continue

                if target in self._polled_functions:
                    raise ValueError(
                        f"{target.__name__} is already being polled"
                        f" by {self._polled_functions[target].name}"
                    )

                self._polled_functions[target] = poller
                self._polled_targets[target] = poller

        def _poll_impl(self, *pollgrps: containers.PollGroup):
            for pollgrp in pollgrps:

                poller = HapticPoller(
                    target=util.function_chain(*pollgrp.targets),
                    wait_for=pollgrp.wait_for,
                    high_precision=pollgrp.high_precision
                )

                if pollgrp.name is not None:
                    poller.name = pollgrp.name

                self._poll_groups_dct[poller.name] = poller
                self._register_polled_funcs(*pollgrp.targets, poller=poller)

                poller.start(self)

        def poll(self, *pollgrps: containers.PollGroup) -> Self:

            if self._haptic_daemon is not None:
                raise RuntimeError(
                    "Mixed use of HapticDaemon and "
                    "HapticDevice.Regulator.poll() is not supported."
                )

            # Default Behavior:
            if len(pollgrps) == 0:
                if self._is_drd_running:
                    self._poll_impl(*self.DEFAULT_REGULATOR_POLL_GROUPS)
                else:
                    self._poll_impl(*self.DEFAULT_POLL_GROUPS)

            self._poll_impl(*pollgrps)

            return self

        @property
        def poll_group_names(self):
            return self._poll_groups_dct.keys()

        @property
        def poll_groups_by_name(self) -> Dict[str, HapticPoller]:
            return _cast(Dict[str, HapticPoller], self._poll_groups_dct_view)

        @property
        def poll_groups_by_target(self) -> Dict[
            Callable[[], Any], HapticPoller
        ]:
            return _cast(
                Dict[Callable[[], Any], HapticPoller],
                self._polled_targets_view
            )

        def stop_poll(self):
            """
            Stop all polling.
            """

            if self._haptic_daemon is not None:
                self._haptic_daemon.stop()
                return self

            self._stop_event.set()

            for poller in self.poll_groups_by_name.values():
                poller.stop()

            return self

        def update(self) -> Self:
            self.update_position_and_orientation()
            self.update_velocity()

            return self

        def update_position_and_orientation(self) -> Self:
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
                raise fdsdk.util.errno_to_exception(dhd.errorGetLast())(
                    op='forcedimension.drd.getPositionAndOrientation()',
                    ID=self._parent._id
                )

            return self

        def update_velocity(self) -> Self:
            if not self._is_drd_running:
                raise RuntimeError("DRD is not running.")

            err = drd.direct.getVelocity(
                self._parent._v,
                self._parent._w,
                self._parent._mock_gripper._v,
                self._parent._id
            )

            if err == -1:
                raise fdsdk.util.errno_to_exception(dhd.errorGetLast())(
                    op='forcedimension.drd.getVelocity()',
                    ID=self._parent._id
                )

            return self

        def set_mot_ratio_max(self, scale: float) -> Self:
            """
            Sets the maximum joint torque applied to all regulated joints
            expressed as a fraction of the maximum torque available for each
            joint.

            In practice, this limits the maximum regulation torque (in joint
            space), making it potentially safer to operate in environments
            where humans or delicate obstacles are present.
            """

            if drd.setMotRatioMax(scale, self._parent._id):
                raise fdsdk.util.errno_to_exception(dhd.errorGetLast())(
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
        ) -> Self:
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
                raise fdsdk.util.errno_to_exception(dhd.errorGetLast())(
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
        ) -> Self:
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
                raise fdsdk.util.errno_to_exception(dhd.errorGetLast())(
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
        ) -> Self:
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
                raise fdsdk.util.errno_to_exception(dhd.errorGetLast())(
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
        ) -> Self:
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
                raise fdsdk.util.errno_to_exception(dhd.errorGetLast())(
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
        ) -> Self:
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
                raise fdsdk.util.errno_to_exception(dhd.errorGetLast())(
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
        ) -> Self:
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
                raise fdsdk.util.errno_to_exception(dhd.errorGetLast())(
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
        ) -> Self:
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
                raise fdsdk.util.errno_to_exception(dhd.errorGetLast())(
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
        ) -> Self:
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
                raise fdsdk.util.errno_to_exception(dhd.errorGetLast())(
                    op='forcedimension.drd.setGripTrackParam()',
                    ID=self._parent._id
                )

            self._config.grip_track_param.vmax = vmax
            self._config.grip_track_param.amax = amax
            self._config.grip_track_param.jerk = jerk

            return self

        def zero(self, block: bool = True) -> Self:
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

        def zero_pos(self, block: bool = True) -> Self:
            """
            Sends the device end-effector position to the origin.
            The motion uses smooth acceleration/deceleration and follows a
            straight line in space. The acceleration and velocity profiles can
            be controlled by adjusting the trajectory generation parameters.
            """

            return self.move_to_pos((0., 0., 0.), block=block)

        def zero_rot(self, block: bool = True) -> Self:
            """
            Sends the device end-effector to zero rotation about the first,
            second, and third joint angles The motion uses smooth
            acceleration/deceleration and follows a straight curve. The
            acceleration and velocity profiles can be controlled by adjusting
            the trajectory generation parameters.
            """
            return self.move_to_rot((0., 0., 0.), block=block)

        def move_to_enc(
            self, *cmds: Array[int, int], block: bool = True
        ) -> Self:
            """
            Sequentially sends the device end-effector to a sequence desired
            delta encoder configurations. The motion uses smooth
            acceleration/deceleration. The acceleration and velocity profiles
            can be controlled by adjusting the trajectory generation
            parameters.

            :param Array[int, int] *cmds:
                A sequence of target delta encoder configurations.

            :param bool block:
                If ``True`` the call will block until the command is completed.
                If ``False`` the call will return immediately. ``False`` values
                are only supported for calls with a single target
                (non-iterable).

            :raises ValueError:
                If more than one command (or an iterable of commands) was
                requested when `block` is ``False``.

            Warning
            -------
            Paths are NOT guarunteed to be continuous if a new command is
            given before the previous command has ended. This can happen if
            `block` is ``False`` and multiple commands are provided without
            proper synchronization. Setting `block` to ``False`` should be
            reserved for use cases for iteratables which continously feed
            inputs spaced over time (i.e. reading from some stream which
            periodically sends commands).


            See Also
            --------
            :func:`forcedimension.Regulator.track_enc()`
            """

            if len(cmds) > 1 and not block:
                raise ValueError(
                    "block=False is only supported for calls with a single "
                    "target."
                )

            for cmd in cmds:
                if drd.moveToEnc(cmd, block, self._parent._id):
                    fdsdk.util.errno_to_exception(dhd.errorGetLast())(
                        op='forcedimension.drd.moveToEnc()',
                        ID=self._parent._id
                    )

            return self

        def track_enc(self, *cmds: Array[int, int]) -> Self:
            """
            Sequentially sends the device end-effector to a sequence of
            delta encoder configurations. This motion is guaunteed to be
            continuous. If motion filters are enabled, the
            motion follows a smooth acceleration/deceleration constraint on
            each encoder axis. The acceleration and velocity profiles can be
            controlled by adjusting the trajectory generation parameters.

            :param Iterable[Array[int, int]] *cmds:
                A sequence of target delta encoder values.
            """

            for cmd in cmds:
                if drd.trackEnc(cmd, self._parent._id):
                    fdsdk.util.errno_to_exception(dhd.errorGetLast())(
                        op='forcedimension.drd.trackEnc()',
                        ID=self._parent._id
                    )

            return self

        def move_to(self, *cmds: Array[int, float], block: bool = True) -> Self:
            """
            Sends the device end-effector to a set of desired
            7-DOF configurations.  The motion uses smooth
            acceleration/deceleration and will follow straight lines in
            position space as well as straight curves in orientation space. The
            acceleration and velocity profiles can be controlled by adjusting
            the trajectory generation parameters.

            :param Array[int, float] *cmds:
                An sequence of target 7 DOF configurat. DOFs 1-3 represent the
                position about the X, Y, and Z axes, respectively (in [m]).
                DOFs 4-6 represent the orientation about the first, second, and
                third wrist joints, respectively (in [rad]). DOF 7
                represents the gripper opening gap (in [m]).

            :param bool block:
                If ``True`` the call will block until the command is completed.
                If ``False`` the call will return immediately. ``False`` values
                are only supported for calls with a single target
                (non-iterable).

            :raises ValueError:
                If more than one command (or an iterable of commands) was
                requested when `block` is ``False``.

            Warning
            -------
            Paths are NOT guarunteed to be continuous if a new command is
            given before the previous command has ended. This can happen if
            `block` is ``False`` and multiple commands are provided without
            proper synchronization. Setting `block` to ``False`` should be
            reserved for use cases for iteratables which continously feed
            inputs spaced over time (i.e. reading from some stream which
            periodically sends commands).


            See Also
            --------
            :func:`forcedimension.Regulator.track_pos()`
            """

            if len(cmds) > 1 and not block:
                raise ValueError(
                    "block=False only supported for calls with a single "
                    "target."
                )

            for cmd in cmds:
                if drd.moveTo(cmd, block, self._parent._id):
                    fdsdk.util.errno_to_exception(dhd.errorGetLast())(
                        op='forcedimension.drd.moveTo()',
                        ID=self._parent._id
                    )

            return self

        def track(self, *cmds: Array[int, float]) -> Self:
            """
            This function sends the device end-effector to a desired Cartesian
            7-DOF configuration. If motion filters are enabled, the motion
            follows a smooth acceleration/deceleration constraint on each
            Cartesian axis. The acceleration and velocity profiles can be
            controlled by adjusting the trajectory generation parameters.

            :param Array[int, float] *cmds:
                An sequence of target 7 DOF configurat. DOFs 1-3 represent the
                position about the X, Y, and Z axes, respectively (in [m]).
                DOFs 4-6 represent the orientation about the first, second, and
                third wrist joints, respectively (in [rad]). DOF 7
                represents the gripper opening gap (in [m]).
            """

            for cmd in cmds:
                if drd.track(cmd, self._parent._id):
                    fdsdk.util.errno_to_exception(dhd.errorGetLast())(
                        op='forcedimension.drd.track()', ID=self._parent._id
                    )

            return self

        def move_to_all_enc(
            self, *cmds: Array[int, int], block: bool = True
        ) -> Self:
            """
            Sends the device end-effector to a set of desired encoder
            positions in each degree-of-freedom. The motion follows a straight
            line in the encoder space, with smooth acceleration/deceleration.
            The acceleration and velocity profiles can be controlled by
            adjusting the trajectory generation parameters.

            :param Array[int, int] *cmds:
                A sequence of target encoder positions for each
                degree-of-freedom.

            :param bool block:
                If ``True`` the call will block until the command is completed.
                If ``False`` the call will return immediately. ``False`` values
                are only supported for calls with a single target
                (non-iterable).

            :raises ValueError:
                If more than one command (or an iterable of commands) was
                requested when `block` is ``False``.

            Warning
            -------
            Paths are NOT guarunteed to be continuous if a new command is
            given before the previous command has ended. This can happen if
            `block` is ``False`` and multiple commands are provided without
            proper synchronization. Setting `block` to ``False`` should be
            reserved for use cases for iteratables which continously feed
            inputs spaced over time (i.e. reading from some stream which
            periodically sends commands).


            See Also
            --------
            :func:`forcedimension.Regulator.track_all_enc()`
            """

            if len(cmds) > 1 and not block:
                raise ValueError(
                    "block=False is only supported for calls with a single "
                    "target."
                )

            for cmd in cmds:
                if drd.moveToAllEnc(cmd, block, self._parent._id):
                    fdsdk.util.errno_to_exception(dhd.errorGetLast())(
                        op='forcedimension.drd.moveToAllEnc()',
                        ID=self._parent._id
                    )

            return self

        def track_all_enc(self, *cmds: Array[int, int]) -> Self:
            """
            Sequentially sends the device end-effector to a set
            of desired encoder positions in each degree of freedom. If motion
            filters are enabled, the motion follows a smooth
            acceleration/deceleration constraint on each encoder axis. The
            acceleration and velocity profiles can be controlled by adjusting
            the trajectory generation parameters.

            :param Array[int, int] *cmds:
                A sequence of target encoder positions for each
                degree-of-freedom.
            """

            for cmd in cmds:
                if drd.trackAllEnc(cmd, self._parent._id):
                    fdsdk.util.errno_to_exception(dhd.errorGetLast())(
                        op='forcedimension.drd.trackAllEnc()',
                        ID=self._parent._id
                    )

            return self

        def move_to_pos(
            self, *cmds: Array[int, float], block: bool = True
        ) -> Self:
            """
            Sends the device end-effector to a set of desired
            positions. The motion follows a straight line in position space,
            with smooth acceleration/deceleration. The acceleration and
            velocity profiles can be controlled by adjusting the trajectory
            generation parameters.

            :param Array[int, float] *cmds:
                An is a sequence target positions of target positions
                follow. Positions are about the X, Y, and Z axes (in [m]).

            :param bool block:
                If ``True`` the call will block until the command is completed.
                If ``False`` the call will return immediately. ``False`` values
                are only supported for calls with a single target
                (non-iterable).

            :raises ValueError:
                If more than one command (or an iterable of commands) was
                requested when `block` is ``False``.

            Warning
            -------
            Paths are NOT guarunteed to be continuous if a new command is
            given before the previous command has ended. This can happen if
            `block` is ``False`` and multiple commands are provided without
            proper synchronization. Setting `block` to ``False`` should be
            reserved for use cases for iteratables which continously feed
            inputs spaced over time (i.e. reading from some stream which
            periodically sends commands).


            See Also
            --------
            :func:`forcedimension.Regulator.track_pos()`
            """

            if len(cmds) > 1 and not block:
                raise ValueError(
                    "block=False is only supported for calls with a single "
                    "target."
                )

            for cmd in cmds:
                if drd.moveToPos(cmd, block, self._parent._id):
                    fdsdk.util.errno_to_exception(dhd.errorGetLast())(
                        op='forcedimension.drd.moveToPos()',
                        ID=self._parent._id
                    )

            return self

        def track_pos(self, *cmds: Array[int, float]) -> Self:
            """
            Sends the device end-effector to a set of desired
            positions. The motion follows a straight line in position space,
            with smooth acceleration/deceleration. The acceleration and
            velocity profiles can be controlled by adjusting the trajectory
            generation parameters.

            :param Array[int, float] *cmds:
                An is a sequence target positions of target positions
                follow. Positions are about the X, Y, and Z axes (in [m]).

            :raises ValueError:
                If more than one command (or an iterable of commands) was
                requested when `block` is ``False``.
            """

            for cmd in cmds:
                if drd.trackPos(cmd, self._parent._id):
                    fdsdk.util.errno_to_exception(dhd.errorGetLast())(
                        op='forcedimension.drd.trackPos()',
                        ID=self._parent._id
                    )

            return self

        def move_to_rot(
            self, *cmds: Array[int, float], block: bool = True
        ) -> Self:
            """
            Sequentially sends the device end-effector to a set of desired
            Cartesian rotations. The motion follows a straight curve between
            each command, with smooth acceleration/deceleration. The
            acceleration and velocity profiles can be controlled by adjusting
            the trajectory generation parameters.

            :param Array[int, float] *cmds:
                An sequence of target orientations. Orientations are
                about the first, second, and third joint angles (in [rad]).

            :param bool block:
                If ``True`` the call will block until the command is completed.
                If ``False`` the call will return immediately. ``False`` values
                are only supported for calls with a single target
                (non-iterable).

            :param bool block:
                If ``True`` the call will block until the command is completed.
                If ``False`` the call will return immediately.

            Info
            ----
            Paths are NOT guarunteed to be continuous a sequence of calls where
            `block` is ``False``. Use
            :func:`forcedimension.HapticDevice.Regulator.track_rot()` for
            that instead.
            """

            if len(cmds) > 1 and not block:
                raise ValueError(
                    "block=False is only supported for calls with a single "
                    "target."
                )

            for cmd in cmds:
                if drd.moveToRot(cmd, block, self._parent._id):
                    fdsdk.util.errno_to_exception(dhd.errorGetLast())(
                        op='forcedimension.drd.moveToRot()',
                        ID=self._parent._id
                    )

            return self

        def track_rot(self, *cmds: Array[int, float]) -> Self:
            """
            Sequentially sends the device end-effector to a set of
            desired Cartesian orientation. If motion filters are enabled, the
            motion follows a smooth acceleration/deceleration curve along each
            Cartesian axis. The acceleration and velocity profiles can be
            controlled by adjusting the trajectory generation parameters.

            :param Array[int, float] *cmds:
                An sequence of target orientations. Orientations are
                about the first, second, and third joint angles (in [rad]).

            :param bool block:
                If ``True`` the call will block until the command is completed.
                If ``False`` the call will return immediately.
            """

            for cmd in cmds:
                if drd.trackRot(cmd, self._parent._id):
                    fdsdk.util.errno_to_exception(dhd.errorGetLast())(
                        op='forcedimension.drd.trackRot()',
                        ID=self._parent._id
                    )

            return self

        def move_to_grip(self, *cmds: float, block: bool = True) -> Self:
            """
            Sequentially sends the device gripper to a set of desired opening
            distances. The motion is executed with smooth
            acceleration/deceleration. The acceleration and velocity profiles
            can be controlled by adjusting the trajectory generation
            parameters.

            :param float *cmds:
                A sequence of target gripper opening distances (in [m]).

            :param bool block:
                If ``True`` the call will block until the command is completed.
                If ``False`` the call will return immediately. ``False`` values
                are only supported for calls with a single target
                (non-iterable).

            :param bool block:
                If ``True`` the call will block until the command is completed.
                If ``False`` the call will return immediately.

            Info
            ----
            Paths are NOT guarunteed to be continuous a sequence of calls where
            `block` is ``False``. Use
            :func:`forcedimension.Regulator.HapticDevice.track_grip()` for that
            instead.
            """

            if len(cmds) > 1 and not block:
                raise ValueError(
                    "block=False is only supported for calls with a single "
                    "target."
                )

            for cmd in cmds:
                if drd.moveToGrip(cmd, block, self._parent._id):
                    fdsdk.util.errno_to_exception(dhd.errorGetLast())(
                        op='forcedimension.drd.moveToGrip()',
                        ID=self._parent._id
                    )

            return self

        def track_grip(self, *cmds: float) -> Self:
            """
            This function sequentially sends the device gripper to a set of
            desired opening distances (in [m]). If motion filters are enabled,
            the motion follows a smooth acceleration/deceleration. The
            acceleration and velocity profiles can be controlled by adjusting
            the trajectory generation parameters.

            :param float *cmds:
                An sequence of target gripper opening distances (in [m]).
            """

            for cmd in cmds:
                if drd.trackGrip(cmd, self._parent._id):
                    fdsdk.util.errno_to_exception(dhd.errorGetLast())(
                        op='forcedimension.drd.trackGrip()',
                        ID=self._parent._id
                    )

            return self

        def hold(self) -> Self:
            """
            This function immediately makes the device hold its current
            position. All motion commands are abandoned.
            """

            if drd.hold(self._parent._id):
                raise fdsdk.util.errno_to_exception(dhd.errorGetLast())(
                    op='forcedimension.drd.hold()', ID=self._parent._id
                )

            return self

        def lock(self, enabled: bool = True) -> Self:
            """
                If ``enabled`` is ``True``, the device moves to its park
                position engages the mechanical locks. If `enabled` is
                ``False``, the mechanical locks are disengaged. It is
                recommended to follow engaging locks with
                `:func:forcedimension.HapticDevice.Regulator.stop()`.

                :param bool enabled:
                    ``True`` to park and engage mechanical locks, ``False`` to
                    disengage locks.

                :raises ArgumentError:
                    If enabled is not implicitly converitble to C uchar.

                :raises DHDErrorNotAvailable:
                    If the device is not equipped with mechanical locks.
            """

            if drd.lock(enabled, False, self._parent._id):
                raise fdsdk.util.errno_to_exception(dhd.errorGetLast())(
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
        def __init__(self, parent: HapticDevice):
            self._parent = parent

        def set_timeguard(
            self, interval: int = constants.DEFAULT_TIMEGUARD_US
        ) -> "HapticDevice":
            """
            Sets the arbitrary minimum period for the TimeGuard feature in the
            Force Dimension SDK. A value of
            :data:`forcedimension.dhd.DEFAULT_TIMEGUARD_US` will reset the
            period to its default value (recommended).

            :raises ArgumentError:
                If interval is not implicitly ocnvertible to C char.

            :raises forcedimension.dhd.adaptors.DHDErrorArgument:
                If interval was an invalid value.
            """

            if dhd.expert.setTimeGuard(interval, self._parent._id):
                raise fdsdk.util.errno_to_exception(dhd.errorGetLast())(
                    op='forcedimension.dhd.expert.setTimeGuard()',
                    ID=self._parent._id
                )

            return self._parent

        def set_com_mode(
            self, mode: dhd.ComMode = dhd.ComMode.ASYNC
        ) -> "HapticDevice":
            """
            Sets the COM operation mode on compatible devices.

            :raises forcedimension.dhd.adaptors.DHDErrorArgument:
                If interval was an invalid value.
            """

            if dhd.expert.setComMode(mode, self._parent._id):
                raise fdsdk.util.errno_to_exception(dhd.errorGetLast())(
                    op='forcedimension.dhd.expert.setComMode()',
                    ID=self._parent._id
                )

            self._parent._config.com_mode = fdsdk.util.com_mode_str(mode)

            return self._parent

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
            If ``True``, the device will fail to instantiate if a firmware or
            internal configuration health check fails.

        :param HapticDeviceConfig config:
            Initial config settings for the Haptic Device. Overrides settings
            set by `config_file`. Settings unset by both `config` and
            `config_file` are unchanged or, if
            `restore_defaults` is ``True``, reset to their defaults.

        :param HapticDeviceConfig config_file:
            Initial config settings for the Haptic Device in a `.json` or
            `.yml` file. Is overridden by settings set in `config`.
            Settings unset by both `config` and
            `config_file` are unchanged or, if
            `restore_defaults` is ``True``, reset to their defaults.

        :param bool restore_defaults:
            If ``True`` will restore all config settings not set by either
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

        self._exception: Optional[Exception] = None

        self._expert = HapticDevice._Expert(self)

        if (dhd.getDeviceCount() <= 0):
            raise dhd.DHDErrorNoDeviceFound()

        if (ID is None) and (devtype is None) and (serial_number is None):
            # Open the first device we find.

            if (id := drd.open()) == -1:
                raise fdsdk.util.errno_to_exception(dhd.errorGetLast())()

            self._id = id

            if (devtype_opened := dhd.getSystemType()) == -1:
                raise fdsdk.util.errno_to_exception(dhd.errorGetLast())(
                    ID=self._id,
                    op='forcedimension.dhd.getSystemType()'
                )

            self._devtype = devtype_opened
        elif ID is not None:
            # Open device of given ID.

            if (id := drd.openID(ID)) == -1:
                raise fdsdk.util.errno_to_exception(dhd.errorGetLast())()

            self._id = id

            if (devtype_opened := dhd.getSystemType()) == -1:
                raise fdsdk.util.errno_to_exception(dhd.errorGetLast())(
                    ID=self._id,
                    op='forcedimension.dhd.getSystemType()'
                )

            self._devtype = devtype_opened
        elif devtype is not None:
            # Open first device of given devtype.

            if (id := dhd.openType(devtype)) == -1:
                raise fdsdk.util.errno_to_exception(dhd.errorGetLast())()

            self._id = id

            if dhd.close(self._id) == -1:
                raise fdsdk.util.errno_to_exception(dhd.errorGetLast())(
                    ID=self._id,
                    op='forcedimension.dhd.close()'
                )

            if drd.openID(self._id) == -1:
                raise fdsdk.util.errno_to_exception(dhd.errorGetLast())()

            self._devtype = devtype
        elif serial_number is not None:
            if (id := dhd.openSerial(serial_number)) == -1:
                raise fdsdk.util.errno_to_exception(dhd.errorGetLast())()

            if dhd.close(self._id) == -1:
                raise fdsdk.util.errno_to_exception(dhd.errorGetLast())(
                    ID=self._id,
                    op='forcedimension.dhd.close()'
                )

            if drd.openID(self._id) == -1:
                raise fdsdk.util.errno_to_exception(dhd.errorGetLast())()

            if (devtype_opened := dhd.getSystemType()) == -1:
                raise fdsdk.util.errno_to_exception(dhd.errorGetLast())(
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
            config_data = config.model_dump(exclude_defaults=True)

        self._init_config(config_file_data, config_data, restore_defaults)

        self._mass = nan
        self._gripper = None

        self._encs = containers.DOFInt()
        self._encs_v = containers.DOFFloat()

        self._joint_state = containers.DOFFloatState()
        self._joint_angles = containers.DOFFloat(
            self._joint_state[0])
        self._joint_v = containers.DOFFloat(self._joint_state[1])

        self._state = containers.DOFFloatState()

        self._pos = containers.Vec3(self._state[0, :3])
        self._v = containers.Vec3(self._state[1, :3])

        self._orientation_angles = containers.Vec3(self._state[0, 3:6])
        self._w = containers.Vec3(self._state[1, 3:6])

        self._f = containers.Vec3()
        self._t = containers.Vec3()

        self._delta_jacobian = containers.Mat3x3()
        self._wrist_jacobian = containers.Mat3x3()
        self._frame = containers.Mat3x3()

        self._inertia_matrix = containers.Mat6x6()

        self._status = containers.Status()

        self._f_req = containers.Vec3()
        self._t_req = containers.Vec3()
        self._vibration_req: List[float] = [0.] * 2
        self._buttons = 0

        self._delta_joint_angles_view = ImmutableWrapper(
            self._joint_angles.delta
        )

        self._wrist_joint_angles_view = ImmutableWrapper(
            self._joint_angles.wrist
        )

        self._base_angles_view = ImmutableWrapper(self._config.base_angles)

        self._state_view = ImmutableWrapper(self._state)
        self._pos_view = ImmutableWrapper(self._pos)
        self._v_view = ImmutableWrapper(self._v)

        self._w_view = ImmutableWrapper(self._w)
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

        handedness = fdsdk.util.handedness(self._devtype)
        has_base = dhd.hasBase(self._id)
        has_wrist = dhd.hasWrist(self._id)
        has_active_wrist = dhd.hasActiveWrist(self._id)
        has_gripper = True if self._gripper is not None else False
        has_active_gripper = dhd.hasActiveGripper(self._id)
        num_dof = fdsdk.util.num_dof(self._devtype)

        if (serial_number := dhd.getSerialNumber(self._id)) < 0:
            if (errno := dhd.errorGetLast()) != ErrorNum.NOT_AVAILABLE:
                raise fdsdk.util.errno_to_exception(errno)(
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
            Name: "{self._specs.name if self._specs.name is not None else ""}"
            Device Model: {
                fdsdk.util.devtype_str(self._specs.devtype, pretty=True)
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
            Handedness: {
                fdsdk.util.handedness_str(self._specs.handedness, pretty= True)
            }
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
            self.update_joint_state,
            self.update_force_and_torque_and_gripper_force,
            self.update_buttons,
            self.update_status
        ]

        self._updater_dct = {
            self.update_state: set([
                self.update_position, self.update_velocity
            ]),

            self.update_encs: set([
                self.update_delta_encs,
                self.update_wrist_encs,
                self._mock_gripper.update_enc
            ]),

            self.update_encs_and_calculate: set([
                self.update_delta_encs,
                self.update_wrist_encs,
                self._mock_gripper.update_enc
            ]),

            self.update_delta_encs_and_calculate: set([
                self.update_delta_encs,
            ]),

            self.update_wrist_encs_and_calculate: set([
                self.update_wrist_encs,
            ]),

            self.update_position_and_orientation: set([
                self.update_position,
                self.update_position_and_orientation,
            ]),

            self.update_wrist_state: set([
                self.update_orientation_angles,
                self.update_angular_velocity
            ]),

            self.regulator.update: set([
                self.regulator.update_position_and_orientation,
                self.regulator.update_velocity
            ])
        }

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
            raise fdsdk.util.errno_to_exception(dhd.errorGetLast())(
                op='forcedimension.dhd.getBaseAngleXRad()',
                ID=self._id
            )

        if dhd.getDeviceAngleRad(
            self._config.base_angles.ptrs[1].contents, self._id
        ):
            raise fdsdk.util.errno_to_exception(dhd.errorGetLast())(
                op='forcedimension.dhd.getDeviceAngleRad()',
                ID=self._id
            )

        if dhd.getBaseAngleZRad(
            self._config.base_angles.ptrs[2].contents, self._id
        ):
            raise fdsdk.util.errno_to_exception(dhd.errorGetLast())(
                op='forcedimension.dhd.getBaseAngleZRad()',
                ID=self._id
            )

    def _init_com_mode(self, restore: bool = False):
        if restore:
            self._expert_set_com_mode()
            return

        if (com_mode := dhd.getComMode(self._id)) == -1:
            raise fdsdk.util.errno_to_exception(dhd.errorGetLast())(
                ID=self._id, op='forcedimension.dhd.getComMode()'
            )

        self._config.com_mode = fdsdk.util.com_mode_str(com_mode)

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
        if (mass := dhd.getEffectorMass(self._id)) < 0:
            raise fdsdk.util.errno_to_exception(dhd.errorGetLast())()

        self._config.mass = mass

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

    def set_config(self, config_data: Dict[str, Any]) -> Self:
        for key, value in config_data:
            HapticDevice._config_setters[key](self, value)

        return self

    def check_exception(self):
        """
        Checks if an exception has occured in an update function and raises it.
        Most property functions implicitly check this.
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
        Constructs a human-readable string that summarizes the current
        configuration of the HapticDevice.
        """

        base_angles = copy(self._config.base_angles)
        base_angles[0] /= 2 * math.pi
        base_angles[1] /= 2 * math.pi
        base_angles[2] /= 2 * math.pi

        return (textwrap.dedent(f"""\
            Force Enabled: {
                "Yes (default)" if self._config.is_force_enabled
                else "No"
            }
            Brakes Enabled: {
                "Yes (default)" if self._config.is_brake_enabled
                else "No"
            }
            Gravity Compensation Enabled: {
                "Yes"
                if self._config.is_gravity_compensation_enabled
                else "No (default)"
            }
            Button Emulation Enabled: {
                "Yes" if self._config.is_button_emulation_enabled
                else "No (default)"
            }
            Base Angles (rev): {base_angles}
            Communication Mode: {
                "async (default)" if self._config.com_mode == 'async'
                else self._config.com_mode
            }
            Timeguard: {
                'default' if self._config.timeguard == constants.DEFAULT_TIMEGUARD_US
                else (
                    'None' if self._config.timeguard == 0
                    else self._config.timeguard
                )
            }
            Linear Velocity Estimator:
                mode: {
                    fdsdk.util.velocity_estimator_mode_str(
                        self._config.linear_velocity_estimator.mode
                    )
                }{
                    " (default)"
                    if self._config.linear_velocity_estimator.mode ==
                    VelocityEstimatorMode.WINDOWING
                    else ""
                }
                window size: {
                    self._config.linear_velocity_estimator.window_size
                } us{
                    " (default)"
                    if self._config.linear_velocity_estimator.window_size ==
                    constants.DEFAULT_VELOCITY_WINDOW else ""
                }
            Angular Velocity Estimator:
                mode: {
                    fdsdk.util.velocity_estimator_mode_str(
                        self._config.angular_velocity_estimator.mode
                    )
                }{
                    " (default)"
                    if self._config.angular_velocity_estimator.mode ==
                    VelocityEstimatorMode.WINDOWING
                    else ""
                }
                window size: {
                    self._config.angular_velocity_estimator.window_size
                } us{
                    " (default)"
                    if self._config.angular_velocity_estimator.window_size ==
                    constants.DEFAULT_VELOCITY_WINDOW else ""
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
                        fdsdk.util.velocity_estimator_mode_str(
                            self._config.gripper.velocity_estimator.mode
                        )
                    }{
                        " (default)" if
                        self._config.gripper.velocity_estimator.mode ==
                        VelocityEstimatorMode.WINDOWING else ""
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
        """
        The current value of the TimeGuard period (in [us]).
        """
        return self._config.timeguard

    @property
    def mass(self) -> float:
        """
        The mass of the end-effector used for gravity compensation
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

        :raises ArgumentError:
            If `m` is not implicitly converitble to a C double.

        :raises DHDErrorArgument:
            If `m` was not a valid value.
        """
        if dhd.setEffectorMass(m, ID=self._id):
            raise fdsdk.util.errno_to_exception(dhd.errorGetLast())()

    @property
    def standard_gravity(self) -> float:
        """
        The acceleration due to gravity (in m/s^2) used for gravity
        compensation.
        """
        return self._standard_gravity

    @property
    def left_handed(self) -> bool:
        return self._specs.handedness == Handedness.LEFT

    @property
    def handedness(self) -> Handedness:
        """
        The handedness of the device. Some devices are not handed and the value
        of this property for those devices is
        :data:`forcedimension.dhd.adaptors.Handedness.NONE`
        """
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
        """
        Gets the COM operation mode of the device.
        """

        return fdsdk.util.com_mode_from_str(self._config.com_mode)

    @property
    def is_neutral(self) -> bool:
        """
        Gets the neutral status of the device.

        See Also
        --------
        :func:`forcedimension.HapticDevice.neutral()`
        """

        return self._is_neutral

    @property
    def is_stopped(self) -> bool:
        """
        Gets the stop status of the device.

        See Also
        --------
        :func:`forcedimension.HapticDevice.stop()`
        """

        return self._is_stopped

    @property
    def is_button_emulation_enabled(self) -> bool:
        """
        Gets the button emulation status.

        See Also
        --------
        :func:`forcedimension.HapticDevice.get_button()`
        """

        return self._button_emulation_enabled

    @property
    def status(self) -> containers.Status:
        """
        Provides a read-only reference to the last-known status of the device.
        Thread-safe.

        :returns:
            A :class:`forcedimension.dhd.adaptors.Status` object representing
            the last-known status of the device.
        """

        return _cast(containers.Status, self._status_view)

    @property
    def base_angles(self) -> containers.Vec3:
        """
        Provides a read-only reference of the device base plate angle
        (in [rad]) about the X, Y, and Z axes.
        """

        return _cast(containers.Vec3, self._base_angles_view)

    def set_base_angles(
        self,
        x: Optional[float] = None,
        y: Optional[float] = None,
        z: Optional[float] = None
    ):
        """
        Sets the device base plate angles (in [rad]) about the X, Y, and Z
        axes. If an angle is not specified, it is not set.
        I.E. set_base_angles(x=1) will set the base angle about the X axis to 1
        radian while keeping the Y and Z base angles unchanged.

        :param Optional[float] x:
            Angle (in [rad]) to set the device plate angle about the X axis to.

        :param Optional[float] y:
            Angle (in [rad]) to set the device plate angle about the Y axis to.

        :param Optional[float] z:
            Angle (in [rad]) to set the device plate angle about the Z axis to.

        :raises ArgumentError:
            If any of `x`, `y`, and `z` are not implicitly convertible to C
            double.

        :raises DHDErrorArgument:
            If any of `x`, `y`, and `z` is an invalid value.
        """

        if x is not None:
            self._config.base_angles[0] = x

            if dhd.setBaseAngleXRad(x):
                raise fdsdk.util.errno_to_exception(dhd.errorGetLast())(
                    op='forcedimension.dhd.setBaseAngleXRad()',
                    ID=self._id
                )

        if y is not None:
            self._config.base_angles[1] = y

            if dhd.setDeviceAngleRad(y):
                raise fdsdk.util.errno_to_exception(dhd.errorGetLast())(
                    op='forcedimension.dhd.setDeviceAngleRad()',
                    ID=self._id
                )

        if z is not None:
            self._config.base_angles[2] = z

            if dhd.setBaseAngleZRad(z):
                raise fdsdk.util.errno_to_exception(dhd.errorGetLast())(
                    op='forcedimension.dhd.setBaseAngleZRad()',
                    ID=self._id
                )

        return self

    def set_output_bits(self, mask: int):
        """
        Sets the user programmable output bits on devices that support it.

        :param int mask:
            Bitwise mask that toggles the progammable output bits.

        :raises ArgumentError:
            If `mask` is not implicitly convertible to a C uint.

        :raises DHDErrorNotAvailable:
            If programmable output bits are not supported on the device.
        """

        if dhd.setOutput(mask, self._id):
            raise fdsdk.util.errno_to_exception(dhd.errorGetLast())

        return self

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
            raise fdsdk.util.errno_to_exception(dhd.errorGetLast())

        self._standard_gravity = g

        return self

    def enable_button_emulation(self, enabled: bool = True):
        """
        Enables the button behavior emulation in devices that feature a
        gripper.

        :param bool enabled:
            ``True`` to enable button emulation, ``False`` to disable.

        :raises ArgumentError:
            If `enabled` is not implicitly convertible to C bool.

        :raises DHDErrorNotAvailable:
            If button emulation is not supported on the device.

        Info
        ----
        Omega.7 devices with firmware version 2.x need to be enabled for the
        button emulation to report the emulated button status.
        """

        self._button_emulation_enabled = enabled

        if dhd.emulateButton(enabled, self._id):
            raise fdsdk.util.errno_to_exception(dhd.errorGetLast())

        return self

    def enable_force(self, enabled: bool = True):
        """
        Enable/disable force on the end-effector.

        :param bool enabled:
            ``True`` to enable, ``False`` to disable.
        """

        if dhd.enableForce(enabled, ID=self._id):
            raise fdsdk.util.errno_to_exception(dhd.errorGetLast())(
                ID=self._id,
                op='forcedimension.dhd.enableForce()'
            )

        return self

    def enable_brakes(self, enabled: bool = True):
        """
        Enable electromagnetic braking on the device. If the brakes are
        disabled, the device is forced into IDLE mode. No forces will be
        applied in that mode.

        :param enabled bool:
            ``True`` to enable electromagnetic braking, ``False`` to disable.
        """

        if dhd.setBrakes(enabled, self._id):
            raise fdsdk.util.errno_to_exception(dhd.errorGetLast())(
                ID=self._id,
                op='forcedimension.dhd.setBrakes()'
            )

        return self

    def enable_gravity_compensation(self, enabled: bool = True):
        """
        Enable built-in gravity compensation for the end-effector that will be
        added on top of the force request.

        :param bool enabled:
            ``True`` to enable, ``False`` to disable.

        See Also
        --------
        :func:`forcedimension.HapticDevice.set_mass()`
        :func:`forcedimension.HapticDevice.set_standard_gravity()`
        """

        dhd.setGravityCompensation(enabled, ID=self._id)

        self._f_req[0] = 0.
        self._f_req[1] = 0.
        self._f_req[2] = 0.

        self._t_req[0] = 0.
        self._t_req[1] = 0.
        self._t_req[2] = 0.

        dhd.stop(self._id)

        return self

    def config_linear_velocity(
        self,
        window_size: int = dhd.DEFAULT_VELOCITY_WINDOW,
        mode: VelocityEstimatorMode = VelocityEstimatorMode.WINDOWING
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
            :data:`forcedimension.VelocityEsimatorMode.WINDOWING` is
            supported by the Force Dimension SDK.

        :raises ArgumentError:
            If either ``window_size`` or ``mode`` are not implicitly
            convertible to C int.

        :raises DHDErrorArgument:
            If either ``window_size`` or ``mode`` was set to an invalid value.
        """

        if dhd.configLinearVelocity(window_size, mode, self._id):
            raise fdsdk.util.errno_to_exception(dhd.errorGetLast())(
                op='forcedimension.dhd.configLinearVelocity()', ID=self._id
            )

        self._config.linear_velocity_estimator.window_size = window_size
        self._config.linear_velocity_estimator.mode = mode

        return self

    def config_angular_velocity(
        self,
        window_size: int = dhd.DEFAULT_VELOCITY_WINDOW,
        mode: VelocityEstimatorMode = VelocityEstimatorMode.WINDOWING
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
            :data:`forcedimension.VelocityEsimatorMode.WINDOWING` is
            supported by the Force Dimension SDK.

        :raises ArgumentError:
            If either ``window_size`` or ``mode`` are not implicitly
            convertible to C int.

        :raises DHDErrorArgument:
            If either ``window_size`` or ``mode`` was set to an invalid value.
        """

        if dhd.configAngularVelocity(window_size, mode, self._id):
            raise fdsdk.util.errno_to_exception(dhd.errorGetLast())(
                op='forcedimension.dhd.configAngularVelocity()', ID=self._id
            )

        self._config.angular_velocity_estimator.window_size = window_size
        self._config.angular_velocity_estimator.mode = mode

        return self

    @property
    def joint_state(self) -> containers.DOFFloatState:
        """
        A read-only reference to the state of the HapticDevice, which is a
        ndarray of [[x, y, z], [vx, vy, vz]]. The position and linear
        velocity are about the X, Y, and Z axes (in [m] and [m/s],
        respectively).

        :raises DHDError:
            If an error has occured with the device, invalidating the
            device state.

        """

        return _cast(containers.DOFFloatState, self._joint_state)

    @property
    def delta_joint_angles(self) -> containers.Vec3:
        """
        A read-only reference to the DELTA joint angles about the first,
        second, and third DELTA joint angles, respectively (in [rad]) at
        the time of the last update.

        :raises DHDError:
            If an error has occured with the device, invalidating the
            device state.
        """

        self.check_exception()
        return _cast(containers.Vec3, self._delta_joint_angles_view)

    @property
    def wrist_joint_angles(self) -> containers.Vec3:
        """
        A read-only reference to the DELTA joint angles about the first,
        second, and third WRIST joint angles, respectively (in [rad]) at
        the time of the last update.

        :raises DHDError:
            If an error has occured with the device, invalidating the
            device state.
        """

        self.check_exception()
        return _cast(containers.Vec3, self._wrist_joint_angles_view)

    @property
    def state(self) -> containers.DOFFloatState:
        """
        A read-only reference to the state of the HapticDevice, which is a
        ndarray of [[x, y, z], [vx, vy, vz]]. The position and linear
        velocity are about the X, Y, and Z axes (in [m] and [m/s],
        respectively).

        :raises DHDError:
            If an error has occured with the device, invalidating the
            device state.

        """

        return _cast(containers.DOFFloatState, self._state_view)

    @property
    def pos(self) -> containers.Vec3:
        """
        A read-only reference to the position of the
        HapticDevice's end-effector about the X, Y, and Z axes (in [m]) at
        the time of the last update. Thread-safe.

        :raises DHDError:
            If an error has occured with the device, invalidating the
            device state.
        """

        self.check_exception()
        return _cast(containers.Vec3, self._pos_view)

    @property
    def v(self) -> containers.Vec3:
        """
        A read-only reference to the linear velocity of the
        HapticDevice's end-effector (in [m/s]) at the time of the last update.
        Thread-safe.

        :raises DHDError:
            If an error has occured with the device, invalidating the
            device state.
        """

        self.check_exception()
        return _cast(containers.Vec3, self._v_view)

    @property
    def w(self) -> containers.Vec3:
        """
        A read-only reference angular velocity of
        the HapticDevice's end-effector (in [rad/s]) at the time of the last
        update. Thread-safe.

        :raises DHDError:
            If an error has occured with the device, invalidating the
            device state.
        """

        self.check_exception()
        return _cast(containers.Vec3, self._w_view)

    @property
    def t(self) -> containers.Vec3:
        """
        A read-only reference to the HapticDevice's end-effector
        (in [Nm]) at the time of the last update. Thread-safe.

        :raises DHDError:
            If an error has occured with the device, invalidating the
            device state.
        """

        self.check_exception()
        return _cast(containers.Vec3, self._t_view)

    @property
    def f(self) -> containers.Vec3:
        """
        A read-only reference to the applied force of the HapticDevice's
        end-effector (in [N]) at the time of the last update. Thread-safe.

        :raises DHDError:
            If an error has occured with the device, invalidating the
            device state.
        """

        self.check_exception()
        return _cast(containers.Vec3, self._f_view)

    @property
    def delta_jacobian(self) -> containers.Mat3x3:
        """
        A read-only reference to the last-known DELTA jacobian based on the
        of the HapticDevice's end-effector configuration at the time of the
        last update. Thread-safe.

        :raises DHDError:
            If an error has occured with the device, invalidating the
            device state.
        """

        self.check_exception()
        return _cast(containers.Mat3x3, self._delta_jacobian_view)

    @property
    def wrist_jacobian(self) -> containers.Mat3x3:
        """
        A read-only reference to the last-known WRIST jacobian based on the
        of the HapticDevice's end-effector configuration at the time of the
        last update. Thread-safe.

        :raises DHDError:
            If an error has occured with the device, invalidating the
            device state.
        """

        self.check_exception()
        return _cast(containers.Mat3x3, self._wrist_jacobian_view)

    @property
    def frame(self) -> containers.Mat3x3:
        """
        A read-only reference to the orientation frame of the of the
        HapticDevice's end-effector at the time of the last update. This
        function only applies to haptic devices with a wrist.
        Thread-safe.

        :raises DHDError:
            If an error has occured with the device, invalidating the
            device state.
        """

        self.check_exception()
        return _cast(containers.Mat3x3, self._frame_view)

    @property
    def inertia_matrix(self) -> containers.Mat6x6:
        """
        A read-only reference to the inertia matrix about the X, Y, and Z axes
        based on the joint configuration at the last update. Thread-safe.
        """

        self.check_exception()
        return _cast(containers.Mat6x6, self._inertia_matrix_view)

    def set_update_list(self, lst: List[Callable[..., Any]]) -> Self:
        """
        Sets the functions to be sequentially called in the default update
        function.

        See Also
        --------
        :func:`forcedimension.HapticDevice.update()`
        """

        if not isinstance(lst, list):
            raise ValueError("Update list must be a list.")

        for func in lst:
            if not isinstance(func, Callable):
                raise ValueError("Each element must be callable.")

        self._update_list = lst

        return self

    def update(self) -> Self:
        """
        Sequentially calls the functions in the internal default update list.

        See Also
        --------
        :func:`forcedimension.HapticDevice.set_update_list()`
        """

        for updater in self._update_list:
            updater()

        return self

    def calculate_pos(self) -> Self:
        """
        Calculates and stores the position of the device given the current
        value of the delta encoders in the internal buffer.
        """

        dhd.expert.direct.deltaEncoderToPosition(
            self._encs.delta, self._pos, self._id
        )

        return self

    def calculate_delta_joint_angles(self) -> Self:
        """
        Calculates and stores the joint angles of the DELTA structure given
        the current end-effector encoder readings in the internal buffer.
        """

        dhd.expert.direct.deltaEncodersToJointAngles(
            self._encs.wrist, self._joint_angles.delta, self._id
        )

        return self

    def calculate_delta_jacobian(self) -> Self:
        """
        Calculates and stores the Jacobian matrix of the DELTA structure given
        current joint angle configuration in the internal buffer.
        """

        self.calculate_delta_joint_angles()

        dhd.expert.direct.deltaJointAnglesToJacobian(
            self._joint_angles.delta, self._delta_jacobian, self._id
        )

        return self

    def calculate_wrist_joint_angles(self) -> Self:
        """
        Calculates and stores the joint angles of the WRIST structure given
        the current end-effector encoder values in the internal buffer.
        """

        dhd.expert.direct.wristEncodersToJointAngles(
            self._encs.wrist, self._joint_angles.wrist, self._id
        )

        return self

    def calculate_wrist_jacobian(self) -> Self:
        """
        Calculates and stores the Jacobian matrix of the WRIST structure given
        current joint angle configuration in the internal buffer.
        """

        dhd.expert.direct.wristJointAnglesToJacobian(
            self._joint_angles.wrist, self._wrist_jacobian, self._id
        )

        return self

    def calculate_inertia_matrix(self) -> Self:
        """
        Calculates the 6x6 inertia matrix (with respect to the X, Y, and Z
        axes) given the current joint angles configuration in the internal
        buffer.
        """

        dhd.expert.direct.jointAnglesToIntertiaMatrix(
            self._joint_angles, self._inertia_matrix, self._id
        )

        return self

    def update_encs_and_calculate(self) -> Self:
        """
        Updates the encoders for each degree-of-freedom and given those values,
        calculates the position of the end-effector, DELTA joint angles, the
        DELTA jacobia, the WRIST joint angles, and the WRIST jacobian.

        :raises DHDErrorCom:
            If a communication error has occured with the device.
        """
        self.update_encs()
        self.calculate_pos()
        self.calculate_wrist_joint_angles()
        self.calculate_delta_jacobian()
        self.calculate_wrist_jacobian()

        return self

    def update_delta_encs_and_calculate(self) -> Self:
        """
        Updates the DELTA encoders and given those values, calculates the
        position of the end-effector, DELTA joint angles, and the DELTA
        jacobian.

        :raises DHDErrorCom:
            If a communication error has occured with the device.
        """

        self.update_delta_encs()
        self.calculate_pos()
        self.calculate_delta_joint_angles()
        self.calculate_delta_jacobian()

        return self

    def update_wrist_encs_and_calculate(self) -> Self:
        """
        Updates the WRIST encoders and given those values, calculates the wrist
        joint angles and the WRIST jacobian.

        :raises DHDErrorCom:
            If a communication error has occured with the device.
        """

        self.update_wrist_encs()
        self.calculate_wrist_joint_angles()
        self.calculate_wrist_jacobian()

        return self

    def update_encs(self) -> Self:
        """
        Performs a blocking read to the HapticDevice, requesting the current
        encoder readers of the DELTA structure that controls the end-effector
        and updates the last-known position with the response. The requested
        values are then loaded into an internal buffer.

        :raises DHDErrorCom:
            If a communication error has occured with the device.
        """

        err = dhd.expert.direct.getEnc(self._encs, 0xff, self._id)

        if err == -1:
            raise fdsdk.util.errno_to_exception(
                dhd.errorGetLast())(
                    ID=self._id,
                    op='forcedimension.dhd.expert.getEnc()'
            )

        return self

    def update_delta_encs(self) -> Self:
        """
        Performs a blocking read to the HapticDevice, requesting the current
        encoder readers of the DELTA structure that controls the end-effector
        and updates the last-known position with the response. The requested
        values are then loaded into an internal buffer.

        :raises DHDError:
            If a communication error has occured with the device.
        """

        err = dhd.expert.direct.getDeltaEncoders(self._encs.delta, self._id)

        if err == -1:
            raise fdsdk.util.errno_to_exception(
                dhd.errorGetLast())(
                    ID=self._id,
                    op='forcedimension.dhd.expert.getDeltaEncoders()'
            )

        return self

    def update_wrist_encs(self) -> Self:
        """
        Performs a blocking read to the HapticDevice, requesting the current
        encoder readers of the DELTA structure that controls the end-effector
        and updates the last-known position with the response. The requested
        values are then loaded into an internal buffer.

        :raises DHDErrorCom:
            If a communication error has occured with the device.
        """

        err = dhd.expert.direct.getWristEncoders(self._encs.wrist, self._id)

        if err == -1:
            raise fdsdk.util.errno_to_exception(
                dhd.errorGetLast())(
                    ID=self._id,
                    op='forcedimension.dhd.expert.getWristEncoders()'
            )

        return self

    def update_enc_velocities(self) -> Self:
        """
        Performs a blocking read to the HapticDevice, requesting the current
        encoder readers of the DELTA structure that controls the end-effector
        and updates the last-known position with the response. The requested
        values are then loaded into an internal buffer.

        :raises DHDErrorCom:
            If a communication error has occured with the device.
        """

        err = dhd.expert.direct.getEncVelocities(self._encs_v, self._id)

        if err == -1:
            raise fdsdk.util.errno_to_exception(
                dhd.errorGetLast())(
                    ID=self._id,
                    op='forcedimension.dhd.expert.getEncVelocities()'
            )

        return self

    def update_joint_state(self) -> Self:
        self.update_joint_angles()
        self.update_joint_angle_velocities()

        return self

    def update_joint_angles(self) -> Self:
        """
        Performs a blocking read to the HapticDevice, requesting the current
        joint angles for each degree-of-freedom (in [rad]). The requested
        values  are then loaded into an internal buffer.

        :raises DHDErrorCom:
            If a communication error has occured with the device.
        """

        err = dhd.expert.direct.getJointAngles(self._joint_angles, self._id)

        if err == -1:
            raise fdsdk.util.errno_to_exception(
                dhd.errorGetLast())(
                    ID=self._id,
                    op='forcedimension.dhd.expert.getJointAngles()'
            )

        return self

    def update_joint_angle_velocities(self) -> Self:
        """
        Performs a blocking read to the HapticDevice, requesting the current
        joint velocities for each degree-of-freedom (in [rad/s]). The requested
        values are then loaded into an internal buffer.

        :raises DHDErrorCom:
            If a communication error has occured with the device.
        """

        err = dhd.expert.direct.getJointVelocities(
            self._joint_v, self._id
        )

        if err == -1:
            raise fdsdk.util.errno_to_exception(
                dhd.errorGetLast())(
                    ID=self._id,
                    op='forcedimension.dhd.expert.getJointVelocities()'
            )

        return self

    def update_state(self) -> Self:
        self.update_delta_state()
        self.update_delta_state()

        return self

    def update_delta_state(self) -> Self:
        """
        Sequentially updates the position and the velocity of end-effectors
        (states affected by the DLETA structure).
        """
        self.update_position()
        self.update_velocity()

        return self

    def update_position(self) -> Self:
        """
        Performs a blocking read to the HapticDevice, requesting the current
        position of the end-effector. The requested values are then loaded into
        an internal buffer.

        :raises DHDErrorCom:
            If a communication error has occured with the device.
        """

        err = dhd.direct.getPosition(self._pos, self._id)

        if err == -1:
            raise fdsdk.util.errno_to_exception(dhd.errorGetLast())(
                ID=self._id,
                op='forcedimension.dhd.getPosition()'
            )

        return self

    def update_velocity(self) -> Self:
        """
        Performs a blocking read to the HapticDevice, requesting the current
        linear velocity of the end-effector. The requested values are then
        loaded into an internal buffer.

        :raises DHDErrorCom:
            If a communication error occurred with the device.
        """

        err = dhd.direct.getLinearVelocity(self._v, self._id)

        if err == -1:
            if dhd.errorGetLast() != ErrorNum.TIMEOUT:
                raise fdsdk.util.errno_to_exception(dhd.errorGetLast())(
                    ID=self._id,
                    op='forcedimension.dhd.getLinearVelocity()'
                )
            else:
                self._v[0] = nan
                self._v[1] = nan
                self._v[2] = nan

        return self

    def update_wrist_state(self) -> Self:
        """
        Sequentially updates the orientation and the angular velocity of end-effector
        (states affected by the WRIST structure).
        """

        self.update_orientation_angles()
        self.update_angular_velocity()

        return self

    def update_orientation_angles(self) -> Self:
        """
        Performs a blocking read to the HapticDevice, requesting the current
        angle of each joint (in [rad]), starting with the one located nearest
        to the wrist base plate and stores it in an internal buffer. For the
        :data:`forcedimension.dhd.constants.DHD_DEVICE_OMEGA6_RIGHT` and
        :data:`forcedimension.dhd.constants.DHD_DEVICE_OMEGA6_LEFT`
        DHD_DEVICE_OMEGA6_LEFT devices, angles are computed with respect to
        their internal reference frame, which is rotated 45 degrees around the
        Y axis.

        :raises DHDErrorNotAvailable:
            If the device does not have a wrist.

        :raises DHDErrorCom:
            If a communication error occurred with the device.
        """

        err = dhd.direct.getOrientationRad(self._orientation_angles, self._id)

        if err == -1:
            raise fdsdk.util.errno_to_exception(dhd.errorGetLast())(
                ID=self._id,
                op='forcedimension.dhd.getLinearVelocity()'
            )

        return self

    def update_angular_velocity(self) -> Self:
        """
        Performs a blocking read to the HapticDevice, requesting the current
        angular velocity of the end-effector. The requested values are then
        loaded into an internal buffer.

        :raises DHDErrorCom:
            If a communication error occurred with the device.
        """

        err = dhd.direct.getAngularVelocityRad(self._w, self._id)

        if err:
            if dhd.errorGetLast() != ErrorNum.TIMEOUT:
                raise fdsdk.util.errno_to_exception(dhd.errorGetLast())(
                    ID=self._id,
                    op='forcedimension.dhd.getAngularVelocityRad()'
                )
            else:
                self._w[0] = nan
                self._w[1] = nan
                self._w[2] = nan

        return self

    def update_force(self) -> Self:
        """
        Performs a blocking read to the HapticDevice, requesting the current
        force the end end-effector is experiencing. The requested values are
        then loaded into an internal buffer.

        :raises DHDErrorCom:
            If a communication error occurred with the device.
        """

        err = dhd.getForce(self._f, self._id)

        if err == -1:
            raise fdsdk.util.errno_to_exception(dhd.errorGetLast())(
                ID=self._id,
                op='forcedimension.dhd.getForce()'
            )

        return self

    def update_force_and_torque(self) -> Self:
        """
        Performs a blocking read to the HapticDevice, requesting in parallel
        the current force and torque applied to the end-effector. The requested
        values are then loaded into internal buffers.

        :raises DHDErrorCom:
            If a communication error occurred with the device.
        """

        if dhd.direct.getForceAndTorque(self._f, self._t, self._id):
            raise fdsdk.util.errno_to_exception(dhd.errorGetLast())(
                ID=self._id,
                op='forcedimension.dhd.getForceAndTorque()'
            )

        return self

    def update_force_and_torque_and_gripper_force(self) -> Self:
        """
        Performs a blocking read to the HapticDevice, requesting in parallel
        the force and torque applied to the end-effector as well as the force
        applied to the gripper. The requested values are then
        loaded into internal buffers.

        :raises DHDErrorCom:
            If a communication error occurred with the device.

        :raises DHDErrorNotAvailable:
            If the device does not have a gripper.

        """

        err = dhd.direct.getForceAndTorqueAndGripperForce(
            self._f, self._t, self._mock_gripper._fg, self._id
        )

        if err:
            raise fdsdk.util.errno_to_exception(dhd.errorGetLast())(
                ID=self._id,
                op='forcedimension.dhd.getForceAndTorqueAndGripperForce()'
            )

        return self

    def update_orientation(self) -> Self:
        """
        Performs a blocking read to the HapticDevice,
        its orientation frame matrix and updates an internal buffer with those
        values.

        :raises DHDErrorCom:
            If a communication error occurred with the device.

        :raises DHDErrorNotAvailable:
            If the device does not have a wrist.

        """

        if dhd.direct.getOrientationFrame(self._frame, self._id):
            raise fdsdk.util.errno_to_exception(dhd.errorGetLast())(
                ID=self._id,
                op='forcedimension.dhd.getOrientationFrame()'
            )

        return self

    def update_position_and_orientation(self) -> Self:
        """
        Performs a blocking read to the HapticDevice, requesting in parallel
        the position of the end effector (in [m]) about the X, Y, and Z axes
        as well as its orientation frame matrix. The requested values are then
        loaded into internal buffers.

        :raises DHDErrorCom:
            If a communication error occurred with the device.

        :raises DHDErrorNotAvailable:
            If the device does not have a wrist.
        """

        err = dhd.direct.getPositionAndOrientationFrame(
            self._pos, self._frame, self._id
        )

        if err == -1:
            raise fdsdk.util.errno_to_exception(dhd.errorGetLast())(
                ID=self._id,
                op='forcedimension.dhd.getPositionAndOrientationFrame()'
            )

        return self

    def update_buttons(self) -> Self:
        """
        Performs a blocking read and gets the state of all buttons on the
        device into a int bit vector buffer.

        See Also
        --------
        :func:`HapticDevice.get_button`
        """

        self._buttons = dhd.getButtonMask(ID=self._id)

        return self

    def update_status(self) -> Self:
        """
        Perform a blocking read to the HapticDevice, requesting all pertinent
        status information.

        :raises DHDErrorCom:
            If a communication error has occured with the device.

        :returns:
            :class:`forcedimension.dhd.adaptors.Status` object containing all
            status information.
        """

        if dhd.getStatus(self._status, ID=self._id):
            raise fdsdk.util.errno_to_exception(dhd.errorGetLast())()

        return self

    def submit(self, respect_neutral_stop: bool = True) -> Self:
        """
        Push the requested forces and torques to the device in a blocking send.

        :raises DHDErrorCom:
            If a communication error has occured with the device.

        See Also
        --------
        :func:`HapticDevice.req`
        """

        if respect_neutral_stop and (self._is_neutral or self._is_stopped):
            return self

        if (not respect_neutral_stop) and self._is_neutral:
            self._is_neutral = False
            self.enable_brakes()

        if (not respect_neutral_stop) and self._is_stopped:
            self._is_neutral = False
            self.enable_force()

        if self._regulator._is_drd_running:
            err = drd.setForceAndTorqueAndGripperForce(
                self._f_req,
                self._t_req,
                self._mock_gripper._fg_req,
                self._id
            )

            if err == -1:
                raise fdsdk.util.errno_to_exception(dhd.errorGetLast())(
                    ID=self._id,
                    op='forcedimension.drd.setForceAndTorqueAndGripperForce()'
                )
        else:
            err = dhd.setForceAndTorqueAndGripperForce(
                self._f_req, self._t_req, self._mock_gripper._fg_req, self._id
            )

            if err == -1:
                raise fdsdk.util.errno_to_exception(dhd.errorGetLast())(
                    ID=self._id,
                    op='forcedimension.dhd.setForceAndTorqueAndGripperForce()'
                )

        return self

    def req(
        self, f: Array[int, float], t: Optional[Array[int, float]] = None
    ) -> Self:
        """
        Load the requested force and request torque buffer for this device.
        This won't send the request to the device. This is used by the
        HapticDaemon.

        :param Array[int, int] f:
            The force (in [N]) to apply to the end effector about
            the X, Y, and Z axes.

        :param Array[int, int] t:
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

        if t is not None:
            self._t_req[0] = t[0]
            self._t_req[1] = t[1]
            self._t_req[2] = t[2]

        return self

    def req_vibration(self, freq: float, amplitude: float) -> Self:
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

    def submit_vibration(self) -> Self:
        """
        Push the requested vibration to the device in a blocking send.

        :raises DHDErrorCom:
            If a communication error has occured with the device.

        See Also
        --------
        :func:`HapticDevice.submit()`
        """

        err = dhd.setVibration(
            self._vibration_req[0], self._vibration_req[1], 0, self._id
        )

        if err == -1:
            raise fdsdk.util.errno_to_exception(dhd.errorGetLast())(
                ID=self._id,
                op='forcedimension.dhd.setVibration()'
            )

        return self

    def zero_force(self) -> Self:
        """
        Sets the forces to 0. This makes the end-effector translationally
        "transparent" (i.e. it will try to be close to frictionless and move
        as though it was in free space).
        """

        self._f_req[0] = 0.0
        self._f_req[1] = 0.0
        self._f_req[2] = 0.0

        return self

    def zero_torque(self) -> Self:
        """
        Sets the torque to 0. This makes the end-effector rotationally
        "transparent" (i.e. it will try to be close to frictionless and move
        as though it was in free space).
        """

        self._t_req[0] = 0.0
        self._t_req[1] = 0.0
        self._t_req[2] = 0.0

        return self

    def zero(self) -> Self:
        """
        Sets the applied force and torque to 0. This makes the end-effector
        rotationally and translationally  "transparent" (i.e. it will try to be
        close to frictionless and move as though it was in free space).
        """

        self.zero_force()
        self.zero_torque()

        return self

    def neutral(self) -> Self:
        """
        Disable electromagnetic braking and put the device in IDLE mode,
        consequently disabling forces. A call to
        :func:`forcedimension.HapticDevice.req` or
        :func:`forcedimension.HapticDevice.submit` with
        `respect_neutral_stop=False` will re-enable forces.
        """

        self._is_neutral = True
        self.enable_brakes(enabled=False)

        return self

    def stop(self) -> Self:
        """
        Disable force and put the device in BRAKE mode. You may feel a viscous
        force that keeps that device from moving too quickly in this mode if
        the electromagnetic brakes are enabled. A call to
        :func:`forcedimension.HapticDevice.req` or
        :func:`forcedimension.HapticDevice.submit` with
        `respect_neutral_stop=False` will re-enable forces.
        """

        self._is_stopped = True
        if dhd.stop(self._id):
            raise fdsdk.util.errno_to_exception(dhd.errorGetLast())(
                ID=self._id,
                op='forcedimension.dhd.stop()'
            )

        return self

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
            ``True`` if the check succeeded, and ``False`` if the firmware of
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

    def set_max_force(self, limit: Optional[float]) -> Self:
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
            raise fdsdk.util.errno_to_exception(dhd.errorGetLast())

        return self

    @property
    def max_torque(self) -> Optional[float]:
        """
        Retrieve the current limit (in [Nm]) to the torque magnitude that can be
        applied by the haptic device. The limit is `None` if there is no limit.
        """

        return self._max_torque

    def set_max_torque(self, limit: Optional[float]) -> Self:
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
            raise fdsdk.util.errno_to_exception(dhd.errorGetLast())()

        return self

    def get_button(self, button_id: int = 0) -> bool:
        """
        See if the button on the device is being pressed.

        :param int button_id:
            The button to check

        :returns:
            ``True`` if the button is being pressed, ``False`` otherwise.

        See Also
        --------
        :class:`forcedimension.dhd.constants.NovintButtonID`

        """

        return bool(self._buttons & _cast(int, 1 << button_id))

    def close(self):
        """
        Shuts down any polling being done on the device and then closes the
        handle to the device.
        """

        self._regulator.stop_poll()
        drd.close(self._id)

    def __enter__(self) -> Self:
        return self

    def __exit__(self, t, value, traceback):
        self.close()

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
        f: Callable[[], Any],
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
        self,
        *args,
        wait_for: Union[Callable[[], Any], float],
        high_precision: bool = True,
        **kwargs
    ):
        if not (
            isinstance(wait_for, int) or
            isinstance(wait_for, float) or
            isinstance(wait_for, Callable)
        ):
            raise TypeError("'wait_for' must be numeric or Callable")

        if isinstance(wait_for, float):
            if wait_for < 0:
                raise ValueError(
                    "'wait_for' must be greater than 0 if it's numeric"
                )
            self._interval = wait_for

        elif isinstance(wait_for, Callable):
            self._wait_func = wait_for
            self._interval = 0.

        super().__init__(*args, **kwargs)

        self._regulator: HapticDevice.Regulator

        if (target := kwargs.get('target')) is None:
            raise ValueError("HapticPoller must have a target.")

        self._target = target

        self._unpause_event = Event()
        self._pause_sync = Event()

        self._target_args = kwargs.get('args', ())
        self._target_kwargs = kwargs.get('kwargs', {})

        self._wait_func: Optional[Callable[[], Any]] = None
        self._is_paused = False

        self._high_precision = high_precision

    def start(self, regulator: HapticDevice.Regulator):
        self._regulator = regulator
        self._stop_event = regulator._stop_event
        super().start()

    def stop(self):
        if self._is_paused:
            self.unpause()

        self._stop_event.set()
        self.join()

    def pause(self, synchronized: bool = True):
        self._unpause_event.clear()
        self._is_paused = True

        if synchronized:
            self._pause_sync.wait()

    def unpause(self):
        self._unpause_event.set()
        self._pause_sync.clear()

    def _execute_target(self):
        self._target(*self._target_args, **self._target_kwargs)

    def _run_zero_interval(self):
        while not self._stop_event.is_set():
            if not self._is_paused:
                self._execute_target()

                if self._wait_func is not None:
                    self._wait_func()
            else:
                self._pause_sync.set()
                self._unpause_event.wait()

    def _run_low_precision(self):
        while not self._stop_event.wait(self._interval):

            if not self._is_paused:
                self._execute_target()
            else:
                self._pause_sync.set()
                self._unpause_event.wait()

    def _run_high_precision(self):
        t0 = time.perf_counter()

        wait_period = self._interval

        if self._interval > 0.001:
            sleep_period = self._interval - 0.001
        else:
            sleep_period = 0

        if not self._stop_event.is_set():
            self._execute_target()

        t_sleep = time.perf_counter()

        while not self._stop_event.is_set():
            # Sleep if possible
            if sleep_period > 0:
                return self._stop_event.wait(sleep_period)

            # Spin without holding the GIL the remaining time
            spin_period = wait_period - (time.perf_counter() - t_sleep)
            if spin_period > 0:
                util.spin(spin_period)

            t0 = time.perf_counter()
            if not self._is_paused:
                self._execute_target()
            else:
                self._pause_sync.set()
                self._unpause_event.wait()

                sleep_period = 0.
                wait_period = 0.

                continue

            # Try to estimate how long the function takes to execute
            wait_period = self._interval - (time.perf_counter() - t0)

            # Function took longer than the interval.
            # Schedule the next call ASAP.
            if wait_period < 0:
                wait_period = 0.
                sleep_period = 0.

                continue

            # Never sleep less than 1ms, but sleep as much as you can.
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

            if self._high_precision:
                self._run_high_precision()
            else:
                self._run_low_precision()

        except Exception as ex:
            self._regulator._parent._exception = ex
            self._regulator.notify_end()
        finally:
            del self._target


@typing_extensions.deprecated(
    "HapticDaemon is deprecated as of Force Dimension Bindings "
    "v0.2.0. Use HapticDevice.Regulator.poll() instead."
)
class HapticDaemon(Thread):
    def __init__(
        self,
        dev: HapticDevice,
        update_list: UpdateOpts = UpdateOpts(),
        forceon=False
    ):

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
                self._dev.update_delta_encs_and_calculate,
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
            _Poller(lambda: self._dev.submit(True),
                    1/update_list.req, self._forceon)
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

        except fdsdk.dhd.adaptors.DHDIOError as ex:
            self._paused = True

            for poller in self._pollers:
                poller.stop()
            self._dev._exception = ex
