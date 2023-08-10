__version__ = '0.2.0'

import ctypes as ct
import time
from math import nan
from threading import Condition, Lock, Thread, Event
from typing import Callable, Generic, List, Optional, Type, TypeVar
from typing import cast as _cast
import warnings

import forcedimension.dhd as dhd
import forcedimension.drd as drd
from forcedimension.containers import (
    DefaultDOFEncsType, DefaultDOFJointAnglesType, DefaultEnc3Type,
    DefaultMat3x3Type, DefaultMat6x6Type, DefaultVecType,
    GripperUpdateOpts, UpdateOpts
)
from forcedimension.dhd import ErrorNum, Status
from forcedimension.typing import GenericVec, IntVectorLike
from forcedimension.util import ImmutableWrapper, spin

T = TypeVar('T', bound=GenericVec)


dhd.expert.enableExpertMode()

class HapticDevice(Generic[T]):
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

        def __init__(self, parent):
            self._parent: HapticDevice = parent

            self._id = parent.ID
            self._enc = ct.c_int()
            self._angle = ct.c_double()
            self._gap = ct.c_double()
            self._v = ct.c_double()
            self._w = ct.c_double()
            self._fg = ct.c_double()

            self._thumb_pos = self._parent._VecType()
            self._finger_pos = self._parent._VecType()

            self._thumb_pos_view = ImmutableWrapper(self._thumb_pos)
            self._finger_pos_view = ImmutableWrapper(self._finger_pos)

            self._fg_req: float = 0.0

        def req(self, fg: float):
            self._fg_req = fg

        def config_velocity(
            self, window_size: int = dhd.DEFAULT_VELOCITY_WINDOW,
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
                    op=dhd.configGripperVelocity, ID=self._id
                )

        def get_max_force(self) -> Optional[float]:
            """
            Retrieve the current limit (in [N]) to the force magnitude that can be
            applied by the haptic device. The limit is `None` if there is no limit.
            """

            limit = dhd.getMaxGripperForce(self._id)

            return limit if limit > 0 else None

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

        @property
        def thumb_pos(self) -> T:
            """
            Provides a read-only reference to the last-known position of the
            thumb rest position (in [m]) of the gripper about the X, Y, and Z
            axes.

            :raises DHDError:
                If an error has occured with the device, invalidating the
                device state.
            """

            self.check_exception()
            return _cast(T, self._thumb_pos_view)

        @property
        def finger_pos(self) -> T:
            """
            Provides a read-only reference to the last-known position of the
            forefinger rest position of the gripper (in [m]) about the X, Y, and Z
            axes.

            :raises DHDError:
                If an error has occured with the device, invalidating the
                device state.
            """

            self.check_exception()
            return _cast(T, self._finger_pos_view)

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

        def update_enc_and_calculate(self):
            """
            Update the value of the gripper encoders and calculate the value of the
            gripper opening (in [m]) and gripper opening angle (in [rad]).

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
            of the gripper forefinger rest position (in [m]) about the X, Y, and
            Z axes.

            :raises DHDError:
                If an error has occured with the device, invalidating the
                device state.
            """

            err = dhd.direct.getGripperFingerPos(self._finger_pos, self._id)

            if err == -1:
                raise dhd.errno_to_exception(dhd.errorGetLast())()

        def check_exception(self):
            self._parent.check_exception()

    class Regulator:
        def __init__(self, parent):
            self._parent: HapticDevice = parent
            self._haptic_daemon: Optional[HapticDaemon] = None

        def stop(self):
            if self._haptic_daemon is not None:
                self._haptic_daemon.stop()
                return

    def __init__(
            self,
            VecType: Type[T] = DefaultVecType,
            *,
            ID: Optional[int] = None,
            devtype: Optional[dhd.DeviceType] = None,
            serial_number: Optional[int] = None,
            ensure_memory: bool = False,
            **kwargs
    ):
        """
        Create a handle to a Force Dimension haptic device. You may specify
        an ID, device type, or serial number to more precisely control which
        device is opened. If none are specified, the first device found is
        opened. It is recommended to use context management
        (i.e. using a `with` statement) to ensure the device is properly
        closed. Once opened, properties of the device are stored as fields
        in the class.

        :param Type[T] VecType:
            The default type for vector buffers used by this class. You may
            make your own custom types, but it's recommended to use
            `forcedimension.containers.DefaultVecType`.

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

        self._VecType = VecType
        self._devtype = dhd.DeviceType.NONE
        self._mass = nan

        self.gripper = None

        self._exception: Optional[dhd.DHDIOError] = None
        self._id = 0

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
                    op=dhd.getSystemType
                )()

            self._devtype = devtype_opened
        elif ID is not None:
            # Open device of given ID.

            if (id := drd.openID(ID)) == -1:
                raise dhd.errno_to_exception(dhd.errorGetLast())()

            self._id = id

            if (devtype_opened := dhd.getSystemType()) == -1:
                raise dhd.errno_to_exception(dhd.errorGetLast())(
                    ID=self._id,
                    op=dhd.getSystemType
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
                    op=dhd.close
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
                    op=dhd.close
                )

            if drd.openID(self._id) == -1:
                raise dhd.errno_to_exception(dhd.errorGetLast())()

            if (devtype_opened := dhd.getSystemType()) == -1:
                raise dhd.errno_to_exception(dhd.errorGetLast())(
                    ID=self._id,
                    op=dhd.getSystemType
                )

            self._devtype = devtype_opened

        if ensure_memory:
            if not self.check_controller_memory():
                raise dhd.DHDErrorConfiguration(ID=self._id)

        if 'wait_for_reset' in kwargs:
            self.wait_for_reset(kwargs['wait_for_reset'])

        self._regulator = HapticDevice.Regulator(self)

        self._encs = DefaultDOFEncsType()
        self._joint_angles = DefaultDOFJointAnglesType()

        self._pos = VecType()
        self._w = VecType()
        self._v = VecType()
        self._f = VecType()
        self._t = VecType()

        self._delta_jacobian = DefaultMat3x3Type()
        self._wrist_jacobian = DefaultMat3x3Type()
        self._frame = DefaultMat3x3Type()

        self._inertia_matrix = DefaultMat6x6Type()

        self._status = Status()

        self._f_req = VecType()
        self._t_req = VecType()
        self._vibration_req: List[float] = [0.] * 2
        self._buttons = 0

        self._delta_joint_angles_view = ImmutableWrapper(
            self._joint_angles.delta
        )

        self._wrist_joint_angles_view = ImmutableWrapper(
            self._joint_angles.wrist
        )

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

        self._gripper = HapticDevice.Gripper(self)
        if dhd.hasGripper(self._id):
            self.gripper = self._gripper

        self._is_neutral = False
        self._is_stopped = False
        self._button_emulation_enabled = False
        self._left_handed = dhd.isLeftHanded(self._id)
        self._has_base = dhd.hasBase(self._id)
        self._has_active_gripper = dhd.hasActiveGripper(self._id)
        self._has_wrist = dhd.hasWrist(self._id)
        self._has_active_wrist = dhd.hasActiveWrist(self._id)

        if (com_mode := dhd.getComMode(self._id)) == -1:
            raise dhd.errno_to_exception(dhd.errorGetLast())(
                ID=self._id, op=dhd.getComMode
            )

        if (max_force := dhd.getMaxForce(self._id)) < 0:
            max_force = None

        if (max_torque := dhd.getMaxTorque(self._id)) < 0:
            max_torque = None

        self._mass, err = dhd.getEffectorMass(self._id)
        if err:
            raise dhd.errno_to_exception(dhd.errorGetLast())()

        self._com_mode = com_mode
        self._max_force = max_force
        self._max_torque = max_torque

        self.update_delta_enc_and_calculate()
        self.update_force_and_torque_and_gripper_force()

    def check_exception(self):
        if self._exception is not None:
            raise self._exception

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
    def left_handed(self) -> Optional[bool]:
        return self._left_handed

    @property
    def has_base(self) -> bool:
        return self._has_base

    @property
    def has_wrist(self) -> bool:
        return self._has_wrist

    @property
    def has_active_wrist(self) -> bool:
        return self._has_active_wrist

    @property
    def has_active_gripper(self) -> bool:
        return self._has_active_gripper

    @property
    def com_mode(self) -> dhd.ComMode:
        return self._com_mode

    @property
    def is_neutral(self) -> bool:
        return self._is_neutral

    @property
    def is_stopped(self) -> bool:
        return self._is_stopped

    @property
    def buton_emulation_enabled(self) -> bool:
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
                feature=dhd.setBrakes
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
                op=dhd.configLinearVelocity, ID=self._id
            )


    def config_angular_velocity(
        self, window_size: int, mode: dhd.VelocityEstimatorMode
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
                op=dhd.configAngularVelocity, ID=self._id
            )



    @property
    def delta_joint_angles(self) -> T:
        self.check_exception()
        return _cast(T, self._delta_joint_angles_view)

    @property
    def wrist_joint_angles(self) -> T:
        self.check_exception()
        return _cast(T, self._wrist_joint_angles_view)

    @property
    def pos(self) -> T:
        """
        Provides a read-only reference to the last-known position of the
        HapticDevice's end-effector (in [m]) about the X, Y, and Z axes.
        Thread-safe.

        :raises DHDError:
            If an error has occured with the device, invalidating the
            device state.
        """

        self.check_exception()
        return _cast(T, self._pos_view)

    @property
    def v(self) -> T:
        """
        Provides a read-only reference to the last-known linear velocity of the
        HapticDevice's end-effector. Thread-safe.

        :raises DHDError:
            If an error has occured with the device, invalidating the
            device state.
        """

        self.check_exception()
        return _cast(T, self._v_view)

    @property
    def w(self) -> T:
        """
        Provides a read-only reference to the last-known angular velocity of the
        HapticDevice's end-effector. Thread-safe.

        :raises DHDError:
            If an error has occured with the device, invalidating the
            device state.
        """

        self.check_exception()
        return _cast(T, self._w_view)

    @property
    def t(self) -> T:
        """
        Provides a read-only reference to the last-known applied torque of the
        HapticDevice's end-effector. Thread-safe.

        :raises DHDError:
            If an error has occured with the device, invalidating the
            device state.
        """

        self.check_exception()
        return _cast(T, self._t_view)

    @property
    def f(self) -> T:
        """
        Provides a read-only reference to the last-known applied force of the
        HapticDevice's end-effector. Thread-safe.

        :raises DHDError:
            If an error has occured with the device, invalidating the
            device state.
        """

        self.check_exception()
        return _cast(T, self._f_view)

    @property
    def delta_jacobian(self) -> DefaultMat3x3Type:
        self.check_exception()
        return _cast(DefaultMat3x3Type, self._delta_jacobian_view)

    @property
    def wrist_jacobian(self) -> DefaultMat3x3Type:
        self.check_exception()
        return _cast(DefaultMat3x3Type, self._wrist_jacobian_view)

    @property
    def frame(self) -> DefaultMat3x3Type:
        self.check_exception()
        return _cast(DefaultMat3x3Type, self._frame_view)

    @property
    def inertia_matrix(self) -> DefaultMat6x6Type:
        self.check_exception()
        return _cast(DefaultMat6x6Type, self._inertia_matrix_view)


    def calculate_pos(self):
        """
        Calculates and stores the position of the device given the current
        end-effector positionin the internal buffer.
        """

        dhd.expert.direct.deltaEncoderToPosition(
            self._encs.delta, self._pos, self._id
        )

    def calculate_delta_joint_angles(self):
        """
        Calculates and stores the joint angles of the DELTA structure given
        the current end-effector encoder readings in the internal buffer.
        """

        dhd.expert.direct.deltaEncodersToJointAngles(
            self._encs.wrist, self._joint_angles.delta, self._id
        )

    def calculate_delta_jacobian(self):
        """
        Calculates and stores the Jacobian matrix of the DELTA structure given
        current joint angle configuration in the internal buffer.
        """

        self.calculate_delta_joint_angles()

        dhd.expert.direct.deltaJointAnglesToJacobian(
            self._joint_angles.delta, self._delta_jacobian, self._id
        )

    def calculate_wrist_joint_angles(self):
        """
        Calculates and stores the joint angles of the WRIST structure given
        the current end-effector encoder readings in the internal buffer.
        """

        dhd.expert.direct.wristEncodersToJointAngles(
            self._encs.wrist, self._joint_angles.wrist, self._id
        )

    def calculate_wrist_jacobian(self):
        """
        Calculates and stores the Jacobian matrix of the WRIST structure given
        current joint angle configuration in the internal buffer.
        """

        dhd.expert.direct.wristJointAnglesToJacobian(
            self._joint_angles.wrist, self._wrist_jacobian, self._id
        )

    def calculate_inertia_matrix(self):
        """
        Calculates the 6x6 inertia matrix (with respect to the X, Y, and Z
        axes) given the current joint angles configuration in the internal
        buffer.
        """

        dhd.expert.direct.jointAnglesToIntertiaMatrix(
            self._joint_angles, self._inertia_matrix, self._id
        )

    def update_delta_enc_and_calculate(self):
        """
        Updates the DELTA encoders and given those values, calculates the
        position of the end-effector, DELTA joint angles, and the DELTA
        jacobian.
        """

        self.update_delta_enc()
        self.calculate_pos()
        self.calculate_delta_joint_angles()
        self.calculate_delta_jacobian()

    def update_wrist_enc_and_calculate(self):
        """
        Updates the WRIST encoders and given those values, calculates the wrist
        joint angles and the WRIST jacobian.
        """

        self.update_wrist_enc()
        self.calculate_wrist_joint_angles()
        self.calculate_wrist_jacobian()

    def update_delta_enc(self):
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
                    feature=dhd.expert.getDeltaEncoders
            )

    def update_wrist_enc(self):
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
                    feature=dhd.expert.getWristEncoders
            )

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
            raise dhd.errno_to_exception(
                dhd.errorGetLast())(
                    ID=self._id,
                    feature=dhd.getPosition
            )

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
                    feature=dhd.getLinearVelocity
                )
            else:
                self._v[0] = nan
                self._v[1] = nan
                self._v[2] = nan

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
                    feature=dhd.getAngularVelocityRad
                )
            else:
                self._w[0] = nan
                self._w[1] = nan
                self._w[2] = nan

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
                feature=dhd.getForce
            )

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
                feature=dhd.getForceAndTorque
            )

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
            self._f, self._t, self._gripper._fg, self._id
        )

        if err:
            raise dhd.errno_to_exception(dhd.errorGetLast())(
                ID=self._id,
                feature=dhd.getForceAndTorqueAndGripperForce
            )

    def update_orientation(self):
        """
        Performs a blocking read to the HapticDevice,
        its orientation frame matrix and updates an internal buffer with those
        values.
        """

        if dhd.direct.getOrientationFrame(self._frame, self._id):
            raise dhd.errno_to_exception(dhd.errorGetLast())(
                ID=self._id,
                feature=dhd.getForceAndTorqueAndGripperForce
            )


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
                feature=dhd.getForceAndTorqueAndGripperForce
            )

    def update_buttons(self):
        """
        Performs a blocking read and gets the state of all buttons on the
        device into a int bit vector buffer.

        See Also
        --------
        :func:`HapticDevice.get_button`
        """

        self._buttons = dhd.getButtonMask(ID=self._id)

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


    def submit(self, respect_neutral_stop=False):
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
            self._f_req, self._t_req, self._gripper._fg_req, self._id
        )

        if err == -1:
            raise dhd.errno_to_exception(dhd.errorGetLast())(
                ID=self._id,
                feature=dhd.setForceAndTorqueAndGripperForce
            )

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

    def submit_vibration(self):
        err = dhd.setVibration(
            self._vibration_req[0], self._vibration_req[1], 0, self._id
        )

        if err == -1:
            raise dhd.errno_to_exception(dhd.errorGetLast())(
                ID=self._id,
                feature=dhd.setVibration
            )

    def neutral(self):
        """
        Disable electromagnetic braking and put the device in IDLE mode,
        consequently disabling forces. A call to
        :func:`forcedimension.HapticDevice.req` or
        :func:`forcedimension.HapticDevice.submit` with `respect_neutral=False`
        will re-enable forces.
        """

        self._is_neutral = True
        self.enable_brakes(enabled=False)

    def stop(self):
        """
        Disable force and put the device in BRAKE mode. You may feel a viscous
        force that keeps that device from moving too quickly in this mode if
        the electromagnetic brakes are enabled. A call to
        :func:`forcedimension.HapticDevice.req` or
        :func:`forcedimension.HapticDevice.submit` with `respect_neutral=False`
        will re-enable forces.
        """

        if dhd.stop(self._id):
            raise dhd.errno_to_exception(dhd.errorGetLast())(
                ID=self._id,
                op=dhd.stop
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
            raise dhd.DHDErrorTimeout(op=dhd.waitForReset, ID=self._id)

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
                    self._dev.gripper is not None
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

        if update_list.gripper is not None and self._dev.gripper is not None:
            funcs = (
                self._dev.gripper.update_enc_and_calculate,
                self._dev.gripper.update_finger_pos,
                self._dev.gripper.update_thumb_pos,
                self._dev.gripper.update_linear_velocity,
                self._dev.gripper.update_angular_velocity
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


