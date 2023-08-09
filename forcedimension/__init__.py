__version__ = '0.2.0'

import ctypes as ct
import time
from math import nan
from threading import Condition, Lock, Thread
from typing import Callable, Generic, List, Optional, Type, TypeVar
from typing import cast as _cast

import forcedimension.dhd as dhd
import forcedimension.drd as drd
from forcedimension.containers import (
    DefaultEnc3Type, DefaultMat3x3Type, DefaultVecType,
    GripperUpdateOpts, UpdateOpts
)
from forcedimension.dhd import ErrorNum, Status
from forcedimension.typing import GenericVec, IntVectorLike
from forcedimension.util import ImmutableWrapper

T = TypeVar('T', bound=GenericVec)


dhd.expert.enableExpertMode()


class HapticDevice(Generic[T]):
    """
    A HapticDevice is a high-level wrapper for any compatible Force Dimension
    device.
    """

    def __init__(
            self,
            ID: Optional[int] = None,
            devtype: Optional[dhd.DeviceType] = None,
            serial_number: Optional[int] = None,
            VecType: Type[T] = DefaultVecType
    ):
        """
        Create a handle to a Force Dimension haptic device. You may specify
        an ID, device type, or serial number to more precisely control which
        device is opened. If none are specified, the first device found is
        opened. It is recommended to use context management
        (i.e. using a `with` statement) to ensure the device is properly
        closed. Once opened, properties of the device are stored as fields
        in the class.

        :param Optional[int] ID:
            If specified, will open the device of the given ID.

        :param Optional[DeviceType] devtype:
            If specified, will open the first device of the give type.

        :param Optional[int] serial_number:
            If specified, will open the device of the given serial number.
            This feature is only available on newer devices.

        :param Type[T] VecType:
            The default type for vector buffers used by this class. You may
            make your own custom types, but it's recommended to use
            `forcedimension.containers.DefaultVecType`.

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

        self._open = True

        self._enc = DefaultEnc3Type()
        self._joint_angles = VecType()

        self._pos = VecType()
        self._w = VecType()
        self._v = VecType()
        self._f = VecType()
        self._t = VecType()
        self._J = DefaultMat3x3Type()
        self._status = Status()

        self._req = False
        self._f_req = VecType()
        self._t_req = VecType()
        self._vibration_req: List[float] = [0.] * 2
        self._buttons = 0

        self._pos_view = ImmutableWrapper(data=self._pos)
        self._w_view = ImmutableWrapper(data=self._w)
        self._v_view = ImmutableWrapper(data=self._v)
        self._f_view = ImmutableWrapper(data=self._f)
        self._t_view = ImmutableWrapper(data=self._t)
        self._J_view = ImmutableWrapper(data=self._J)
        self._status_view = ImmutableWrapper(data=self._status)

        self._devtype = dhd.DeviceType.NONE
        self._mass = nan

        self.gripper = None

        self._thread_exception: Optional[dhd.DHDIOError] = None

        self._haptic_daemon: Optional[HapticDaemon] = None
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

        self._left_handed = dhd.isLeftHanded(self._id)
        self._gripper = Gripper(self, self._id)

        if dhd.hasGripper(self._id):
            self.gripper = self._gripper

        self._mass, err = dhd.getEffectorMass(self._id)

        if err:
            raise dhd.errno_to_exception(dhd.errorGetLast())()

        self.update_enc_and_calculate()
        self.update_force_and_torque_and_gripper_force()

    def check_threadex(self):
        if self._thread_exception is not None:
            raise self._thread_exception

    @property
    def ID(self) -> int:
        """
        Provides a read-only reference to the ID of the HapticDevice.
        Thread-safe.

        :returns: The ID of the HapticDevices
        """

        self.check_threadex()
        return self._id

    @property
    def devtype(self) -> dhd.DeviceType:
        return self._devtype

    @property
    def pos(self) -> T:
        """
        Provides a read-only reference to the last-known position of the
        HapticDevice's end-effector. Thread-safe.

        :returns:
            A mutable sequence of [x, y, z] where x, y, and z are the
            end-effector's position given in [m].
        """

        self.check_threadex()
        return _cast(T, self._pos_view)

    @property
    def mass(self) -> float:
        """
        Get the mass of the end-effector used for gravity compensation in [kg].
        Thread-safe.

        :returns: the set mass of the end-effector in [kg]
        """

        return self._mass

    def set_mass(self, m: float):
        """
        Sets the mass of the end-effector used for gravity compensation
        (in [kg]). Thread-safe.
        """
        if dhd.setEffectorMass(m, ID=self._id):
            raise dhd.errno_to_exception(dhd.errorGetLast())()

    @property
    def v(self) -> T:
        """
        Provides a read-only reference to the last-known linear velocity of the
        HapticDevice's end-effector. Thread-safe.

        :returns:
            A mutable sequence of [vx, vy, vz] where vx, vy, and vz are
            the end-effector's linear velocity given in [m/s].
        """

        self.check_threadex()
        return _cast(T, self._v_view)

    @property
    def w(self) -> T:
        """
        Provides a read-only reference to the last-known angular velocity of the
        HapticDevice's end-effector. Thread-safe.

        :returns:
            A mutable sequence of [wx, wy, wz] where wx, wy, and wz are
            the end-effector's linear velocity given in [rad/s].
        """

        self.check_threadex()
        return _cast(T, self._w_view)

    @property
    def t(self) -> T:
        """
        Provides a read-only reference to the last-known applied torque of the
        HapticDevice's end-effector. Thread-safe.

        :returns:
            A mutable sequence of [tx, ty, tz] where tx, ty, and tz are
            the torque experienced by the end-effector in [Nm]
        """

        self.check_threadex()
        return _cast(T, self._t_view)

    @property
    def f(self) -> T:
        """
        Provides a read-only reference to the last-known applied force of the
        HapticDevice's end-effector. Thread-safe.

        :returns:
            A mutable sequence of [fx, fy, fz] where fx, fy, and fz are
            the torque experienced by the end-effector in [N]
        """

        self.check_threadex()
        return _cast(T, self._f_view)

    @property
    def left_handed(self) -> Optional[bool]:
        return self._left_handed

    @property
    def status(self) -> Status:
        """
        Provides a read-only reference to the last-known status of the device.
        Thread-safe.

        :returns:
            A Status object representing the last-known status of the device.
        """
        return _cast(Status, self._status_view)

    def calculate_pos(self):
        """
        Calculates and stores the position of the device given the current
        end-effector position.
        """

        dhd.expert.direct.deltaEncoderToPosition(
            self._enc, self._pos, self._id
        )

    def calculate_joint_angles(self):
        """
        Calculates and stores the joint angles of the device given the current
        end-effector encoder readings.
        """

        dhd.expert.direct.deltaEncodersToJointAngles(
            self._enc, self._joint_angles, self._id
        )

    def calculate_jacobian(self):
        """
        Calculates and stores the Jacobian matrix of the device given the
        current end-effector position.
        """

        self.calculate_joint_angles()

        dhd.expert.direct.deltaJointAnglesToJacobian(
            self._joint_angles, self._J, self._id
        )

    def update_enc_and_calculate(self):
        self.update_enc()
        self.calculate_pos()
        self.calculate_jacobian()

    def update_enc(self):
        """
        Performs a blocking read to the HapticDevice, requesting the current
        encoder readers of the (DELTA structure that controls the) end-effector
        and updates the last-known position  with the response.
        """

        err = dhd.expert.getDeltaEncoders(self._enc, self._id)

        if err == -1:
            raise dhd.errno_to_exception(
                dhd.errorGetLast())(
                    ID=self._id,
                    feature=dhd.expert.getDeltaEncoders
            )

    def update_position(self):
        """
        Performs a blocking read to the HapticDevice, requesting the current
        position of the end-effector and updates the last-known position with
        the response.
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
        velocity with the response.
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
        angular velocity with the response.
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
        force with the response.
        """

        err = dhd.getForce(self._f, self._id)

        if err == -1:
            raise dhd.errno_to_exception(ErrorNum(
                dhd.errorGetLast()))(
                    ID=self._id,
                    feature=dhd.getForce
            )

    def update_force_and_torque(self):
        """
        Performs a blocking read to the HapticDevice, requesting in parallel
        the current force and torque applied to the end-effector and
        updates the last-known force and torque with the response.
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
        applied to the gripper. Then, the last known force, torque, and gripper
        force are updated.

        The reason this is in HapticDevice is largely due to an implement
        detail in dhd and as a way to optimize requests to the device.
        """

        err = dhd.direct.getForceAndTorqueAndGripperForce(
            self._f, self._t, self._gripper._fg, self._id
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

        :returns:
            Status object containing all status information.
        """

        if dhd.getStatus(self._status, ID=self._id):
            raise dhd.errno_to_exception(dhd.errorGetLast())()


    def submit(self):
        """
        Push the requested forces and torques to the device in a blocking send.

        See Also
        --------
        :func:`HapticDevice.req`
        """

        self._req = False

        if dhd.setForceAndTorque(self._f_req, self._t_req, self._id) == -1:
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
            The force in [N] to apply to the end effector about
            the x, y, and z axes.

        :param IntVectorLike t:
            The torque in [Nm] to apply to the end effector about the
            x, y,and z axes.

        See Also
        --------
        :func:`HapticDevice.submit`
        """

        self._req = True

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

        :param float freq: frequency of the vibration (in [Hz])
        :param float amplitude: amplitude of the vibration

        See Also
        --------
        :func:`HapticDevice.submit_vibration`
        """

        self._vibration_req[0] = freq
        self._vibration_req[1] = amplitude

    def neutral(self):
        """
        Disable forces and put the device in IDLE mode. Brakes will be turned
        off and gravity compensation disabled. No forces will be allowed to be
        put on the device.
        """

        self.enable_force(enabled=False)
        self.enable_brakes(enabled=False)
        self.enable_gravity_compensation(enabled=False)

    def stop(self):
        """
        Disable force and put the device in BRAKE mode. You may feel a viscous
        force that keeps that device from moving too quickly in this mode.
        """

        dhd.stop(self._id)

    def submit_vibration(self):
        err = dhd.setVibration(
            self._vibration_req[0], self._vibration_req[1], 0, self._id
        )

        if err == -1:
            raise dhd.errno_to_exception(dhd.errorGetLast())(
                ID=self._id,
                feature=dhd.setVibration
            )

    def enable_force(self, enabled: bool = True):
        """
        Enable/disable force on the end-effector.

        :param bool enabled: `True` to enable, `False` to disable
        """

        dhd.enableForce(enabled, ID=self._id)

    def enable_brakes(self, enabled: bool = True):
        """
        Enable electromagnetic braking on the device.

        :param enabled bool:
            `True` to enable electromagnetic braking, `False` to disable.
        """

        if dhd.setBrakes(enabled):
            raise dhd.errno_to_exception(dhd.errorGetLast())(
                ID=self._id,
                feature=dhd.setVibration
            )

    def get_max_force(self) -> Optional[float]:
        """
        Retrieve the current limit (in N) to the force magnitude that can be
        applied by the haptic device.

        :returns:
            The current limit (in N) to the force magnitude that can be
            applied by the haptic device to the end-effector. If there is no
            limit, None is returned instead.
        """

        limit = dhd.getMaxForce(ID=self._id)

        return limit if limit > 0 else None

    def set_max_force(self, limit: Optional[float]):
        """
        Define or disable a limit (in N) to the force magnitude that can be
        applied by the haptic device.

        :param Optional[float] limit:
            The desired limit (in N) to the force magnitude that can be
            applied. If the limit is None, the force limit is disabled.
        """

        if limit is not None:
            if limit < 0:
                raise ValueError("limit must be greater than 0 or None.")

        if limit is None:
            limit = -1.0

        if dhd.setMaxForce(limit, self._id):
            raise dhd.errno_to_exception(dhd.errorGetLast())

    def get_max_torque(self) -> Optional[float]:
        """
        Retrieve the current limit (in Nm) to the torque magnitude that can be
        applied by the haptic device.

        :returns:
            The current limit (in Nm) to the force magnitude that can be
            applied by the haptic device to the end-effector. If there is no
            limit, None is returned instead.
        """

        limit = dhd.getMaxTorque(ID=self._id)

        return limit if limit > 0 else None

    def set_max_torque(self, limit: Optional[float]):
        """
        Define or disable a limit (in N) to the force magnitude that can be
        applied by the haptic device.

        :param Optional[float] limit:
            The desired limit (in N) to the force magnitude that can be
            applied. If the limit is None, the force limit is disabled.
        """

        if limit is not None:
            if limit < 0:
                raise ValueError("limit must be greater than 0 or None.")

        if limit is None:
            limit = -1.0

        if dhd.setMaxTorque(limit, self._id):
            raise dhd.errno_to_exception(dhd.errorGetLast())()

    def enable_gravity_compensation(self, enabled: bool = True):
        """
        Enable built-in gravity compensation for the end-effector that will be
        added on top of the force request.

        :param bool enabled: True to enable, False to disable
        """

        dhd.setGravityCompensation(enabled, ID=self._id)

        self._req = True

        self._f_req[0] = 0.
        self._f_req[1] = 0.
        self._f_req[2] = 0.

        self._t_req[0] = 0.
        self._t_req[1] = 0.
        self._t_req[2] = 0.

        dhd.stop(self._id)

    def get_button(self, button_id: int = 0) -> bool:
        """
        See if the button on the device is being pressed.

        See Also
        --------
        :class:`forcedimension.dhd.constants.NovintButtonID`


        :param int button_id: The button to check

        :returns: True if the button is being pressed, False otherwise
        """

        return bool(self._buttons & _cast(int, 1 << button_id))

    def close(self):
        self.open = False
        if self._haptic_daemon is not None:
            self._haptic_daemon.stop()
        drd.close(self._id)

    def __enter__(self):
        return self

    def __exit__(self, t, value, traceback):
        self.close()


class Gripper(Generic[T]):
    """
    A high-level wrapper for methods and kinematic data of a Gripper on a
    HapticDevice.

    Certain kinds of HapticDevices opened will have grippers. If that is the
    case, a Gripper object will be instantiated as well containing methods to
    get kinematic information about the Gripper.
    """

    def __init__(
            self,
            parent: HapticDevice,
            ID: Optional[int] = None,
            VecType: Type[T] = DefaultVecType
    ):

        if ID is not None:
            if (ID < 0):
                raise ValueError("ID must be greater than 0.")

            self._id: int = ID

        self._enc = ct.c_int()
        self._angle = ct.c_double()
        self._gap = ct.c_double()
        self._v = ct.c_double()
        self._w = ct.c_double()
        self._fg = ct.c_double()

        self._thumb_pos: T = VecType()
        self._finger_pos: T = VecType()

        self._parent: HapticDevice = parent

        self._thumb_pos_view: ImmutableWrapper[T] = (
            ImmutableWrapper(self._thumb_pos)
        )

        self._finger_pos_view: ImmutableWrapper[T] = (
            ImmutableWrapper(self._finger_pos)
        )

        self._fg_req: float = 0.0

    def request(self, fg: float):
        self._fg_req = fg

    def submit(self):
        self._req = False
        dhd.setForceAndTorqueAndGripperForce(
            self._parent._f_req,
            self._parent._t_req,
            self.fg
        )

    def req(self, fg: float):
        self._parent._req = True
        self._fg_req = fg

    def get_max_force(self) -> Optional[float]:
        """
        Retrieve the current limit (in N) to the force magnitude that can be
        applied by the haptic device.

        :returns:
            The current limit (in N) to the force magnitude that can be
            applied by the haptic device to the end-effector. If there is no
            limit, None is returned instead.
        """

        limit = dhd.getMaxGripperForce(self._id)

        return limit if limit > 0 else None

    def set_max_force(self, limit: Optional[float]):
        """
        Define or disable a limit (in N) to the force magnitude that can be
        applied by the haptic device.

        :param Optional[float] limit:
            The desired limit (in N) to the force magnitude that can be
            applied. If the limit is None, the force limit is disabled.
        """

        if limit is not None:
            if limit < 0:
                raise ValueError("limit must be greater than 0 or None.")

        if limit is None:
            limit = -1.0

        if dhd.setMaxGripperForce(limit, self._id):
            raise dhd.errno_to_exception(dhd.errorGetLast())()

    @property
    def thumb_pos(self):
        self.check_threadex()
        return self._thumb_pos_view

    @property
    def finger_pos(self):
        self.check_threadex()
        return self._finger_pos_view

    @property
    def gap(self) -> float:
        self.check_threadex()
        return self._gap.value

    @property
    def angle(self) -> float:
        self.check_threadex()
        return self._angle.value

    @property
    def v(self) -> float:
        self.check_threadex()
        return self._v.value

    @property
    def w(self) -> float:
        self.check_threadex()
        return self._w.value

    @property
    def fg(self) -> float:
        self.check_threadex()
        return self._fg.value

    def calculate_gap(self):
        err = dhd.expert.gripperEncoderToGap(
            self._enc.value, self._gap, self._id
        )

        if err:
            raise dhd.errno_to_exception(dhd.errorGetLast())()

    def calculate_angle(self):
        dhd.expert.gripperEncoderToAngleRad(
            self._enc.value, self._angle, self._id
        )

    def update_enc(self):
        err = dhd.expert.getGripperEncoder(self._enc, self._id)

        if err == -1:
            raise dhd.errno_to_exception(dhd.errorGetLast())()

    def update_enc_and_calculate(self):
        self.update_enc()
        self.calculate_angle()
        self.calculate_gap()

    def update_linear_velocity(self):
        err = dhd.getGripperLinearVelocity(self._v, self._id)

        if err == -1:
            raise dhd.errno_to_exception(dhd.errorGetLast())()

    def update_angular_velocity(self):
        err = dhd.getGripperAngularVelocityRad(self._w, self._id)

        if err == -1:
            raise dhd.errno_to_exception(dhd.errorGetLast())()

    def update_angle(self):
        err = dhd.getGripperAngleRad(self._angle, self._id)

        if err == -1:
            raise dhd.errno_to_exception(dhd.errorGetLast())()

    def update_gap(self):
        err = dhd.getGripperGap(self._gap, self._id)

        if err == -1:
            raise dhd.errno_to_exception(dhd.errorGetLast())()

    def update_thumb_pos(self):
        err = dhd.direct.getGripperThumbPos(self._thumb_pos, self._id)

        if err == -1:
            raise dhd.errno_to_exception(dhd.errorGetLast())()

    def update_finger_pos(self):
        err = dhd.direct.getGripperFingerPos(self._finger_pos, self._id)

        if err == -1:
            raise dhd.errno_to_exception(dhd.errorGetLast())()

    def check_threadex(self):
        self._parent.check_threadex()


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


class HapticDaemon(Thread):
    def __init__(
        self,
        dev: HapticDevice,
        update_list: UpdateOpts = UpdateOpts(),
        forceon=False
    ):

        super().__init__()

        if not isinstance(dev, HapticDevice):
            raise TypeError("Daemon needs an instance of HapticDevice")

        self._paused = False
        self._dev = dev
        self._dev._haptic_daemon = self
        self._forceon = forceon
        self._pollers = []

        self._set_pollers(update_list)

        if (self._dev.gripper is not None):
            self._req_func: Callable[[], None] = self._dev.gripper.submit
        else:
            self._req_func = self._dev.submit

        super().__init__(daemon=True)

    def _set_pollers(self, update_list):
        if update_list is not None:
            funcs = (
                self._dev.update_enc_and_calculate,
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
                _Poller(
                    self._dev.gripper.submit,
                    1/update_list.req,
                    self._forceon
                )
            )
        else:
            self._force_poller = (
                _Poller(self._dev.submit, 1/update_list.req, self._forceon)
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
            self._dev._thread_exception = ex
