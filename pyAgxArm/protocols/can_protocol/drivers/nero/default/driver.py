from typing import Optional, List
from typing_extensions import Literal

from .parser import Parser, NeroDefaultDriverAPIOptions, NeroDefaultDriverAPIProtoAdapter
from ...core.arm_driver_abstract import ArmDriverAbstract
from ....msgs.core import MessageAbstract
from ......utiles.numeric_codec import RAD2DEG
from ......utiles.vaildator import Validator
from ....msgs.nero.default import (
    ArmMsgModeCtrl,
    ArmMsgFeedbackJointStates,
    ArmMsgFeedbackEndPose,
    ArmMsgFeedbackStatus,
    ArmMsgFeedbackStatusEnum,
    ArmMsgFeedbackLowSpd,
    ArmMsgFeedbackHighSpd,
    ArmMsgMotorEnableDisableConfig,
    ArmMsgMotionCtrl,
    ArmMsgMasterSlaveModeConfig,
)


class Driver(ArmDriverAbstract):
    """Nero CAN driver.

    Terminology
    -----------
    `flange`:
    - The mounting face / connection interface on the robotic arm's last link
      (mechanical tool interface).

    Common conventions
    ------------------
    `timeout` (for request/response style APIs):
    - `timeout < 0.0` raises ValueError.
    - `timeout == 0.0`: non-blocking; evaluate readiness once and return
      immediately.
    - `timeout > 0.0`: blocking; poll until ready or timeout expires.

    `joint_index`:
    - `joint_index == 255` means "all joints".

    `set_*` return semantics:
    - Many `set_*` APIs are ACK-only: True means the controller acknowledged the
      request.
      This does not strictly guarantee the setting is already applied.
    - Some `set_*` APIs additionally verify by reading back state; their
      docstrings will mention the verification method if applicable.
    """

    @property
    def OPTIONS(self):
        return NeroDefaultDriverAPIOptions

    ARM_STATUS = ArmMsgFeedbackStatusEnum

    _JOINT_NUMS = 7

    _Parser = Parser

    # Can be overridden by subclasses to specify the message types used by the
    # driver.
    _MSG_ModeCtrl = ArmMsgModeCtrl
    _MSG_MotorEnableDisableConfig = ArmMsgMotorEnableDisableConfig

    def __init__(self, config):
        super().__init__(config)
        self._parser: Parser = self._parser
        self._msg_mode = self._MSG_ModeCtrl()

    def _set_mode(self) -> None:
        """Send cached mode message (`self._msg_mode`) to the controller."""
        self._send_msg(self._msg_mode)

    def _deal_move_p_msgs(self, pose: List[float]):
        """Get pose control messages."""
        pose = Validator.clamp_pose6(
            pose,
            name="flange_pose"
        )

        # Radians to degrees conversion
        rpy = [i * RAD2DEG for i in pose[3:]]

        # Convert user inputs to protocol fields.
        x = round(pose[0] * 1e6)
        y = round(pose[1] * 1e6)
        z = round(pose[2] * 1e6)

        # Convert orientation to protocol fields.
        roll = round(rpy[0] * 1e3)
        pitch = round(rpy[1] * 1e3)
        yaw = round(rpy[2] * 1e3)

        return self._parser._make_end_pose_ctrl_msgs(
            x_um=x,
            y_um=y,
            z_um=z,
            roll_mdeg=roll,
            pitch_mdeg=pitch,
            yaw_mdeg=yaw,
        )

    def _deal_move_j_msgs(self, joints: List[float]):
        """Get joint control messages."""
        joints = Validator.clamp_joints(
            joints,
            length=self._JOINT_NUMS,
            joints_limit=list(
                self._config.get(
                    "joint_limits", {}
                ).values()
            )
        )

        # Convert user inputs to protocol fields.
        joints_mdeg = [round(j * RAD2DEG * 1e3) for j in joints]
        return self._parser._make_joint_ctrl_msgs(joints_mdeg)

    # -------------------------- Get --------------------------

    def get_joint_angles(self):
        """Get current joint angles feedback.

        Returns
        -------
        MessageAbstract[list[float]] | None
            The joint angles feedback.
            If the joint angles are not available, return None.

        Message
        -------
        `list[float]`: joint angles, unit: rad

        Examples
        --------
        >>> ja = robot.get_joint_angles()
        >>> if ja is not None:
        >>>     print(ja.msg)
        >>>     print(ja.hz, ja.timestamp)
        """
        joint_angles: Optional[
            MessageAbstract[ArmMsgFeedbackJointStates]
        ] = None
        if getattr(self, "_joint_angles", None) is None:
            self._joint_angles = MessageAbstract(
                msg=list([0.0] * self._JOINT_NUMS),
                msg_type=ArmMsgFeedbackJointStates.type_,
            )
        if getattr(self._parser, "joint_12", None) is not None:
            joint_angles = self._parser.joint_12
            self._joint_angles.msg[0] = joint_angles.msg.joint_1
            self._joint_angles.msg[1] = joint_angles.msg.joint_2
        if getattr(self._parser, "joint_34", None) is not None:
            joint_angles = self._parser.joint_34
            self._joint_angles.msg[2] = joint_angles.msg.joint_3
            self._joint_angles.msg[3] = joint_angles.msg.joint_4
        if getattr(self._parser, "joint_56", None) is not None:
            joint_angles = self._parser.joint_56
            self._joint_angles.msg[4] = joint_angles.msg.joint_5
            self._joint_angles.msg[5] = joint_angles.msg.joint_6
        if getattr(self._parser, "joint_7", None) is not None:
            joint_angles = self._parser.joint_7
            self._joint_angles.msg[6] = joint_angles.msg.joint_7
        if joint_angles is not None:
            self._joint_angles.timestamp = joint_angles.timestamp
            self._joint_angles.hz = self._ctx.fps.get_fps(
                joint_angles.msg_type)
            if Validator.is_joints(
                self._joint_angles.msg,
                length=self._JOINT_NUMS
            ):
                return self._joint_angles
        return None

    def get_flange_pose(self):
        """Get current flange pose feedback.

        Returns
        -------
        MessageAbstract[list[float]] | None
            The end pose feedback. If the end pose is not available, return
            None.

        Message
        -------
        `list[float]`: `[x, y, z, roll, pitch, yaw]`

        `x, y, z`: Position, unit: m

        `roll, pitch, yaw`: Orientation, unit: rad

        Rotation Convention
        -------------------
        The Euler angles follow `XYZ extrinsic` (fixed frame) convention, which
        is equivalent to `ZYX intrinsic` (body frame) convention:

        1. `Extrinsic XYZ`: Rotate about global X (RX), then Y (RY), then Z
           (RZ)
        2. `Intrinsic ZYX`: Rotate about body Z (RZ), then Y (RY), then X (RX)

        Rotation matrix: `R = R_z(RZ) * R_y(RY) * R_x(RX)`

        Examples
        --------
        >>> fp = robot.get_flange_pose()
        >>> if fp is not None:
        >>>     x, y, z, roll, pitch, yaw = fp.msg
        >>>     print(x, y, z, roll, pitch, yaw)
        >>>     print(fp.hz, fp.timestamp)
        """
        end_pose = None
        if getattr(self, "_end_pose", None) is None:
            self._end_pose = MessageAbstract(
                msg=list([0.0] * 6), msg_type=ArmMsgFeedbackEndPose.type_
            )
        if getattr(self._parser, "end_pose_xy", None) is not None:
            end_pose = self._parser.end_pose_xy
            self._end_pose.msg[0] = end_pose.msg.X_axis
            self._end_pose.msg[1] = end_pose.msg.Y_axis
        if getattr(self._parser, "end_pose_zrx", None) is not None:
            end_pose = self._parser.end_pose_zrx
            self._end_pose.msg[2] = end_pose.msg.Z_axis
            self._end_pose.msg[3] = end_pose.msg.RX_axis
        if getattr(self._parser, "end_pose_ryrz", None) is not None:
            end_pose = self._parser.end_pose_ryrz
            self._end_pose.msg[4] = end_pose.msg.RY_axis
            self._end_pose.msg[5] = end_pose.msg.RZ_axis
        if end_pose is not None:
            self._end_pose.timestamp = end_pose.timestamp
            self._end_pose.hz = self._ctx.fps.get_fps(end_pose.msg_type)
            if Validator.is_pose6(
                self._end_pose.msg,
                name="flange_pose"
            ):
                return self._end_pose
        return None

    def get_arm_status(self):
        """Get the arm status feedback.

        Returns
        -------
        MessageAbstract[ArmMsgFeedbackStatus] | None
            The arm status feedback. If the arm status is not available, return
            None.

        Message
        -------
        `ctrl_mode`: Control mode

        `arm_status`: Robotic arm status

        `mode_feedback`: Mode feedback

        `teach_status`: Teaching status

        `motion_status`: Motion status

        `trajectory_num`: Current trajectory point number

        `err_status`: Error status

        Details
        ---------
        `ctrl_mode`: Control mode
        - 0: Standby mode
        - 1: CAN instruction control mode
        - 2: Teaching mode
        - 3: Ethernet control mode
        - 4: Wi-Fi control mode
        - 5: Remote control mode
        - 6: Linkage teaching input mode
        - 7: Offline trajectory mode
        - 8: TCP control mode

        `arm_status`: Robotic arm status
        - 0: Normal
        - 1: Emergency stop
        - 2: No solution
        - 3: Singularity point
        - 4: Target angle exceeds limit
        - 5: Joint communication exception
        - 6: Joint brake not released
        - 7: Collision occurred
        - 8: Overspeed during teaching drag
        - 9: Joint status abnormal
        - 10: Other exception
        - 11: Teaching record
        - 12: Teaching execution
        - 13: Teaching pause
        - 14: Main controller NTC over temperature
        - 15: Release resistor NTC over temperature

        `mode_feedback`: Mode feedback
        - 0: MOVE P
        - 1: MOVE J
        - 2: MOVE L
        - 3: MOVE C
        - 4: MOVE MIT
        - 5: MOVE CPV

        `teach_status`: Teaching status
        - 0: Off
        - 1: Start teaching record (enter drag teaching mode)
        - 2: End teaching record (exit drag teaching mode)
        - 3: Execute teaching trajectory (reproduce drag teaching trajectory)
        - 4: Pause execution
        - 5: Continue execution (continue trajectory reproduction)
        - 6: Terminate execution
        - 7: Move to trajectory starting point

        `motion_status`: Motion status
        - 0: Reached the target position
        - 1: Not yet reached the target position

        `trajectory_num`: Current trajectory point number
        - 0~255 (feedback in offline trajectory mode)

        `err_status`: Error status
        - `joint_1_angle_limit`:
            Joint 1 angle limit exceeded (False: normal, True: abnormal)
        - `joint_2_angle_limit`:
            Joint 2 angle limit exceeded (False: normal, True: abnormal)
        - `joint_3_angle_limit`:
            Joint 3 angle limit exceeded (False: normal, True: abnormal)
        - `joint_4_angle_limit`:
            Joint 4 angle limit exceeded (False: normal, True: abnormal)
        - `joint_5_angle_limit`:
            Joint 5 angle limit exceeded (False: normal, True: abnormal)
        - `joint_6_angle_limit`:
            Joint 6 angle limit exceeded (False: normal, True: abnormal)
        - `joint_7_angle_limit`:
            Joint 7 angle limit exceeded (False: normal, True: abnormal)
        - `communication_status_joint_1`:
            Joint 1 communication exception (False: normal, True: abnormal)
        - `communication_status_joint_2`:
            Joint 2 communication exception (False: normal, True: abnormal)
        - `communication_status_joint_3`:
            Joint 3 communication exception (False: normal, True: abnormal)
        - `communication_status_joint_4`:
            Joint 4 communication exception (False: normal, True: abnormal)
        - `communication_status_joint_5`:
            Joint 5 communication exception (False: normal, True: abnormal)
        - `communication_status_joint_6`:
            Joint 6 communication exception (False: normal, True: abnormal)
        - `communication_status_joint_7`:
            Joint 7 communication exception (False: normal, True: abnormal)

        Examples
        --------
        >>> arm_status = robot.get_arm_status()
        >>> if arm_status is not None:
        >>>     print(arm_status.msg)
        >>>     print(arm_status.hz, arm_status.timestamp)
        >>>     # unit: Hz, s
        """
        arm_status: Optional[MessageAbstract[ArmMsgFeedbackStatus]] = getattr(
            self._parser, "arm_status", None
        )
        if arm_status is not None:
            arm_status.hz = self._ctx.fps.get_fps(arm_status.msg_type)
            return arm_status
        else:
            return None

    def get_driver_states(self, joint_index: Literal[1, 2, 3, 4, 5, 6, 7]):
        """Get low-speed driver state feedback.

        Parameters
        ----------
        `joint_index`: Literal[1, 2, 3, 4, 5, 6, 7]
        - 1~7: get the driver state of the specified joint

        Returns
        -------
        MessageAbstract[ArmMsgFeedbackLowSpd] | None
            The specified joint's driver state, or None if not available.

        Message
        -------
        `vol`: Current driver voltage.

        `foc_temp`: Driver temperature, unit: °C

        `motor_temp`: Motor temperature, unit: °C

        `bus_current`: Current driver current, unit: A

        `foc_status`: Driver status.
        - `voltage_too_low`: Power voltage too low, type: bool
        - `motor_overheating`: Motor over-temperature, type: bool
        - `driver_overcurrent`: Driver over-current, type: bool
        - `driver_overheating`: Driver over-temperature, type: bool
        - `collision_status`: Collision protection status, type: bool
        - `driver_error_status`: Driver error status, type: bool
        - `driver_enable_status`: Driver enable status, type: bool
        - `stall_status`: Stalling protection status, type: bool

        Examples
        --------
        >>> ds = robot.get_driver_states(1)
        >>> if ds is not None:
        >>>     print(ds.msg.foc_status.driver_enable_status)
        >>>     print(ds.hz, ds.timestamp)
        """
        if joint_index not in self._JOINT_INDEX_LIST[:-1]:
            raise ValueError(
                f"Joint index should be {self._JOINT_INDEX_LIST[:-1]}")

        driver_state: Optional[
            MessageAbstract[ArmMsgFeedbackLowSpd]
        ] = getattr(self._parser, f"driver_state_{joint_index}", None)
        if driver_state is not None:
            driver_state.hz = self._ctx.fps.get_fps(driver_state.msg_type)
            return driver_state
        else:
            return None

    def get_motor_states(self, joint_index: Literal[1, 2, 3, 4, 5, 6, 7]):
        """Get high-speed motor state feedback.

        Parameters
        ----------
        `joint_index`: Literal[1, 2, 3, 4, 5, 6, 7]
        - 1~7: get the motor state of the specified joint

        Returns
        -------
        MessageAbstract[ArmMsgFeedbackHighSpd] | None
            The specified joint's motor state, or None if not available.

        Message
        -------
        `pos`: Current motor position, unit: rad

        `motor_speed`: Current motor speed, unit: rad/s

        `current`: Current motor current, unit: A

        `torque`: Current motor torque, unit: N·m

        Examples
        --------
        >>> ms = robot.get_motor_states(1)
        >>> if ms is not None:
        >>>     print(ms.msg.pos, ms.msg.motor_speed, ms.msg.torque)
        >>>     print(ms.hz, ms.timestamp)
        """
        if joint_index not in self._JOINT_INDEX_LIST[:-1]:
            raise ValueError(
                f"Joint index should be {self._JOINT_INDEX_LIST[:-1]}")

        motor_state: Optional[
            MessageAbstract[ArmMsgFeedbackHighSpd]
        ] = getattr(self._parser, f"motor_state_{joint_index}", None)
        if motor_state is not None:
            motor_state.hz = self._ctx.fps.get_fps(motor_state.msg_type)
            return motor_state
        else:
            return None

    def get_joint_enable_status(
        self, joint_index: Literal[1, 2, 3, 4, 5, 6, 7, 255]
    ):
        """Get the enable status of the specified joint motor.

        Parameters
        ----------
        `joint_index`: Literal[1, 2, 3, 4, 5, 6, 7, 255]
        - 1~7: get the enable status of the specified joint motor
        - 255: get the enable status of all joint motors (True only if all
          joints are enabled)

        Returns
        -------
        bool
            Enable status of the specified joint motor (False if not
            available).

        Examples
        --------
        Get the enable status of joint 1:
        >>> enable_status = robot.get_joint_enable_status(1)
        >>> if enable_status:
        >>>     print("Joint 1 motor is enabled")
        """
        if joint_index == 255:
            return all(self.get_joints_enable_status_list())

        if joint_index not in self._JOINT_INDEX_LIST[:-1]:
            raise ValueError(
                f"Joint index should be {self._JOINT_INDEX_LIST[:-1]}")

        msg: Optional[MessageAbstract[ArmMsgFeedbackLowSpd]] = (
            self.get_driver_states(joint_index=joint_index)
        )
        if msg is not None:
            return msg.msg.foc_status.driver_enable_status
        else:
            return False

    def get_joints_enable_status_list(self):
        """Get the enable status of all joint motors.

        Returns
        -------
        list[bool]
            Enable status of all joint motors.
        """
        return [self.get_joint_enable_status(i)
                for i in self._JOINT_INDEX_LIST[:-1]]

    # -------------------------- Enable/Disable --------------------------

    def enable(self, joint_index: Literal[1, 2, 3, 4, 5, 6, 7, 255] = 255):
        """Enable one joint motor or all joint motors.

        Parameters
        ----------
        `joint_index`: Literal[1, 2, 3, 4, 5, 6, 7, 255], optional
        - 1~7: enable the specified joint
        - 255: enable all joints (default)

        Returns
        -------
        bool
            True if the joint is enabled, False otherwise.

        Examples
        --------
        >>> if robot.enable():
        >>>     print("All joints enabled")
        """
        if joint_index not in self._JOINT_INDEX_LIST:
            raise ValueError(f"Joint index should be {self._JOINT_INDEX_LIST}")

        def send_enable_msg(joint_index):
            msg = self._MSG_MotorEnableDisableConfig(
                joint_index=joint_index, enable_flag=2)
            self._send_msg(msg)

        if joint_index == 255:
            send_enable_msg(self._JOINT_NUMS + 1)
            return all(self.get_joints_enable_status_list())
        else:
            send_enable_msg(joint_index)
            return self.get_joint_enable_status(joint_index=joint_index)

    def disable(self, joint_index: Literal[1, 2, 3, 4, 5, 6, 7, 255] = 255):
        """Disable one joint motor or all joint motors.

        Parameters
        ----------
        `joint_index`: Literal[1, 2, 3, 4, 5, 6, 7, 255], optional
        - 1~7: disable the specified joint
        - 255: disable all joints (default)

        Returns
        -------
        bool
            True if the joint is disabled, False otherwise.

        Examples
        --------
        >>> if robot.disable():
        >>>     print("All joints disabled")
        """
        if joint_index not in self._JOINT_INDEX_LIST:
            raise ValueError(f"Joint index should be {self._JOINT_INDEX_LIST}")

        def send_disable_msg(joint_index):
            msg = self._MSG_MotorEnableDisableConfig(
                joint_index=joint_index, enable_flag=1)
            self._send_msg(msg)

        if joint_index == 255:
            send_disable_msg(self._JOINT_NUMS + 1)
            return all(not self.get_joint_enable_status(i)
                       for i in self._JOINT_INDEX_LIST[:-1])
        else:
            send_disable_msg(joint_index)
            return not self.get_joint_enable_status(joint_index=joint_index)

    # -------------------- Reset and Emergency Stop --------------------

    def reset(self):
        """Reset motion controller state.

        This issues a motion control command to reset the robotic arm's motion state.

        Examples
        --------
        Reset the robotic arm:
        >>> robot.reset()
        """
        msg = ArmMsgMotionCtrl(2)
        self._send_msg(msg)

    def electronic_emergency_stop(self):
        """Trigger a damped emergency stop.

        Initiates a controlled emergency stop by applying damping to all joints
        and allowing rapid deceleration without mechanical shock.
        """
        msg = ArmMsgMotionCtrl(1)
        self._send_msg(msg)

    # -------------------------- Set --------------------------

    def set_speed_percent(self, percent: int = 100):
        """Set the percent of movement speed.

        This setting controls the relative speed of the robotic arm.

        Parameters
        ----------
        `percent`: int, optional
            The speed percent value ranges from [0, 100]. Default is 100.

        Raises
        ------
        ValueError
            If `percent` is outside [0, 100].
        """
        if not isinstance(percent, int):
            raise ValueError("Percent should be an integer")
        if percent < 0 or percent > 100:
            raise ValueError("Percent should be between 0 and 100")
        self._msg_mode.move_spd_rate_ctrl = percent
        self._set_mode()

    def set_motion_mode(
        self,
        motion_mode: Literal['p', 'j'] = 'p'
    ):
        """Set movement mode and MIT mode.

        Parameters
        ----------
        `motion_mode`: Literal['p', 'j']
        - `OPTIONS.MOTION_MODE.P`: move p
        - `OPTIONS.MOTION_MODE.J`: move j

        Raises
        ------
        ValueError
            If `motion_mode` is not in
            ['p', 'j'].

        Examples
        --------
        >>> robot.set_motion_mode(robot.OPTIONS.MOTION_MODE.P)
        """
        if motion_mode not in self.OPTIONS.MOTION_MODE.value_list():
            raise ValueError(
                "Invalid motion mode, should be in MOTION_MODE: "
                f"{self.OPTIONS.MOTION_MODE.value_list()}"
            )
        self._msg_mode.move_mode = NeroDefaultDriverAPIProtoAdapter.motion_mode(motion_mode)
        self._msg_mode.mit_mode = NeroDefaultDriverAPIProtoAdapter.mit_mode(motion_mode)
        self._msg_mode.enable_can_push = 0x01
        self._set_mode()
        self._msg_mode.enable_can_push = 0x00

    # -------------------------- Move --------------------------

    def move_p(self, pose: List[float]):
        """Move the robotic arm flange to specified pose in Cartesian space.

        Parameters
        ----------
        `pose`: list[float]
        - `list[float]` - > `[x, y, z, roll, pitch, yaw]`
        - `x, y, z`: Position coordinates in meters.
            (Numerical precision: 1e-6 m)
        - `roll, pitch, yaw`: Rotation angles around X, Y, Z axes respectively
            in radians. (Numerical precision: 1.74532925199e-5 rad)
          - `roll`, `yaw` must be within `[-pi, pi]`
          - `pitch` must be within `[-pi/2, pi/2]`

        Raises
        ------
        ValueError
            If pose is not a list or has incorrect length (not 6 elements).

            If `roll`, `yaw` is outside `[-pi, pi]` or `pitch` is outside
            `[-pi/2, pi/2]`.
        """
        # Prepare control messages
        msgs = self._deal_move_p_msgs(pose)

        # Set motion mode and send commands
        self.set_motion_mode('p')
        self._send_msgs(msgs)

    def move_j(self, joints: List[float]):
        """Move the robotic arm joints to the specified target angles (joint space).

        Parameters
        ----------
        `joints`: list[float]
        - `list[float]` - > `[j1, j2, j3, j4, j5, j6, j7]`
        - `j1..j7`: Joint angles in radians.
            (Numerical precision: 1.74532925199e-5 rad)

        Raises
        ------
        ValueError
            If `joints` is not a list or does not have length 7.

        Examples
        --------
        Move to joint angles:
        >>> robot.move_j([0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        """
        # Prepare control messages
        msgs = self._deal_move_j_msgs(joints)

        # Set motion mode and send commands
        self.set_motion_mode('j')
        self._send_msgs(msgs)

    def move_l(self, pose: List[float]):
        """Move the robotic arm flange to target pose in Cartesian space with
        linear motion.

        Parameters
        ----------
        `pose`: list[float]
        - `list[float]` - > `[x, y, z, roll, pitch, yaw]`
        - `x, y, z`: Position coordinates in meters.
            (Numerical precision: 1e-6 m)
        - `roll, pitch, yaw`: Rotation angles around X, Y, Z axes respectively
            in radians. (Numerical precision: 1.74532925199e-5 rad)
          - `roll`, `yaw` must be within `[-pi, pi]`
          - `pitch` must be within `[-pi/2, pi/2]`

        Raises
        ------
        ValueError
            If pose is not a list or has incorrect length (not 6 elements).

            If `roll`, `yaw` is outside `[-pi, pi]` or `pitch` is outside
            `[-pi/2, pi/2]`.
        """
        # Prepare control messages
        msgs = self._deal_move_p_msgs(pose)

        # Set motion mode and send commands
        self.set_motion_mode('l')
        self._send_msgs(msgs)

    # -------------------------- Master-Slave --------------------------

    def _set_master_slave_config(
        self,
        linkage_config: Literal[0x00, 0xFA, 0xFC] = 0x00,
        feedback_offset: Literal[0x00, 0x10, 0x20] = 0x00,
        ctrl_offset: Literal[0x00, 0x10, 0x20] = 0x00,
        linkage_offset: Literal[0x00, 0x10, 0x20] = 0x00,
    ):
        """Set the master-slave configuration."""
        if linkage_config not in [0x00, 0xFA, 0xFC]:
            raise ValueError("Linkage config should be 0x00, 0xFA, 0xFC")
        if feedback_offset not in [0x00, 0x10, 0x20]:
            raise ValueError("Feedback offset should be 0x00, 0x10, 0x20")
        if ctrl_offset not in [0x00, 0x10, 0x20]:
            raise ValueError("Ctrl offset should be 0x00, 0x10, 0x20")
        if linkage_offset not in [0x00, 0x10, 0x20]:
            raise ValueError("Linkage offset should be 0x00, 0x10, 0x20")
        msg = ArmMsgMasterSlaveModeConfig(
            linkage_config, feedback_offset, ctrl_offset, linkage_offset
        )
        self._send_msg(msg)

    def set_normal_mode(self):
        """Set the robotic arm to the normal controlled mode (single arm)."""
        self._set_master_slave_config(linkage_config=0x00)
        self._msg_mode.enable_can_push = 0x01
        self._set_mode()
        self._msg_mode.enable_can_push = 0x00

    def set_master_mode(self):
        """Set the arm to the master arm zero force drag mode (master arm)."""
        self._msg_mode.enable_can_push = 0x02
        self._set_mode()
        self._msg_mode.enable_can_push = 0x00
        self._set_master_slave_config(linkage_config=0xFA)

    def set_slave_mode(self):
        """Set the arm to the slave arm controlled mode (slave arm)."""
        self._msg_mode.enable_can_push = 0x02
        self._set_mode()
        self._msg_mode.enable_can_push = 0x00
        self._set_master_slave_config(linkage_config=0xFC)
