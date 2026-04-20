import copy
from typing import Optional, Callable, TypeVar, List
from typing_extensions import Literal

from .parser import Parser, PiperDefaultDriverAPIOptions, PiperDefaultDriverAPIProtoAdapter
from ...core.arm_driver_abstract import ArmDriverAbstract
from ....msgs.core import MessageAbstract
from ......utiles.numeric_codec import (
    NumericCodec as nc,
    RAD2DEG,
    DEG2RAD,
)
from ......utiles.validator import Validator
from ....msgs.piper.default import (
    ArmMsgModeCtrl,
    ArmMsgFeedbackJointStates,
    ArmMsgFeedbackEndPose,
    ArmMsgFeedbackStatus,
    ArmMsgFeedbackStatusEnum,
    ArmMsgFeedbackLowSpd,
    ArmMsgFeedbackHighSpd,
    ArmMsgFeedbackCurrentMotorAngleLimitMaxSpd,
    ArmMsgFeedbackCurrentMotorMaxAccLimit,
    ArmMsgFeedbackCurrentEndVelAccParam,
    ArmMsgFeedbackCPVResponse,
    ArmMsgMotorEnableDisableConfig,
    ArmMsgReqFirmware,
    ArmMsgSearchMotorMaxAngleSpdAccLimit,
    ArmMsgMotorAngleLimitMaxSpdSet,
    ArmMsgJointConfig,
    ArmMsgParamEnquiryAndConfig,
    ArmMsgEndVelAccParamConfig,
    ArmMsgCrashProtectionRatingConfig,
    ArmMsgJointAssistanceRatingConfig,
    ArmMsgMotionCtrl,
    ArmMsgLeaderArmMoveToHome,
    ArmMsgLeaderFollowerModeConfig,
    ArmMsgJointCtrl,
)


T = TypeVar("T")


class Driver(ArmDriverAbstract):
    """Piper CAN driver.

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
        return PiperDefaultDriverAPIOptions

    ARM_STATUS = ArmMsgFeedbackStatusEnum

    _Parser = Parser

    # Can be overridden by subclasses to specify the message types used by the
    # driver.
    _MSG_ModeCtrl = ArmMsgModeCtrl
    _MSG_MotorEnableDisableConfig = ArmMsgMotorEnableDisableConfig
    _MSG_SearchMotorMaxAngleSpdAccLimit = ArmMsgSearchMotorMaxAngleSpdAccLimit
    _MSG_ParamEnquiryAndConfig = ArmMsgParamEnquiryAndConfig
    _MSG_JointConfig = ArmMsgJointConfig
    _MSG_MotorAngleLimitMaxSpdSet = ArmMsgMotorAngleLimitMaxSpdSet
    _MSG_EndVelAccParamConfig = ArmMsgEndVelAccParamConfig
    _MSG_CrashProtectionRatingConfig = ArmMsgCrashProtectionRatingConfig
    _MSG_JointAssistanceRatingConfig = ArmMsgJointAssistanceRatingConfig

    def __init__(self, config: dict):
        super().__init__(config)
        self._parser: Parser = self._parser
        self._msg_mode = self._MSG_ModeCtrl()

    def _set_mode(self) -> None:
        """Send cached mode message (`self._msg_mode`) to the controller."""
        self._send_msg(self._msg_mode)

    def _maybe_set_motion_mode(
        self, motion_mode: Literal['p', 'j', 'l', 'c', 'mit', 'js', 'cpv']
    ) -> None:
        """Set motion mode only when auto mode-setting is enabled."""
        if self._auto_set_motion_mode_enabled:
            self.set_motion_mode(motion_mode)

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
        if self._joint_limits_enabled:
            joints = Validator.clamp_joints(
                joints,
                length=self._JOINT_NUMS,
                joints_limit=list(
                    self._config.get(
                        "joint_limits", {}
                    ).values()
                )
            )
        else:
            joints = Validator.clamp_joints(
                joints,
                length=self._JOINT_NUMS
            )

        # Convert user inputs to protocol fields.
        joints_mdeg = [round(j * RAD2DEG * 1e3) for j in joints]
        return self._parser._make_joint_ctrl_msgs(joints_mdeg)

    def _mit_position_limits(self, joint_index: int):
        if not self._joint_limits_enabled:
            return -12.5, 12.5
        limits = self._config.get(
            "joint_limits", {}
        ).get(f"joint{joint_index}", None)
        if limits is not None:
            return limits[0], limits[1]
        return -12.5, 12.5

    def _all_joints_bool(self, fn: Callable[[int], bool]) -> bool:
        """Apply a bool-returning function to all joints and AND results."""
        return all(fn(i) for i in self._JOINT_INDEX_LIST[:-1])

    # -------------------------
    # Common "set_*" templates (ACK-only / ACK+verify)
    # -------------------------
    def _clear_resp_set_instruction(self) -> None:
        """Clear cached `resp_set_instruction` message if present."""
        if getattr(self._parser, "resp_set_instruction", None) is not None:
            self._parser.resp_set_instruction.msg.clear()

    def _is_resp_set_instruction(self, instruction_index: int) -> bool:
        """Return True if cached `resp_set_instruction` matches the index."""
        return (
            getattr(self._parser, "resp_set_instruction", None) is not None
            and self._parser.resp_set_instruction.msg.instruction_index
            == instruction_index
        )

    def _ack_only_set(
        self,
        *,
        request: Callable[[], None],
        instruction_index: int,
        timeout: float = 1.0,
        stamp_key: str,
        clear_before: bool = True,
    ) -> bool:
        """Template for set_* APIs where success is defined as "ACK received".

        Returns False on timeout. True means the controller acknowledged the
        request.
        """
        if clear_before:
            self._clear_resp_set_instruction()

        def is_ready() -> bool:
            return self._is_resp_set_instruction(instruction_index)

        def get_value() -> bool:
            return True

        return bool(
            self._ctx._request_and_get(
                request=request,
                is_ready=is_ready,
                get_value=get_value,
                timeout=timeout,
                min_interval=0.0,
                stamp_attr=stamp_key,
            )
        )

    def _ack_and_check_set(
        self,
        *,
        request: Callable[[], None],
        instruction_index: int,
        check: Callable[[], bool],
        timeout: float = 1.0,
        stamp_key: str,
        clear_before: bool = True,
    ) -> bool:
        """Template for set_* APIs where success is "ACK received and state
        verified".

        Returns False on timeout, or if `check()` fails.
        """
        if clear_before:
            self._clear_resp_set_instruction()

        def is_ready() -> bool:
            return (
                self._is_resp_set_instruction(instruction_index)
                and check()
            )

        def get_value() -> bool:
            return True

        return bool(
            self._ctx._request_and_get(
                request=request,
                is_ready=is_ready,
                get_value=get_value,
                timeout=timeout,
                min_interval=0.0,
                stamp_attr=stamp_key,
            )
        )

    def _resp_set_instruction_get(
        self,
        *,
        request: Callable[[], None],
        instruction_index: int,
        get_value: Callable[[], T],
        timeout: float = 1.0,
        stamp_key: str,
        clear_before: bool = True,
    ) -> Optional[T]:
        """Wait for `resp_set_instruction` with a specific instruction index,
        then compute and return a typed value via `get_value()`."""
        if clear_before:
            self._clear_resp_set_instruction()

        def is_ready() -> bool:
            return self._is_resp_set_instruction(instruction_index)

        return self._ctx._request_and_get(
            request=request,
            is_ready=is_ready,
            get_value=get_value,
            timeout=timeout,
            min_interval=0.0,
            stamp_attr=stamp_key,
        )

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

        Examples
        --------
        >>> arm_status = robot.get_arm_status()
        >>> if arm_status is not None:
        >>>     print(arm_status.msg)
        >>>     print(
        ...         arm_status.msg.arm_status
        ...         == robot.ARM_STATUS.ArmStatus.NORMAL
        ...     )
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

    def get_driver_states(self, joint_index: Literal[1, 2, 3, 4, 5, 6]):
        """Get low-speed driver state feedback.

        Parameters
        ----------
        `joint_index`: Literal[1, 2, 3, 4, 5, 6]
        - 1~6: get the driver state of the specified joint

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

    def get_motor_states(self, joint_index: Literal[1, 2, 3, 4, 5, 6]):
        """Get high-speed motor state feedback.

        Parameters
        ----------
        `joint_index`: Literal[1, 2, 3, 4, 5, 6]
        - 1~6: get the motor state of the specified joint

        Returns
        -------
        MessageAbstract[ArmMsgFeedbackHighSpd] | None
            The specified joint's motor state, or None if not available.

        Message
        -------
        `position`: Current motor position, unit: rad

        `velocity`: Current motor speed, unit: rad/s

        `current`: Current motor current, unit: A

        `torque`: Current motor torque, unit: N·m

        Examples
        --------
        >>> ms = robot.get_motor_states(1)
        >>> if ms is not None:
        >>>     print(ms.msg.position, ms.msg.velocity, ms.msg.torque)
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
        self, joint_index: Literal[1, 2, 3, 4, 5, 6, 255]
    ):
        """Get the enable status of the specified joint motor.

        Parameters
        ----------
        `joint_index`: Literal[1, 2, 3, 4, 5, 6, 255]
        - 1~6: get the enable status of the specified joint motor
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

    def get_firmware(self, timeout: float = 1.0, min_interval: float = 1.0):
        """Get the firmware information of the arm.

        Parameters
        ----------
        `timeout`: float, optional
        - Timeout in seconds (see `Driver` docstring: Common conventions ->
          `timeout`).
        - Default is 1.0.

        `min_interval`: float, optional
        - Minimum interval in seconds between two consecutive requests.
        - Default is 1.0.

        Returns
        -------
        dict | None
            The firmware information of the arm.
            If the firmware information is not available, return None.

        Dict keys
        ---------
        `hardware_version`: Hardware version with batch (e.g. H-V1.2-1)

        `motor_ratio_and_batch`: Motor ratio and batch (e.g. 10)

        `node_type`: Node type (e.g. ARM_MC)

        `software_version`: Software version (e.g. S-V1.8-2)

        `production_date`: Production date (e.g. 250925)

        `node_number`: Node number (e.g. 15)

        Examples
        --------
        >>> firmware = robot.get_firmware()
        >>> if firmware is not None:
        >>>     print(
        ...         firmware["hardware_version"],
        ...         firmware["software_version"],
        ...     )
        >>> # Non-blocking: call with `timeout=0.0` (see `Driver` conventions).
        """
        def request() -> None:
            self._send_msg(ArmMsgReqFirmware())

        def is_ready() -> bool:
            if (getattr(self._parser, "firmware_info", None) is not None
                and len(self._parser.firmware_info.msg.data_seg) >= 3
                and (self._parser.firmware_info.msg.data_seg[0:3] != bytearray(b'H-V')
                     or self.get_fps() <= 0)):
                self._parser.firmware_info.msg.clear()
                return False
            
            return (
                getattr(self._parser, "firmware_info", None) is not None
                and len(self._parser.firmware_info.msg.data_seg) == 8 * 11
            )

        def get_value() -> dict:
            data = self._parser.firmware_info.msg.data_seg
            return {
                "hardware_version": data[0:8].decode("utf-8"),
                "motor_ratio_and_batch": data[16:18].decode("utf-8"),
                "node_type": data[32:38].decode("utf-8"),
                "software_version": data[60:68].decode("utf-8"),
                "production_date": data[68:74].decode("utf-8"),
                "node_number": data[76:78].decode("utf-8"),
            }

        def clear() -> None:
            self._parser.firmware_info.msg.clear()

        return self._ctx._request_and_get(
            request=request,
            is_ready=is_ready,
            get_value=get_value,
            clear=clear,
            timeout=timeout,
            min_interval=min_interval,
            stamp_attr="firmware_info",
        )

    # -------------------------- Enable/Disable --------------------------

    def enable(self, joint_index: Literal[1, 2, 3, 4, 5, 6, 255] = 255):
        """Enable one joint motor or all joint motors.

        Parameters
        ----------
        `joint_index`: Literal[1, 2, 3, 4, 5, 6, 255], optional
        - 1~6: enable the specified joint
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

    def disable(self, joint_index: Literal[1, 2, 3, 4, 5, 6, 255] = 255):
        """Disable one joint motor or all joint motors.

        Parameters
        ----------
        `joint_index`: Literal[1, 2, 3, 4, 5, 6, 255], optional
        - 1~6: disable the specified joint
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

    # -------------------- Emergency Stop and Reset --------------------

    def electronic_emergency_stop(self):
        """Trigger a damped emergency stop.

        Initiates a controlled emergency stop by applying damping to all joints
        and allowing rapid deceleration without mechanical shock.
        """
        msg = ArmMsgMotionCtrl(1)
        self._send_msg(msg)

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
        temp = self._msg_mode.move_mode
        self._msg_mode.move_mode = 255
        self._set_mode()
        self._msg_mode.move_mode = temp

    def set_installation_pos(
        self, pos: Literal['horizontal', 'left', 'right'] = 'horizontal'
    ):
        """Set base installation orientation.

        Parameters
        ----------
        `pos`: Literal['horizontal', 'left', 'right']
        - `OPTIONS.INSTALLATION_POS.HORIZONTAL`: horizontal installation (default)
        - `OPTIONS.INSTALLATION_POS.LEFT`: left-side installation
        - `OPTIONS.INSTALLATION_POS.RIGHT`: right-side installation

        Raises
        ------
        ValueError
            If `pos` is not in ['horizontal', 'left', 'right'].

        Examples
        --------
        >>> robot.set_installation_pos(robot.OPTIONS.INSTALLATION_POS.HORIZONTAL)
        >>> robot.set_installation_pos(robot.OPTIONS.INSTALLATION_POS.LEFT)
        >>> robot.set_installation_pos(robot.OPTIONS.INSTALLATION_POS.RIGHT)
        """
        if pos not in self.OPTIONS.INSTALLATION_POS.value_list():
            raise ValueError(
                "Installation position should be in OPTIONS.INSTALLATION_POS: "
                f"{self.OPTIONS.INSTALLATION_POS.value_list()}"
            )
        installation_pos = PiperDefaultDriverAPIProtoAdapter.installation_pos(pos)
        self._msg_mode.installation_pos = installation_pos
        temp = self._msg_mode.move_mode
        self._msg_mode.move_mode = 255
        self._set_mode()
        self._msg_mode.installation_pos = 0
        self._msg_mode.move_mode = temp

    def set_motion_mode(
        self,
        motion_mode: Literal['p', 'j', 'l', 'c', 'mit', 'js', 'cpv'] = 'p'
    ):
        """Set movement mode and MIT mode.

        Parameters
        ----------
        `motion_mode`: Literal['p', 'j', 'l', 'c', 'mit', 'js', 'cpv']
        - `OPTIONS.MOTION_MODE.P`: move p
        - `OPTIONS.MOTION_MODE.J`: move j
        - `OPTIONS.MOTION_MODE.L`: move l
        - `OPTIONS.MOTION_MODE.C`: move c
        - `OPTIONS.MOTION_MODE.MIT`: move mit (MIT)
        - `OPTIONS.MOTION_MODE.JS`: move js (MIT)
        - `OPTIONS.MOTION_MODE.CPV`: move cpv (CPV)

        Raises
        ------
        ValueError
            If `motion_mode` is not in
            ['p', 'j', 'l', 'c', 'mit', 'js', 'cpv'].

        Examples
        --------
        >>> robot.set_motion_mode(robot.OPTIONS.MOTION_MODE.P)
        """
        if motion_mode not in self.OPTIONS.MOTION_MODE.value_list():
            raise ValueError(
                "Invalid motion mode, should be in OPTIONS.MOTION_MODE: "
                f"{self.OPTIONS.MOTION_MODE.value_list()}"
            )
        self._msg_mode.move_mode = PiperDefaultDriverAPIProtoAdapter.motion_mode(motion_mode)
        self._msg_mode.mit_mode = PiperDefaultDriverAPIProtoAdapter.mit_mode(motion_mode)
        self._set_mode()

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

        Examples
        --------
        >>> robot.move_p(
        ...     [0.1, 0.0, 0.3, 0.0, 1.5707963267948966, 0.0]
        ... )
        """
        # Prepare control messages
        msgs = self._deal_move_p_msgs(pose)

        # Set motion mode and send commands
        self._maybe_set_motion_mode('p')
        self._send_msgs(msgs)

    def move_j(self, joints: List[float]):
        """Move the robotic arm joints to the specified target angles in joint space.

        Parameters
        ----------
        `joints`: list[float]
        - `list[float]` - > `[j1, j2, j3, j4, j5, j6]`
        - `j1..j6`: Joint angles in radians.
            (Numerical precision: 1.74532925199e-5 rad)

        Raises
        ------
        ValueError
            If `joints` is not a list or does not have length 6.

        Examples
        --------
        Move to joint angles:
        >>> robot.move_j([0.5, 0.0, 0.0, 0.0, 0.0, 0.0])
        """
        # Prepare control messages
        msgs = self._deal_move_j_msgs(joints)

        # Set motion mode and send commands
        self._maybe_set_motion_mode('j')
        self._send_msgs(msgs)

    def move_js(self, joints: List[float]):
        """Move the robotic arm joints to the specified target angles in joint
        space with "JS" mode enabled.

        This is similar to `move_j`, but sets a specific `mit_mode`
        before sending the joint command messages.

        WARNING
        -------
        This API is intended for "instantaneous" response:
        - No smoothing.
        - No trajectory planning.
        - The controller/driver will try to respond as fast as possible
          (not infinitely fast) to reach the target.

        This may cause severe mechanical shock, oscillation, or instability.

        Risk level: EXTREMELY HIGH.

        Parameters
        ----------
        `joints`: list[float]
        - `list[float]` - > `[j1, j2, j3, j4, j5, j6]`
        - `j1..j6`: Joint angles in radians.
            (Numerical precision: 1.74532925199e-5 rad)

        Raises
        ------
        ValueError
            If `joints` is not a list or does not have length 6.

        Examples
        --------
        Fast-response joint move:
        >>> robot.move_js([0.5, 0.0, 0.0, 0.0, 0.0, 0.0])
        """
        # Prepare control messages
        msgs = self._deal_move_j_msgs(joints)

        # Set motion mode and send commands
        self._maybe_set_motion_mode('js')
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

        Examples
        --------
        >>> robot.move_l([0.2, 0.0, 0.3, 0.0, 1.5708, 0.0])
        """
        # Prepare control messages
        msgs = self._deal_move_p_msgs(pose)

        # Set motion mode and send commands
        self._maybe_set_motion_mode('l')
        self._send_msgs(msgs)

    def move_c(
        self,
        start_pose: List[float],
        mid_pose: List[float],
        end_pose: List[float],
    ):
        """Move the robotic arm flange to specified pose in Cartesian space with
        circular motion.

        Parameters
        ----------
        `start_pose` | `mid_pose` | `end_pose`: list[float]
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
            If start_pose, mid_pose, or end_pose
            is not a list or has incorrect length (not 6 elements).

            If `roll`, `yaw` is outside `[-pi, pi]` or `pitch` is outside
            `[-pi/2, pi/2]`.

        Examples
        --------
        >>> sp = [0.2, 0.0, 0.3, 0.0, 1.5708, 0.0]
        >>> mp = [0.2, 0.05, 0.35, 0.0, 1.5708, 0.0]
        >>> ep = [0.2, 0.0, 0.4, 0.0, 1.5708, 0.0]
        >>> robot.move_c(sp, mp, ep)
        """
        # Prepare control messages
        msgs = self._deal_move_p_msgs(start_pose)
        msgs.append(self._parser._make_circular_coord_num_update_msg(0x01))
        msgs += self._deal_move_p_msgs(mid_pose)
        msgs.append(self._parser._make_circular_coord_num_update_msg(0x02))
        msgs += self._deal_move_p_msgs(end_pose)
        msgs.append(self._parser._make_circular_coord_num_update_msg(0x03))

        # Set motion mode and send commands
        self._maybe_set_motion_mode('c')
        self._send_msgs(msgs)

    def move_mit(
        self,
        joint_index: Literal[1, 2, 3, 4, 5, 6],
        p_des: float = 0.0,
        v_des: float = 0.0,
        kp: float = 10.0,
        kd: float = 0.8,
        t_ff: float = 0.0,
    ):
        """Control a single joint in MIT (impedance/torque) style mode.

        This API sends an MIT control message for a specific joint with desired
        position/velocity, PD gains, and feed-forward torque.

        The controller conceptually computes a reference torque:

            T_ref = kp * (p_des - p) + kd * (v_des - v) + t_ff

        where `p/v` are the measured joint `position/velocity`.

        Parameters
        ----------
        `joint_index`: Literal[1, 2, 3, 4, 5, 6]

        `p_des`: float, optional
        - Desired position reference (unit: rad). Range: [-12.5, 12.5].
          Default is
            0.0. (Numerical precision: 3.8147554741741054e-4 rad)

        `v_des`: float, optional
        - Desired velocity reference (unit: rad/s). Range: [-45.0, 45.0].
          Default is
            0.0. (Numerical precision: 2.197802197802198e-2 rad/s)

        `kp`: float, optional
        - Proportional gain. Range: [0.0, 500.0]. Default is 10.0.
            (Numerical precision: 1.221001221001221e-1)

        `kd`: float, optional
        - Derivative gain. Range: [-5.0, 5.0]. Default is 0.8.
            (Numerical precision: 2.442002442002442e-3)

        `t_ff`: float, optional
        - Feed-forward torque reference (unit: N·m). Default is 0.0.
          Joint 1-3: Range [-32.0, 32.0].
            (Numerical precision: 2.509803921568627e-1 N·m)
          Joint 4-6: Range [-8.0, 8.0].
            (Numerical precision: 6.274509803921569e-2 N·m)

        Raises
        ------
        ValueError
            If any parameter is outside the allowed range, or if `joint_index`
            is not in {1, 2, 3, 4, 5, 6}.

        Notes
        -----
        - This uses MIT move mode.
        - Typical usage patterns:
          - Velocity control: set `kp = 0`, `kd != 0`, command `v_des`.
          - Torque control: set `kp = 0`, `kd = 0`, command `t_ff`.
          - Position control: avoid `kd = 0` when `kp != 0` to reduce
            oscillation risk.

        Examples
        --------
        Hold joint 1 at a target position:
        >>> robot.move_mit(
        ...     joint_index=1, p_des=0.5, v_des=0.0, kp=10.0, kd=0.8, t_ff=0.0
        ... )

        Damped motion on joint 1 (increase kd for more damping):
        >>> robot.move_mit(
        ...     joint_index=1, p_des=0.0, v_des=0.0, kp=10.0, kd=2.0
        ... )

        Apply feed-forward torque on joint 1 (with low gains):
        >>> robot.move_mit(
        ...     joint_index=1, p_des=0.0, v_des=0.0, kp=2.0, kd=0.5, t_ff=1.5
        ... )
        """
        if joint_index not in self._JOINT_INDEX_LIST[:-1]:
            raise ValueError(
                f"Joint index should be {self._JOINT_INDEX_LIST[:-1]}")

        lower_limit, upper_limit = self._mit_position_limits(joint_index)
        
        if not Validator.is_within_limit(p_des, lower_limit, upper_limit):
            print(
                f"Warning: Desired position {p_des} rad is outside "
                f"joint {joint_index} limits [{lower_limit}, {upper_limit}] rad. "
            )
            p_des = Validator.clamp(p_des, lower_limit, upper_limit)

        if not Validator.is_within_limit(v_des, -45.0, 45.0):
            print(
                f"Warning: Desired velocity {v_des} rad/s is outside "
                f"joint {joint_index} limits [-45.0, 45.0] rad/s. "
            )
            v_des = Validator.clamp(v_des, -45.0, 45.0)

        if not Validator.is_within_limit(kp, 0.0, 500.0):
            print(
                f"Warning: Proportional gain {kp} is outside "
                f"joint {joint_index} limits [0.0, 500.0]. "
            )
            kp = Validator.clamp(kp, 0.0, 500.0)

        if not Validator.is_within_limit(kd, -5.0, 5.0):
            print(
                f"Warning: Derivative gain {kd} is outside "
                f"joint {joint_index} limits [-5.0, 5.0]. "
            )
            kd = Validator.clamp(kd, -5.0, 5.0)

        if joint_index in (1, 2, 3):
            t_ff_min = -32.0
            t_ff_max = 32.0
        else:
            t_ff_min = -8.0
            t_ff_max = 8.0

        if not Validator.is_within_limit(t_ff, t_ff_min, t_ff_max):
            print(
                f"Warning: Feed-forward torque {t_ff} N·m is outside "
                f"joint {joint_index} limits [{t_ff_min}, {t_ff_max}]. "
            )
            t_ff = Validator.clamp(t_ff, t_ff_min, t_ff_max)
        
        if joint_index in (1, 2, 3):
            t_ff *= 0.25

        p_des = nc.FloatToUint(p_des, -12.5, 12.5, 16)
        v_des = nc.FloatToUint(v_des, -45.0, 45.0, 12)
        kp = nc.FloatToUint(kp, 0.0, 500.0, 12)
        kd = nc.FloatToUint(kd, -5.0, 5.0, 12)
        t_ff = nc.FloatToUint(t_ff, -8.0, 8.0, 8)

        msg = self._parser._make_joint_mit_ctrl_msg(
            joint_index=joint_index,
            p_des=p_des,
            v_des=v_des,
            kp=kp,
            kd=kd,
            t_ff=t_ff,
        )

        # Set motion mode and send commands
        self._maybe_set_motion_mode('mit')
        self._send_msg(msg)

    # -------------------------- Leader-Follower --------------------------

    def _set_leader_follower_config(
        self,
        linkage_config: Literal[0x00, 0xFA, 0xFC] = 0x00,
        feedback_offset: Literal[0x00, 0x10, 0x20] = 0x00,
        ctrl_offset: Literal[0x00, 0x10, 0x20] = 0x00,
        linkage_offset: Literal[0x00, 0x10, 0x20] = 0x00,
    ):
        """Set the leader-follower configuration."""
        if linkage_config not in [0x00, 0xFA, 0xFC]:
            raise ValueError("Linkage config should be 0x00, 0xFA, 0xFC")
        if feedback_offset not in [0x00, 0x10, 0x20]:
            raise ValueError("Feedback offset should be 0x00, 0x10, 0x20")
        if ctrl_offset not in [0x00, 0x10, 0x20]:
            raise ValueError("Ctrl offset should be 0x00, 0x10, 0x20")
        if linkage_offset not in [0x00, 0x10, 0x20]:
            raise ValueError("Linkage offset should be 0x00, 0x10, 0x20")
        msg = ArmMsgLeaderFollowerModeConfig(
            linkage_config, feedback_offset, ctrl_offset, linkage_offset
        )
        self._send_msg(msg)

    def set_leader_mode(self):
        """Set the arm to the leader arm zero force drag mode (leader arm)."""
        self._set_leader_follower_config(linkage_config=0xFA)

    def set_follower_mode(self):
        """Set the arm to the follower arm controlled mode (follower arm)."""
        self._set_leader_follower_config(linkage_config=0xFC)

    def move_leader_to_home(self):
        """Move the leader arm to the home position (leader arm).

        Notes
        -----
        - This function will send a message to the controller to move the leader
          arm to the home position.
        - After calling this function, you must call `restore_leader_drag_mode`
          to restore the leader arm to the zero force drag mode.
        """
        msg = ArmMsgLeaderArmMoveToHome(mode=1)
        self._send_msg(msg)

    def move_leader_follower_to_home(self):
        """Move both leader and follower arms to the home position together.

        Notes
        -----
        - This function will send a message to the controller to move both the
          leader and follower arms to the home position together.
        - After calling this function, you must call `restore_leader_drag_mode`
          to restore the leader-follower arm mode (zero force drag for leader).
        """
        msg = ArmMsgLeaderArmMoveToHome(mode=2)
        self._send_msg(msg)

    def restore_leader_drag_mode(self):
        """Restore the leader arm to the zero force drag mode (leader arm)."""
        msg = ArmMsgLeaderArmMoveToHome(mode=0)
        self._send_msg(msg)

    def get_leader_joint_angles(self):
        """Get the leader arm joint angles,
        can be used to control the follower arm.

        Returns
        -------
        MessageAbstract[list[float]] | None
            The joint angles feedback of the leader arm.
            If the joint angles are not available, return None.

        Message
        -------
        `list[float]`: joint angles, unit: rad

        Examples
        --------
        >>> mja = robot.get_leader_joint_angles()
        >>> if mja is not None:
        >>>     print(mja.msg)
        >>>     print(mja.hz, mja.timestamp)
        """
        leader_joint_angles: Optional[MessageAbstract[ArmMsgJointCtrl]] = None
        if getattr(self, "_leader_joint_angles", None) is None:
            self._leader_joint_angles = MessageAbstract(
                msg=list([0.0] * self._JOINT_NUMS),
                msg_type=ArmMsgJointCtrl.type_,
            )
        if getattr(self._parser, "leader_joint_12", None) is not None:
            leader_joint_angles = self._parser.leader_joint_12
            self._leader_joint_angles.msg[0] = leader_joint_angles.msg.joint_1
            self._leader_joint_angles.msg[1] = leader_joint_angles.msg.joint_2
        if getattr(self._parser, "leader_joint_34", None) is not None:
            leader_joint_angles = self._parser.leader_joint_34
            self._leader_joint_angles.msg[2] = leader_joint_angles.msg.joint_3
            self._leader_joint_angles.msg[3] = leader_joint_angles.msg.joint_4
        if getattr(self._parser, "leader_joint_56", None) is not None:
            leader_joint_angles = self._parser.leader_joint_56
            self._leader_joint_angles.msg[4] = leader_joint_angles.msg.joint_5
            self._leader_joint_angles.msg[5] = leader_joint_angles.msg.joint_6
        if leader_joint_angles is not None:
            self._leader_joint_angles.timestamp = leader_joint_angles.timestamp
            self._leader_joint_angles.hz = self._ctx.fps.get_fps(
                leader_joint_angles.msg_type
            )
            if Validator.is_joints(
                self._leader_joint_angles.msg,
                length=self._JOINT_NUMS
            ):
                return self._leader_joint_angles
        return None

    # -------------------------- Other --------------------------

    def get_joint_angle_vel_limits(
        self,
        joint_index: Literal[1, 2, 3, 4, 5, 6],
        timeout: float = 1.0,
        min_interval: float = 1.0,
    ):
        """Get the joint angle and velocity limits.

        Parameters
        ----------
        `joint_index`: Literal[1, 2, 3, 4, 5, 6]
        - 1~6: get the message of the specified joint.

        `timeout`: float, optional
        - Timeout in seconds (see `Driver` docstring: Common conventions -> `timeout`).
        - Default is 1.0.

        `min_interval`: float, optional
        - Minimum interval in seconds between two consecutive requests.
        - Default is 1.0.

        Returns
        -------
        MessageAbstract[ArmMsgFeedbackCurrentMotorAngleLimitMaxSpd] | None
            The specified joint's limits, or None if not available.

        Message
        -------
        `min_angle_limit`: Joint minimum angle limit, unit: rad

        `max_angle_limit`: Joint maximum angle limit, unit: rad

        `min_joint_spd`: Joint minimum velocity, unit: rad/s

        `max_joint_spd`: Joint maximum velocity, unit: rad/s

        Examples
        --------
        >>> limit = robot.get_joint_angle_vel_limits(1)
        >>> if limit is not None:
        >>>     print(limit.msg.min_angle_limit, limit.msg.max_angle_limit)
        >>>     print(limit.msg.min_joint_spd, limit.msg.max_joint_spd)
        >>> # Non-blocking: `robot.get_joint_angle_vel_limits(1, timeout=0.0)`
        """
        # Input validation
        self._ctx._validate_timeout(timeout)
        if joint_index not in self._JOINT_INDEX_LIST[:-1]:
            raise ValueError(
                f"Joint index should be {self._JOINT_INDEX_LIST[:-1]}")

        def request() -> None:
            self._send_msg(
                self._MSG_SearchMotorMaxAngleSpdAccLimit(
                    joint_index=joint_index, search_content=1
                )
            )

        def is_ready() -> bool:
            return (
                getattr(self._parser, "motor_angle_limit_max_spd",
                        None) is not None
                and self._parser.motor_angle_limit_max_spd.msg.joints[
                    joint_index - 1
                ].min_angle_limit
                is not None
            )

        def get_value(
        ) -> MessageAbstract[ArmMsgFeedbackCurrentMotorAngleLimitMaxSpd]:
            self._parser.motor_angle_limit_max_spd.hz = self._ctx.fps.get_fps(
                self._parser.motor_angle_limit_max_spd.msg_type
            )
            temp = copy.deepcopy(
                self._parser.motor_angle_limit_max_spd.msg.joints[joint_index - 1]
            )
            temp = MessageAbstract(msg=temp, msg_type=temp.type_)
            temp.hz = self._parser.motor_angle_limit_max_spd.hz
            temp.timestamp = self._parser.motor_angle_limit_max_spd.timestamp
            return temp

        def clear() -> None:
            self._parser.motor_angle_limit_max_spd.msg.joints[joint_index - 1].clear(
            )

        return self._ctx._request_and_get(
            request=request,
            is_ready=is_ready,
            get_value=get_value,
            clear=clear,
            timeout=timeout,
            min_interval=min_interval,
            stamp_attr=f"joint_angle_vel:{joint_index}",
        )

    def get_joint_acc_limits(
        self,
        joint_index: Literal[1, 2, 3, 4, 5, 6],
        timeout: float = 1.0,
        min_interval: float = 1.0,
    ):
        """Get the joint acceleration limits.

        Parameters
        ----------
        `joint_index`: Literal[1, 2, 3, 4, 5, 6]
        - 1~6: get the message of the specified joint.

        `timeout`: float, optional
        - Timeout in seconds (see `Driver` docstring: Common conventions -> `timeout`).
        - Default is 1.0.

        `min_interval`: float, optional
        - Minimum interval in seconds between two consecutive requests.
        - Default is 1.0.

        Returns
        -------
        MessageAbstract[ArmMsgFeedbackCurrentMotorMaxAccLimit] | None
            The specified joint's limits, or None if not available.

        Message
        -------
        `max_joint_acc`: Joint maximum acceleration, unit: rad/s^2

        Examples
        --------
        >>> limit = robot.get_joint_acc_limits(1)
        >>> if limit is not None:
        >>>     print(limit.msg.max_joint_acc)
        >>> # Non-blocking: `robot.get_joint_acc_limits(1, timeout=0.0)`
        """
        # Input validation
        self._ctx._validate_timeout(timeout)
        if joint_index not in self._JOINT_INDEX_LIST[:-1]:
            raise ValueError(
                f"Joint index should be {self._JOINT_INDEX_LIST[:-1]}")

        def request() -> None:
            self._send_msg(
                self._MSG_SearchMotorMaxAngleSpdAccLimit(
                    joint_index=joint_index, search_content=2
                )
            )

        def is_ready() -> bool:
            return (
                getattr(self._parser, "motor_max_acc_limit", None) is not None
                and self._parser.motor_max_acc_limit.msg.joints[
                    joint_index - 1
                ].max_joint_acc
                is not None
            )

        def get_value(
        ) -> MessageAbstract[ArmMsgFeedbackCurrentMotorMaxAccLimit]:
            self._parser.motor_max_acc_limit.hz = self._ctx.fps.get_fps(
                self._parser.motor_max_acc_limit.msg_type
            )
            temp = copy.deepcopy(
                self._parser.motor_max_acc_limit.msg.joints[joint_index - 1]
            )
            temp = MessageAbstract(msg=temp, msg_type=temp.type_)
            temp.hz = self._parser.motor_max_acc_limit.hz
            temp.timestamp = self._parser.motor_max_acc_limit.timestamp
            return temp

        def clear() -> None:
            self._parser.motor_max_acc_limit.msg.joints[joint_index - 1].clear()

        return self._ctx._request_and_get(
            request=request,
            is_ready=is_ready,
            get_value=get_value,
            clear=clear,
            timeout=timeout,
            min_interval=min_interval,
            stamp_attr=f"joint_acc:{joint_index}",
        )

    def get_flange_vel_acc_limits(
        self,
        timeout: float = 1.0,
        min_interval: float = 1.0
    ):
        """Get the flange velocity and acceleration limits.

        Parameters
        ----------
        `timeout`: float, optional
        - Timeout in seconds (see `Driver` docstring: Common conventions -> `timeout`).
        - Default is 1.0.

        `min_interval`: float, optional
        - Minimum interval in seconds between two consecutive requests.
        - Default is 1.0.

        Returns
        -------
        MessageAbstract[ArmMsgFeedbackCurrentEndVelAccParam] | None
            The end effector velocity and acceleration limits.
            If the end effector velocity and acceleration limits is not available,
            return None.

        Message
        -------
        `end_max_linear_vel`: End effector maximum linear velocity, unit: m/s

        `end_max_angular_vel`: End effector maximum angular velocity, unit: rad/s

        `end_max_linear_acc`: End effector maximum linear acceleration, unit: m/s^2

        `end_max_angular_acc`: End effector maximum angular acceleration, unit: rad/s^2

        Examples
        --------
        >>> limit = robot.get_flange_vel_acc_limits()
        >>> if limit is not None:
        >>>     print(limit.msg.end_max_linear_vel, limit.msg.end_max_angular_vel)
        >>>     print(limit.msg.end_max_linear_acc, limit.msg.end_max_angular_acc)
        >>> # Non-blocking: `robot.get_flange_vel_acc_limits(timeout=0.0)`
        """
        def request() -> None:
            self._send_msg(self._MSG_ParamEnquiryAndConfig(param_enquiry=1))

        def is_ready() -> bool:
            return (
                getattr(self._parser, "end_vel_acc_param", None) is not None
                and self._parser.end_vel_acc_param.msg.end_max_linear_vel is not None
            )

        def get_value(
        ) -> MessageAbstract[ArmMsgFeedbackCurrentEndVelAccParam]:
            self._parser.end_vel_acc_param.hz = self._ctx.fps.get_fps(
                self._parser.end_vel_acc_param.msg_type
            )
            return copy.deepcopy(self._parser.end_vel_acc_param)

        def clear() -> None:
            self._parser.end_vel_acc_param.msg.clear()

        res = self._ctx._request_and_get(
            request=request,
            is_ready=is_ready,
            get_value=get_value,
            clear=clear,
            timeout=timeout,
            min_interval=min_interval,
            stamp_attr="flange_vel_acc",
        )
        return res

    def get_crash_protection_rating(
        self, timeout: float = 1.0, min_interval: float = 1.0
    ):
        """Get the crash protection rating.

        Parameters
        ----------
        `timeout`: float, optional
        - Timeout in seconds (see `Driver` docstring: Common conventions -> `timeout`).
        - Default is 1.0.

        `min_interval`: float, optional
        - Minimum interval in seconds between two consecutive requests.
        - Default is 1.0.

        Returns
        -------
        MessageAbstract[list[int]] | None
            The crash protection rating.
            If the crash protection rating is not available, return None.

        Message
        -------
        `list[int]`: Collision protection level for each joint.
        - 0: No collision detection
        - 1-8: Collision detection thresholds increase (higher values represent more
            sensitive thresholds)

        Examples
        --------
        >>> rating = robot.get_crash_protection_rating()
        >>> if rating is not None:
        >>>     print(rating.msg)
        >>>     print(rating.hz, rating.timestamp)
        """
        def request() -> None:
            self._send_msg(self._MSG_ParamEnquiryAndConfig(param_enquiry=2))

        def is_ready() -> bool:
            return (
                getattr(self._parser, "crash_protection_rating",
                        None) is not None
                and self._parser.crash_protection_rating.msg.joint_1 is not None
            )

        def get_value() -> MessageAbstract[List[int]]:
            self._parser.crash_protection_rating.hz = self._ctx.fps.get_fps(
                self._parser.crash_protection_rating.msg_type
            )
            temp: MessageAbstract[List[int]] = copy.deepcopy(
                self._parser.crash_protection_rating)
            temp.msg = [
                getattr(
                    temp.msg,
                    f"joint_{i}") for i in range(1, self._JOINT_NUMS + 1)
            ]
            return temp

        def clear() -> None:
            self._parser.crash_protection_rating.msg.clear()

        return self._ctx._request_and_get(
            request=request,
            is_ready=is_ready,
            get_value=get_value,
            clear=clear,
            timeout=timeout,
            min_interval=min_interval,
            stamp_attr="crash_protection_rating",
        )

    def get_joint_assistance_rating(
        self, timeout: float = 1.0, min_interval: float = 1.0
    ):
        """Get the joint assistance rating.

        Parameters
        ----------
        `timeout`: float, optional
        - Timeout in seconds (see `Driver` docstring: Common conventions -> `timeout`).
        - Default is 1.0.

        `min_interval`: float, optional
        - Minimum interval in seconds between two consecutive requests.
        - Default is 1.0.

        Returns
        -------
        MessageAbstract[list[int]] | None
            The joint assistance rating of all joints.
            If the rating is not available, return None.
        """
        def request() -> None:
            self._send_msg(self._MSG_ParamEnquiryAndConfig(param_enquiry=0x05))

        def is_ready() -> bool:
            return (
                getattr(self._parser, "joint_assistance_rating", None) is not None
                and self._parser.joint_assistance_rating.msg.joint_1 is not None
            )

        def get_value() -> MessageAbstract[List[int]]:
            self._parser.joint_assistance_rating.hz = self._ctx.fps.get_fps(
                self._parser.joint_assistance_rating.msg_type
            )
            temp: MessageAbstract[List[int]] = copy.deepcopy(
                self._parser.joint_assistance_rating
            )
            temp.msg = [
                getattr(temp.msg, f"joint_{i}")
                for i in range(1, self._JOINT_NUMS + 1)
            ]
            return temp

        def clear() -> None:
            self._parser.joint_assistance_rating.msg.clear()

        return self._ctx._request_and_get(
            request=request,
            is_ready=is_ready,
            get_value=get_value,
            clear=clear,
            timeout=timeout,
            min_interval=min_interval,
            stamp_attr="joint_assistance_rating",
        )

    def calibrate_joint(
        self,
        joint_index: Literal[1, 2, 3, 4, 5, 6, 255] = 255,
        timeout: float = 1.0
    ):
        """Calibrate the joint.

        This function will set the current position as the joint zero point.

        Parameters
        ----------
        `joint_index`: Literal[1, 2, 3, 4, 5, 6, 255]
        - 1~6: calibrate the specified joint without offset.
        - 255: calibrate all joints with offset. See the official tutorial for pre-calibration joint poses.

        `timeout`: float, optional
        - Timeout in seconds. Default is 1.0.

        Returns
        -------
        bool
            True if the joint is calibrated, False otherwise.

        Examples
        --------
        >>> robot.disable(1)
        >>> input("Please move the joint to the zero position...")
        >>> if robot.calibrate_joint(1):
        >>>     robot.enable(1)
        >>>     robot.move_j([0.0] * 6)
        """
        # Input validation
        self._ctx._validate_timeout(timeout)
        if joint_index not in self._JOINT_INDEX_LIST:
            raise ValueError(f"Joint index should be {self._JOINT_INDEX_LIST}")

        # Clear previous response
        self._clear_resp_set_instruction()

        def request() -> None:
            self._send_msg(
                self._MSG_JointConfig(
                    joint_index=joint_index,
                    set_motor_current_pos_as_zero=0xAE
                )
            )

        def get_value() -> bool:
            return (
                self._parser.resp_set_instruction.msg.is_set_zero_successfully
                == 1
            )

        res = self._resp_set_instruction_get(
            request=request,
            get_value=get_value,
            instruction_index=0x75,
            timeout=timeout,
            stamp_key=f"calibrate_joint:{joint_index}",
        )
        return bool(res)

    def clear_joint_error(
        self,
        joint_index: Literal[1, 2, 3, 4, 5, 6, 255] = 255,
        timeout: float = 1.0,
    ) -> bool:
        """Clear joint error code.

        Parameters
        ----------
        `joint_index`: Literal[1, 2, 3, 4, 5, 6, 255]
        - 1~6: clear error code on the specified joint.
        - 255: clear error code on all joints.

        `timeout`: float, optional
        - Wait time in seconds. Default is 1.0.

        Returns
        -------
        bool
            True if ACK/response is received within `timeout`, False otherwise.

            Notes
            -----
            See `Driver` docstring: Common conventions -> `set_*` return semantics.
            This API is ACK-only and does not include a read-back verifier.
        """
        # Input validation
        self._ctx._validate_timeout(timeout)
        if joint_index not in self._JOINT_INDEX_LIST:
            raise ValueError(f"Joint index should be {self._JOINT_INDEX_LIST}")

        def request() -> None:
            self._send_msg(
                self._MSG_JointConfig(
                    joint_index=joint_index,
                    clear_joint_err=0xAE,
                )
            )

        return self._ack_only_set(
            request=request,
            instruction_index=0x75,
            timeout=timeout,
            stamp_key=f"clear_joint_error:{joint_index}",
        )

    def set_joint_angle_vel_limits(
        self,
        joint_index: Literal[1, 2, 3, 4, 5, 6, 255] = 255,
        min_angle_limit: Optional[float] = None,
        max_angle_limit: Optional[float] = None,
        max_joint_spd: Optional[float] = None,
        timeout: float = 1.0,
    ):
        """Set the joint angle and velocity limits.

        Parameters
        ----------
        `joint_index`: Literal[1, 2, 3, 4, 5, 6, 255]
        - 1~6: set the joint angle and velocity limits of the specified joint.
        - 255: set the joint angle and velocity limits of all joints.

        `min_angle_limit`: float
        - The minimum angle limit in rad.
            (Numerical precision: 1.74532925199e-3 rad)

        `max_angle_limit`: float
        - The maximum angle limit in rad.
            (Numerical precision: 1.74532925199e-3 rad)

        `max_joint_spd`: float
        - The maximum joint speed in rad/s.
            (Numerical precision: 1e-2 rad/s)

        `timeout`: float, optional
        - Timeout in seconds. Default is 1.0.

        Returns
        -------
        bool
            True if the joint angle and velocity limits are set successfully,
            False otherwise.

        Examples
        --------
        >>> success = robot.set_joint_angle_vel_limits(
        ...     joint_index=1, min_angle_limit=-2.618, max_angle_limit=2.618
        ... )
        >>> if success:
        >>>     print("Joint angle and velocity limits set successfully")

        >>> success = robot.set_joint_angle_vel_limits(joint_index=1, max_joint_spd=3.0)
        >>> if success:
        >>>     print("Joint angle and velocity limits set successfully")
        """
        # Input validation
        self._ctx._validate_timeout(timeout)
        if joint_index not in self._JOINT_INDEX_LIST:
            raise ValueError(f"Joint index should be {self._JOINT_INDEX_LIST}")

        if joint_index == 255:
            return self._all_joints_bool(
                lambda i: self.set_joint_angle_vel_limits(
                    i, min_angle_limit, max_angle_limit, max_joint_spd
                )
            )
        else:
            # Clear previous response
            self._clear_resp_set_instruction()

            # Prepare control messages
            min_angle_limit = (
                0x7FFF
                if min_angle_limit is None
                else round(min_angle_limit * RAD2DEG * 1e1)
            )
            max_angle_limit = (
                0x7FFF
                if max_angle_limit is None
                else round(max_angle_limit * RAD2DEG * 1e1)
            )
            max_joint_spd = (
                0x7FFF if max_joint_spd is None else round(
                    abs(max_joint_spd) * 1e2)
            )

            def request() -> None:
                self._send_msg(
                    self._MSG_MotorAngleLimitMaxSpdSet(
                        joint_index,
                        max_angle_limit,
                        min_angle_limit,
                        max_joint_spd
                    )
                )

            def check() -> bool:
                res = self.get_joint_angle_vel_limits(joint_index)
                return not (
                    res is None
                    or min_angle_limit != 0x7FFF
                    and min_angle_limit
                    != round(res.msg.min_angle_limit * RAD2DEG * 1e1)
                    or max_angle_limit != 0x7FFF
                    and max_angle_limit
                    != round(res.msg.max_angle_limit * RAD2DEG * 1e1)
                    or max_joint_spd != 0x7FFF
                    and max_joint_spd != round(abs(res.msg.max_joint_spd) * 1e2)
                )

            res = self._ack_and_check_set(
                request=request,
                instruction_index=0x74,
                check=check,
                timeout=timeout,
                stamp_key=f"set_joint_angle_vel_limits:{joint_index}",
            )
            return bool(res)

    def set_joint_acc_limits(
        self,
        joint_index: Literal[1, 2, 3, 4, 5, 6, 255] = 255,
        max_joint_acc: Optional[float] = None,
        timeout: float = 1.0,
    ):
        """Set the joint acceleration limits.

        Parameters
        ----------
        `joint_index`: Literal[1, 2, 3, 4, 5, 6, 255]
        - 1~6: set the joint acceleration limits of the specified joint.
        - 255: set the joint acceleration limits of all joints.

        `max_joint_acc`: float
        - The maximum joint acceleration in rad/s^2.
            (Numerical precision: 1e-2 rad/s^2)

        `timeout`: float, optional
        - Timeout in seconds. Default is 1.0.

        Returns
        -------
        bool
            True if the maximum joint acceleration is set successfully, False
            otherwise.
        """
        # Input validation
        self._ctx._validate_timeout(timeout)
        if joint_index not in self._JOINT_INDEX_LIST:
            raise ValueError(f"Joint index should be {self._JOINT_INDEX_LIST}")

        if joint_index == 255:
            return self._all_joints_bool(
                lambda i: self.set_joint_acc_limits(
                    i, max_joint_acc))
        else:
            # Clear previous response
            self._clear_resp_set_instruction()

            # Prepare control messages
            max_joint_acc = (
                0x7FFF if max_joint_acc is None else round(
                    abs(max_joint_acc) * 1e2)
            )

            def request() -> None:
                self._send_msg(
                    self._MSG_JointConfig(
                        joint_index=joint_index,
                        acc_param_config_is_effective_or_not=0xAE,
                        max_joint_acc=max_joint_acc,
                    )
                )

            def check() -> bool:
                res = self.get_joint_acc_limits(joint_index)
                return not (
                    res is None
                    or max_joint_acc != 0x7FFF
                    and max_joint_acc != round(abs(res.msg.max_joint_acc) * 1e2)
                )

            res = self._ack_and_check_set(
                request=request,
                instruction_index=0x75,
                check=check,
                timeout=timeout,
                stamp_key=f"set_joint_acc_limits:{joint_index}",
            )
            return bool(res)

    def set_payload(
        self,
        payload: Literal['empty', 'half', 'full'] = 'empty',
        timeout: float = 1.0
    ):
        """Set the arm payload.

        Parameters
        ----------
        `load`: Literal['empty', 'half', 'full']
        - `OPTIONS.PAYLOAD.EMPTY`: empty payload
        - `OPTIONS.PAYLOAD.HALF`: half payload
        - `OPTIONS.PAYLOAD.FULL`: full payload

        `timeout`: float, optional
        - Timeout in seconds. Default is 1.0.

        Returns
        -------
        bool
            True if an ACK/response is received within `timeout`, False otherwise.

            Notes
            -----
            See `Driver` docstring: Common conventions -> `set_*` return semantics.

        Examples
        --------
        >>> # 1. Set payload to empty
        >>> success = robot.set_payload(robot.OPTIONS.PAYLOAD.EMPTY)
        >>> if success:
        >>>     print("Payload set successfully")
        >>>
        >>> # 2. Set payload to half
        >>> success = robot.set_payload(robot.OPTIONS.PAYLOAD.HALF)
        >>> if success:
        >>>     print("Payload set successfully")
        >>>
        >>> # 3. Set payload to full
        >>> success = robot.set_payload(robot.OPTIONS.PAYLOAD.FULL)
        >>> if success:
        >>>     print("Payload set successfully")
        """
        # Input validation
        self._ctx._validate_timeout(timeout)
        if payload not in self.OPTIONS.PAYLOAD.value_list():
            raise ValueError(
                f"Load should be in PAYLOAD: {self.OPTIONS.PAYLOAD.value_list()}")

        payload_code = PiperDefaultDriverAPIProtoAdapter.payload(payload)

        # Clear previous response
        self._clear_resp_set_instruction()

        def request() -> None:
            self._send_msg(
                self._MSG_ParamEnquiryAndConfig(
                    end_load_param_setting_effective=0xAE,
                    set_end_load=payload_code))

        return self._ack_only_set(
            request=request,
            instruction_index=0x77,
            timeout=timeout,
            stamp_key="set_payload",
        )

    def set_flange_vel_acc_limits(
        self,
        max_linear_vel: Optional[float] = None,
        max_angular_vel: Optional[float] = None,
        max_linear_acc: Optional[float] = None,
        max_angular_acc: Optional[float] = None,
        timeout: float = 1.0,
    ):
        """Set the flange velocity and acceleration limits.

        Parameters
        ----------
        `max_linear_vel`: float
        - The maximum linear velocity in m/s.
            (Numerical precision: 1e-3 m/s)

        `max_angular_vel`: float
        - The maximum angular velocity in rad/s.
            (Numerical precision: 1e-3 rad/s)

        `max_linear_acc`: float
        - The maximum linear acceleration in m/s^2.
            (Numerical precision: 1e-3 m/s^2)

        `max_angular_acc`: float
        - The maximum angular acceleration in rad/s^2.
            (Numerical precision: 1e-3 rad/s^2)

        `timeout`: float, optional
        - Timeout in seconds. Default is 1.0.

        Returns
        -------
        bool
            True if the end effector velocity and acceleration limits are set
            successfully, False otherwise.

        Examples
        --------
        >>> success = robot.set_flange_vel_acc_limits(
        ...     max_linear_vel=0.5,
        ...     max_angular_vel=0.13,
        ...     max_linear_acc=0.8,
        ...     max_angular_acc=0.2,
        ... )
        >>> if success:
        >>>     print("Flange velocity and acceleration limits set successfully")
        """
        # Input validation
        self._ctx._validate_timeout(timeout)

        # Clear previous response
        self._clear_resp_set_instruction()

        # Prepare control messages
        max_linear_vel = (
            0x7FFF if max_linear_vel is None else round(
                abs(max_linear_vel) * 1e3)
        )
        max_angular_vel = (
            0x7FFF if max_angular_vel is None else round(
                abs(max_angular_vel) * 1e3)
        )
        max_linear_acc = (
            0x7FFF if max_linear_acc is None else round(
                abs(max_linear_acc) * 1e3)
        )
        max_angular_acc = (
            0x7FFF if max_angular_acc is None else round(
                abs(max_angular_acc) * 1e3)
        )

        def request() -> None:
            self._send_msg(
                self._MSG_EndVelAccParamConfig(
                    max_linear_vel,
                    max_angular_vel,
                    max_linear_acc,
                    max_angular_acc
                )
            )

        def check() -> bool:
            res = self.get_flange_vel_acc_limits()
            return not (
                res is None
                or max_linear_vel != 0x7FFF
                and max_linear_vel != round(abs(res.msg.end_max_linear_vel) * 1e3)
                or max_angular_vel != 0x7FFF
                and max_angular_vel != round(abs(res.msg.end_max_angular_vel) * 1e3)
                or max_linear_acc != 0x7FFF
                and max_linear_acc != round(abs(res.msg.end_max_linear_acc) * 1e3)
                or max_angular_acc != 0x7FFF
                and max_angular_acc != round(abs(res.msg.end_max_angular_acc) * 1e3)
            )

        return self._ack_and_check_set(
            request=request,
            instruction_index=0x79,
            check=check,
            timeout=timeout,
            stamp_key="set_flange_vel_acc_limits",
        )

    def set_crash_protection_rating(
        self,
        joint_index: Literal[1, 2, 3, 4, 5, 6, 255] = 255,
        rating: Literal[0, 1, 2, 3, 4, 5, 6, 7, 8] = 0,
        timeout: float = 1.0,
    ):
        """Set the crash protection rating.

        Parameters
        ----------
        `joint_index`: Literal[1, 2, 3, 4, 5, 6, 255]
        - 1~6: set the crash protection rating of the specified joint.
        - 255: set the crash protection rating of all joints.

        `rating`: Literal[0, 1, 2, 3, 4, 5, 6, 7, 8]
        - 0~8: set the crash protection rating of the specified joint.

        `timeout`: float, optional
        - Timeout in seconds. Default is 1.0.

        Returns
        -------
        bool
            True if the crash protection rating is set successfully, False otherwise.

        Examples
        --------
        Set the crash protection rating of joint 1 to 1:
        >>> success = robot.set_crash_protection_rating(joint_index=1, rating=1)
        >>> if success:
        >>>     print("Crash protection rating set successfully")

        Set the crash protection rating of all joints to 0:
        >>> success = robot.set_crash_protection_rating(joint_index=255, rating=0)
        >>> if success:
        >>>     print("Crash protection rating set successfully")
        """
        # Input validation
        self._ctx._validate_timeout(timeout)
        if joint_index not in self._JOINT_INDEX_LIST:
            raise ValueError(f"Joint index should be {self._JOINT_INDEX_LIST}")
        if rating < 0 or rating > 8:
            raise ValueError(
                "Crash protection rating should be between 0 and 8")

        # Clear previous response
        self._clear_resp_set_instruction()

        # Get current crash protection rating
        current_rating = self.get_crash_protection_rating()
        if current_rating is None:
            return False

        if joint_index == 255:
            joints = [rating] * self._JOINT_NUMS
        else:
            joints = current_rating.msg.copy()
            joints[joint_index - 1] = rating

        def request() -> None:
            self._send_msg(self._MSG_CrashProtectionRatingConfig(*joints))

        def check() -> bool:
            res = self.get_crash_protection_rating()
            return not (res is None or res.msg != joints)

        return self._ack_and_check_set(
            request=request,
            instruction_index=0x7A,
            check=check,
            timeout=timeout,
            stamp_key="set_crash_protection_rating",
        )

    def set_joint_assistance_rating(
        self,
        joint_index: Literal[1, 2, 3, 4, 5, 6, 255] = 255,
        rating: Literal[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10] = 0,
        timeout: float = 1.0,
    ):
        """Set the joint assistance rating.

        Parameters
        ----------
        `joint_index`: Literal[1, 2, 3, 4, 5, 6, 255]
        - 1~6: set the assistance rating of the specified joint.
        - 255: set the assistance rating of all joints.

        `rating`: Literal[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
        - 0~10: set the assistance rating of the specified joint.

        `timeout`: float, optional
        - Timeout in seconds. Default is 1.0.

        Returns
        -------
        bool
            True if the rating is set successfully, False otherwise.
        """
        self._ctx._validate_timeout(timeout)
        if joint_index not in self._JOINT_INDEX_LIST:
            raise ValueError(f"Joint index should be {self._JOINT_INDEX_LIST}")
        if rating < 0 or rating > 10:
            raise ValueError("Joint assistance rating should be between 0 and 10")

        self._clear_resp_set_instruction()
        current_rating = self.get_joint_assistance_rating()
        if current_rating is None:
            return False

        if joint_index == 255:
            joints = [rating] * self._JOINT_NUMS
        else:
            joints = current_rating.msg.copy()
            joints[joint_index - 1] = rating

        def request() -> None:
            self._send_msg(self._MSG_JointAssistanceRatingConfig(*joints))

        def check() -> bool:
            res = self.get_joint_assistance_rating()
            return not (res is None or res.msg != joints)

        return self._ack_and_check_set(
            request=request,
            instruction_index=0x87,
            check=check,
            timeout=timeout,
            stamp_key="set_joint_assistance_rating",
        )

    def set_flange_vel_acc_limits_to_default(self, timeout: float = 1.0):
        """Set the flange velocity and acceleration limits to default.

        Parameters
        ----------
        `timeout`: float, optional
        - Timeout in seconds. Default is 1.0.

        Returns
        -------
        bool
            True if an ACK/response is received within `timeout`, False otherwise.

            Notes
            -----
            See `Driver` docstring: Common conventions -> `set_*` return semantics.

        Examples
        --------
        >>> success = robot.set_flange_vel_acc_limits_to_default()
        >>> if success:
        >>>     print(
        ...         "Flange velocity and acceleration limits set to default "
        ...         "successfully"
        ...     )
        """
        # Input validation
        self._ctx._validate_timeout(timeout)

        # Clear previous response
        self._clear_resp_set_instruction()

        def request() -> None:
            self._send_msg(self._MSG_ParamEnquiryAndConfig(param_setting=1))

        return self._ack_only_set(
            request=request,
            instruction_index=0x77,
            timeout=timeout,
            stamp_key="set_flange_vel_acc_limits_to_default",
        )

    def set_joint_angle_vel_acc_limits_to_default(self, timeout: float = 1.0):
        """Set the joint angle, velocity and acceleration limits to default.

        Parameters
        ----------
        `timeout`: float, optional
        - Timeout in seconds. Default is 1.0.

        Returns
        -------
        bool
            True if an ACK/response is received within `timeout`, False otherwise.

            Notes
            -----
            See `Driver` docstring: Common conventions -> `set_*` return semantics.

        Examples
        --------
        >>> success = robot.set_joint_angle_vel_acc_limits_to_default()
        >>> if success:
        >>>     print(
        ...         "Joint angle, velocity and acceleration limits set to default "
        ...         "successfully"
        ...     )
        """
        # Input validation
        self._ctx._validate_timeout(timeout)

        # Clear previous response
        self._clear_resp_set_instruction()

        def request() -> None:
            self._send_msg(self._MSG_ParamEnquiryAndConfig(param_setting=2))

        return self._ack_only_set(
            request=request,
            instruction_index=0x77,
            timeout=timeout,
            stamp_key="set_joint_angle_vel_acc_limits_to_default",
        )

    def set_links_vel_acc_period_feedback(
        self, enable: bool = False, timeout: float = 1.0
    ):
        """Set each joint link Cartesian velocity and acceleration period
        feedback.

        In the lower-level main control, this function has been deprecated, but
        it will still periodically feedback data. The feedback data is all 0
        and has no meaning. It is recommended to disable periodic feedback
        through this function.

        Parameters
        ----------
        `enable`: bool
        - True: enable the joint links Cartesian velocity and acceleration
            period feedback.
        - False: disable the joint links Cartesian velocity and acceleration
            period feedback.

        `timeout`: float, optional
        - Timeout in seconds. Default is 1.0.

        Returns
        -------
        bool
            True if an ACK/response is received within `timeout`, False
            otherwise.

            Notes
            -----
            See `Driver` docstring: Common conventions -> `set_*` return
            semantics.
            This API does not provide a direct read-back verifier; observe
            periodic feedback frames to confirm behavior.

            A practical way to verify is using `candump` and checking whether
            CAN IDs `0x481~0x486` appear on the bus (these are the `0x48x`
            periodic frames).
            For example, run:

                candump can0 | grep "48[1-6]"

            before/after calling this API and compare the output.
        """
        # Input validation
        self._ctx._validate_timeout(timeout)

        # Clear previous response
        self._clear_resp_set_instruction()

        def request() -> None:
            self._send_msg(
                self._MSG_ParamEnquiryAndConfig(
                    data_feedback_0x48x=1 if enable else 2)
            )

        return self._ack_only_set(
            request=request,
            instruction_index=0x77,
            timeout=timeout,
            stamp_key="set_links_vel_acc_period_feedback",
        )

    # -------------------------- CPV --------------------------

    _CPV_VALUE_SCALE = {
        'po': (1e-3 * DEG2RAD, 1.0 / (1e-3 * DEG2RAD)),
        'sp': (1e-3, 1e3),
        'ac': (1e-2, 1e2),
        'dc': (1e-2, 1e2),
        'vv': (1e-3, 1e3),
        'pp': (1e-2, 1e2),
        'kp': (1e-2, 1e2),
        'ki': (1e-2, 1e2),
    }

    def _cpv_get_scale(
        self,
        type_: Literal['po', 'sp', 'ac', 'dc', 'vv', 'pp', 'kp', 'ki']
    ) -> float:
        return self._CPV_VALUE_SCALE[type_][0]

    def _cpv_set_scale(
        self,
        type_: Literal['po', 'sp', 'ac', 'dc', 'vv', 'pp', 'kp', 'ki']
    ) -> float:
        return self._CPV_VALUE_SCALE[type_][1]

    def _cpv_po_joints_flag(
        self,
        joint_index: Literal[1, 2, 3, 4, 5, 6]
    ) -> None:
        if getattr(self._parser, "_cpv_po_joints_flag", None) is None:
            self._parser._cpv_po_joints_flag = [True] * self._JOINT_NUMS
        
        arm_status: Optional[MessageAbstract[ArmMsgFeedbackStatus]] = getattr(
            self._parser, "arm_status", None
        )
        if (arm_status is not None
            and (self._parser.arm_status.msg.ctrl_mode != self.ARM_STATUS.CtrlMode.CAN_CTRL
                 or self._parser.arm_status.msg.mode_feedback in [
                     self.ARM_STATUS.ModeFeedback.MOVE_J,
                     self.ARM_STATUS.ModeFeedback.MOVE_MIT]
            ) and not self._parser._cpv_po_joints_flag[joint_index - 1]):
            self._parser._cpv_po_joints_flag = [True] * self._JOINT_NUMS

    def _move_cpv(
        self,
        joint_index: Literal[1, 2, 3, 4, 5, 6],
        type_: Literal['po', 'sp'],
        value: float,
    ) -> None:
        if joint_index not in self._JOINT_INDEX_LIST[:-1]:
            raise ValueError(
                f"Joint index should be {self._JOINT_INDEX_LIST[:-1]}")
        
        msg = self._parser._make_cpv_settings_and_queries_msg(
            joint_index=joint_index,
            mode='w',
            type_=type_,
            value=round(value * self._cpv_set_scale(type_)),
        )
        self._cpv_po_joints_flag(joint_index)
        self._maybe_set_motion_mode('cpv')
        self._send_msg(msg)

        if type_ == "po" and self._parser._cpv_po_joints_flag[joint_index - 1]:
            self._send_msg(msg)
            self._parser._cpv_po_joints_flag[joint_index - 1] = False

    def _get_cpv(
        self,
        joint_index: Literal[1, 2, 3, 4, 5, 6],
        type_: Literal['po', 'sp', 'ac', 'dc', 'vv', 'pp', 'kp', 'ki'],
        timeout: float = 1.0,
        min_interval: float = 1.0,
    ) -> Optional[float]:
        self._ctx._validate_timeout(timeout)
        if joint_index not in self._JOINT_INDEX_LIST[:-1]:
            raise ValueError(
                f"Joint index should be {self._JOINT_INDEX_LIST[:-1]}")

        def request() -> None:
            self._cpv_po_joints_flag(joint_index)
            self._maybe_set_motion_mode('cpv')
            self._send_msg(
                self._parser._make_cpv_settings_and_queries_msg(
                    joint_index=joint_index,
                    mode='r',
                    type_=type_,
                    value=0,
                )
            )

        def get_msg() -> Optional[MessageAbstract[ArmMsgFeedbackCPVResponse]]:
            return getattr(
                self._parser,
                f"cpv_response_{joint_index}",
                None
            )

        def is_ready() -> bool:
            msg = get_msg()
            return (
                msg is not None
                and msg.msg.type_value.get(type_) is not None
            )

        def get_value() -> Optional[float]:
            msg = get_msg()
            if msg is None:
                return None
            return msg.msg.type_value.get(type_) * self._cpv_get_scale(type_)

        def clear() -> None:
            msg = get_msg()
            if msg is not None:
                msg.msg.type_value.pop(type_, None)

        return self._ctx._request_and_get(
            request=request,
            is_ready=is_ready,
            get_value=get_value,
            clear=clear,
            timeout=timeout,
            min_interval=min_interval,
            stamp_attr=f"get_cpv_{type_}:{joint_index}",
        )

    def _set_cpv(
        self,
        joint_index: Literal[1, 2, 3, 4, 5, 6],
        type_: Literal['ac', 'dc', 'vv', 'pp', 'kp', 'ki'],
        value: float,
        get: Callable[[Literal[1, 2, 3, 4, 5, 6], float, float], Optional[float]],
        timeout: float = 1.0,
    ) -> bool:
        self._ctx._validate_timeout(timeout)
        if joint_index not in self._JOINT_INDEX_LIST[:-1]:
            raise ValueError(
                f"Joint index should be {self._JOINT_INDEX_LIST[:-1]}")

        value = round(abs(value) * self._cpv_set_scale(type_))

        def request() -> None:
            self._cpv_po_joints_flag(joint_index)
            self._maybe_set_motion_mode('cpv')
            self._send_msg(
                self._parser._make_cpv_settings_and_queries_msg(
                    joint_index=joint_index,
                    mode='w',
                    type_=type_,
                    value=value,
                )
            )

        def check() -> bool:
            res = get(joint_index)
            if res is None:
                return False
            return round(abs(res) * self._cpv_set_scale(type_)) == value

        return bool(self._ctx._request_and_get(
            request=request,
            is_ready=check,
            get_value=lambda: True,
            timeout=timeout,
            min_interval=0.0,
            stamp_attr=f"set_cpv_{type_}:{joint_index}",
        ))

    def move_cpv_pos(
        self,
        joint_index: Literal[1, 2, 3, 4, 5, 6],
        pos: float,
    ) -> None:
        """Command joint position in CPV motion mode.

        Parameters
        ----------
        `joint_index`: Literal[1, 2, 3, 4, 5, 6]

        `pos`: float
        - Target joint angle in radians.
        """
        lower_limit, upper_limit = self._mit_position_limits(joint_index)
        if not Validator.is_within_limit(pos, lower_limit, upper_limit):
            print(
                f"Warning: Desired position {pos} rad is outside "
                f"joint {joint_index} limits [{lower_limit}, {upper_limit}] rad. "
            )
            pos = Validator.clamp(pos, lower_limit, upper_limit)

        self._move_cpv(
            joint_index=joint_index,
            type_='po',
            value=pos,
        )

    def move_cpv_vel(
        self,
        joint_index: Literal[1, 2, 3, 4, 5, 6],
        vel: float,
    ) -> None:
        """Command joint velocity reference in CPV motion mode.

        Parameters
        ----------
        `joint_index`: Literal[1, 2, 3, 4, 5, 6]

        `vel`: float
        - Desired joint velocity in rad/s.
        """
        self._move_cpv(
            joint_index=joint_index,
            type_='sp',
            value=vel,
        )

    def get_cpv_pos(
        self,
        joint_index: Literal[1, 2, 3, 4, 5, 6],
        timeout: float = 1.0,
        min_interval: float = 1.0,
    ) -> Optional[float]:
        """Read joint position from the CPV feedback channel.

        Issues a CPV read request and waits for the corresponding response
        field on the parser.

        Parameters
        ----------
        `joint_index`: Literal[1, 2, 3, 4, 5, 6]

        `timeout`: float, optional
        - Wait time in seconds. Default is 1.0.

        `min_interval`: float, optional
        - Minimum spacing between requests. Default is 1.0.

        Returns
        -------
        Optional[float]
            Joint angle in radians, or None on timeout.
        """
        return self._get_cpv(
            joint_index=joint_index,
            type_='po',
            timeout=timeout,
            min_interval=min_interval,
        )

    def get_cpv_vel(
        self,
        joint_index: Literal[1, 2, 3, 4, 5, 6],
        timeout: float = 1.0,
        min_interval: float = 1.0,
    ) -> Optional[float]:
        """Read joint velocity from the CPV feedback channel.

        Issues a CPV read request and waits for the corresponding response
        field on the parser.

        Parameters
        ----------
        `joint_index`: Literal[1, 2, 3, 4, 5, 6]

        `timeout`: float, optional
        - Wait time in seconds. Default is 1.0.

        `min_interval`: float, optional
        - Minimum spacing between requests. Default is 1.0.

        Returns
        -------
        Optional[float]
            Velocity in rad/s, or None on timeout.
        """
        return self._get_cpv(
            joint_index=joint_index,
            type_='sp',
            timeout=timeout,
            min_interval=min_interval,
        )

    def get_cpv_acc(
        self,
        joint_index: Literal[1, 2, 3, 4, 5, 6],
        timeout: float = 1.0,
        min_interval: float = 1.0,
    ) -> Optional[float]:
        """Read CPV joint acceleration parameter.

        Parameters
        ----------
        `joint_index`: Literal[1, 2, 3, 4, 5, 6]

        `timeout`: float, optional
        - Wait time in seconds. Default is 1.0.

        `min_interval`: float, optional
        - Minimum spacing between requests. Default is 1.0.

        Returns
        -------
        Optional[float]
            Acceleration in rad/s^2, or None on timeout.
        """
        return self._get_cpv(
            joint_index=joint_index,
            type_='ac',
            timeout=timeout,
            min_interval=min_interval,
        )

    def get_cpv_dcc(
        self,
        joint_index: Literal[1, 2, 3, 4, 5, 6],
        timeout: float = 1.0,
        min_interval: float = 1.0,
    ) -> Optional[float]:
        """Read CPV joint deceleration parameter.

        Parameters
        ----------
        `joint_index`: Literal[1, 2, 3, 4, 5, 6]

        `timeout`: float, optional
        - Wait time in seconds. Default is 1.0.

        `min_interval`: float, optional
        - Minimum spacing between requests. Default is 1.0.

        Returns
        -------
        Optional[float]
            Deceleration in rad/s^2, or None on timeout.
        """
        return self._get_cpv(
            joint_index=joint_index,
            type_='dc',
            timeout=timeout,
            min_interval=min_interval,
        )

    def get_cpv_cv(
        self,
        joint_index: Literal[1, 2, 3, 4, 5, 6],
        timeout: float = 1.0,
        min_interval: float = 1.0,
    ) -> Optional[float]:
        """Read CPV contour / profile velocity.

        Parameters
        ----------
        `joint_index`: Literal[1, 2, 3, 4, 5, 6]

        `timeout`: float, optional
        - Wait time in seconds. Default is 1.0.

        `min_interval`: float, optional
        - Minimum spacing between requests. Default is 1.0.

        Returns
        -------
        Optional[float]
            Contour velocity in rad/s, or None on timeout.
        """
        return self._get_cpv(
            joint_index=joint_index,
            type_='vv',
            timeout=timeout,
            min_interval=min_interval,
        )

    def get_cpv_pp(
        self,
        joint_index: Literal[1, 2, 3, 4, 5, 6],
        timeout: float = 1.0,
        min_interval: float = 1.0,
    ) -> Optional[float]:
        """Read CPV position-loop proportional gain.

        Parameters
        ----------
        `joint_index`: Literal[1, 2, 3, 4, 5, 6]

        `timeout`: float, optional
        - Wait time in seconds. Default is 1.0.

        `min_interval`: float, optional
        - Minimum spacing between requests. Default is 1.0.

        Returns
        -------
        Optional[float]
            Position-loop Kp, or None on timeout.
        """
        return self._get_cpv(
            joint_index=joint_index,
            type_='pp',
            timeout=timeout,
            min_interval=min_interval,
        )

    def get_cpv_kp(
        self,
        joint_index: Literal[1, 2, 3, 4, 5, 6],
        timeout: float = 1.0,
        min_interval: float = 1.0,
    ) -> Optional[float]:
        """Read CPV velocity-loop proportional gain.

        Parameters
        ----------
        `joint_index`: Literal[1, 2, 3, 4, 5, 6]

        `timeout`: float, optional
        - Wait time in seconds. Default is 1.0.

        `min_interval`: float, optional
        - Minimum spacing between requests. Default is 1.0.

        Returns
        -------
        Optional[float]
            Velocity-loop Kp, or None on timeout.
        """
        return self._get_cpv(
            joint_index=joint_index,
            type_='kp',
            timeout=timeout,
            min_interval=min_interval,
        )

    def get_cpv_ki(
        self,
        joint_index: Literal[1, 2, 3, 4, 5, 6],
        timeout: float = 1.0,
        min_interval: float = 1.0,
    ) -> Optional[float]:
        """Read CPV velocity-loop integral gain.

        Parameters
        ----------
        `joint_index`: Literal[1, 2, 3, 4, 5, 6]

        `timeout`: float, optional
        - Wait time in seconds. Default is 1.0.

        `min_interval`: float, optional
        - Minimum spacing between requests. Default is 1.0.

        Returns
        -------
        Optional[float]
            Velocity-loop Ki, or None on timeout.
        """
        return self._get_cpv(
            joint_index=joint_index,
            type_='ki',
            timeout=timeout,
            min_interval=min_interval,
        )

    def set_cpv_acc(
        self,
        joint_index: Literal[1, 2, 3, 4, 5, 6],
        acc: float,
        timeout: float = 1.0,
    ) -> bool:
        """Set CPV joint acceleration and verify by read-back.

        Parameters
        ----------
        `joint_index`: Literal[1, 2, 3, 4, 5, 6]

        `acc`: float
        - Acceleration in rad/s^2.

        `timeout`: float, optional
        - Wait time in seconds. Default is 1.0.

        Returns
        -------
        bool
            True if the read-back equals ``abs(acc)``, False otherwise.
        """
        return self._set_cpv(
            joint_index=joint_index,
            type_='ac',
            value=acc,
            get=self.get_cpv_acc,
            timeout=timeout,
        )

    def set_cpv_dcc(
        self,
        joint_index: Literal[1, 2, 3, 4, 5, 6],
        dcc: float,
        timeout: float = 1.0,
    ) -> bool:
        """Set CPV joint deceleration and verify by read-back.

        Parameters
        ----------
        `joint_index`: Literal[1, 2, 3, 4, 5, 6]

        `dcc`: float
        - Deceleration in rad/s^2.

        `timeout`: float, optional
        - Wait time in seconds. Default is 1.0.

        Returns
        -------
        bool
            True if the read-back equals ``abs(dcc)``, False otherwise.
        """
        return self._set_cpv(
            joint_index=joint_index,
            type_='dc',
            value=dcc,
            get=self.get_cpv_dcc,
            timeout=timeout,
        )

    def set_cpv_cv(
        self,
        joint_index: Literal[1, 2, 3, 4, 5, 6],
        cv: float,
        timeout: float = 1.0,
    ) -> bool:
        """Set CPV contour velocity and verify by read-back.

        Parameters
        ----------
        `joint_index`: Literal[1, 2, 3, 4, 5, 6]

        `cv`: float
        - Contour velocity in rad/s.

        `timeout`: float, optional
        - Wait time in seconds. Default is 1.0.

        Returns
        -------
        bool
            True if the read-back equals ``abs(cv)``, False otherwise.
        """
        return self._set_cpv(
            joint_index=joint_index,
            type_='vv',
            value=cv,
            get=self.get_cpv_cv,
            timeout=timeout,
        )

    def set_cpv_pp(
        self,
        joint_index: Literal[1, 2, 3, 4, 5, 6],
        pp: float,
        timeout: float = 1.0,
    ) -> bool:
        """Set CPV position-loop Kp and verify by read-back.

        Parameters
        ----------
        `joint_index`: Literal[1, 2, 3, 4, 5, 6]

        `pp`: float
        - Position-loop proportional gain.

        `timeout`: float, optional
        - Wait time in seconds. Default is 1.0.

        Returns
        -------
        bool
            True if the read-back equals ``abs(pp)``, False otherwise.
        """
        return self._set_cpv(
            joint_index=joint_index,
            type_='pp',
            value=pp,
            get=self.get_cpv_pp,
            timeout=timeout,
        )

    def set_cpv_kp(
        self,
        joint_index: Literal[1, 2, 3, 4, 5, 6],
        kp: float,
        timeout: float = 1.0,
    ) -> bool:
        """Set CPV velocity-loop Kp and verify by read-back.

        Parameters
        ----------
        `joint_index`: Literal[1, 2, 3, 4, 5, 6]

        `kp`: float
        - Velocity-loop proportional gain.

        `timeout`: float, optional
        - Wait time in seconds. Default is 1.0.

        Returns
        -------
        bool
            True if the read-back equals ``abs(kp)``, False otherwise.
        """
        return self._set_cpv(
            joint_index=joint_index,
            type_='kp',
            value=kp,
            get=self.get_cpv_kp,
            timeout=timeout,
        )

    def set_cpv_ki(
        self,
        joint_index: Literal[1, 2, 3, 4, 5, 6],
        ki: float,
        timeout: float = 1.0,
    ) -> bool:
        """Set CPV velocity-loop Ki and verify by read-back.

        Parameters
        ----------
        `joint_index`: Literal[1, 2, 3, 4, 5, 6]

        `ki`: float
        - Velocity-loop integral gain.

        `timeout`: float, optional
        - Wait time in seconds. Default is 1.0.

        Returns
        -------
        bool
            True if the read-back equals ``abs(ki)``, False otherwise.
        """
        return self._set_cpv(
            joint_index=joint_index,
            type_='ki',
            value=ki,
            get=self.get_cpv_ki,
            timeout=timeout,
        )
