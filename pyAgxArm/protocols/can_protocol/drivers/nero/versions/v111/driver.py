from typing import List, Optional
from typing_extensions import Literal

from .......utiles.numeric_codec import (
    NumericCodec as nc,
    RAD2DEG,
)
from .......utiles.validator import Validator
from ...default.driver import Driver as DefaultDriver
from .....msgs.core import MessageAbstract
from .....msgs.nero.default import (
    ArmMsgFeedbackEndPose,
    ArmMsgFeedbackHighSpd,
)
from .....msgs.nero.versions import ArmMsgFeedbackStatusEnumV111, ArmMsgModeCtrlV111
from .parser import Parser, NeroV111DriverAPIProtoAdapter


class Driver(DefaultDriver):
    """Nero CAN driver for firmware == v111 (1.11).

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

    _Parser = Parser
    ARM_STATUS = ArmMsgFeedbackStatusEnumV111
    _MSG_ModeCtrl = ArmMsgModeCtrlV111

    def set_motion_mode(
        self,
        motion_mode: Literal['p', 'j', 'l', 'c', 'mit', 'js'] = 'p'
    ):
        """Set movement mode and MIT mode.

        Parameters
        ----------
        `motion_mode`: Literal['p', 'j', 'l', 'c', 'mit', 'js']
        - `OPTIONS.MOTION_MODE.P`: move p
        - `OPTIONS.MOTION_MODE.J`: move j
        - `OPTIONS.MOTION_MODE.L`: move l
        - `OPTIONS.MOTION_MODE.C`: move c
        - `OPTIONS.MOTION_MODE.MIT`: move mit (MIT)
        - `OPTIONS.MOTION_MODE.JS`: move js (MIT)

        Raises
        ------
        ValueError
            If `motion_mode` is not in
            ['p', 'j', 'l', 'c', 'mit', 'js'].

        Examples
        --------
        >>> robot.set_motion_mode(robot.OPTIONS.MOTION_MODE.P)
        """
        if motion_mode not in self.OPTIONS.MOTION_MODE.value_list():
            raise ValueError(
                "Invalid motion mode, should be in OPTIONS.MOTION_MODE: "
                f"{self.OPTIONS.MOTION_MODE.value_list()}"
            )
        self._msg_mode.move_mode = NeroV111DriverAPIProtoAdapter.motion_mode(motion_mode)
        self._msg_mode.mit_mode = NeroV111DriverAPIProtoAdapter.mit_mode(motion_mode)
        self._set_mode()

    def _maybe_set_motion_mode(
        self, motion_mode: Literal['p', 'j', 'l', 'c', 'mit', 'js']
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

        rpy = [i * RAD2DEG for i in pose[3:]]

        x = round(pose[0] * 1e6)
        y = round(pose[1] * 1e6)
        z = round(pose[2] * 1e6)

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

    def get_flange_pose(self):
        """Get current flange pose feedback."""
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
            # TODO: remove this after the bug is fixed
            # corrected in version 1.20
            motor_state.msg.velocity = 0.0
            return motor_state
        else:
            return None

    def move_mit(
        self,
        joint_index: Literal[1, 2, 3, 4, 5, 6, 7],
        p_des: float = 0.0,
        v_des: float = 0.0,
        kp: float = 10.0,
        kd: float = 0.8,
        t_ff: float = 0.0,
    ):
        """Control a single joint in MIT (impedance/torque) style mode.

        Firmware version: >= v111 (1.11)

        This API sends an MIT control message for a specific joint with desired
        position/velocity, PD gains, and feed-forward torque.

        The controller conceptually computes a reference torque:

            T_ref = kp * (p_des - p) + kd * (v_des - v) + t_ff

        where `p/v` are the measured joint `position/velocity`.

        Parameters
        ----------
        `joint_index`: Literal[1, 2, 3, 4, 5, 6, 7]

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
        - Feed-forward torque reference (unit: N·m). Range: [-16.0, 16.0].
          Default is 0.0.
            (Numerical precision: 7.814407814407814e-3 N·m)

        Raises
        ------
        ValueError
            If any parameter is outside the allowed range, or if `joint_index`
            is not in {1, 2, 3, 4, 5, 6, 7}.

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

        # TODO: remove this after the bug is fixed
        if joint_index != 6:
            v_des *= -1

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

        if not Validator.is_within_limit(t_ff, -16, 16):
            print(
                f"Warning: Feed-forward torque {t_ff} N·m is outside "
                f"joint {joint_index} limits [-16, 16]. "
            )
            t_ff = Validator.clamp(t_ff, -16, 16)

        p_des = nc.FloatToUint(p_des, -12.5, 12.5, 16)
        v_des = nc.FloatToUint(v_des, -45.0, 45.0, 12)
        kp = nc.FloatToUint(kp, 0.0, 500.0, 12)
        kd = nc.FloatToUint(kd, -5.0, 5.0, 12)
        t_ff = nc.FloatToUint(t_ff, -16.0, 16.0, 12)

        msg = self._parser._make_joint_mit_ctrl_msg(
            joint_index=joint_index,
            p_des=p_des,
            v_des=v_des,
            kp=kp,
            kd=kd,
            t_ff=t_ff,
        )

        self._maybe_set_motion_mode('mit')
        self._send_msg(msg)

    def calibrate_joint(
        self,
        joint_index: Literal[1, 2, 3, 4, 5, 6, 7, 255] = 255,
    ):
        """Calibrate a joint by setting current position as zero.

        Parameters
        ----------
        `joint_index`: Literal[1, 2, 3, 4, 5, 6, 7, 255]
        - 1~7: calibrate the specified joint without offset.
        - 255: calibrate all joints with offset (controller-side behavior).
        """
        if joint_index not in self._JOINT_INDEX_LIST:
            raise ValueError(f"Joint index should be {self._JOINT_INDEX_LIST}")

        self._send_msg(
            self._MSG_JointConfig(
                joint_index=joint_index,
                set_motor_current_pos_as_zero=0xAE,
            )
        )
