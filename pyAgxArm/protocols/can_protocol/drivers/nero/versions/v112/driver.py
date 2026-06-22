import warnings
from typing import Optional
from typing_extensions import Literal

from .......utiles.numeric_codec import DEG2RAD
from .......utiles.validator import Validator
from .....msgs.core import MessageAbstract
from .....msgs.piper.default import ArmMsgFeedbackCPVResponse
from .....msgs.nero.default import (
    ArmMsgFeedbackLeaderJointStates,
    ArmMsgFeedbackStatus,
)
from .....msgs.nero.versions.v112 import ArmMsgFeedbackIKJointStates
from ...versions.v111.driver import Driver as V111Driver

from .parser import (
    NeroV112DriverAPIOptions,
    NeroV112DriverAPIProtoAdapter,
    Parser,
)


class Driver(V111Driver):
    """Nero CAN driver for firmware == v112 (1.12).

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

    @property
    def OPTIONS(self):
        return NeroV112DriverAPIOptions

    def set_motion_mode(
        self,
        motion_mode: Literal['p', 'j', 'l', 'c', 'mit', 'js', 'cpv'] = 'p'
    ):
        """Set movement mode and MIT mode.

        Firmware version: >= v112 (1.12). CPV mode is supported from v112.

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
        self._msg_mode.move_mode = NeroV112DriverAPIProtoAdapter.motion_mode(motion_mode)
        self._msg_mode.mit_mode = NeroV112DriverAPIProtoAdapter.mit_mode(motion_mode)
        self._set_mode()

    def _maybe_set_motion_mode(
        self, motion_mode: Literal['p', 'j', 'l', 'c', 'mit', 'js', 'cpv']
    ) -> None:
        """Set motion mode only when auto mode-setting is enabled."""
        if self._auto_set_motion_mode_enabled:
            self.set_motion_mode(motion_mode)

    def set_normal_mode(self):
        """Set the robotic arm to the normal controlled mode (single arm).

        Firmware ≤ 1.11 (``NeroFW.DEFAULT`` / ``NeroFW.V111``)
            Switches to normal mode and opens CAN feedback push when enabled.

        Firmware ≥ 1.12 (``NeroFW.V112``)
            No effect (compatibility no-op). Only **leader** and **follower**
            modes remain (aligned with Piper). Default is follower; CAN
            feedback push is on at power-up — no manual CAN-push setup.
            Use :meth:`set_follower_mode` or :meth:`set_leader_mode`.

        See Also
        --------
        set_follower_mode, set_leader_mode
        """
        if not getattr(self, "_set_normal_mode_v112_warned", False):
            warnings.warn(
                "set_normal_mode() has no effect on Nero firmware >= 1.12 "
                "(NeroFW.V112): only leader/follower modes (Piper-aligned); "
                "default is follower with CAN push on power-up. Use "
                "set_follower_mode() or set_leader_mode().",
                UserWarning,
                stacklevel=2,
            )
            self._set_normal_mode_v112_warned = True
        return None

    def enable(self, joint_index: Literal[1, 2, 3, 4, 5, 6, 7, 255] = 255, timeout: float = 1.5):
        """Enable one joint motor or all joint motors.

        Parameters
        ----------
        `joint_index`: Literal[1, 2, 3, 4, 5, 6, 7, 255], optional
        - 1~7: enable the specified joint
        - 255: enable all joints (default)

        `timeout`: float, optional
        - 0.0: non-blocking; evaluate readiness once and return immediately.
        - > 0.0: blocking; poll until ready or timeout expires.

        Returns
        -------
        bool
            True if the joint is enabled, False otherwise.

        Examples
        --------
        >>> while not robot.enable():
        >>>     time.sleep(0.01)
        >>> print("All joints enabled")
        """
        if joint_index not in self._JOINT_INDEX_LIST:
            raise ValueError(f"Joint index should be {self._JOINT_INDEX_LIST}")

        def function():
            result = (self.get_arm_status() is None or
            self.get_arm_status().msg.ctrl_mode != self.ARM_STATUS.CtrlMode.CAN_CTRL)

            if result:
                if self._msg_mode.enable_can_push == self._msg_mode.Enums.CanActiveMsgReporting.INVALID:
                    self._msg_mode.enable_can_push = self._msg_mode.Enums.CanActiveMsgReporting.ENABLE
                else:
                    self._msg_mode.enable_can_push = self._msg_mode.Enums.CanActiveMsgReporting.INVALID
                self._set_mode()
                self._ctx._read_stop_event.wait(timeout=0.02)

            return not result

        self._ctx._wait_for_response(function, timeout=0.2)

        def send_enable_msg(joint_index):
            msg = self._MSG_MotorEnableDisableConfig(
                joint_index=joint_index, enable_flag=2)
            self._send_msg(msg)

        if joint_index == 255:
            send_enable_msg(self._JOINT_NUMS + 1)
            return self._ctx._wait_for_response(
                lambda: all(self.get_joints_enable_status_list()),
                timeout=timeout,
            )
        else:
            send_enable_msg(joint_index)
            return self._ctx._wait_for_response(
                lambda: self.get_joint_enable_status(joint_index),
                timeout=timeout,
            )

    def disable(self, joint_index: Literal[1, 2, 3, 4, 5, 6, 7, 255] = 255, timeout: float = 1.5):
        """Disable one joint motor or all joint motors.

        Parameters
        ----------
        `joint_index`: Literal[1, 2, 3, 4, 5, 6, 7, 255], optional
        - 1~7: disable the specified joint
        - 255: disable all joints (default)

        `timeout`: float, optional
        - 0.0: non-blocking; evaluate readiness once and return immediately.
        - > 0.0: blocking; poll until ready or timeout expires.

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

        def function():
            result = (self.get_arm_status() is None or
            self.get_arm_status().msg.ctrl_mode != self.ARM_STATUS.CtrlMode.CAN_CTRL)

            if result:
                if self._msg_mode.enable_can_push == self._msg_mode.Enums.CanActiveMsgReporting.INVALID:
                    self._msg_mode.enable_can_push = self._msg_mode.Enums.CanActiveMsgReporting.ENABLE
                else:
                    self._msg_mode.enable_can_push = self._msg_mode.Enums.CanActiveMsgReporting.INVALID
                self._set_mode()
                self._ctx._read_stop_event.wait(timeout=0.02)

            return not result

        self._ctx._wait_for_response(function, timeout=0.2)

        def send_disable_msg(joint_index):
            msg = self._MSG_MotorEnableDisableConfig(
                joint_index=joint_index, enable_flag=1)
            self._send_msg(msg)

        if joint_index == 255:
            send_disable_msg(self._JOINT_NUMS + 1)
            return self._ctx._wait_for_response(
                lambda: all(not self.get_joint_enable_status(i) 
                for i in self._JOINT_INDEX_LIST[:-1]),
                timeout=timeout,
            )
        else:
            send_disable_msg(joint_index)
            return self._ctx._wait_for_response(
                lambda: not self.get_joint_enable_status(joint_index),
                timeout=timeout,
            )

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
        leader_joint_angles: Optional[
            MessageAbstract[ArmMsgFeedbackLeaderJointStates]
        ] = None
        if getattr(self, "_leader_joint_angles", None) is None:
            self._leader_joint_angles = MessageAbstract(
                msg=list([0.0] * self._JOINT_NUMS),
                msg_type=ArmMsgFeedbackLeaderJointStates.type_,
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
        if getattr(self._parser, "leader_joint_7", None) is not None:
            leader_joint_angles = self._parser.leader_joint_7
            self._leader_joint_angles.msg[6] = leader_joint_angles.msg.joint_7
        if leader_joint_angles is not None:
            self._leader_joint_angles.timestamp = leader_joint_angles.timestamp
            self._leader_joint_angles.hz = self._ctx.fps.get_fps(
                leader_joint_angles.msg_type)
            if Validator.is_joints(
                self._leader_joint_angles.msg,
                length=self._JOINT_NUMS
            ):
                return self._leader_joint_angles
        return None

    def get_ik_joint_angles(self):
        """Get the IK joint angles feedback.

        Notes
        -----
        Only available after calling `move_p/l/c`.

        Returns
        -------
        MessageAbstract[list[float]] | None
            The IK joint angles feedback.
            If the IK joint angles are not available, return None.
        
        Message
        -------
        `list[float]`: IK joint angles, unit: rad

        Examples
        --------
        >>> ika = robot.get_ik_joint_angles()
        >>> if ika is not None:
        >>>     print(ika.msg)
        >>>     print(ika.hz, ika.timestamp)
        """
        ik_joint_angles: Optional[
            MessageAbstract[ArmMsgFeedbackIKJointStates]
        ] = None
        if getattr(self, "_ik_joint_angles", None) is None:
            self._ik_joint_angles = MessageAbstract(
                msg=list([0.0] * self._JOINT_NUMS),
                msg_type=ArmMsgFeedbackIKJointStates.type_,
            )
        if getattr(self._parser, "ik_joint_12", None) is not None:
            ik_joint_angles = self._parser.ik_joint_12
            self._ik_joint_angles.msg[0] = ik_joint_angles.msg.joint_1
            self._ik_joint_angles.msg[1] = ik_joint_angles.msg.joint_2
        if getattr(self._parser, "ik_joint_34", None) is not None:
            ik_joint_angles = self._parser.ik_joint_34
            self._ik_joint_angles.msg[2] = ik_joint_angles.msg.joint_3
            self._ik_joint_angles.msg[3] = ik_joint_angles.msg.joint_4
        if getattr(self._parser, "ik_joint_56", None) is not None:
            ik_joint_angles = self._parser.ik_joint_56
            self._ik_joint_angles.msg[4] = ik_joint_angles.msg.joint_5
            self._ik_joint_angles.msg[5] = ik_joint_angles.msg.joint_6
        if getattr(self._parser, "ik_joint_7", None) is not None:
            ik_joint_angles = self._parser.ik_joint_7
            self._ik_joint_angles.msg[6] = ik_joint_angles.msg.joint_7
        if ik_joint_angles is not None:
            self._ik_joint_angles.timestamp = ik_joint_angles.timestamp
            self._ik_joint_angles.hz = self._ctx.fps.get_fps(
                ik_joint_angles.msg_type)
            if Validator.is_joints(
                self._ik_joint_angles.msg,
                length=self._JOINT_NUMS
            ):
                return self._ik_joint_angles
        return None

    # -------------------------- CPV --------------------------

    _CPV_VALUE_SCALE = {
        'po': (1e-3 * DEG2RAD, 1.0 / (1e-3 * DEG2RAD)),
        # TODO: check the scale of sp, 1e-3 -> 1e-6
        'sp': (1e-6, 1e3),
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
        joint_index: Literal[1, 2, 3, 4, 5, 6, 7]
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
        joint_index: Literal[1, 2, 3, 4, 5, 6, 7],
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
        joint_index: Literal[1, 2, 3, 4, 5, 6, 7],
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
        joint_index: Literal[1, 2, 3, 4, 5, 6, 7],
        type_: Literal['ac', 'dc', 'vv', 'pp', 'kp', 'ki'],
        value: float,
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

        def get_msg() -> Optional[MessageAbstract[ArmMsgFeedbackCPVResponse]]:
            return getattr(
                self._parser,
                f"cpv_response_{joint_index}",
                None
            )

        def is_ready() -> bool:
            msg = get_msg()
            if msg is None:
                return False
            if msg.msg.type_value.get(type_) != 172:
                return False
            # Clears the 172 (0xAC) write-ack entry after success.
            msg.msg.type_value.pop(type_, None)
            return True

        return bool(self._ctx._request_and_get(
            request=request,
            is_ready=is_ready,
            get_value=lambda: True,
            timeout=timeout,
            min_interval=0.0,
            stamp_attr=f"set_cpv_{type_}:{joint_index}",
        ))

    def move_cpv_pos(
        self,
        joint_index: Literal[1, 2, 3, 4, 5, 6, 7],
        pos: float,
    ) -> None:
        """Command joint position in CPV motion mode.

        Parameters
        ----------
        `joint_index`: Literal[1, 2, 3, 4, 5, 6, 7]

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
        joint_index: Literal[1, 2, 3, 4, 5, 6, 7],
        vel: float,
    ) -> None:
        """Command joint velocity reference in CPV motion mode.

        Parameters
        ----------
        `joint_index`: Literal[1, 2, 3, 4, 5, 6, 7]

        `vel`: float
        - Desired joint velocity in rad/s.
        """
        # TODO: remove this after the bug is fixed
        if joint_index != 6:
            vel *= -1

        self._move_cpv(
            joint_index=joint_index,
            type_='sp',
            value=vel,
        )

    def get_cpv_pos(
        self,
        joint_index: Literal[1, 2, 3, 4, 5, 6, 7],
        timeout: float = 1.0,
        min_interval: float = 1.0,
    ) -> Optional[float]:
        """Read joint position from the CPV feedback channel.

        Issues a CPV read request and waits for the corresponding response
        field on the parser.

        Parameters
        ----------
        `joint_index`: Literal[1, 2, 3, 4, 5, 6, 7]

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
        joint_index: Literal[1, 2, 3, 4, 5, 6, 7],
        timeout: float = 1.0,
        min_interval: float = 1.0,
    ) -> Optional[float]:
        """Read joint velocity from the CPV feedback channel.

        Issues a CPV read request and waits for the corresponding response
        field on the parser.

        Parameters
        ----------
        `joint_index`: Literal[1, 2, 3, 4, 5, 6, 7]

        `timeout`: float, optional
        - Wait time in seconds. Default is 1.0.

        `min_interval`: float, optional
        - Minimum spacing between requests. Default is 1.0.

        Returns
        -------
        Optional[float]
            Velocity in rad/s, or None on timeout.
        """
        vel = self._get_cpv(
            joint_index=joint_index,
            type_='sp',
            timeout=timeout,
            min_interval=min_interval,
        )

        # TODO: remove this after the bug is fixed
        # corrected in version 1.20
        if joint_index != 6:
            vel *= -1

        return vel

    def get_cpv_acc(
        self,
        joint_index: Literal[1, 2, 3, 4, 5, 6, 7],
        timeout: float = 1.0,
        min_interval: float = 1.0,
    ) -> Optional[float]:
        """Read CPV joint acceleration parameter.

        Parameters
        ----------
        `joint_index`: Literal[1, 2, 3, 4, 5, 6, 7]

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
        joint_index: Literal[1, 2, 3, 4, 5, 6, 7],
        timeout: float = 1.0,
        min_interval: float = 1.0,
    ) -> Optional[float]:
        """Read CPV joint deceleration parameter.

        Parameters
        ----------
        `joint_index`: Literal[1, 2, 3, 4, 5, 6, 7]

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
        joint_index: Literal[1, 2, 3, 4, 5, 6, 7],
        timeout: float = 1.0,
        min_interval: float = 1.0,
    ) -> Optional[float]:
        """Read CPV contour / profile velocity.

        Parameters
        ----------
        `joint_index`: Literal[1, 2, 3, 4, 5, 6, 7]

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
        joint_index: Literal[1, 2, 3, 4, 5, 6, 7],
        timeout: float = 1.0,
        min_interval: float = 1.0,
    ) -> Optional[float]:
        """Read CPV position-loop proportional gain.

        Parameters
        ----------
        `joint_index`: Literal[1, 2, 3, 4, 5, 6, 7]

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
        joint_index: Literal[1, 2, 3, 4, 5, 6, 7],
        timeout: float = 1.0,
        min_interval: float = 1.0,
    ) -> Optional[float]:
        """Read CPV velocity-loop proportional gain.

        Parameters
        ----------
        `joint_index`: Literal[1, 2, 3, 4, 5, 6, 7]

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
        joint_index: Literal[1, 2, 3, 4, 5, 6, 7],
        timeout: float = 1.0,
        min_interval: float = 1.0,
    ) -> Optional[float]:
        """Read CPV velocity-loop integral gain.

        Parameters
        ----------
        `joint_index`: Literal[1, 2, 3, 4, 5, 6, 7]

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
        joint_index: Literal[1, 2, 3, 4, 5, 6, 7],
        acc: float,
        timeout: float = 1.0,
    ) -> bool:
        """Set CPV joint acceleration and verify by read-back.

        Parameters
        ----------
        `joint_index`: Literal[1, 2, 3, 4, 5, 6, 7]

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
            timeout=timeout,
        )

    def set_cpv_dcc(
        self,
        joint_index: Literal[1, 2, 3, 4, 5, 6, 7],
        dcc: float,
        timeout: float = 1.0,
    ) -> bool:
        """Set CPV joint deceleration and verify by read-back.

        Parameters
        ----------
        `joint_index`: Literal[1, 2, 3, 4, 5, 6, 7]

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
            timeout=timeout,
        )

    def set_cpv_cv(
        self,
        joint_index: Literal[1, 2, 3, 4, 5, 6, 7],
        cv: float,
        timeout: float = 1.0,
    ) -> bool:
        """Set CPV contour velocity and verify by read-back.

        Parameters
        ----------
        `joint_index`: Literal[1, 2, 3, 4, 5, 6, 7]

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
            timeout=timeout,
        )

    def set_cpv_pp(
        self,
        joint_index: Literal[1, 2, 3, 4, 5, 6, 7],
        pp: float,
        timeout: float = 1.0,
    ) -> bool:
        """Set CPV position-loop Kp and verify by read-back.

        Parameters
        ----------
        `joint_index`: Literal[1, 2, 3, 4, 5, 6, 7]

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
            timeout=timeout,
        )

    def set_cpv_kp(
        self,
        joint_index: Literal[1, 2, 3, 4, 5, 6, 7],
        kp: float,
        timeout: float = 1.0,
    ) -> bool:
        """Set CPV velocity-loop Kp and verify by read-back.

        Parameters
        ----------
        `joint_index`: Literal[1, 2, 3, 4, 5, 6, 7]

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
            timeout=timeout,
        )

    def set_cpv_ki(
        self,
        joint_index: Literal[1, 2, 3, 4, 5, 6, 7],
        ki: float,
        timeout: float = 1.0,
    ) -> bool:
        """Set CPV velocity-loop Ki and verify by read-back.

        Parameters
        ----------
        `joint_index`: Literal[1, 2, 3, 4, 5, 6, 7]

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
            timeout=timeout,
        )
