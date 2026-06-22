from typing import Optional
from typing_extensions import Literal

from .....msgs.core import MessageAbstract
from .....msgs.nero.default import (
    ArmMsgFeedbackHighSpd,
)
from ...versions.v112.driver import Driver as V112Driver


class Driver(V112Driver):
    """Nero CAN driver for firmware >= v120 (1.20).

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
            return motor_state
        else:
            return None

    # -------------------------- CPV --------------------------

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
        return self._get_cpv(
            joint_index=joint_index,
            type_='sp',
            timeout=timeout,
            min_interval=min_interval,
        )
