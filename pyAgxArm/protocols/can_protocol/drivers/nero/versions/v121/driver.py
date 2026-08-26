from typing_extensions import Literal

from ..v120.driver import Driver as V120Driver


class Driver(V120Driver):
    """Nero CAN driver for firmware >= v121 (1.21).

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

    def move_mit(
        self,
        joint_index: Literal[1, 2, 3, 4, 5, 6, 7],
        p_des: float = 0.0,
        v_des: float = 0.0,
        kp: float = 10.0,
        kd: float = 0.8,
        t_ff: float = 0.0,
    ):
        v_des = v_des if joint_index == 6 else -v_des
        return super().move_mit(
            joint_index=joint_index,
            p_des=p_des,
            v_des=v_des,
            kp=kp,
            kd=kd,
            t_ff=t_ff,
        )

    def move_cpv_vel(
        self,
        joint_index: Literal[1, 2, 3, 4, 5, 6, 7],
        vel: float,
    ) -> None:
        vel = vel if joint_index == 6 else -vel
        return super().move_cpv_vel(joint_index, vel)
