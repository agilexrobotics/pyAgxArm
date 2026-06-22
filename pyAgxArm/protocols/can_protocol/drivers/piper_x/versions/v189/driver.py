from typing_extensions import Literal

from ....piper.versions.v189.driver import Driver as PiperDriverV189


class Driver(PiperDriverV189):
    """
    PiperX CAN driver for firmware >= v189 (S-V1.8-9).

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
      request, but it does not strictly guarantee the setting is already applied.
    - Some `set_*` APIs additionally verify by reading back state; their
      docstrings will mention the verification method if applicable.
    """

    def move_cpv_pos(
        self,
        joint_index: Literal[1, 2, 3, 4, 5, 6],
        pos: float,
    ) -> None:
        # TODO: remove this after the bug is fixed
        if joint_index in [4, 5]:
            pos = -pos
        super().move_cpv_pos(
            joint_index,
            pos,
        )
