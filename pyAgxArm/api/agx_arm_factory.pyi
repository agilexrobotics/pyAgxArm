# agx_arm_factory.pyi
from typing_extensions import Literal
from typing import Any, TypeVar, overload
from ..protocols.can_protocol.drivers import (
    NeroDriverDefault,
    NeroDriverV111,
    NeroDriverV112,
    NeroDriverV120,
    PiperDriverDefault,
    PiperDriverV183,
    PiperDriverV188,
    PiperDriverV189,
    PiperHDriverDefault,
    PiperHDriverV183,
    PiperHDriverV188,
    PiperHDriverV189,
    PiperLDriverDefault,
    PiperLDriverV183,
    PiperLDriverV188,
    PiperLDriverV189,
    PiperXDriverDefault,
    PiperXDriverV183,
    PiperXDriverV188,
    PiperXDriverV189,
)

# ---------- Phantom config types ----------

class NeroCanDefaultConfig():
    pass
class NeroCanV111Config():
    pass
class NeroCanV112Config():
    pass
class NeroCanV120Config():
    pass

class PiperCanDefaultConfig():
    pass
class PiperCanV183Config():
    pass
class PiperCanV188Config():
    pass
class PiperCanV189Config():
    pass

class PiperHCanDefaultConfig():
    pass
class PiperHCanV183Config():
    pass
class PiperHCanV188Config():
    pass
class PiperHCanV189Config():
    pass

class PiperLCanDefaultConfig():
    pass
class PiperLCanV183Config():
    pass
class PiperLCanV188Config():
    pass
class PiperLCanV189Config():
    pass

class PiperXCanDefaultConfig():
    pass
class PiperXCanV183Config():
    pass
class PiperXCanV188Config():
    pass
class PiperXCanV189Config():
    pass

# ---------- create_agx_arm_config overloads ----------

# --- nero ---

@overload
def create_agx_arm_config(
    robot: Literal["nero"],
    comm: Literal["can"] = ...,
    firmeware_version: Literal["default"] = ...,
    **kwargs: Any
) -> NeroCanDefaultConfig: ...

@overload
def create_agx_arm_config(
    robot: Literal["nero"],
    comm: Literal["can"] = ...,
    firmeware_version: Literal["v111"] = ...,
    **kwargs: Any
) -> NeroCanV111Config: ...

@overload
def create_agx_arm_config(
    robot: Literal["nero"],
    comm: Literal["can"] = ...,
    firmeware_version: Literal["v112"] = ...,
    **kwargs: Any
) -> NeroCanV112Config: ...

@overload
def create_agx_arm_config(
    robot: Literal["nero"],
    comm: Literal["can"] = ...,
    firmeware_version: Literal["v120"] = ...,
    **kwargs: Any
) -> NeroCanV120Config: ...

# --- piper ---

@overload
def create_agx_arm_config(
    robot: Literal["piper"],
    comm: Literal["can"] = ...,
    firmeware_version: Literal["default"] = ...,
    **kwargs: Any
) -> PiperCanDefaultConfig: ...

@overload
def create_agx_arm_config(
    robot: Literal["piper"],
    comm: Literal["can"] = ...,
    firmeware_version: Literal["v183"] = ...,
    **kwargs: Any
) -> PiperCanV183Config: ...

@overload
def create_agx_arm_config(
    robot: Literal["piper"],
    comm: Literal["can"] = ...,
    firmeware_version: Literal["v188"] = ...,
    **kwargs: Any
) -> PiperCanV188Config: ...

@overload
def create_agx_arm_config(
    robot: Literal["piper"],
    comm: Literal["can"] = ...,
    firmeware_version: Literal["v189"] = ...,
    **kwargs: Any
) -> PiperCanV189Config: ...

# --- piper_h ---

@overload
def create_agx_arm_config(
    robot: Literal["piper_h"],
    comm: Literal["can"] = ...,
    firmeware_version: Literal["default"] = ...,
    **kwargs: Any
) -> PiperHCanDefaultConfig: ...

@overload
def create_agx_arm_config(
    robot: Literal["piper_h"],
    comm: Literal["can"] = ...,
    firmeware_version: Literal["v183"] = ...,
    **kwargs: Any
) -> PiperHCanV183Config: ...

@overload
def create_agx_arm_config(
    robot: Literal["piper_h"],
    comm: Literal["can"] = ...,
    firmeware_version: Literal["v188"] = ...,
    **kwargs: Any
) -> PiperHCanV188Config: ...

@overload
def create_agx_arm_config(
    robot: Literal["piper_h"],
    comm: Literal["can"] = ...,
    firmeware_version: Literal["v189"] = ...,
    **kwargs: Any
) -> PiperHCanV189Config: ...

# --- piper_l ---

@overload
def create_agx_arm_config(
    robot: Literal["piper_l"],
    comm: Literal["can"] = ...,
    firmeware_version: Literal["default"] = ...,
    **kwargs: Any
) -> PiperLCanDefaultConfig: ...

@overload
def create_agx_arm_config(
    robot: Literal["piper_l"],
    comm: Literal["can"] = ...,
    firmeware_version: Literal["v183"] = ...,
    **kwargs: Any
) -> PiperLCanV183Config: ...

@overload
def create_agx_arm_config(
    robot: Literal["piper_l"],
    comm: Literal["can"] = ...,
    firmeware_version: Literal["v188"] = ...,
    **kwargs: Any
) -> PiperLCanV188Config: ...

@overload
def create_agx_arm_config(
    robot: Literal["piper_l"],
    comm: Literal["can"] = ...,
    firmeware_version: Literal["v189"] = ...,
    **kwargs: Any
) -> PiperLCanV189Config: ...

# --- piper_x ---

@overload
def create_agx_arm_config(
    robot: Literal["piper_x"],
    comm: Literal["can"] = ...,
    firmeware_version: Literal["default"] = ...,
    **kwargs: Any
) -> PiperXCanDefaultConfig: ...

@overload
def create_agx_arm_config(
    robot: Literal["piper_x"],
    comm: Literal["can"] = ...,
    firmeware_version: Literal["v183"] = ...,
    **kwargs: Any
) -> PiperXCanV183Config: ...

@overload
def create_agx_arm_config(
    robot: Literal["piper_x"],
    comm: Literal["can"] = ...,
    firmeware_version: Literal["v188"] = ...,
    **kwargs: Any
) -> PiperXCanV188Config: ...

@overload
def create_agx_arm_config(
    robot: Literal["piper_x"],
    comm: Literal["can"] = ...,
    firmeware_version: Literal["v189"] = ...,
    **kwargs: Any
) -> PiperXCanV189Config: ...

# ---------- AgxArmFactory ----------

T = TypeVar("T", bound=Any)

class AgxArmFactory:

    @classmethod
    @overload
    def create_arm(cls, config: None, **kwargs) -> None:
        """
        Create a robotic arm Driver instance.
        """
        ...

    # --- nero ---

    @classmethod
    @overload
    def create_arm(cls, config: NeroCanDefaultConfig, **kwargs) -> NeroDriverDefault:
        """Nero CAN driver (default, firmware <= v110).

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
        ...

    @classmethod
    @overload
    def create_arm(cls, config: NeroCanV111Config, **kwargs) -> NeroDriverV111:
        """Nero CAN driver for firmware == v111."""
        ...

    @classmethod
    @overload
    def create_arm(cls, config: NeroCanV112Config, **kwargs) -> NeroDriverV112:
        """Nero CAN driver for firmware 1.12."""
        ...

    @classmethod
    @overload
    def create_arm(cls, config: NeroCanV120Config, **kwargs) -> NeroDriverV120:
        """Nero CAN driver for firmware >= v120 (1.20)."""
        ...

    # --- piper ---

    @classmethod
    @overload
    def create_arm(cls, config: PiperCanDefaultConfig, **kwargs) -> PiperDriverDefault:
        """Piper CAN driver (default, firmware <= v182).

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
        ...

    @classmethod
    @overload
    def create_arm(cls, config: PiperCanV183Config, **kwargs) -> PiperDriverV183:
        """Piper CAN driver for firmware v183~v187 (S-V1.8-3 ~ S-V1.8-7)."""
        ...

    @classmethod
    @overload
    def create_arm(cls, config: PiperCanV188Config, **kwargs) -> PiperDriverV188:
        """Piper CAN driver for firmware == v188 (S-V1.8-8)."""
        ...

    @classmethod
    @overload
    def create_arm(cls, config: PiperCanV189Config, **kwargs) -> PiperDriverV189:
        """Piper CAN driver for firmware >= v189 (S-V1.8-9)."""
        ...

    # --- piper_h ---

    @classmethod
    @overload
    def create_arm(cls, config: PiperHCanDefaultConfig, **kwargs) -> PiperHDriverDefault:
        """PiperH CAN driver (default, firmware <= v182).

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
        ...

    @classmethod
    @overload
    def create_arm(cls, config: PiperHCanV183Config, **kwargs) -> PiperHDriverV183:
        """PiperH CAN driver for firmware v183~v187 (S-V1.8-3 ~ S-V1.8-7)."""
        ...

    @classmethod
    @overload
    def create_arm(cls, config: PiperHCanV188Config, **kwargs) -> PiperHDriverV188:
        """PiperH CAN driver for firmware == v188 (S-V1.8-8)."""
        ...

    @classmethod
    @overload
    def create_arm(cls, config: PiperHCanV189Config, **kwargs) -> PiperHDriverV189:
        """PiperH CAN driver for firmware >= v189 (S-V1.8-9)."""
        ...

    # --- piper_l ---

    @classmethod
    @overload
    def create_arm(cls, config: PiperLCanDefaultConfig, **kwargs) -> PiperLDriverDefault:
        """PiperL CAN driver (default, firmware <= v182).

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
        ...

    @classmethod
    @overload
    def create_arm(cls, config: PiperLCanV183Config, **kwargs) -> PiperLDriverV183:
        """PiperL CAN driver for firmware v183~v187 (S-V1.8-3 ~ S-V1.8-7)."""
        ...

    @classmethod
    @overload
    def create_arm(cls, config: PiperLCanV188Config, **kwargs) -> PiperLDriverV188:
        """PiperL CAN driver for firmware == v188 (S-V1.8-8)."""
        ...

    @classmethod
    @overload
    def create_arm(cls, config: PiperLCanV189Config, **kwargs) -> PiperLDriverV189:
        """PiperL CAN driver for firmware >= v189 (S-V1.8-9)."""
        ...

    # --- piper_x ---

    @classmethod
    @overload
    def create_arm(cls, config: PiperXCanDefaultConfig, **kwargs) -> PiperXDriverDefault:
        """PiperX CAN driver (default, firmware <= v182).

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
        ...

    @classmethod
    @overload
    def create_arm(cls, config: PiperXCanV183Config, **kwargs) -> PiperXDriverV183:
        """PiperX CAN driver for firmware v183~v187 (S-V1.8-3 ~ S-V1.8-7)."""
        ...

    @classmethod
    @overload
    def create_arm(cls, config: PiperXCanV188Config, **kwargs) -> PiperXDriverV188:
        """PiperX CAN driver for firmware == v188 (S-V1.8-8)."""
        ...

    @classmethod
    @overload
    def create_arm(cls, config: PiperXCanV189Config, **kwargs) -> PiperXDriverV189:
        """PiperX CAN driver for firmware >= v189 (S-V1.8-9)."""
        ...
