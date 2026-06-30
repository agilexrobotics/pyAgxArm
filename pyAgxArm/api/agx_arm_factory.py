import copy
import inspect
from typing import Type, Dict, TypeVar
from typing_extensions import Literal
from .constants import (
    ROBOT_OPTION_FIELDS,
    ROBOT_JOINT_LIMIT_PRESET,
    ROBOT_JOINT_NAME,
    ROBOT_JOINT_TORQUE_K,
    ROBOT_JOINT_TORQUE_B,
)
from ..protocols.can_protocol.comms import *
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


def extract_kwargs(func, source: dict) -> dict:
    sig = inspect.signature(func)
    return {
        k: source[k]
        for k in sig.parameters.keys()
        if k in source
    }


def create_agx_arm_config(
        robot: Literal["nero", "piper", "piper_h", "piper_l", "piper_x"],
        comm: Literal["can"] = "can",
        firmeware_version: str = "default",
        **kwargs):
    """Generate the configuration dictionary required by the robotic arm.

    Parameters
    ----------
    robot : str
        Robotic arm model. Use ``ArmModel`` constants for IDE hints::

            from pyAgxArm import ArmModel
            ArmModel.PIPER  / ArmModel.PIPER_H / ArmModel.PIPER_L
            ArmModel.PIPER_X / ArmModel.NERO

    comm : str
        Communication type. Currently only ``"can"`` is supported.
    firmeware_version : str
        Main controller firmware version. Use per-robot-series
        constants for IDE hints:

        **Piper series** (piper / piper_h / piper_l / piper_x) — ``PiperFW``::

            from pyAgxArm import PiperFW
            PiperFW.DEFAULT  # firmware ≤ S-V1.8-2
            PiperFW.V183     # firmware S-V1.8-3 ~ S-V1.8-7
            PiperFW.V188     # firmware = S-V1.8-8
            PiperFW.V189     # firmware ≥ S-V1.8-9

        **Nero series** — ``NeroFW``::

            from pyAgxArm import NeroFW
            NeroFW.DEFAULT   # firmware ≤ 1.10
            NeroFW.V111      # firmware = 1.11
            NeroFW.V112      # firmware 1.12
            NeroFW.V120      # firmware ≥ 1.20

        String literals equal to ``PiperFW`` / ``NeroFW`` member values are also
        accepted (backward-compatible; see ``arm_options.py`` for the full list).

    **kwargs
        Additional keyword arguments forwarded to the comm layer
        (e.g. ``channel``, ``interface``, ``bitrate``), and robot options
        (e.g. ``joint_limits``).
    """
    config = {
        "robot": robot,
        "firmeware_version": firmeware_version,
        "log": {
            "level": kwargs.get("log_level", "INFO"),
            "path": kwargs.get("log_path", ""),
        },
    }

    # ---------- robot-specific options ----------
    allowed_fields = ROBOT_OPTION_FIELDS.get(robot, set())

    for field in allowed_fields:
        if field in kwargs:
            config[field] = kwargs[field]
    # ---------- joint name ----------
    config["joint_names"] = ROBOT_JOINT_NAME.get(robot)
    # ---------- joint torque k ----------
    config["joint_torque_k"] = ROBOT_JOINT_TORQUE_K.get(
        robot, ROBOT_JOINT_TORQUE_K["piper"]
    )
    # ---------- joint torque b ----------
    config["joint_torque_b"] = ROBOT_JOINT_TORQUE_B.get(
        robot, ROBOT_JOINT_TORQUE_B["piper"]
    )
    # ---------- joint limit ----------
    preset_joint_limits = ROBOT_JOINT_LIMIT_PRESET.get(robot)
    if preset_joint_limits is None:
        raise ValueError(f"No joint limit preset for robot={robot}")

    # 使用深拷贝，避免污染全局 preset
    final_joint_limits = copy.deepcopy(preset_joint_limits)

    user_joint_limits = kwargs.get("joint_limits")
    if user_joint_limits is not None:
        if not isinstance(user_joint_limits, dict):
            raise TypeError("joint_limits must be a dict")

        for joint, limit in user_joint_limits.items():
            if joint not in final_joint_limits:
                raise ValueError(f"Invalid joint name: {joint}")
            if not (isinstance(limit, (list, tuple)) and len(limit) == 2):
                raise ValueError(f"Invalid limit format for {joint}")
            final_joint_limits[joint] = list(limit)

    config["joint_limits"] = final_joint_limits
    # ---------- comm ----------
    if comm == "can":
        config["comm"] = {
            "type": "can",
            "can": create_can_comm_config(
                    **extract_kwargs(create_can_comm_config, kwargs)
            ),
        }
    else:
        raise ValueError(f"Unsupported comm type: {comm}")

    return config


T = TypeVar("T")


class AgxArmFactory:

    _registry: Dict[str, Dict[str, Dict[str, Type]]] = {
        "piper": {
            "can": {
                "default": PiperDriverDefault,
                "v183": PiperDriverV183,
                "v188": PiperDriverV188,
                "v189": PiperDriverV189,
            },
        },
        "nero": {
            "can": {
                "default": NeroDriverDefault,
                "v111": NeroDriverV111,
                "v112": NeroDriverV112,
                "v120": NeroDriverV120,
            },
        },
        "piper_h": {
            "can": {
                "default": PiperHDriverDefault,
                "v183": PiperHDriverV183,
                "v188": PiperHDriverV188,
                "v189": PiperHDriverV189,
            },
        },
        "piper_l": {
            "can": {
                "default": PiperLDriverDefault,
                "v183": PiperLDriverV183,
                "v188": PiperLDriverV188,
                "v189": PiperLDriverV189,
            },
        },
        "piper_x": {
            "can": {
                "default": PiperXDriverDefault,
                "v183": PiperXDriverV183,
                "v188": PiperXDriverV188,
                "v189": PiperXDriverV189,
            },
        },
    }

    # -------------------------------------------------
    @classmethod
    def register_arm(
        cls,
        *,
        robot: str,
        comm: str,
        firmeware_version: str,
        driver_cls: Type,
    ) -> None:
        """
        注册 Driver

        robot   : piper / nero / piper_h / piper_l / piper_x
        comm    : can
        firmeware_version :
            Piper 系列: default / v183 / v188 / v189
            Nero 系列 : default / v111 / v112 / v120
        """
        cls._registry.setdefault(robot, {})
        cls._registry[robot].setdefault(comm, {})
        cls._registry[robot][comm][firmeware_version] = driver_cls

    # -------------------------------------------------
    @classmethod
    def load_class(cls, config: dict) -> Type:
        """
        根据 config 获取 Driver 类（不实例化）
        """
        robot = config["robot"]
        comm = config["comm"]["type"]
        firmeware_version = config.get("firmeware_version", "default")

        try:
            return cls._registry[robot][comm][firmeware_version]
        except KeyError as e:
            raise KeyError(
                f"Driver not registered: robot={robot}, comm={comm}, version={firmeware_version}"
            ) from e

    # -------------------------------------------------
    @classmethod
    def create_arm(cls, config: dict, **kwargs) -> T:
        """
        Create a robotic arm Driver instance.
        """
        arm_cls: Type[T] = cls.load_class(config)
        return arm_cls(config=config, **kwargs)
