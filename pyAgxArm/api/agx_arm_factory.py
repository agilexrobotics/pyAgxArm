import copy
import hashlib
import inspect
import json
import threading
import time
import weakref
from typing import Any, Dict, List, Optional, Type, TypeVar
from typing_extensions import Literal
from .constants import ROBOT_OPTION_FIELDS, ROBOT_JOINT_LIMIT_PRESET, ROBOT_JOINT_NAME
from ..protocols.can_protocol.comms import *
from ..protocols.can_protocol.drivers import (
    NeroDriverDefault,
    NeroDriverV111,
    PiperDriverDefault,
    PiperDriverV183,
    PiperDriverV188,
    PiperHDriverDefault,
    PiperHDriverV183,
    PiperHDriverV188,
    PiperLDriverDefault,
    PiperLDriverV183,
    PiperLDriverV188,
    PiperXDriverDefault,
    PiperXDriverV183,
    PiperXDriverV188,
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
            PiperFW.V188     # firmware ≥ S-V1.8-8

        **Nero series** — ``NeroFW``::

            from pyAgxArm import NeroFW
            NeroFW.DEFAULT   # firmware ≤ 1.10
            NeroFW.V111      # firmware ≥ 1.11

        Raw strings (``"default"`` / ``"v183"`` / ``"v188"`` / ``"v111"``) are also accepted.

    **kwargs
        Additional keyword arguments forwarded to the comm layer
        (e.g. ``channel``, ``interface``, ``bitrate``), and robot options
        (e.g. ``joint_limits``, ``auto_set_motion_mode``).
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
            },
        },
        "nero": {
            "can": {
                "default": NeroDriverDefault,
                "v111": NeroDriverV111,
            },
        },
        "piper_h": {
            "can": {
                "default": PiperHDriverDefault,
                "v183": PiperHDriverV183,
                "v188": PiperHDriverV188,
            },
        },
        "piper_l": {
            "can": {
                "default": PiperLDriverDefault,
                "v183": PiperLDriverV183,
                "v188": PiperLDriverV188,
            },
        },
        "piper_x": {
            "can": {
                "default": PiperXDriverDefault,
                "v183": PiperXDriverV183,
                "v188": PiperXDriverV188,
            },
        },
    }
    _instance_cache: Dict[str, Dict[str, Any]] = {}
    _cache_lock = threading.RLock()
    _reuse_policy: Literal["new", "reuse", "replace"] = "replace"

    # -------------------------------------------------
    @classmethod
    def set_reuse_policy(cls, reuse_policy: Literal["new", "reuse", "replace"]) -> None:
        """Set global instance reuse policy used by create_arm.
        
        Parameters
        ----------
        reuse_policy : Literal["new", "reuse", "replace"]
            - "new": always create a new instance and refresh cache entry.
            - "reuse": return cached live instance if available.
            - "replace": disconnect cached live instance first, then create new.
        """
        if reuse_policy not in {"new", "reuse", "replace"}:
            raise ValueError(
                f"Invalid reuse_policy={reuse_policy!r}. "
                "Expected one of: 'new', 'reuse', 'replace'."
            )
        with cls._cache_lock:
            cls._reuse_policy = reuse_policy

    @classmethod
    def get_reuse_policy(cls) -> Literal["new", "reuse", "replace"]:
        """Get current global instance reuse policy.
        
        Returns
        -------
        Literal["new", "reuse", "replace"]
            - "new": always create a new instance and refresh cache entry.
            - "reuse": return cached live instance if available.
            - "replace": disconnect cached live instance first, then create new.
        """
        with cls._cache_lock:
            return cls._reuse_policy

    # -------------------------------------------------
    @classmethod
    def _normalize_for_fingerprint(cls, value: Any) -> Any:
        """Normalize config values into JSON-serializable structures."""
        if isinstance(value, dict):
            return {
                str(k): cls._normalize_for_fingerprint(v)
                for k, v in value.items()
            }
        if isinstance(value, (list, tuple)):
            return [cls._normalize_for_fingerprint(v) for v in value]
        if isinstance(value, set):
            normalized = [cls._normalize_for_fingerprint(v) for v in value]
            return sorted(normalized, key=lambda item: repr(item))
        if isinstance(value, (str, int, float, bool)) or value is None:
            return value
        return repr(value)

    @classmethod
    def _fingerprint_config(cls, config: dict) -> str:
        """Build a stable hash fingerprint from full config content."""
        normalized = cls._normalize_for_fingerprint(config)
        serialized = json.dumps(
            normalized,
            sort_keys=True,
            ensure_ascii=True,
            separators=(",", ":"),
        )
        return hashlib.sha256(serialized.encode("utf-8")).hexdigest()

    @classmethod
    def _purge_cache_entry(cls, fingerprint: str, expected_ref: Optional[weakref.ref] = None) -> None:
        with cls._cache_lock:
            cache_entry = cls._instance_cache.get(fingerprint)
            if cache_entry is None:
                return
            if expected_ref is not None and cache_entry.get("instance_ref") is not expected_ref:
                return
            cls._instance_cache.pop(fingerprint, None)

    @classmethod
    def _get_cached_instance(cls, config: dict) -> Optional[T]:
        """Get live cached arm instance by config, or None."""
        fingerprint = cls._fingerprint_config(config)
        with cls._cache_lock:
            cache_entry = cls._instance_cache.get(fingerprint)
            if cache_entry is None:
                return None
            instance_ref: Optional[weakref.ref] = cache_entry.get("instance_ref")
            if instance_ref is None:
                cls._instance_cache.pop(fingerprint, None)
                return None
            instance = instance_ref()
            if instance is None:
                cls._instance_cache.pop(fingerprint, None)
                return None
            return instance

    @classmethod
    def _cache_instance(cls, config: dict, instance: T) -> str:
        """Cache arm instance with weakref and finalize cleanup."""
        fingerprint = cls._fingerprint_config(config)
        instance_ref = weakref.ref(instance)
        finalizer = weakref.finalize(
            instance,
            cls._purge_cache_entry,
            fingerprint,
            instance_ref,
        )
        with cls._cache_lock:
            cls._instance_cache[fingerprint] = {
                "instance_ref": instance_ref,
                "created_at": time.time(),
                "finalizer": finalizer,
            }
        return fingerprint

    # -------------------------------------------------
    @classmethod
    def detect_can_configs(
        cls,
        interfaces: Any = None,
        *,
        timeout: float = 5.0,
    ) -> List[Dict[str, Any]]:
        """
        Probe CAN backends using python-can ``detect_available_configs``.

        Returns a list of plain dicts (typically ``interface``, ``channel``, …).
        Returns an empty list if the installed python-can has no
        ``detect_available_configs`` API. If the API exists but enumerates no
        adapters, the list is also empty.

        Raises
        ------
        Exception
            Any exception raised by ``detect_available_configs`` (e.g. backend
            errors) is propagated to the caller.
        """
        import can

        detect_fn = getattr(can, "detect_available_configs", None)
        if detect_fn is None:
            print(
                "[AgxArmFactory.detect_can_configs] python-can has no "
                "detect_available_configs; returning empty list."
            )
            return []

        out: List[Dict[str, Any]] = []
        for item in detect_fn(interfaces=interfaces, timeout=timeout):
            if isinstance(item, dict):
                out.append(dict(item))
                continue
            asdict = getattr(item, "_asdict", None)
            if callable(asdict):
                out.append(dict(asdict()))
            else:
                out.append({"value": repr(item)})
        return out

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
            Piper 系列: default / v183 / v188
            Nero 系列 : default / v111
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
        reuse_policy = cls.get_reuse_policy()

        cached_instance: Optional[T] = None
        if reuse_policy in {"reuse", "replace"}:
            cached_instance = cls._get_cached_instance(config)
            if reuse_policy == "reuse" and cached_instance is not None:
                return cached_instance

        if reuse_policy == "replace" and cached_instance is not None:
            disconnect = getattr(cached_instance, "disconnect", None)
            if callable(disconnect):
                disconnect()

        arm_cls: Type[T] = cls.load_class(config)
        arm_instance = arm_cls(config=config, **kwargs)
        cls._cache_instance(config, arm_instance)
        return arm_instance
