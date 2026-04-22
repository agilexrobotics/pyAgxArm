import threading
import weakref
from typing import Optional, TYPE_CHECKING, overload, List

from typing_extensions import Literal
from .arm_driver_interface import ArmDriverInterface
from .driver_context import DriverContext
from ...msgs.core import AttributeBase, MessageAbstract
from .protocol_parser_interface import ProtocolParserInterface
from .protocol_parser_abstract import DriverAPIOptions
from ..core.arm_driver_context import ArmDriverContext
from .....utiles.logger import Logger
from .....utiles.validator import Validator
from .....utiles.mdh_kinematics import (
    fk_from_mdh,
    get_mdh
)
from .....utiles.tf import (
    T16_to_pose6,
    inv_T16,
    matmul16_to,
    pose6_to_T16,
    pose6_to_T16_into,
)

if TYPE_CHECKING:
    from ..effector.agx_gripper import AgxGripperDriverDefault
    from ..effector.revo2 import Revo2DriverDefault


class ArmDriverAbstract(ArmDriverInterface):
    _instances = {}
    _lock = threading.Lock()

    _JOINT_NUMS = 6
    _JOINT_INDEX_LIST = [i for i in range(1, _JOINT_NUMS + 1)] + [255]

    _Parser = ProtocolParserInterface

    @staticmethod
    def _shutdown_ctx_on_finalize(
        ctx: DriverContext,
        log: Optional[Logger] = None,
    ) -> None:
        """Best-effort cleanup when driver instance is garbage collected."""
        try:
            if log is not None:
                log.shutdown()
            ctx.shutdown()
            ctx._parser_packet_fun_list.clear()
            ctx._data_monitor_fun_list.clear()
        except Exception:
            # Finalizer must never raise; explicit disconnect remains preferred.
            pass

    @property
    def OPTIONS(self):
        return DriverAPIOptions

    def __init_subclass__(cls, **kwargs):
        super().__init_subclass__(**kwargs)
        cls._JOINT_INDEX_LIST = [i for i in range(1, cls._JOINT_NUMS + 1)] + [255]

    def __init__(self, config: dict):
        self._config = config.copy()
        _id = format(id(self), "x")
        _robot = self._config["robot"]
        _comm_cfg = self._config["comm"]
        self._comm_type = _comm_cfg["type"]
        self._comm_channel = _comm_cfg[self._comm_type]["channel"]
        self.log = Logger("%s.%s.%s" % (_robot, self._comm_channel, _id))
        self._validator = Validator(logger_=self.log.get_child("validator"))
        self._ctx = DriverContext(config, logger_=self.log.get_child("ctx"))
        self._gc_finalizer = weakref.finalize(
            self,
            self._shutdown_ctx_on_finalize,
            self._ctx,
            self.log,
        )
        self._connected = False
        self._effector_kind: Optional[str] = None
        self._effector = None
        self._parser = self._Parser(
            self._ctx.fps,
            logger_=self.log.get_child("parser"),
        )
        self._arm_ctx = ArmDriverContext(config, self._ctx, self._parser)

        # CONFIG
        auto_set_motion_mode = self._config.get("auto_set_motion_mode", True)
        if not isinstance(auto_set_motion_mode, bool):
            raise ValueError("Config `auto_set_motion_mode` should be bool")
        self._auto_set_motion_mode_enabled = auto_set_motion_mode

        enable_joint_limits = self._config.get("enable_joint_limits", True)
        if not isinstance(enable_joint_limits, bool):
            raise ValueError("Config `enable_joint_limits` should be bool")
        self._joint_limits_enabled = enable_joint_limits

        # TCP
        self._tcp_offset_pose: List[float] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self._T_f_t = None
        self._T_t_f = None
        self._tcp_buf_a = [0.0] * 16
        self._tcp_buf_b = [0.0] * 16

        # MDH
        self._mdh = get_mdh(self._config.get("robot"))

    def _send_msg(self, msg: AttributeBase) -> None:
        """Send one control message.

        Parameters
        ----------
        `msg`: AttributeBase
        - The message to send.
        """
        if isinstance(msg, AttributeBase):
            data = self._parser.pack(msg)
            if data is not None:
                comm = self._ctx.get_comm()
                if comm is None:
                    self.log.warning(
                        "Failed to send %s: comm is None (driver not connected).",
                        type(msg).__name__,
                    )
                    raise RuntimeError(
                        "Robot is not connected (comm is None). "
                        "Call `connect()` before sending commands."
                    )
                try:
                    comm.send(data)
                except Exception as exc:
                    self.log.error(
                        "Failed to send %s on channel '%s' (%s: %s).",
                        type(msg).__name__,
                        comm.get_channel(),
                        type(exc).__name__,
                        exc,
                        exc_info=True,
                    )
                    raise RuntimeError(
                        f"Failed to send {type(msg).__name__} on channel '{comm.get_channel()}': {exc}"
                    ) from exc
        else:
            raise TypeError(
                "msg must be AttributeBase"
            )

    def _send_msgs(
        self,
        msgs: List[AttributeBase]
    ) -> None:
        """Send a sequence of control messages (with optional intervals).

        Parameters
        ----------
        `msgs`: list[AttributeBase]
        - The messages to send.
        """
        if isinstance(msgs, list):
            for i, msg in enumerate(msgs):
                self._send_msg(msg)
        else:
            raise TypeError(
                "msgs must be a list of AttributeBase"
            )

    @property
    def joint_nums(self):
        return self._JOINT_NUMS
    
    def get_context(self):
        return self._ctx

    @overload
    def init_effector(
        self, effector: None
    ) -> None:
        """Initialize end-effector driver exactly once and return the driver instance.

        Notes
        -----
        - This method is intentionally one-shot to avoid registering multiple
          callbacks into the same DriverContext threads.
        - If called again, it raises RuntimeError.
        """
        ...

    @overload
    def init_effector(
        self, effector: Literal["agx_gripper"]
    ) -> "AgxGripperDriverDefault":
        """agx_gripper end-effector driver.
        """
        ...

    @overload
    def init_effector(
        self, effector: Literal["revo2"]
    ) -> "Revo2DriverDefault":
        """revo2 end-effector driver.
        """
        ...

    def init_effector(self, effector: str):
        """Initialize end-effector driver exactly once and return the driver instance.
        """
        if self._effector_kind is not None:
            raise RuntimeError(
                f"effector already initialized: {self._effector_kind}. "
                "Create a new robotic arm instance if you need a different effector."
            )

        effector_kind = str(effector).strip().lower()
        try:
            if effector_kind == self.OPTIONS.EFFECTOR.AGX_GRIPPER:
                from ..effector.agx_gripper import AgxGripperDriverDefault

                effector_driver = AgxGripperDriverDefault(
                    self._config,
                    self.get_context(),
                )
            elif effector_kind == self.OPTIONS.EFFECTOR.REVO2:
                from ..effector.revo2 import Revo2DriverDefault

                effector_driver = Revo2DriverDefault(
                    self._config,
                    self.get_context(),
                )
            else:
                raise ValueError(f"Unsupported effector kind: {effector}")
        except Exception as exc:
            self.log.error(
                "Init effector '%s' failed (%s: %s).",
                effector_kind,
                type(exc).__name__,
                exc,
                exc_info=True,
            )
            raise

        self._effector_kind = effector_kind
        self._effector = effector_driver
        return self._effector

    def get_driver_version(self):
        raise NotImplementedError

    def create_comm(self, config: Optional[dict] = None, comm: str = "can"):
        return self._ctx.create_comm(config, comm)

    def has_comm_error(self) -> bool:
        return self._ctx.has_comm_error()
    
    def get_comm_error(self):
        return self._ctx.get_comm_error()

    def connect(self, start_read_thread: bool = True) -> None:
        """
        Initialize and connect the underlying communication, optionally
        starting background reader/monitor/FPS threads.

        This method is idempotent: repeated calls while already connected
        will be ignored. Use `disconnect()` to stop threads and close comm
        when the driver instance is no longer needed.
        """
        try:
            if self._ctx.has_comm_error():
                self.disconnect()
            comm = self._ctx.get_comm()
            if comm is None:
                comm = self._ctx.init_comm()
            if comm is None:
                raise ValueError("comm is None")
            if not comm.is_connected():
                comm.connect()
            if not comm.is_connected():
                raise RuntimeError("Failed to establish robot communication.")
            with self._lock:
                if self._connected:
                    return
                self._connected = True
            if start_read_thread:
                self._ctx.start_th()
        except Exception as exc:
            self.log.error(
                "Connect failed for %s channel '%s' (%s: %s).",
                self._comm_type,
                self._comm_channel,
                type(exc).__name__,
                exc,
                exc_info=True,
            )
            raise
        self.log.info(
            "Connected to %s channel '%s'.",
            self._comm_type,
            self._comm_channel,
        )

    def disconnect(self, join_timeout: float = 1.0) -> None:
        """
        Disconnect from the arm and release underlying threads and CAN resources.

        This method is idempotent and can be safely called when the arm
        instance is no longer needed, e.g. after reading firmware version
        and before creating a new instance.
        """
        with self._lock:
            already_disconnected = (not self._connected and self._ctx.get_comm() is None)
            self._connected = False

        if already_disconnected:
            self.log.shutdown()
            return

        try:
            self._ctx.shutdown(join_timeout=join_timeout)
        except Exception as exc:
            self.log.error(
                "Disconnect failed for %s channel '%s' (%s: %s).",
                self._comm_type,
                self._comm_channel,
                type(exc).__name__,
                exc,
                exc_info=True,
            )
            self.log.shutdown()
            raise
        self.log.info(
            "Disconnected from %s channel '%s'.",
            self._comm_type,
            self._comm_channel,
        )
        self.log.shutdown()

    def reconnect(self, join_timeout: float = 1.0, start_read_thread: bool = True) -> None:
        """
        Explicitly rebuild the current connection session.

        This is the recommended recovery path after a communication error or
        device reconnect event: fully disconnect the current session first,
        then create a fresh connection and restart background threads.
        """
        self.disconnect(join_timeout=join_timeout)
        self.connect(start_read_thread=start_read_thread)

    def is_connected(self) -> bool:
        comm = self._ctx.get_comm()
        if comm is None:
            return False
        return comm.is_connected()

    def is_ok(self):
        return self._arm_ctx.is_ok()

    def get_fps(self):
        return self._arm_ctx.get_fps()

    def get_config(self) -> dict:
        return self._config

    def get_type(self):
        comm = self._ctx.get_comm()
        if comm is None:
            return None
        return comm.get_type()

    def get_channel(self):
        comm = self._ctx.get_comm()
        if comm is None:
            return None
        return comm.get_channel()

    def get_joint_angles(self):
        raise NotImplementedError

    def get_flange_pose(self):
        raise NotImplementedError

    def get_arm_status(self):
        raise NotImplementedError

    def get_driver_states(self):
        raise NotImplementedError

    def get_motor_states(self):
        raise NotImplementedError

    def enable(self):
        raise NotImplementedError

    def disable(self):
        raise NotImplementedError

    def reset(self):
        raise NotImplementedError

    def electronic_emergency_stop(self):
        raise NotImplementedError

    # -------------------------- TCP --------------------------

    def _world_flange_to_tcp_noclamp(self, flange_pose: List[float]) -> List[float]:
        """``flange_pose`` must already satisfy ``Validator.clamp_pose6``."""
        if self._T_f_t is None:
            return flange_pose
        pose6_to_T16_into(self._tcp_buf_a, flange_pose)
        matmul16_to(self._tcp_buf_b, self._tcp_buf_a, self._T_f_t)
        return T16_to_pose6(self._tcp_buf_b)

    def _world_tcp_to_flange_noclamp(self, tcp_pose: List[float]) -> List[float]:
        """``tcp_pose`` must already satisfy ``Validator.clamp_pose6``."""
        if self._T_t_f is None:
            return tcp_pose
        pose6_to_T16_into(self._tcp_buf_a, tcp_pose)
        matmul16_to(self._tcp_buf_b, self._tcp_buf_a, self._T_t_f)
        return T16_to_pose6(self._tcp_buf_b)

    def set_tcp_offset(self, pose: List[float]):
        """Set TCP offset pose in the flange frame.

        Parameters
        ----------
        `pose`: list[float]
        - `[x, y, z, roll, pitch, yaw]` in flange frame.
        - `x, y, z`: meters.
        - `roll, pitch, yaw`: radians (Euler angles around `X/Y/Z`).
          - `roll`, `yaw` must be within `[-pi, pi]`
          - `pitch` must be within `[-pi/2, pi/2]`
        """
        self._tcp_offset_pose = Validator.clamp_pose6(
            pose,
            name="tcp_offset_pose"
        )
        if all(x == 0.0 for x in self._tcp_offset_pose):
            self._T_f_t = None
            self._T_t_f = None
        else:
            self._T_f_t = pose6_to_T16(self._tcp_offset_pose)
            self._T_t_f = inv_T16(self._T_f_t)

    def get_tcp_pose(self):
        """Get TCP pose by applying the configured TCP offset to the flange pose.

        Returns
        -------
        MessageAbstract[list[float]] | None
            `msg`: `[x, y, z, roll, pitch, yaw]` (TCP pose in base frame)
        """
        flange: Optional[MessageAbstract] = self.get_flange_pose()
        if flange is None:
            return None

        fp = Validator.clamp_pose6(
            flange.msg,
            name="flange_pose"
        )
        return MessageAbstract(
            msg_type="tcp_pose",
            msg=self._world_flange_to_tcp_noclamp(fp),
            timestamp=flange.timestamp,
            hz=flange.hz,
        )

    def get_flange2tcp_pose(self, flange_pose: List[float]):
        """Convert a flange pose (base frame) to the corresponding TCP pose.

        Parameters
        ----------
        `flange_pose`: `[x, y, z, roll, pitch, yaw]`
        - `x, y, z`: meters.
        - `roll, pitch, yaw`: radians (Euler angles around `X/Y/Z`).
          - `roll`, `yaw` must be within `[-pi, pi]`
          - `pitch` must be within `[-pi/2, pi/2]`

        Returns
        -------
        `tcp_pose`: `[x, y, z, roll, pitch, yaw]`
        - `x, y, z`: meters.
        - `roll, pitch, yaw`: radians (Euler angles around `X/Y/Z`).
          - `roll`, `yaw` must be within `[-pi, pi]`
          - `pitch` must be within `[-pi/2, pi/2]`

        Examples
        --------
        >>> robot.set_tcp_offset([0, 0, 0.1, 0, 0, 0])
        >>> flange_pose = robot.get_flange_pose()
        >>> if flange_pose is not None:
        >>>     print(robot.get_flange2tcp_pose(flange_pose.msg))
        """
        flange_pose = Validator.clamp_pose6(
            flange_pose,
            name="flange_pose"
        )
        return self._world_flange_to_tcp_noclamp(flange_pose)

    def get_tcp2flange_pose(self, tcp_pose: List[float]):
        """Convert a target TCP pose (base frame) to the corresponding flange pose.

        Notes
        -----
        If you call:
            `flange_pose = robot.get_tcp2flange_pose(target_tcp_pose)`

            `robot.move_p(flange_pose)`

        the TCP will move to `target_tcp_pose` (subject to kinematics and controller).
        """
        tcp_pose = Validator.clamp_pose6(
            tcp_pose,
            name="tcp_pose"
        )
        return self._world_tcp_to_flange_noclamp(tcp_pose)

    # -------------------------- FK --------------------------

    def fk(self, joint_angles: List[float]):
        """Forward kinematics (modified DH) to flange pose.

        Parameters
        ----------
        joint_angles :
            Joint angles in radians
            (``joint_1`` … ``joint_n``).

        Returns
        -------
        list[float]
            ``[x, y, z, roll, pitch, yaw]`` — position in meters, Euler angles in radians.

        Examples
        --------
        >>> ja = robot.get_joint_angles()
        >>> if ja is not None:
        ...     fp = robot.fk(ja.msg)
        ...     x, y, z, roll, pitch, yaw = fp
        """
        return fk_from_mdh(self._mdh, joint_angles)

    # -------------------------- CONFIG --------------------------

    def set_auto_set_motion_mode_enabled(self, enabled: bool) -> None:
        """Enable/disable auto motion-mode switching at runtime."""
        if not isinstance(enabled, bool):
            raise ValueError("`enabled` should be bool")
        self._auto_set_motion_mode_enabled = enabled
        self._config["auto_set_motion_mode"] = enabled

    def set_joint_limits_enabled(self, enabled: bool) -> None:
        """Enable/disable software joint-angle limits at runtime."""
        if not isinstance(enabled, bool):
            raise ValueError("`enabled` should be bool")
        self._joint_limits_enabled = enabled
        self._config["enable_joint_limits"] = enabled
