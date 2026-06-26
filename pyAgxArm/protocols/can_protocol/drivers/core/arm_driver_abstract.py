import threading
from typing import Optional, TYPE_CHECKING, overload, List

from typing_extensions import Literal
from .arm_driver_interface import ArmDriverInterface
from .driver_context import DriverContext
from ...msgs.core import AttributeBase, MessageAbstract
from .protocol_parser_interface import ProtocolParserInterface
from .protocol_parser_abstract import DriverAPIOptions
from ..core.arm_driver_context import ArmDriverContext
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
    from ..effector.revo2_touch import Revo2TouchDriverDefault


class ArmDriverAbstract(ArmDriverInterface):
    _instances = {}
    _lock = threading.Lock()

    _JOINT_NUMS = 6
    _JOINT_INDEX_LIST = [i for i in range(1, _JOINT_NUMS + 1)] + [255]

    _Parser = ProtocolParserInterface

    @property
    def OPTIONS(self):
        return DriverAPIOptions

    def __init_subclass__(cls, **kwargs):
        super().__init_subclass__(**kwargs)
        cls._JOINT_INDEX_LIST = [i for i in range(1, cls._JOINT_NUMS + 1)] + [255]

    def __init__(self, config: dict):
        self._config = config.copy()
        self._ctx = DriverContext(config)
        self._connected = False
        self._effector_kind: Optional[str] = None
        self._effector = None
        self._parser = self._Parser(self._ctx.fps)
        self._arm_ctx = ArmDriverContext(config, self._ctx, self._parser)
        self._auto_set_motion_mode_enabled = True
        self._joint_limits_enabled = False

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
                    raise RuntimeError(
                        "Robot is not connected (comm is None). "
                        "Call `connect()` before sending commands."
                    )
                if not self._ctx.should_send_tx_frame(data):
                    return
                try:
                    comm.send(data)
                except Exception as exc:
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

    @overload
    def init_effector(
        self, effector: Literal["revo2_touch"]
    ) -> "Revo2TouchDriverDefault":
        """revo2 touch-hand end-effector driver.
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
        self._effector_kind = effector_kind

        if effector_kind == self.OPTIONS.EFFECTOR.AGX_GRIPPER:
            from ..effector.agx_gripper import AgxGripperDriverDefault

            self._effector = AgxGripperDriverDefault(self._config, self.get_context())
            return self._effector

        if effector_kind == self.OPTIONS.EFFECTOR.REVO2:
            from ..effector.revo2 import Revo2DriverDefault

            self._effector = Revo2DriverDefault(self._config, self.get_context())
            return self._effector

        if effector_kind == self.OPTIONS.EFFECTOR.REVO2_TOUCH:
            from ..effector.revo2_touch import Revo2TouchDriverDefault

            self._effector = Revo2TouchDriverDefault(self._config, self.get_context())
            return self._effector

        raise ValueError(f"Unsupported effector kind: {effector}")

    def get_driver_version(self):
        raise NotImplementedError

    def create_comm(self, config: Optional[dict] = None, comm: str = "can"):
        return self._ctx.create_comm(config, comm)

    def has_comm_error(self) -> bool:
        """
        Check whether the communication layer is currently in an error state.

        Returns:
            bool: `True` if a communication error has been recorded;
                otherwise `False`.
        """
        return self._ctx.has_comm_error()
    
    def get_comm_error(self):
        """
        Get the most recent communication error information.

        Returns:
            Any: The error object stored in the communication context.
                Usually returns `None` when there is no current error.
        """
        return self._ctx.get_comm_error()

    def connect(self, start_read_thread: bool = True) -> None:
        """
        Initialize and connect the underlying communication, optionally
        starting background reader/monitor/FPS threads.

        This method is idempotent: repeated calls while already connected
        will be ignored. Use `disconnect()` to stop threads and close comm
        when the driver instance is no longer needed.
        """
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

    def disconnect(self, join_timeout: float = 1.0) -> None:
        """
        Disconnect from the arm and release underlying threads and CAN resources.

        This method is idempotent and can be safely called when the arm
        instance is no longer needed, e.g. after reading firmware version
        and before creating a new instance.
        """
        with self._lock:
            if not self._connected and self._ctx.get_comm() is None:
                # Already disconnected and comm torn down
                return
            self._connected = False

        # Stop internal DriverContext threads and FPS manager, and close comm
        self._ctx.shutdown(join_timeout=join_timeout)

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

        fp = Validator.clamp_pose6(flange.msg, name="flange_pose")
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

    def get_auto_set_motion_mode_enabled(self) -> bool:
        """Get the auto set motion mode enabled state."""
        return self._auto_set_motion_mode_enabled

    def set_joint_limits_enabled(self, enabled: bool) -> None:
        """Enable/disable software joint-angle limits at runtime."""
        if not isinstance(enabled, bool):
            raise ValueError("`enabled` should be bool")
        self._joint_limits_enabled = enabled

    def get_joint_limits_enabled(self) -> bool:
        """Get the joint limits enabled state."""
        return self._joint_limits_enabled

