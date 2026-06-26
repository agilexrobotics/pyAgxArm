import time
import threading
from typing import Callable, TypeVar, Optional, Dict

import can
from ...comms import CommsFactory, create_comm_config
from .....utiles.fps import FPSManager


T = TypeVar('T')


class DriverContext:
    def __init__(self, config: dict):
        self._config = config.copy()
        self.comm = None

        self.fps = FPSManager()
        self.fps.set_cal_fps_time_interval(0.1)

        self._parser_packet_fun_list = []
        self._data_monitor_fun_list = []
        self._read_stop_event = threading.Event()
        self._monitor_stop_event = threading.Event()

        self._read_th = None
        self._monitor_th = None

        self._comm_initialized = False

        # Stamp dictionary for request throttling (used by _request_and_get).
        # Key is an arbitrary string (e.g. "firmware", "joint_acc:3").
        self._req_stamp: Dict[str, float] = {}
        self._tx_repeat_lock = threading.Lock()
        self._tx_repeat_min_interval: Dict[int, float] = {
            0x151: 0.1,
        }
        self._tx_last_payload: Dict[int, bytes] = {}
        self._tx_last_stamp: Dict[int, float] = {}

    def has_comm_error(self) -> bool:
        return bool(self.comm is not None and getattr(self.comm, "last_error", None) is not None)
    
    def get_comm_error(self):
        if self.comm is None:
            return None
        return getattr(self.comm, "last_error", None)

    def _resolve_comm_config(self, config: Optional[dict], comm: str) -> dict:
        source = config if config is not None else self._config
        if not source:
            return create_comm_config(comm)

        if "comm" in source:
            comm_cfg = source["comm"]
            if not isinstance(comm_cfg, dict):
                raise TypeError("config['comm'] must be a dict")
            actual_comm = comm_cfg.get("type", comm)
            if actual_comm != comm:
                raise ValueError(
                    f"Comm config type mismatch: expected '{comm}', got '{actual_comm}'."
                )
            if comm not in comm_cfg or not isinstance(comm_cfg[comm], dict):
                raise KeyError(f"Missing comm config payload for '{comm}'.")
            return comm_cfg[comm].copy()

        return source.copy()

    def create_comm(self, config: Optional[dict] = None, comm: str = "can"):
        cfg = self._resolve_comm_config(config, comm)
        self.comm = CommsFactory.create_comm(comm, "impl", config=cfg, comm_type=comm)
        if self.comm is None:
            raise RuntimeError(f"Failed to create {comm} communication instance.")
        self.comm.set_callback(self._run_parser_packet_funs)
        if hasattr(self.comm, "last_error"):
            self.comm.last_error = None
        self._comm_initialized = True
        return self.comm

    def init_comm(self):
        if self._comm_initialized:
            return self.comm
        comm_cfg = self._config["comm"]
        comm_type = comm_cfg["type"]
        comm = None
        try:
            comm = CommsFactory.create_comm(
                comm_type,
                "impl",
                config=comm_cfg[comm_type],
                comm_type=comm_type
            )
            if comm is None:
                raise RuntimeError(f"Failed to create {comm_type} communication instance.")
            comm.set_callback(self._run_parser_packet_funs)
            if not comm.is_connected():
                comm.connect()
            if not comm.is_connected():
                raise RuntimeError(f"Failed to connect {comm_type} communication instance.")
        except Exception:
            if comm is not None:
                try:
                    comm.close()
                except Exception:
                    pass
            self.comm = None
            self._comm_initialized = False
            raise

        self.comm = comm
        if hasattr(self.comm, "last_error"):
            self.comm.last_error = None
        self._comm_initialized = True
        return self.comm

    def is_comm_init(self):
        return self._comm_initialized

    def get_comm(self):
        return self.comm

    def register_parser_packet_fun(self, cb):
        self._parser_packet_fun_list.append(cb)

    def _run_parser_packet_funs(self, rx_data):
        for cb in self._parser_packet_fun_list:
            cb(rx_data)

    def register_data_monitor_fun(self, cb):
        self._data_monitor_fun_list.append(cb)

    def _run_data_monitor_funs(self):
        for cb in self._data_monitor_fun_list:
            cb()

    def start_th(self):
        if not self.comm:
            raise NotImplementedError("comm is None!! Please init comm")

        self._read_stop_event.clear()
        self._monitor_stop_event.clear()
        if hasattr(self.comm, "last_error"):
            self.comm.last_error = None

        self._read_th = threading.Thread(
            target=self._read_loop, daemon=True
        )
        self._read_th.start()

        self._monitor_th = threading.Thread(
            target=self._monitor_loop, daemon=True
        )
        self._monitor_th.start()

        self.fps.start()

    def stop_th(self, join_timeout: float = 1.0):
        """
        Stop internal reader/monitor threads and FPS manager.

        This method is idempotent and can be safely called multiple times.
        """
        # Signal threads to exit
        self._read_stop_event.set()
        self._monitor_stop_event.set()

        # Join threads if they were started
        if self._read_th is not None and self._read_th.is_alive():
            self._read_th.join(timeout=join_timeout)
        if self._monitor_th is not None and self._monitor_th.is_alive():
            self._monitor_th.join(timeout=join_timeout)

        self._read_th = None
        self._monitor_th = None

        # Stop FPS manager thread
        self.fps.stop()

    def shutdown(self, join_timeout: float = 1.0):
        """
        Fully shut down communication and clean up threads/resources.

        - Stop internal threads and FPSManager
        - Close underlying comm callback and session resources
        """
        # Stop internal threads first
        self.stop_th(join_timeout=join_timeout)

        # Then close communication resources
        if self.comm is not None:
            try:
                # Clear callback to avoid dangling references
                if hasattr(self.comm, "clear_callback"):
                    self.comm.clear_callback()
            except Exception:
                # Ignore callback clear failure, do not block shutdown
                pass

            try:
                self.comm.close()
            except Exception:
                # Ignore close errors from underlying comm layer
                pass

            self.comm = None
            self._comm_initialized = False
        self._req_stamp.clear()
        with self._tx_repeat_lock:
            self._tx_last_payload.clear()
            self._tx_last_stamp.clear()

    def _read_loop(self):
        while not self._read_stop_event.is_set():
            try:
                self.comm.recv()
            except Exception as exc:
                self._read_stop_event.set()
                self._monitor_stop_event.set()
                raise exc

    def _monitor_loop(self):
        while not self._monitor_stop_event.is_set():
            self._run_data_monitor_funs()
            time.sleep(0.05)

    # -------------------------
    # Common "request_and_get" helpers
    # -------------------------
    def _validate_timeout(self, timeout: float) -> None:
        """Validate common `timeout` argument."""
        if timeout < 0.0:
            raise ValueError("Timeout should be greater than 0.0")

    def _validate_min_interval(self, min_interval: float) -> None:
        """Validate common `min_interval` argument."""
        if min_interval < 0.0:
            raise ValueError("min_interval should be greater than or equal to 0.0")

    def _wait_for_response(self, func: Callable, timeout: float = 1.0):
        """
        Poll `func()` until it returns truthy or timeout expires.

        Returns
        -------
        bool
            True if `func()` returned True before timeout, False otherwise.
        """
        self._validate_timeout(timeout)
        if self.has_comm_error():
            return False

        if timeout == 0.0:
            # Non-blocking mode.
            return func()

        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.has_comm_error():
                return False
            if func():
                return True
            time.sleep(0.0005)
        return False

    def _request_and_get(
        self,
        *,
        request: Callable[[], None],
        is_ready: Callable[[], bool],
        get_value: Callable[[], T],
        clear: Optional[Callable[[], None]]= None,
        timeout: float = 1.0,
        min_interval: float = 0.0,
        stamp_attr: str,
    ):
        """
        Compose "request → wait → read cached value → clear cache" into one template.

        This is intended for query/config APIs that follow a request/response pattern:
        - send a request frame (once per `min_interval`)
        - wait for parser-side cache to be updated (`_wait_for_response`)
        - read and optionally clear cached data

        Notes
        -----
        - `timeout == 0.0` runs `is_ready()` once and returns immediately.
        - This function does NOT change the waiting mechanism; it reuses the existing
          polling implementation in `_wait_for_response`.
        """
        self._validate_timeout(timeout)
        self._validate_min_interval(min_interval)

        def throttle_request() -> None:
            last_ts = self._req_stamp.get(stamp_attr, 0.0)
            now = time.time()
            if now - last_ts >= min_interval:
                request()
                self._req_stamp[stamp_attr] = now

        throttle_request()
        if not self._wait_for_response(is_ready, timeout):
            return None

        value = get_value()
        if clear is not None:
            clear()
        self._req_stamp[stamp_attr] = 0.0
        return value

    # -------------------------
    # TX repeat-frame throttling
    # -------------------------
    def set_tx_repeat_min_interval(self, can_id: int, min_interval: float) -> None:
        """Set the minimum resend interval for identical frames on a CAN ID."""
        self._validate_min_interval(min_interval)
        with self._tx_repeat_lock:
            if min_interval == 0.0:
                self._tx_repeat_min_interval.pop(can_id, None)
            else:
                self._tx_repeat_min_interval[can_id] = min_interval

    def should_send_tx_frame(self, frame: can.Message) -> bool:
        """Return whether a TX frame should be sent after duplicate throttling."""
        min_interval = self._tx_repeat_min_interval.get(frame.arbitration_id, 0.0)
        if min_interval <= 0.0:
            return True

        payload = bytes(frame.data)
        now = time.monotonic()
        with self._tx_repeat_lock:
            last_payload = self._tx_last_payload.get(frame.arbitration_id)
            last_stamp = self._tx_last_stamp.get(frame.arbitration_id, 0.0)
            if last_payload == payload and now - last_stamp < min_interval:
                return False
            self._tx_last_payload[frame.arbitration_id] = payload
            self._tx_last_stamp[frame.arbitration_id] = now
            return True
