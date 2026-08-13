"""
Adapt ``bc-stark-sdk`` CAN FD transport to the Agilex wrist CAN 2.0 tunnel.

Official SDK calls synchronous ``set_can_tx`` / ``set_can_rx`` callbacks.
TX is fragmented onto classic CAN 0x711; RX reassembles 0x712 in the arm
read-loop via :class:`~.....comms.Can20CanfdRx`.
"""

import asyncio
import threading
import time
from typing import Any, Coroutine, List, Optional, Tuple, TypeVar

from .....comms import Can20CanfdRx, CanFdMessage
from ....core.driver_context import DriverContext

try:
    import bc_stark_sdk.main_mod as libstark
    from bc_stark_sdk.main_mod import DeviceContext, StarkProtocolType
except ImportError as exc:  # pragma: no cover
    libstark = None  # type: ignore[assignment]
    DeviceContext = Any  # type: ignore[misc, assignment]
    StarkProtocolType = Any  # type: ignore[misc, assignment]
    _IMPORT_ERROR = exc
else:
    _IMPORT_ERROR = None

T = TypeVar("T")

# BrainCo CAN FD id: (slave_id << 16) | (master_id << 8) | pdu_len
DEFAULT_MASTER_ID = 0x08
DEFAULT_TUNNEL_TIMEOUT_S = 0.1  # _can_rx wait; wrist tunnel may be slower on hardware
DEFAULT_TUNNEL_GAP_S = 0.0  # pause between 0x711 fragments
DEFAULT_PROBE_TIMEOUT_MS = 500  # scan_canfd_devices


def _require_sdk() -> None:
    if libstark is None:
        raise ImportError(
            "bc-stark-sdk is required for Revo2 Pro/Touch SDK support. "
            "Install with: pip install bc-stark-sdk"
        ) from _IMPORT_ERROR


def _slave_id_from_can_id(can_id: int) -> int:
    return (can_id >> 16) & 0xFF


def _master_id_from_can_id(can_id: int) -> int:
    return (can_id >> 8) & 0xFF


class LibStarkBridge:
    """
    Run bc-stark-sdk on a dedicated asyncio thread over the wrist tunnel.

    Requires the arm read-loop (``ctx.start_th()``) so 0x712 frames reach
    :meth:`_on_can_frame`. SDK ``can_rx`` blocks that asyncio thread, not
    the read-loop.
    """

    def __init__(
        self,
        ctx: DriverContext,
        *,
        master_id: int = DEFAULT_MASTER_ID,
        tunnel_timeout_s: float = DEFAULT_TUNNEL_TIMEOUT_S,
        tunnel_gap_s: float = DEFAULT_TUNNEL_GAP_S,
    ) -> None:
        _require_sdk()
        self._ctx = ctx
        self._master_id = master_id
        self._tunnel_timeout_s = tunnel_timeout_s
        self._tunnel_gap_s = tunnel_gap_s

        self._loop = asyncio.new_event_loop()
        self._loop_ready = threading.Event()
        self._loop_thread = threading.Thread(
            target=self._run_event_loop, name="revo2-pro-sdk", daemon=True
        )
        self._client: Optional[DeviceContext] = None
        self._started = False

        self._rx_collector = Can20CanfdRx()
        self._rx_lock = threading.Lock()
        ctx.register_parser_packet_fun(self._on_can_frame)

    @property
    def client(self) -> DeviceContext:
        if self._client is None:
            raise RuntimeError("LibStarkBridge is not started")
        return self._client

    def start(self) -> None:
        """Register SDK callbacks and init ``DeviceContext`` (idempotent)."""
        if self._started:
            return
        self._loop_thread.start()
        if not self._loop_ready.wait(timeout=5.0):
            raise RuntimeError("SDK event loop failed to start")
        libstark.set_can_tx_callback(self._can_tx)
        libstark.set_can_rx_callback(self._can_rx)
        self._client = libstark.init_device_handler(
            StarkProtocolType.CanFd, self._master_id
        )
        self._started = True

    def run(self, coro: Coroutine[Any, Any, T], *, timeout: Optional[float] = None) -> T:
        """Submit *coro* to the SDK asyncio thread and block for the result."""
        if not self._started:
            raise RuntimeError(
                "LibStarkBridge is not started. "
                "Ensure the arm read-loop is running before using the touch hand."
            )
        wait_s = timeout if timeout is not None else self._tunnel_timeout_s + 2.0
        future = asyncio.run_coroutine_threadsafe(coro, self._loop)
        return future.result(timeout=wait_s)

    async def probe_slave_id(
        self,
        candidate_ids: Optional[List[int]] = None,
        *,
        timeout_ms: int = DEFAULT_PROBE_TIMEOUT_MS,
    ) -> Optional[int]:
        """Wrap official ``scan_canfd_devices`` (default candidates 0x7E, 0x7F)."""
        return await libstark.scan_canfd_devices(
            self.client,
            candidate_ids or [0x7E, 0x7F],
            timeout_ms,
        )

    def _run_event_loop(self) -> None:
        asyncio.set_event_loop(self._loop)
        self._loop_ready.set()
        self._loop.run_forever()

    def _on_can_frame(self, message) -> None:
        with self._rx_lock:
            self._rx_collector.feed(message)

    def _can_tx(self, _slave_id: int, can_id: int, data: list) -> bool:
        comm = self._ctx.get_comm()
        if comm is None:
            return False

        # New SDK transaction: drop stale 0x712 fragments.
        with self._rx_lock:
            self._rx_collector.reset()

        request = CanFdMessage(can_id, bytes(data))
        gap = self._tunnel_gap_s
        for frame in request.msgs:
            comm.send(frame)
            if gap > 0:
                time.sleep(gap)
        return True

    def _can_rx(
        self,
        _slave_id: int,
        expected_can_id: int,
        _expected_frames: int,
    ) -> Tuple[int, bytes]:
        # Called synchronously from SDK; blocks until read-loop feeds a match.
        expected_slave = _slave_id_from_can_id(expected_can_id)
        expected_master = _master_id_from_can_id(expected_can_id)

        def _matches(msg: CanFdMessage) -> bool:
            return (
                _slave_id_from_can_id(msg.arbitration_id) == expected_slave
                and _master_id_from_can_id(msg.arbitration_id) == expected_master
            )

        got = self._rx_collector.wait_matching(self._tunnel_timeout_s, _matches)
        if got is None:
            return 0, bytes([])
        return got.arbitration_id, got.data
