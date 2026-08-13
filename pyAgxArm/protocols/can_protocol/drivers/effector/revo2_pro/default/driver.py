"""Revo2 Pro driver: wrist CAN tunnel + bc-stark-sdk + left/right routing."""

import asyncio
import inspect
from typing import (
    TYPE_CHECKING,
    Any,
    Callable,
    List,
    Optional,
    Sequence,
    TypeVar,
    Union,
)
from typing_extensions import Literal

from ....core.driver_context import DriverContext
from .libstark_bridge import LibStarkBridge

if TYPE_CHECKING:
    from bc_stark_sdk.main_mod import (
        ButtonPressEvent,
        DeviceContext,
        DeviceInfo,
        FingerId,
        FingerUnitMode,
        HandType,
        LedColor,
        LedInfo,
        LedMode,
        MotorSettings,
        MotorStatusData,
        SkuType,
    )
else:
    try:
        from bc_stark_sdk.main_mod import (
            ButtonPressEvent,
            DeviceContext,
            DeviceInfo,
            FingerId,
            FingerUnitMode,
            HandType,
            LedColor,
            LedInfo,
            LedMode,
            MotorSettings,
            MotorStatusData,
            SkuType,
        )
    except ImportError:  # pragma: no cover
        ButtonPressEvent = object  # type: ignore[misc, assignment]
        DeviceContext = object  # type: ignore[misc, assignment]
        DeviceInfo = object  # type: ignore[misc, assignment]
        FingerId = object  # type: ignore[misc, assignment]
        FingerUnitMode = object  # type: ignore[misc, assignment]
        HandType = object  # type: ignore[misc, assignment]
        LedColor = object  # type: ignore[misc, assignment]
        LedInfo = object  # type: ignore[misc, assignment]
        LedMode = object  # type: ignore[misc, assignment]
        MotorSettings = object  # type: ignore[misc, assignment]
        MotorStatusData = object  # type: ignore[misc, assignment]
        SkuType = object  # type: ignore[misc, assignment]

T = TypeVar("T")

HandSide = Literal["left", "right"]

_DEVICE_ID_BY_HAND = {
    "left": 0x7E,
    "right": 0x7F,
}


def _normalize_hand_side(value: object) -> HandSide:
    if not isinstance(value, str):
        raise TypeError("hand_side must be a string: 'left' or 'right'")
    key = value.strip().lower()
    if key in _DEVICE_ID_BY_HAND:
        return key  # type: ignore[return-value]
    raise ValueError("hand_side must be exactly 'left' or 'right'")


def _device_id_for_hand(hand: HandSide) -> int:
    return _DEVICE_ID_BY_HAND[hand]


def _hand_for_device_id(device_id: int) -> HandSide:
    for hand, dev_id in _DEVICE_ID_BY_HAND.items():
        if dev_id == device_id:
            return hand  # type: ignore[return-value]
    raise ValueError(f"unknown touch-hand device id: 0x{device_id:02X}")


def _other_hand_side(hand: HandSide) -> HandSide:
    return "right" if hand == "left" else "left"


async def _invoke_sdk(
    fn: Callable[..., Any],
    slave_id: int,
    *args: Any,
    **kwargs: Any,
) -> Any:
    """
    bc-stark-sdk methods may return synchronously (e.g. bool capability probes)
    or a Future that must be awaited on the SDK asyncio loop.
    """
    result = fn(slave_id, *args, **kwargs)
    if inspect.iscoroutine(result) or asyncio.isfuture(result):
        return await result
    if hasattr(result, "__await__"):
        return await result
    return result


class Driver:
    """
    Revo2 Pro transport and hand-side routing over the bc-stark-sdk.

    Unwrapped SDK helpers may still be invoked via :meth:`run_sdk` or :attr:`client`.

    Quick start (``hand`` is this driver)::

        from pyAgxArm import AgxArmFactory

        robot = AgxArmFactory.create_arm(cfg)
        robot.connect()
        hand = robot.init_effector(robot.OPTIONS.EFFECTOR.REVO2_PRO)
        hand.set_hand_side("right")  # optional; default auto-probes 0x7E/0x7F
        hand.set_finger_position(hand.FingerId.Index, 500)
        info = hand.get_device_info()
    """

    # bc-stark-sdk types used by Driver method parameters (avoid separate import).
    FingerId = FingerId
    FingerUnitMode = FingerUnitMode
    LedColor = LedColor
    LedInfo = LedInfo
    LedMode = LedMode
    MotorSettings = MotorSettings

    def __init__(self, config: dict, ctx: DriverContext):
        del config
        self._bridge = LibStarkBridge(ctx)
        self._bridge.start()
        self._auto_mode = True
        self._active_side: Optional[HandSide] = None

    @property
    def client(self) -> DeviceContext:
        """Official bc-stark-sdk ``DeviceContext`` (async methods).

        Example::

            ctx = hand.client  # raw bc-stark-sdk DeviceContext
        """
        return self._bridge.client

    @property
    def hand_side(self) -> Optional[str]:
        """``"left"``, ``"right"``, or ``None`` when auto mode is not yet bound.

        Example::

            side = hand.hand_side
        """
        return self._active_side

    @property
    def slave_id(self) -> Optional[int]:
        """Bound Modbus slave id (``0x7E`` / ``0x7F``), or ``None`` if not yet known.

        Example::

            sid = hand.slave_id  # e.g. 127 (0x7F)
        """
        if self._active_side is None:
            return None
        return _device_id_for_hand(self._active_side)

    def set_hand_side(self, side: Optional[Union[str, HandSide]] = None) -> None:
        """Select left/right routing or revert to auto probe/failover.

        Args:
            side: ``"left"``, ``"right"``, or ``None`` for auto mode (default).

        Example::

            hand.set_hand_side("right")  # or None for auto-probe
        """
        if side is None:
            self._auto_mode = True
            self._active_side = None
            return
        hand = _normalize_hand_side(side)
        self._auto_mode = False
        self._active_side = hand

    # --- device info ---

    def get_device_info(self) -> Optional[DeviceInfo]:
        """Get device information (serial number, firmware version, hardware type)

        Wrapped bc-stark-sdk API; ``slave_id`` is routed automatically.

        Important: Call once after connect; SDK caches hardware type for capability routing.

        Returns:
            DeviceInfo: serial_number, firmware_version, hardware_type, hand_type, etc.

        Example::

            info = hand.get_device_info()
        """
        return self.run_sdk(self.client.get_device_info)

    def get_device_sn(self) -> Optional[str]:
        """Get device serial number.

        Wrapped bc-stark-sdk API; ``slave_id`` is routed automatically.

        Returns:
            str: Device serial number

        Example::

            sn = hand.get_device_sn()  # e.g. "BCXTR1354J2500009"
        """
        return self.run_sdk(self.client.get_device_sn)

    def get_device_fw_version(self) -> Optional[str]:
        """Get device firmware version.

        Wrapped bc-stark-sdk API; ``slave_id`` is routed automatically.

        Returns:
            str: Firmware version string

        Example::

            ver = hand.get_device_fw_version()  # e.g. "1.0.16.U"
        """
        return self.run_sdk(self.client.get_device_fw_version)

    def get_sku_type(self) -> Optional[SkuType]:
        """Get device SKU type (hand side: left/right).

        Wrapped bc-stark-sdk API; ``slave_id`` is routed automatically.

        Returns:
            SkuType: Device SKU type

        Example::

            sku = hand.get_sku_type()
        """
        return self.run_sdk(self.client.get_sku_type)

    def get_hand_type(self) -> Optional[HandType]:
        """Get device hand type (simplified left/right).

        Wrapped bc-stark-sdk API; ``slave_id`` is routed automatically.

        Returns:
            HandType enum (Left = 0, Right = 1)

        Example::

            ht = hand.get_hand_type()
        """
        return self.run_sdk(self.client.get_hand_type)

    def get_button_event(self) -> Optional[ButtonPressEvent]:
        """Get button press event.

        Wrapped bc-stark-sdk API; ``slave_id`` is routed automatically.

        Returns:
            ButtonPressEvent: Button press event data

        Example::

            evt = hand.get_button_event()
        """
        return self.run_sdk(self.client.get_button_event)

    # --- device config ---

    def get_auto_calibration_enabled(self) -> Optional[bool]:
        """Check if auto calibration is enabled.

        Wrapped bc-stark-sdk API; ``slave_id`` is routed automatically.

        Returns:
            bool: True if auto calibration is enabled

        Example::

            enabled = hand.get_auto_calibration_enabled()
        """
        return self.run_sdk(self.client.get_auto_calibration_enabled)

    def set_auto_calibration(self, enabled: bool) -> None:
        """Enable or disable auto calibration.

        Wrapped bc-stark-sdk API; ``slave_id`` is routed automatically.

        Args:
            enabled: True to enable, False to disable

        Example::

            hand.set_auto_calibration(True)
        """
        self.run_sdk(self.client.set_auto_calibration, enabled)

    def calibrate_position(self) -> None:
        """Trigger manual position calibration.

        Wrapped bc-stark-sdk API; ``slave_id`` is routed automatically.

        Example::

            hand.calibrate_position()
        """
        self.run_sdk(self.client.calibrate_position)

    def reset_default_settings(self) -> None:
        """Reset all settings to factory defaults.

        Wrapped bc-stark-sdk API; ``slave_id`` is routed automatically.

        Example::

            hand.reset_default_settings()
        """
        self.run_sdk(self.client.reset_default_settings)

    def reboot(self) -> None:
        """Reboot the device.

        Wrapped bc-stark-sdk API; ``slave_id`` is routed automatically.

        Example::

            hand.reboot()
        """
        self.run_sdk(self.client.reboot)

    # --- motor control: position ---

    def set_finger_position(self, finger_id: FingerId, position: int) -> None:
        """Set finger position

        Wrapped bc-stark-sdk API; ``slave_id`` is routed automatically.

        Args:
            finger_id: Finger ID
            position: Position value, unified range **0~1000** (all devices, all protocols)
            0 = fully open, 1000 = fully closed

        Note: SDK automatically converts to internal range based on device type and protocol.

        Example::

            hand.set_finger_position(hand.FingerId.Index, 500)
        """
        self.run_sdk(self.client.set_finger_position, finger_id, position)

    def set_finger_position_with_millis(
        self, finger_id: FingerId, position: int, milliseconds: int
    ) -> None:
        """Set finger position with duration

        Wrapped bc-stark-sdk API; ``slave_id`` is routed automatically.

        Args:
            finger_id: Finger ID
            position: Position value, unified range **0~1000** (all protocols)
            milliseconds: Duration in milliseconds (1~2000ms)

        Note: Only supported on Revo2 devices.

        Example::

            hand.set_finger_position_with_millis(hand.FingerId.Index, 500, 800)
        """
        self.run_sdk(
            self.client.set_finger_position_with_millis,
            finger_id,
            position,
            milliseconds,
        )

    def set_finger_position_with_speed(
        self, finger_id: FingerId, position: int, speed: int
    ) -> None:
        """Set finger position with speed

        Wrapped bc-stark-sdk API; ``slave_id`` is routed automatically.

        Args:
            finger_id: Finger ID
            position: Position value, unified range **0~1000** (all protocols)
            speed: Speed value (1~1000)

        Note: Only supported on Revo2 devices.

        Example::

            hand.set_finger_position_with_speed(hand.FingerId.Index, 500, 200)
        """
        self.run_sdk(
            self.client.set_finger_position_with_speed,
            finger_id,
            position,
            speed,
        )

    def set_finger_positions(self, positions: Sequence[int]) -> None:
        """Set multiple finger positions

        Wrapped bc-stark-sdk API; ``slave_id`` is routed automatically.

        Args:
            positions: Position array (length 6), unified range **0~1000** (all devices, all protocols)

        Note: SDK automatically converts to internal range based on device type and protocol.

        Example::

            hand.set_finger_positions([0, 500, 500, 500, 500, 500])
        """
        self.run_sdk(self.client.set_finger_positions, list(positions))

    def set_finger_positions_and_durations(
        self, positions: Sequence[int], durations: Sequence[int]
    ) -> None:
        """Set multiple finger positions with durations (Revo2 only).

        Wrapped bc-stark-sdk API; ``slave_id`` is routed automatically.

        Args:
            positions: Position array (length 6), unified range 0~1000
            durations: Duration array (length 6), in milliseconds (1~2000ms)

        Example::

            hand.set_finger_positions_and_durations([500] * 6, [1000] * 6)
        """
        self.run_sdk(
            self.client.set_finger_positions_and_durations,
            list(positions),
            list(durations),
        )

    def set_finger_positions_and_speeds(
        self, positions: Sequence[int], speeds: Sequence[int]
    ) -> None:
        """Set multiple finger positions with speeds (Revo2 only).

        Wrapped bc-stark-sdk API; ``slave_id`` is routed automatically.

        Args:
            positions: Position array (length 6), unified range 0~1000
            speeds: Speed array (length 6), range 1~1000

        Example::

            hand.set_finger_positions_and_speeds([500] * 6, [200] * 6)
        """
        self.run_sdk(
            self.client.set_finger_positions_and_speeds,
            list(positions),
            list(speeds),
        )

    def get_finger_positions(self) -> Optional[List[int]]:
        """Get finger positions

        Wrapped bc-stark-sdk API; ``slave_id`` is routed automatically.

        Returns:
            Position array (length 6), unified range **0~1000** (all devices, all protocols)

        Note: SDK automatically converts internal values to unified external range.

        Example::

            pos = hand.get_finger_positions()
        """
        return self.run_sdk(self.client.get_finger_positions)

    # --- motor control: speed ---

    def set_finger_speed(self, finger_id: FingerId, speed: int) -> None:
        """Set finger speed

        Wrapped bc-stark-sdk API; ``slave_id`` is routed automatically.

        Args:
            finger_id: Finger ID
            speed: Speed value, unified range **-1000~+1000** (all devices, all protocols)
            Positive = close, Negative = open, 0 = stop

        Note: SDK automatically converts to internal range based on device type and protocol.

        Example::

            hand.set_finger_speed(hand.FingerId.Index, 300)
        """
        self.run_sdk(self.client.set_finger_speed, finger_id, speed)

    def set_finger_speeds(self, speeds: Sequence[int]) -> None:
        """Set multiple finger speeds

        Wrapped bc-stark-sdk API; ``slave_id`` is routed automatically.

        Args:
            speeds: Speed array (length 6), unified range **-1000~+1000** (all devices, all protocols)

        Note: SDK automatically converts to internal range based on device type and protocol.

        Example::

            hand.set_finger_speeds([0, 300, 0, 0, 0, 0])
        """
        self.run_sdk(self.client.set_finger_speeds, list(speeds))

    def get_finger_speeds(self) -> Optional[List[int]]:
        """Get finger speeds

        Wrapped bc-stark-sdk API; ``slave_id`` is routed automatically.

        Returns:
            Speed array (length 6), unified range **-1000~+1000** (all devices, all protocols)

        Note: SDK automatically converts internal values to unified external range.

        Example::

            speeds = hand.get_finger_speeds()
        """
        return self.run_sdk(self.client.get_finger_speeds)

    # --- motor control: current ---

    def set_finger_current(self, finger_id: FingerId, current: int) -> None:
        """Set finger current

        Wrapped bc-stark-sdk API; ``slave_id`` is routed automatically.

        Args:
            finger_id: Finger ID
            current: Current value, unified range **-1000~+1000** (all devices, all protocols)
            Positive = close, Negative = open

        Note: SDK automatically converts to internal range based on device type and protocol.

        Example::

            hand.set_finger_current(hand.FingerId.Index, 200)
        """
        self.run_sdk(self.client.set_finger_current, finger_id, current)

    def set_finger_currents(self, currents: Sequence[int]) -> None:
        """Set multiple finger currents

        Wrapped bc-stark-sdk API; ``slave_id`` is routed automatically.

        Args:
            currents: Current array (length 6), unified range **-1000~+1000** (all devices, all protocols)

        Note: SDK automatically converts to internal range based on device type and protocol.

        Example::

            hand.set_finger_currents([0, 200, 0, 0, 0, 0])
        """
        self.run_sdk(self.client.set_finger_currents, list(currents))

    def get_finger_currents(self) -> Optional[List[int]]:
        """Get finger currents

        Wrapped bc-stark-sdk API; ``slave_id`` is routed automatically.

        Returns:
            Current array (length 6), unified range **-1000~+1000** (all devices, all protocols)

        Note: SDK automatically converts internal values to unified external range.

        Example::

            currents = hand.get_finger_currents()
        """
        return self.run_sdk(self.client.get_finger_currents)

    # --- motor status ---

    def get_motor_status(self) -> Optional[MotorStatusData]:
        """Get motor status

        Wrapped bc-stark-sdk API; ``slave_id`` is routed automatically.

        Returns:
            MotorStatusData containing: - positions: Position array, unified range **0~1000** (all devices, all protocols) - speeds: Speed array, unified range **-1000~+1000** (all devices, all protocols) - currents: Current array, unified range **-1000~+1000** (all devices, all protocols) - states: Motor state array

        Note: SDK automatically converts internal values to unified external range.

        Example::

            status = hand.get_motor_status()
        """
        return self.run_sdk(self.client.get_motor_status)

    def get_motor_state(self) -> Optional[List[int]]:
        """Get motor state (running, idle, stalled, etc.).

        Wrapped bc-stark-sdk API; ``slave_id`` is routed automatically.

        Returns:
            list[int]: Motor state code array (length 6).

        Example::

            states = hand.get_motor_state()
        """
        return self.run_sdk(self.client.get_motor_state)

    # --- motor settings ---

    def get_finger_unit_mode(self) -> Optional[FingerUnitMode]:
        """Get finger unit mode (Revo2 only).

        Wrapped bc-stark-sdk API; ``slave_id`` is routed automatically.

        Returns:
            FingerUnitMode: Current finger unit mode

        Example::

            mode = hand.get_finger_unit_mode()
        """
        return self.run_sdk(self.client.get_finger_unit_mode)

    def set_finger_unit_mode(self, mode: FingerUnitMode) -> None:
        """Set finger unit mode (Revo2 only).

        Wrapped bc-stark-sdk API; ``slave_id`` is routed automatically.

        Args:
            mode: FingerUnitMode (FiveFingers or SixFingers)

        Example::

            hand.set_finger_unit_mode(hand.FingerUnitMode.Normalized)
        """
        self.run_sdk(self.client.set_finger_unit_mode, mode)

    def get_all_finger_settings(self) -> Optional[List[MotorSettings]]:
        """Get motor settings for all fingers (Revo2 only).

        Wrapped bc-stark-sdk API; ``slave_id`` is routed automatically.

        Returns:
            list: MotorSettings for all fingers

        Example::

            settings = hand.get_all_finger_settings()
        """
        return self.run_sdk(self.client.get_all_finger_settings)

    def get_finger_settings(self, finger_id: FingerId) -> Optional[MotorSettings]:
        """Get motor settings for a single finger (Revo2 only).

        Wrapped bc-stark-sdk API; ``slave_id`` is routed automatically.

        Args:
            finger_id: Finger ID

        Returns:
            MotorSettings: Settings for the specified finger

        Example::

            cfg = hand.get_finger_settings(hand.FingerId.Thumb)
        """
        return self.run_sdk(self.client.get_finger_settings, finger_id)

    def set_finger_settings(self, finger_id: FingerId, settings: MotorSettings) -> None:
        """Set motor settings for a finger (Revo2 only).

        Wrapped bc-stark-sdk API; ``slave_id`` is routed automatically.

        Args:
            finger_id: Finger ID
            settings: MotorSettings object

        Example::

            cfg = hand.get_finger_settings(hand.FingerId.Thumb)
            hand.set_finger_settings(hand.FingerId.Thumb, cfg)
        """
        self.run_sdk(self.client.set_finger_settings, finger_id, settings)

    def get_finger_min_position(self, finger_id: FingerId) -> Optional[int]:
        """Get minimum position limit for a finger (Revo2 only).

        Wrapped bc-stark-sdk API; ``slave_id`` is routed automatically.

        Args:
            finger_id: Finger ID

        Returns:
            int: Minimum position value

        Example::

            lo = hand.get_finger_min_position(hand.FingerId.Index)
        """
        return self.run_sdk(self.client.get_finger_min_position, finger_id)

    def set_finger_min_position(self, finger_id: FingerId, position: int) -> None:
        """Set minimum position limit for a finger (Revo2 only).

        Wrapped bc-stark-sdk API; ``slave_id`` is routed automatically.

        Args:
            finger_id: Finger ID
            position: Minimum position value

        Example::

            hand.set_finger_min_position(hand.FingerId.Index, 0)
        """
        self.run_sdk(self.client.set_finger_min_position, finger_id, position)

    def get_finger_max_position(self, finger_id: FingerId) -> Optional[int]:
        """Get maximum position limit for a finger (Revo2 only).

        Wrapped bc-stark-sdk API; ``slave_id`` is routed automatically.

        Args:
            finger_id: Finger ID

        Returns:
            int: Maximum position value

        Example::

            hi = hand.get_finger_max_position(hand.FingerId.Index)
        """
        return self.run_sdk(self.client.get_finger_max_position, finger_id)

    def set_finger_max_position(self, finger_id: FingerId, position: int) -> None:
        """Set maximum position limit for a finger (Revo2 only).

        Wrapped bc-stark-sdk API; ``slave_id`` is routed automatically.

        Args:
            finger_id: Finger ID
            position: Maximum position value

        Example::

            hand.set_finger_max_position(hand.FingerId.Index, 1000)
        """
        self.run_sdk(self.client.set_finger_max_position, finger_id, position)

    def get_finger_max_speed(self, finger_id: FingerId) -> Optional[int]:
        """Get maximum speed limit for a finger (Revo2 only).

        Wrapped bc-stark-sdk API; ``slave_id`` is routed automatically.

        Args:
            finger_id: Finger ID

        Returns:
            int: Maximum speed value

        Example::

            max_spd = hand.get_finger_max_speed(hand.FingerId.Index)
        """
        return self.run_sdk(self.client.get_finger_max_speed, finger_id)

    def set_finger_max_speed(self, finger_id: FingerId, speed: int) -> None:
        """Set maximum speed limit for a finger (Revo2 only).

        Wrapped bc-stark-sdk API; ``slave_id`` is routed automatically.

        Args:
            finger_id: Finger ID
            speed: Maximum speed value

        Example::

            hand.set_finger_max_speed(hand.FingerId.Index, 500)
        """
        self.run_sdk(self.client.set_finger_max_speed, finger_id, speed)

    def get_finger_max_current(self, finger_id: FingerId) -> Optional[int]:
        """Get maximum current limit for a finger (Revo2 only).

        Wrapped bc-stark-sdk API; ``slave_id`` is routed automatically.

        Args:
            finger_id: Finger ID

        Returns:
            int: Maximum current value

        Example::

            max_cur = hand.get_finger_max_current(hand.FingerId.Index)
        """
        return self.run_sdk(self.client.get_finger_max_current, finger_id)

    def set_finger_max_current(self, finger_id: FingerId, current: int) -> None:
        """Set maximum current limit for a finger (Revo2 only).

        Wrapped bc-stark-sdk API; ``slave_id`` is routed automatically.

        Args:
            finger_id: Finger ID
            current: Maximum current value

        Example::

            hand.set_finger_max_current(hand.FingerId.Index, 800)
        """
        self.run_sdk(self.client.set_finger_max_current, finger_id, current)

    def get_finger_protected_current(self, finger_id: FingerId) -> Optional[int]:
        """Get protected current for a single finger (Revo2 only).

        Wrapped bc-stark-sdk API; ``slave_id`` is routed automatically.

        Args:
            finger_id: Finger ID

        Returns:
            int: Protected current value

        Example::

            pc = hand.get_finger_protected_current(hand.FingerId.Index)
        """
        return self.run_sdk(self.client.get_finger_protected_current, finger_id)

    def set_finger_protected_current(self, finger_id: FingerId, current: int) -> None:
        """Set protected current for a single finger (Revo2 only).

        Wrapped bc-stark-sdk API; ``slave_id`` is routed automatically.

        Args:
            finger_id: Finger ID
            current: Protected current value

        Example::

            hand.set_finger_protected_current(hand.FingerId.Index, 500)
        """
        self.run_sdk(self.client.set_finger_protected_current, finger_id, current)

    def get_finger_protected_currents(self) -> Optional[List[int]]:
        """Get protected currents for all fingers (Revo2 only).

        Wrapped bc-stark-sdk API; ``slave_id`` is routed automatically.

        Returns:
            list: Protected current array (length 6)

        Example::

            pcs = hand.get_finger_protected_currents()
        """
        return self.run_sdk(self.client.get_finger_protected_currents)

    def set_finger_protected_currents(self, currents: Sequence[int]) -> None:
        """Set protected currents for all fingers (Revo2 only). Protected current is the threshold for stall detection.

        Wrapped bc-stark-sdk API; ``slave_id`` is routed automatically.

        Args:
            protected_currents: Current array (length 6)

        Example::

            hand.set_finger_protected_currents([500] * 6)
        """
        self.run_sdk(self.client.set_finger_protected_currents, list(currents))

    def get_thumb_aux_lock_current(self) -> Optional[int]:
        """Get thumb auxiliary motor lock current (Revo2 only).

        Wrapped bc-stark-sdk API; ``slave_id`` is routed automatically.

        Returns:
            int: Lock current value

        Example::

            lock = hand.get_thumb_aux_lock_current()
        """
        return self.run_sdk(self.client.get_thumb_aux_lock_current)

    def set_thumb_aux_lock_current(self, lock_current: int) -> None:
        """Set thumb auxiliary motor lock current (Revo2 only).

        Wrapped bc-stark-sdk API; ``slave_id`` is routed automatically.

        Args:
            lock_current: Lock current value

        Example::

            hand.set_thumb_aux_lock_current(100)
        """
        self.run_sdk(self.client.set_thumb_aux_lock_current, lock_current)

    # --- LED / buzzer / vibration ---

    def get_led_enabled(self) -> Optional[bool]:
        """Check if LED is enabled.

        Wrapped bc-stark-sdk API; ``slave_id`` is routed automatically.

        Returns:
            bool: True if LED is enabled

        Example::

            on = hand.get_led_enabled()
        """
        return self.run_sdk(self.client.get_led_enabled)

    def get_led_info(self) -> Optional[LedInfo]:
        """Get LED information (color, mode, brightness).

        Wrapped bc-stark-sdk API; ``slave_id`` is routed automatically.

        Returns:
            LedInfo: LED configuration

        Example::

            led = hand.get_led_info()
        """
        return self.run_sdk(self.client.get_led_info)

    def set_led_enabled(self, enabled: bool) -> None:
        """Enable or disable LED.

        Wrapped bc-stark-sdk API; ``slave_id`` is routed automatically.

        Args:
            enabled: True to enable, False to disable

        Example::

            hand.set_led_enabled(True)
        """
        self.run_sdk(self.client.set_led_enabled, enabled)

    def set_led_info(self, led_info: LedInfo) -> None:
        """Set LED information (color, mode, brightness).

        Wrapped bc-stark-sdk API; ``slave_id`` is routed automatically.

        Args:
            led_info: LedInfo object

        Example::

            led = hand.LedInfo(hand.LedColor.RGB, hand.LedMode.Blink2Hz)
            hand.set_led_info(led)
        """
        self.run_sdk(self.client.set_led_info, led_info)

    def get_buzzer_enabled(self) -> Optional[bool]:
        """Check if buzzer is enabled.

        Wrapped bc-stark-sdk API; ``slave_id`` is routed automatically.

        Returns:
            bool: True if buzzer is enabled

        Example::

            on = hand.get_buzzer_enabled()
        """
        return self.run_sdk(self.client.get_buzzer_enabled)

    def set_buzzer_enabled(self, enabled: bool) -> None:
        """Enable or disable buzzer.

        Wrapped bc-stark-sdk API; ``slave_id`` is routed automatically.

        Args:
            enabled: True to enable, False to disable

        Example::

            hand.set_buzzer_enabled(True)
        """
        self.run_sdk(self.client.set_buzzer_enabled, enabled)

    def get_vibration_enabled(self) -> Optional[bool]:
        """Check if vibration motor is enabled.

        Wrapped bc-stark-sdk API; ``slave_id`` is routed automatically.

        Returns:
            bool: True if vibration motor is enabled

        Example::

            on = hand.get_vibration_enabled()
        """
        return self.run_sdk(self.client.get_vibration_enabled)

    def set_vibration_enabled(self, enabled: bool) -> None:
        """Enable or disable vibration motor.

        Wrapped bc-stark-sdk API; ``slave_id`` is routed automatically.

        Args:
            enabled: True to enable, False to disable

        Example::

            hand.set_vibration_enabled(True)
        """
        self.run_sdk(self.client.set_vibration_enabled, enabled)

    # --- transport helpers ---

    def scan_slave_ids(
        self,
        candidate_ids: Optional[Sequence[int]] = None,
        *,
        timeout_ms: int = 500,
    ) -> Optional[int]:
        """Probe wrist CAN tunnel for a touch-hand Modbus slave id.

        Uses bc-stark-sdk ``scan_canfd_devices`` under the hood. Does not
        change :attr:`hand_side` binding.

        Args:
            candidate_ids: IDs to try (default ``[0x7E, 0x7F]``).
            timeout_ms: Probe timeout per candidate.

        Returns:
            First responding slave id, or ``None``.

        Example::

            sid = hand.scan_slave_ids([0x7E, 0x7F])
        """
        ids = list(candidate_ids) if candidate_ids is not None else None
        return self._bridge.run(
            self._bridge.probe_slave_id(ids, timeout_ms=timeout_ms)
        )

    def run_sdk(
        self,
        fn: Callable[..., Any],
        *args: Any,
        **kwargs: Any,
    ) -> Optional[T]:
        """
        Run a bc-stark-sdk ``DeviceContext`` method on the SDK thread.

        Pass ``hand.client.<method>``; ``slave_id`` routing is injected
        automatically. Handles both synchronous returns (e.g. capability
        probes) and async Futures. Returns ``None`` on timeout or SDK error.

        Example::

            pos = hand.run_sdk(hand.client.get_finger_positions)
            hand.run_sdk(hand.client.set_finger_positions, [0, 500, 500, 500, 500, 500])
            hand.get_device_info()
            hand.run_sdk(hand.client.is_touch_hand)
        """
        async def _once(slave_id: int) -> T:
            return await _invoke_sdk(fn, slave_id, *args, **kwargs)

        def _run(slave_id: int) -> T:
            return self._bridge.run(_once(slave_id))

        auto_mode = self._auto_mode
        active = self._active_side

        if not auto_mode:
            if active is None:
                raise RuntimeError("pinned hand side is not set")
            try:
                return _run(_device_id_for_hand(active))
            except Exception:
                return None

        if active is None:
            slave_id = self._probe_and_bind()
            if slave_id is None:
                return None
            try:
                return _run(slave_id)
            except Exception:
                self._active_side = None
                return None

        primary = _device_id_for_hand(active)
        try:
            return _run(primary)
        except Exception:
            fallback_hand = _other_hand_side(active)
            try:
                result = _run(_device_id_for_hand(fallback_hand))
                self._active_side = fallback_hand
                return result
            except Exception:
                self._active_side = None
                return None

    def _probe_and_bind(self) -> Optional[int]:
        found = self._bridge.run(self._bridge.probe_slave_id())
        if found is None:
            return None
        self._active_side = _hand_for_device_id(found)
        return found
