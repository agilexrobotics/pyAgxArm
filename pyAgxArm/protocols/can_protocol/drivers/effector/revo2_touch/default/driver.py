"""Revo2 Touch driver extending Revo2 Pro with capacitive touch APIs."""

from typing import TYPE_CHECKING, List, Optional

from ...revo2_pro.default.driver import Driver as Revo2ProDriver

if TYPE_CHECKING:
    from bc_stark_sdk.main_mod import TouchFingerItem, TouchRawData
else:
    try:
        from bc_stark_sdk.main_mod import TouchFingerItem, TouchRawData
    except ImportError:  # pragma: no cover
        TouchFingerItem = object  # type: ignore[misc, assignment]
        TouchRawData = object  # type: ignore[misc, assignment]


class Driver(Revo2ProDriver):
    """Revo2 Pro driver plus the capacitive touch sensor operations."""

    def get_touch_sensor_enabled(self) -> Optional[int]:
        """Get which touch sensors are enabled.

        Wrapped bc-stark-sdk API; ``slave_id`` is routed automatically.

        Returns:
            int: Bitmask of enabled sensors (bit 0-4 for each finger)

        Example::

            mask = hand.get_touch_sensor_enabled()  # bit i = finger i
        """
        return self.run_sdk(self.client.get_touch_sensor_enabled)

    def get_touch_sensor_fw_versions(self) -> Optional[List[str]]:
        """Get touch sensor firmware versions.

        Wrapped bc-stark-sdk API; ``slave_id`` is routed automatically.

        Returns:
            list: Firmware versions for each touch sensor

        Example::

            vers = hand.get_touch_sensor_fw_versions()
        """
        return self.run_sdk(self.client.get_touch_sensor_fw_versions)

    def get_touch_sensor_raw_data(self) -> Optional[TouchRawData]:
        """Get touch sensor raw data (capacitive sensors).

        Wrapped bc-stark-sdk API; ``slave_id`` is routed automatically.

        Returns:
            TouchRawData: Raw capacitive sensor data

        Example::

            raw = hand.get_touch_sensor_raw_data()
        """
        return self.run_sdk(self.client.get_touch_sensor_raw_data)

    def get_touch_sensor_status(self) -> Optional[List[TouchFingerItem]]:
        """Get touch sensor status for all fingers (capacitive sensors).

        Wrapped bc-stark-sdk API; ``slave_id`` is routed automatically.

        Returns:
            list[TouchFingerItem]: Capacitive status for fingers 0–4.

        Example::

            items = hand.get_touch_sensor_status()
        """
        return self.run_sdk(self.client.get_touch_sensor_status)

    def get_single_touch_sensor_status(self, index: int) -> Optional[TouchFingerItem]:
        """Get touch sensor status for a single finger (capacitive sensors).

        Wrapped bc-stark-sdk API; ``slave_id`` is routed automatically.

        Args:
            index: Finger index (0-4)

        Returns:
            TouchFingerItem: Capacitive status for one finger.

        Example::

            item = hand.get_single_touch_sensor_status(0)
        """
        return self.run_sdk(self.client.get_single_touch_sensor_status, index)

    def touch_sensor_setup(self, bits: int) -> None:
        """Setup touch sensors (enable/disable specific fingers).

        Wrapped bc-stark-sdk API; ``slave_id`` is routed automatically.

        Args:
            bits: Bitmask of sensors to enable (bit 0-4 for each finger)

        Example::

            hand.touch_sensor_setup(0b11111)  # enable fingers 0-4"""
        self.run_sdk(self.client.touch_sensor_setup, bits)

    def touch_sensor_reset(self, bits: int) -> None:
        """Reset touch sensors.

        Wrapped bc-stark-sdk API; ``slave_id`` is routed automatically.

        Args:
            bits: Bitmask of sensors to reset (bit 0-4 for each finger)

        Example::

            hand.touch_sensor_reset(0b00001)  # reset finger 0"""
        self.run_sdk(self.client.touch_sensor_reset, bits)

    def touch_sensor_calibrate(self, bits: int) -> None:
        """Calibrate touch sensors.

        Wrapped bc-stark-sdk API; ``slave_id`` is routed automatically.

        Args:
            bits: Bitmask of sensors to calibrate (bit 0-4 for each finger)

        Example::

            hand.touch_sensor_calibrate(0b11111)
        """
        self.run_sdk(self.client.touch_sensor_calibrate, bits)
