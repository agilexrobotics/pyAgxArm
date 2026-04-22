#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import errno
import logging

import can
from can.message import Message
from typing import Optional
from platform import system

from .core.can_comm_base import CanCommBase
from .can_sys_utils import CanSystemInfoBase, LinuxSocketCanSystemInfo

_SUPPORTED_PLATFORMS = {"Linux", "Windows", "Darwin"}


def create_can_comm_config(
    *,
    channel: str = "can0",
    interface: str = "socketcan",
    bitrate: int = 1_000_000,
    enable_check_can: bool = True,
    auto_connect: bool = True,
    timeout: float = 0.001,
    receive_own_messages: bool = False,
    local_loopback: bool = False,
):
    return {
        "channel": channel,
        "interface": interface,
        "bitrate": bitrate,
        "enable_check_can": enable_check_can,
        "auto_connect": auto_connect,
        "timeout": timeout,
        "receive_own_messages": receive_own_messages,
        "local_loopback": local_loopback,
    }


class CanComm:
    """
    Platform selector for python-can based communication.
    """
    def __new__(
        cls,
        config: dict,
        comm_type: str = "can",
        logger_: Optional[logging.Logger] = None,
    ):
        platform_system = system()
        if platform_system not in _SUPPORTED_PLATFORMS:
            supported_text = ", ".join(sorted(_SUPPORTED_PLATFORMS))
            raise RuntimeError(
                "Unsupported platform: %s. Supported platforms: %s."
                % (platform_system, supported_text)
            )
        return CanCommImpl(config, comm_type, logger_=logger_)


class CanCommImpl(CanCommBase):
    def __init__(
        self,
        config: dict,
        comm_type: str = "can",
        logger_: Optional[logging.Logger] = None,
    ) -> None:
        super().__init__()
        self.recv_bus = None
        self.send_bus = None
        self.sysinfo: Optional[CanSystemInfoBase] = None
        self._config = config.copy()
        self._type = comm_type
        self._logger = logger_ if logger_ is not None else logging.getLogger(__name__)
        self._channel = self._config["channel"]
        self._interface = (
            self._config["interface"]
            if "interface" in self._config
            else self._config.get("bustype", "socketcan")
        )
        self._bitrate = self._config.get("bitrate", 1000000)
        self._enable_check_can = self._config.get("enable_check_can", False)
        self._auto_connect = self._config.get("auto_connect", False)
        self._timeout = self._config.get("timeout", 0.001)
        self._receive_own_messages = self._config.get("receive_own_messages", False)
        self._local_loopback = self._config.get("local_loopback", False)
        self._is_connected = False
        self._can_link_down_active = False
        if system() == "Linux" and self._interface == "socketcan":
            self.sysinfo = LinuxSocketCanSystemInfo
        if self._enable_check_can and self.sysinfo is not None:
            self._check_can_status()
        if self._auto_connect:
            self.connect()

    @property
    def logger(self) -> logging.Logger:
        """Expose comm logger for downstream components."""
        return self._logger

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass

    def _check_can_status(self) -> None:
        if not self.sysinfo.is_exists(self._channel):
            raise ValueError("Device '%s' does not exist." % (self._channel,))
        if not self.sysinfo.is_up(self._channel):
            self._logger.warning("Device '%s' is DOWN.", self._channel)
            self._can_link_down_active = True
        actual_bitrate = self.sysinfo.get_bitrate(self._channel)
        if (
            self._bitrate is not None
            and actual_bitrate is not None
            and actual_bitrate != self._bitrate
        ):
            self._logger.warning(
                "Device '%s' CAN port bitrate is %s bps, expected %s bps.",
                self._channel,
                actual_bitrate,
                self._bitrate,
            )

    def _classify_can_error(self, exc: Exception) -> Optional[str]:
        """
        Classify CAN send/recv exceptions.

        Returns
        -------
        - "hard_disconnect": device/netdev disappears (e.g. USB-CAN unplugged).
        - "link_down": interface is down (ENETDOWN-like)
        - "no_buffer": tx queue/buffer full (ENOBUFS-like)
        - None: unknown/unclassified error (caller should decide whether to raise)

        Notes
        -----
        - For send path, "no_buffer" / "link_down" are treated as tolerable and
          can be absorbed with warning prints.
        - For recv path, only "link_down" is treated as tolerable; "no_buffer"
          is generally a TX-side condition.
        """
        if self.sysinfo is not None and not self.sysinfo.is_exists(self._channel):
            return "hard_disconnect"
        candidates = [exc, getattr(exc, "__cause__", None), getattr(exc, "__context__", None)]
        for err in candidates:
            if err is None:
                continue
            if isinstance(err, OSError):
                eno = getattr(err, "errno", None)
                if eno == errno.ENETDOWN:
                    return "link_down"
                if eno == errno.ENODEV:
                    return "hard_disconnect"
                if eno == errno.ENOBUFS:
                    return "no_buffer"
            err_text = str(err).lower()
            if (
                "no buffer space available" in err_text
                or "transmit buffer full" in err_text
                or "buffer full" in err_text
            ):
                return "no_buffer"
            if "network is down" in err_text:
                return "link_down"
            if "no such device" in err_text:
                return "hard_disconnect"
        return None

    def _check_can_link_up(self) -> None:
        if not self._can_link_down_active:
            return
        if self.sysinfo and not self.sysinfo.is_up(self._channel):
            self._logger.warning("Device '%s' is DOWN.", self._channel)
            self._can_link_down_active = True
        else:
            self._logger.info("Device '%s' is UP.", self._channel)
            self._can_link_down_active = False

    def connect(self, **kwargs):
        if self.recv_bus is not None and self.send_bus is not None:
            return

        common_kwargs = dict(
            channel=self._channel,
            interface=self._interface,
            bitrate=self._bitrate,
            receive_own_messages=self._receive_own_messages,
            local_loopback=self._local_loopback,
        )

        try:
            self.recv_bus = can.interface.Bus(**common_kwargs)
            self.send_bus = self.recv_bus
            if self.sysinfo:
                # self.send_bus = can.interface.Bus(**common_kwargs)
                pass
            else:
                self._can_link_down_active = False
            self._is_connected = True
        except:
            self.close()
            raise can.CanInitializationError(
                "Failed to open CAN bus "
                "(interface='%s', channel='%s', bitrate=%s)."
                % (self._interface, self._channel, self._bitrate)
            )

    def close(self):
        try:
            self.recv_bus.shutdown()
        except Exception:
            pass

        try:
            self.send_bus.shutdown()
        except Exception:
            pass

        self.recv_bus = None
        self.send_bus = None
        self._is_connected = False

    def send(self, msg: Message, timeout=None):
        if self.send_bus is None:
            self.close()
            raise RuntimeError("CAN bus is not connected.")

        try:
            self.send_bus.send(msg, timeout)
            self._check_can_link_up()
        except Exception as exc:
            err_kind = self._classify_can_error(exc)
            if err_kind == "hard_disconnect":
                self.close()
                raise RuntimeError(
                    "Device '%s' is disconnected." % (self._channel,)
                ) from None
            if err_kind == "no_buffer":
                self._logger.warning("Device '%s' send buffer is FULL.", self._channel)
                return
            if err_kind == "link_down":
                self._logger.warning("Device '%s' is DOWN.", self._channel)
                self._can_link_down_active = True
                return
            raise

    def recv(self):
        if self.recv_bus is None:
            self.close()
            raise RuntimeError("CAN bus is not connected.")

        try:
            msg = self.recv_bus.recv(self._timeout)
            if msg is not None:
                if not msg.is_error_frame:
                    self._trigger_callback(msg)
                    return msg
            self._check_can_link_up()
        except Exception as exc:
            err_kind = self._classify_can_error(exc)
            if err_kind == "hard_disconnect":
                self.close()
                raise RuntimeError(
                    "Device '%s' is disconnected." % (self._channel,)
                ) from None
            if err_kind == "link_down":
                self._logger.warning("Device '%s' is DOWN.", self._channel)
                self._can_link_down_active = True
                return
            raise
