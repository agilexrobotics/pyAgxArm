from typing import Any, Callable, Type, Optional, Union, List, Tuple, Dict

import can
import logging

from ...msgs.core.msg_abstract import MessageAbstract
from ...msgs.core.attritube_base import AttributeBase
from .....utiles.fps import FPSManager


Decoder = Callable[[Any, bytearray], Any]
Encoder = Callable[[Any], Union[List[int], bytearray]]

RxSpec = Tuple[str, Type, Decoder]  # (attr_name, msg_cls, decoder)
TxSpec = Tuple[int, Encoder]        # (can_id, encoder)


class TableDriven:
    """
    通用表驱动 CAN Parser/packer：

    - RX: can_id -> (attr_name, msg_cls, decoder)
      - 统一缓存 MessageAbstract + msg_type + fps 统计 + timestamp
      - msg_type 默认取 msg_cls.type_（AttributeBase 自动提供）

    - TX: msg.type_ -> (can_id, encoder)
      - encoder 只负责把 msg 打包成 data（8 bytes list[int] / bytearray）
    """

    _rx_map: Dict[int, RxSpec]
    _tx_map: Dict[str, TxSpec]

    def __init__(
        self,
        fps_manager: FPSManager,
        logger_: Optional[logging.Logger] = None,
    ):
        self._fps_manager = fps_manager
        self._rx_map = {}
        self._tx_map = {}
        self._logger = logger_

    @property
    def logger(self) -> Optional[logging.Logger]:
        """Expose parser logger for downstream components."""
        return self._logger

    # ---------- RX ----------
    def _get_or_create_cached_msg(self, attr_name: str, msg_cls: Type) -> MessageAbstract:
        cached = getattr(self, attr_name, None)
        if cached is None:
            cached = MessageAbstract()
            cached.msg_type = getattr(msg_cls, "type_", msg_cls.__name__)
            cached.msg = msg_cls()
            self._fps_manager.add_variable(cached.msg_type)
            setattr(self, attr_name, cached)
        self._fps_manager.increment(cached.msg_type)
        return cached

    def parse_packet(self, rx_can_frame: can.Message) -> Optional[MessageAbstract]:
        spec = self._rx_map.get(rx_can_frame.arbitration_id)
        if spec is None:
            if self._logger is not None:
                self._logger.debug(
                    "Ignoring unknown CAN frame id=0x%03X dlc=%d in %s.",
                    rx_can_frame.arbitration_id,
                    len(rx_can_frame.data),
                    type(self).__name__,
                )
            return None

        attr_name, msg_cls, decoder = spec
        cached = self._get_or_create_cached_msg(attr_name=attr_name, msg_cls=msg_cls)
        decoder(cached.msg, rx_can_frame.data)
        cached.timestamp = rx_can_frame.timestamp
        return cached

    # ---------- TX ----------
    def pack(self, msg: AttributeBase) -> Optional[can.Message]:
        spec = self._tx_map.get(msg.type_)
        if spec is None:
            return None

        can_id, encoder = spec
        return can.Message(is_extended_id=False, arbitration_id=can_id, data=encoder(msg))
