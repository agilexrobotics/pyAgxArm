import logging
from typing import TYPE_CHECKING, Callable, Optional, Union, List, Dict, Tuple, Type

from ....core.table_driven import TableDriven
from ....core.protocol_parser_interface import ProtocolParserInterface
from .....msgs.effector.revo2.default import (
    FeedbackFingerCurrent,
    FeedbackFingerPos,
    FeedbackFingerSpd,
    FeedbackHandStatus,
    FingerPosCtrl,
    FingerSpdCtrl,
    FingerCurrentCtrl,
    FingerPosTimeCtrl,
)
from .....msgs.core.msg_abstract import MessageAbstract
from pyAgxArm.utiles.fps import FPSManager
from pyAgxArm.utiles.numeric_codec import NumericCodec as nc


class Codec:
    """Minimal codec for revo2 messages."""

    def decode_1C0_hand_status(self, m: FeedbackHandStatus, d: bytearray) -> None:
        m.left_or_right = nc.ConvertToNegative_8bit(nc.ConvertBytesToInt(d,0,1), False)
        m.thumb_tip = nc.ConvertToNegative_8bit(nc.ConvertBytesToInt(d,2,3), False)
        m.thumb_base = nc.ConvertToNegative_8bit(nc.ConvertBytesToInt(d,3,4), False)
        m.index_finger = nc.ConvertToNegative_8bit(nc.ConvertBytesToInt(d,4,5), False)
        m.middle_finger = nc.ConvertToNegative_8bit(nc.ConvertBytesToInt(d,5,6), False)
        m.ring_finger = nc.ConvertToNegative_8bit(nc.ConvertBytesToInt(d,6,7), False)
        m.pinky_finger = nc.ConvertToNegative_8bit(nc.ConvertBytesToInt(d,7,8), False)

    def decode_1C1_finger_pos(self, m: FeedbackFingerPos, d: bytearray) -> None:
        m.thumb_tip = nc.ConvertToNegative_8bit(nc.ConvertBytesToInt(d,2,3), False)
        m.thumb_base = nc.ConvertToNegative_8bit(nc.ConvertBytesToInt(d,3,4), False)
        m.index_finger = nc.ConvertToNegative_8bit(nc.ConvertBytesToInt(d,4,5), False)
        m.middle_finger = nc.ConvertToNegative_8bit(nc.ConvertBytesToInt(d,5,6), False)
        m.ring_finger = nc.ConvertToNegative_8bit(nc.ConvertBytesToInt(d,6,7), False)
        m.pinky_finger = nc.ConvertToNegative_8bit(nc.ConvertBytesToInt(d,7,8), False)

    def decode_1C2_finger_spd(self, m: FeedbackFingerSpd, d: bytearray) -> None:
        m.thumb_tip = nc.ConvertToNegative_8bit(nc.ConvertBytesToInt(d,2,3))
        m.thumb_base = nc.ConvertToNegative_8bit(nc.ConvertBytesToInt(d,3,4))
        m.index_finger = nc.ConvertToNegative_8bit(nc.ConvertBytesToInt(d,4,5))
        m.middle_finger = nc.ConvertToNegative_8bit(nc.ConvertBytesToInt(d,5,6))
        m.ring_finger = nc.ConvertToNegative_8bit(nc.ConvertBytesToInt(d,6,7))
        m.pinky_finger = nc.ConvertToNegative_8bit(nc.ConvertBytesToInt(d,7,8))

    def decode_1C3_finger_current(self, m: FeedbackFingerCurrent, d: bytearray) -> None:
        m.thumb_tip = nc.ConvertToNegative_8bit(nc.ConvertBytesToInt(d,2,3))
        m.thumb_base = nc.ConvertToNegative_8bit(nc.ConvertBytesToInt(d,3,4))
        m.index_finger = nc.ConvertToNegative_8bit(nc.ConvertBytesToInt(d,4,5))
        m.middle_finger = nc.ConvertToNegative_8bit(nc.ConvertBytesToInt(d,5,6))
        m.ring_finger = nc.ConvertToNegative_8bit(nc.ConvertBytesToInt(d,6,7))
        m.pinky_finger = nc.ConvertToNegative_8bit(nc.ConvertBytesToInt(d,7,8))

    def encode_1B1_finget_pos_ctrl(self, msg: FingerPosCtrl) -> List[int]:
        return (
            [0] * 2
            + nc.ConvertToList_8bit(msg.thumb_tip, False)
            + nc.ConvertToList_8bit(msg.thumb_base, False)
            + nc.ConvertToList_8bit(msg.index_finger, False)
            + nc.ConvertToList_8bit(msg.middle_finger, False)
            + nc.ConvertToList_8bit(msg.ring_finger, False)
            + nc.ConvertToList_8bit(msg.pinky_finger, False)
        )
    
    def encode_1B2_finget_spd_ctrl(self, msg: FingerSpdCtrl) -> List[int]:
        return (
            [0] * 2
            + nc.ConvertToList_8bit(msg.thumb_tip)
            + nc.ConvertToList_8bit(msg.thumb_base)
            + nc.ConvertToList_8bit(msg.index_finger)
            + nc.ConvertToList_8bit(msg.middle_finger)
            + nc.ConvertToList_8bit(msg.ring_finger)
            + nc.ConvertToList_8bit(msg.pinky_finger)
        )

    def encode_1B3_finget_current_ctrl(self, msg: FingerCurrentCtrl) -> List[int]:
        return (
            [0] * 2
            + nc.ConvertToList_8bit(msg.thumb_tip)
            + nc.ConvertToList_8bit(msg.thumb_base)
            + nc.ConvertToList_8bit(msg.index_finger)
            + nc.ConvertToList_8bit(msg.middle_finger)
            + nc.ConvertToList_8bit(msg.ring_finger)
            + nc.ConvertToList_8bit(msg.pinky_finger)
        )

    def encode_1B5_finget_pos_time_ctrl(self, msg: FingerPosTimeCtrl) -> List[int]:
        return (
            nc.ConvertToList_8bit(msg.mode)
            + [0]
            + nc.ConvertToList_8bit(msg.thumb_tip, False)
            + nc.ConvertToList_8bit(msg.thumb_base, False)
            + nc.ConvertToList_8bit(msg.index_finger, False)
            + nc.ConvertToList_8bit(msg.middle_finger, False)
            + nc.ConvertToList_8bit(msg.ring_finger, False)
            + nc.ConvertToList_8bit(msg.pinky_finger, False)
        )


class Parser(TableDriven, ProtocolParserInterface):
    """
    revo2 parser/packer.

    This parser is designed to be fed by the SAME `rx_data` stream as the arm
    parser (i.e. the arm driver's comm callback).
    """
    
    if TYPE_CHECKING:
        # These attributes are created dynamically by TableDriven.parse_packet()
        # based on the RX map keys (the first element of each rx_map tuple).
        hand_status: Optional[MessageAbstract[FeedbackHandStatus]]
        finger_pos: Optional[MessageAbstract[FeedbackFingerPos]]
        finger_spd: Optional[MessageAbstract[FeedbackFingerSpd]]
        finger_current: Optional[MessageAbstract[FeedbackFingerCurrent]]
        finger_pos_ctrl: Optional[MessageAbstract[FingerPosCtrl]]
        finger_spd_ctrl: Optional[MessageAbstract[FingerSpdCtrl]]
        finger_current_ctrl: Optional[MessageAbstract[FingerCurrentCtrl]]
        finger_pos_time_ctrl: Optional[MessageAbstract[FingerPosTimeCtrl]]

    def __init__(
        self,
        fps_manager: FPSManager,
        logger_: Optional[logging.Logger] = None,
    ):
        super().__init__(fps_manager=fps_manager, logger_=logger_)
        self._codec = Codec()
        self._rx_map = self._build_rx_map()
        self._tx_map = self._build_tx_map()

    def _build_rx_map(
        self,
    ) -> Dict[int, Tuple[str, Type, Callable[[object, bytearray], None]]]:
        return {
            0x1C0: (
                "hand_status",
                FeedbackHandStatus,
                self._codec.decode_1C0_hand_status
            ),
            0x1C1: (
                "finger_pos",
                FeedbackFingerPos,
                self._codec.decode_1C1_finger_pos,
            ),
            0x1C2: (
                "finger_spd",
                FeedbackFingerSpd,
                self._codec.decode_1C2_finger_spd
            ),
            0x1C3: (
                "finger_current",
                FeedbackFingerCurrent,
                self._codec.decode_1C3_finger_current,
            ),
        }

    def _build_tx_map(
        self,
    ) -> Dict[str, Tuple[int, Callable[[object], Union[List[int], bytearray]]]]:
        return {
            FingerPosCtrl.type_: (
                0x1B1,
                self._codec.encode_1B1_finget_pos_ctrl
            ),
            FingerSpdCtrl.type_: (
                0x1B2,
                self._codec.encode_1B2_finget_spd_ctrl
            ),
            FingerCurrentCtrl.type_: (
                0x1B3,
                self._codec.encode_1B3_finget_current_ctrl
            ),
            FingerPosTimeCtrl.type_: (
                0x1B5,
                self._codec.encode_1B5_finget_pos_time_ctrl
            ),
        }
