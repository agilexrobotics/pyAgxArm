from typing import TYPE_CHECKING, Callable, Optional, Union, List, Dict, Tuple, Type

from ....core.table_driven import TableDriven
from ....core.protocol_parser_interface import ProtocolParserInterface
from .....msgs.effector.agx_gripper.default import (
    ArmMsgFeedbackGripper,
    ArmMsgFeedbackGripperTeachingPendantParam,
    ArmMsgGripperCtrl,
    ArmMsgGripperTeachingPendantParamConfig,
)
from .....msgs.piper.default import ArmMsgFeedbackRespSetInstruction, ArmMsgParamEnquiryAndConfig
from .....msgs.core.msg_abstract import MessageAbstract
from pyAgxArm.utiles.fps import FPSManager
from pyAgxArm.utiles.numeric_codec import NumericCodec as nc


class Codec:
    """Minimal codec for agx_gripper messages."""

    def decode_2A8_gripper(self, m: ArmMsgFeedbackGripper, d: bytearray) -> None:
        value = nc.ConvertToNegative_32bit(nc.ConvertBytesToInt(d, 0, 4))
        m.force = nc.ConvertToNegative_16bit(nc.ConvertBytesToInt(d, 4, 6)) * 1e-3
        m.status_code = nc.ConvertToNegative_8bit(
            nc.ConvertBytesToInt(d, 6, 7), False
        )
        mode = nc.ConvertToNegative_8bit(
            nc.ConvertBytesToInt(d, 7, 8), False
        )
        if mode == 0x01:
            m.mode = "angle"
            m.value = value * 1e-3
        else:
            m.mode = "width"
            m.value = value * 1e-6

    def decode_159_gripper_ctrl(self, m: ArmMsgGripperCtrl, d: bytearray) -> None:
        m.value = nc.ConvertToNegative_32bit(nc.ConvertBytesToInt(d, 0, 4)) * 1e-6
        m.force = nc.ConvertToNegative_16bit(nc.ConvertBytesToInt(d, 4, 6)) * 1e-3
        m.status_code = nc.ConvertToNegative_8bit(
            nc.ConvertBytesToInt(d, 6, 7), False
        )
        m.set_zero = nc.ConvertToNegative_8bit(nc.ConvertBytesToInt(d, 7, 8), False)

    def decode_476_resp_set_instruction(
        self, m: ArmMsgFeedbackRespSetInstruction, d: bytearray
    ) -> None:
        m.instruction_index = nc.ConvertToNegative_8bit(
            nc.ConvertBytesToInt(d, 0, 1), False
        )
        m.is_set_zero_successfully = nc.ConvertToNegative_8bit(
            nc.ConvertBytesToInt(d, 1, 2), False
        )

    def decode_47E_gripper_teaching_pendant_param(
        self, m: ArmMsgFeedbackGripperTeachingPendantParam, d: bytearray
    ) -> None:
        m.teaching_range_per = nc.ConvertToNegative_8bit(
            nc.ConvertBytesToInt(d, 0, 1), False
        )
        m.max_range_config = nc.ConvertToNegative_8bit(
            nc.ConvertBytesToInt(d, 1, 2), False
        ) * 1e-3
        m.teaching_friction = nc.ConvertToNegative_8bit(
            nc.ConvertBytesToInt(d, 2, 3), False
        )

    def encode_159_gripper_ctrl(self, msg: ArmMsgGripperCtrl) -> List[int]:
        return (
            nc.ConvertToList_32bit(msg.value, True)
            + nc.ConvertToList_16bit(msg.force, False)
            + nc.ConvertToList_8bit(msg.status_code, False)
            + nc.ConvertToList_8bit(msg.set_zero, False)
        )

    def encode_477_param_enquiry_and_config(
        self, msg: ArmMsgParamEnquiryAndConfig
    ) -> List[int]:
        return (
            nc.ConvertToList_8bit(msg.param_enquiry, False)
            + nc.ConvertToList_8bit(msg.param_setting, False)
            + nc.ConvertToList_8bit(msg.data_feedback_0x48x, False)
            + nc.ConvertToList_8bit(msg.end_load_param_setting_effective, False)
            + nc.ConvertToList_8bit(msg.set_end_load, False)
            + [0] * 3
        )
    
    def encode_47D_gripper_teaching_pendant_param_config(
        self, msg: ArmMsgGripperTeachingPendantParamConfig
    ) -> List[int]:
        return (
            nc.ConvertToList_8bit(msg.teaching_range_per, False)
            + nc.ConvertToList_8bit(msg.max_range_config, False)
            + nc.ConvertToList_8bit(msg.teaching_friction, False)
            + [0] * 5
        )


class Parser(TableDriven, ProtocolParserInterface):
    """
    agx_gripper parser/packer.

    This parser is designed to be fed by the SAME `rx_data` stream as the arm
    parser (i.e. the arm driver's comm callback). It only handles gripper-related
    CAN IDs and message types.
    """
    
    if TYPE_CHECKING:
        # These attributes are created dynamically by TableDriven.parse_packet()
        # based on the RX map keys (the first element of each rx_map tuple).
        gripper: Optional[MessageAbstract[ArmMsgFeedbackGripper]]
        gripper_ctrl_feedback: Optional[MessageAbstract[ArmMsgGripperCtrl]]
        gripper_teaching_pendant_param: Optional[
            MessageAbstract[ArmMsgFeedbackGripperTeachingPendantParam]
        ]
        resp_set_instruction: Optional[
            MessageAbstract[ArmMsgFeedbackRespSetInstruction]
        ]

    def __init__(self, fps_manager: FPSManager):
        super().__init__(fps_manager=fps_manager)
        self._codec = Codec()
        self._rx_map = self._build_rx_map()
        self._tx_map = self._build_tx_map()

    def _build_rx_map(
        self,
    ) -> Dict[int, Tuple[str, Type, Callable[[object, bytearray], None]]]:
        return {
            0x2A8: ("gripper", ArmMsgFeedbackGripper, self._codec.decode_2A8_gripper),
            0x159: (
                "gripper_ctrl_feedback",
                ArmMsgGripperCtrl,
                self._codec.decode_159_gripper_ctrl,
            ),
            0x476: (
                "resp_set_instruction",
                ArmMsgFeedbackRespSetInstruction,
                self._codec.decode_476_resp_set_instruction
            ),
            0x47E: (
                "gripper_teaching_pendant_param",
                ArmMsgFeedbackGripperTeachingPendantParam,
                self._codec.decode_47E_gripper_teaching_pendant_param,
            ),
        }

    def _build_tx_map(
        self,
    ) -> Dict[str, Tuple[int, Callable[[object], Union[List[int], bytearray]]]]:
        return {
            ArmMsgGripperCtrl.type_: (0x159, self._codec.encode_159_gripper_ctrl),
            ArmMsgParamEnquiryAndConfig.type_: (
                0x477,
                self._codec.encode_477_param_enquiry_and_config
            ),
            ArmMsgGripperTeachingPendantParamConfig.type_: (
                0x47D,
                self._codec.encode_47D_gripper_teaching_pendant_param_config,
            ),
        }
