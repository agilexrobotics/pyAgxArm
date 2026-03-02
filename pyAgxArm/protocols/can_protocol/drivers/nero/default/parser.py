from typing import TYPE_CHECKING, Callable, Optional, List, Dict, Tuple, Type

from ......utiles.fps import FPSManager
from ......utiles.numeric_codec import (
    NumericCodec as nc,
    DEG2RAD
)
from ....msgs.core.attritube_base import AttributeBase
from ....msgs.core.msg_abstract import MessageAbstract
from ...piper.default.parser import Codec as PiperCodec
from ...piper.default.parser import Parser as PiperParser
from ....msgs.nero.default import (
    ArmMsgFeedbackJointStates7,
    ArmMsgJointCtrl7,
    ArmMsgJointMitCtrl7,
    ArmMsgFeedbackHighSpd7,
    ArmMsgFeedbackLowSpd7,
    ArmMsgFeedbackStatus,
    ArmMsgModeCtrl,
    ArmMsgFeedbackMasterJointStates1,
    ArmMsgFeedbackMasterJointStates2,
    ArmMsgFeedbackMasterJointStates3,
    ArmMsgFeedbackMasterJointStates4,
    ArmMsgFeedbackMasterJointStates5,
    ArmMsgFeedbackMasterJointStates6,
    ArmMsgFeedbackMasterJointStates7,
)
from ...core.protocol_parser_abstract import DriverAPIOptions, DriverAPIProtoAdapter
from ....msgs.core import StrStruct

class NeroDefaultDriverAPIOptions(DriverAPIOptions):

    class PAYLOAD(StrStruct):
        EMPTY = "empty"
        HALF = "half"
        FULL = "full"

    class MOTION_MODE(StrStruct):
        P = "p"
        J = "j"
        L = "l"
        C = "c"
        MIT = "mit"
        JS = "js"

class NeroDefaultDriverAPIProtoAdapter(DriverAPIProtoAdapter):

    _MOVE_CODE = {
        NeroDefaultDriverAPIOptions.MOTION_MODE.P: ArmMsgModeCtrl.Enums.MotionMode.P,
        NeroDefaultDriverAPIOptions.MOTION_MODE.J: ArmMsgModeCtrl.Enums.MotionMode.J,
        NeroDefaultDriverAPIOptions.MOTION_MODE.L: ArmMsgModeCtrl.Enums.MotionMode.L,
        NeroDefaultDriverAPIOptions.MOTION_MODE.C: ArmMsgModeCtrl.Enums.MotionMode.C,
        NeroDefaultDriverAPIOptions.MOTION_MODE.MIT: ArmMsgModeCtrl.Enums.MotionMode.MIT,
        NeroDefaultDriverAPIOptions.MOTION_MODE.JS: ArmMsgModeCtrl.Enums.MotionMode.J,
    }

    _MIT_CODE = {
        NeroDefaultDriverAPIOptions.MOTION_MODE.MIT: ArmMsgModeCtrl.Enums.MitMode.MIT,
        NeroDefaultDriverAPIOptions.MOTION_MODE.JS: ArmMsgModeCtrl.Enums.MitMode.MIT,
    }

    @classmethod
    def motion_mode(cls, value: str) -> tuple[int, int]:
        return cls._MOVE_CODE[value]
    
    @classmethod
    def mit_mode(cls, value: str) -> tuple[int, int]:
        return cls._MIT_CODE.get(value, ArmMsgModeCtrl.Enums.MitMode.POS_VEL)

class Codec(PiperCodec):
    """Nero 编解码器：在 Piper 基础上扩展第 7 轴相关编解码。"""

    def decode_2A9_joint_7(self, m: ArmMsgFeedbackJointStates7, d: bytearray) -> None:
        m.joint_7 = (
            nc.ConvertToNegative_32bit(nc.ConvertBytesToInt(d, 0, 4))
            * 1e-3
            * DEG2RAD
        )

    def decode_master_joint_state_by_index(self, index: int) -> Callable[[object, bytearray], None]:
        def decoder(m: AttributeBase, d: bytearray) -> None:
            setattr(m, f"joint_{index}", nc.from_bytes_to_float(d))
        return decoder

    def encode_151_mode_ctrl(self, msg: ArmMsgModeCtrl):
        d = super().encode_151_mode_ctrl(msg)
        d[6] = nc.ConvertToList_8bit(msg.enable_can_push, False)[0]
        return d

    def encode_170_joint_ctrl_7(self, msg: ArmMsgJointCtrl7) -> List[int]:
        return nc.ConvertToList_32bit(msg.joint_7) + [0] * 4


class Parser(PiperParser):
    # Override message classes used by PiperParser driver-side builders.
    _MSG_JointCtrl7 = ArmMsgJointCtrl7

    _MSG_JointMitCtrlByIndex: Dict[int, Type[AttributeBase]] = {
        **PiperParser._MSG_JointMitCtrlByIndex,
        7: ArmMsgJointMitCtrl7,
    }
    
    if TYPE_CHECKING:
        arm_status: Optional[MessageAbstract[ArmMsgFeedbackStatus]]
        joint_7: Optional[MessageAbstract[ArmMsgFeedbackJointStates7]]
        motor_state_7: Optional[MessageAbstract[ArmMsgFeedbackHighSpd7]]
        driver_state_7: Optional[MessageAbstract[ArmMsgFeedbackLowSpd7]]

        master_joint_1: Optional[MessageAbstract[ArmMsgFeedbackMasterJointStates1]]
        master_joint_2: Optional[MessageAbstract[ArmMsgFeedbackMasterJointStates2]]
        master_joint_3: Optional[MessageAbstract[ArmMsgFeedbackMasterJointStates3]]
        master_joint_4: Optional[MessageAbstract[ArmMsgFeedbackMasterJointStates4]]
        master_joint_5: Optional[MessageAbstract[ArmMsgFeedbackMasterJointStates5]]
        master_joint_6: Optional[MessageAbstract[ArmMsgFeedbackMasterJointStates6]]
        master_joint_7: Optional[MessageAbstract[ArmMsgFeedbackMasterJointStates7]]

    def __init__(self, fps_manager: FPSManager, codec: Optional[Codec] = None):
        # Reuse Piper Parser init; only replace codec with Nero version.
        super().__init__(fps_manager=fps_manager, codec=codec or Codec())
        self._codec = codec or Codec()

    def _build_rx_map(
        self,
    ) -> Dict[int, Tuple[str, Type, Callable[[object, bytearray], None]]]:
        rx = super()._build_rx_map()

        # Nero 精简协议：移除Nero不支持的接收映射
        for can_id in (0x155, 0x156, 0x157, 0x473,
                       0x476, 0x478, 0x47B, 0x47C,
                       0x4AF,):
            rx.pop(can_id, None)

        # Nero 增量：第 7 轴相关 CAN-ID
        rx.update(
            {
                0x257: (
                    "motor_state_7",
                    ArmMsgFeedbackHighSpd7,
                    self._codec.decode_high_spd
                ),
                0x267: (
                    "driver_state_7",
                    ArmMsgFeedbackLowSpd7,
                    self._codec.decode_low_spd
                ),
                0x2A9: (
                    "joint_7",
                    ArmMsgFeedbackJointStates7,
                    self._codec.decode_2A9_joint_7
                ),

                # 覆盖具有差异的消息
                0x2A1: (
                    "arm_status",
                    ArmMsgFeedbackStatus,
                    self._codec.decode_2A1_status
                ),

                # 主臂关节消息
                0x501: (
                    "master_joint_1",
                    ArmMsgFeedbackMasterJointStates1,
                    self._codec.decode_master_joint_state_by_index(1)
                ),
                0x502: (
                    "master_joint_2",
                    ArmMsgFeedbackMasterJointStates2,
                    self._codec.decode_master_joint_state_by_index(2)
                ),
                0x503: (
                    "master_joint_3",
                    ArmMsgFeedbackMasterJointStates3,
                    self._codec.decode_master_joint_state_by_index(3)
                ),
                0x504: (
                    "master_joint_4",
                    ArmMsgFeedbackMasterJointStates4,
                    self._codec.decode_master_joint_state_by_index(4)
                ),
                0x505: (
                    "master_joint_5",
                    ArmMsgFeedbackMasterJointStates5,
                    self._codec.decode_master_joint_state_by_index(5)
                ),
                0x506: (
                    "master_joint_6",
                    ArmMsgFeedbackMasterJointStates6,
                    self._codec.decode_master_joint_state_by_index(6)
                ),
                0x507: (
                    "master_joint_7",
                    ArmMsgFeedbackMasterJointStates7,
                    self._codec.decode_master_joint_state_by_index(7)
                ),
            }
        )
        return rx

    def _build_tx_map(self) -> Dict[str, Tuple[int, Callable]]:
        tx = super()._build_tx_map()

        # Nero 精简协议：移除Nero不支持的发送映射
        remove_can_ids = {0x191, 0x472,
                          0x474, 0x475, 0x477,
                          0x479, 0x47A, 0x4AF,}
        for msg_type, (can_id, _enc) in list(tx.items()):
            if can_id in remove_can_ids:
                tx.pop(msg_type, None)

        # Nero 增量：第 7 轴控制
        tx.update(
            {
                ArmMsgJointCtrl7.type_: (
                    0x170,
                    self._codec.encode_170_joint_ctrl_7
                ),
                ArmMsgJointMitCtrl7.type_: (
                    0x160,
                    self._codec.pack_joint_mit_ctrl
                ),
            }
        )
        return tx
