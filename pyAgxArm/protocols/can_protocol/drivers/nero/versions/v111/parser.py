from typing import Callable, Dict, Optional, Tuple, Type

from .......utiles.numeric_codec import NumericCodec as nc
from .....msgs.nero.versions import (
    ArmMsgFeedbackStatusV111,
    ArmMsgModeCtrlV111,
)
from ...default.parser import (
    Codec as DefaultCodec,
    Parser as DefaultParser,
    NeroDefaultDriverAPIProtoAdapter,
    NeroDefaultDriverAPIOptions,
)
from .....msgs.piper.default import (
    ArmMsgJointMitCtrl,
)


class NeroV111DriverAPIProtoAdapter(NeroDefaultDriverAPIProtoAdapter):
    _MOVE_CODE = {
        **NeroDefaultDriverAPIProtoAdapter._MOVE_CODE,
        NeroDefaultDriverAPIOptions.MOTION_MODE.MIT: ArmMsgModeCtrlV111.Enums.MotionMode.MIT,
    }


class Codec(DefaultCodec):
    """v111+ codec: 12-bit t_ff, no CRC."""

    def pack_joint_mit_ctrl(self, joint_mit_ctrl: ArmMsgJointMitCtrl) -> bytearray:
        """v188+: 12-bit t_ff, no CRC.

        Byte layout (8 bytes total):
            Byte 0-1: p_des  [15:0]   (16 bit)
            Byte 2  : v_des  [11:4]
            Byte 3  : v_des  [3:0]  | kp [11:8]
            Byte 4  : kp     [7:0]
            Byte 5  : kd     [11:4]
            Byte 6  : kd     [3:0]  | t_ff [11:8]
            Byte 7  : t_ff   [7:0]
        """
        data = bytearray(
            nc.ConvertToList_16bit(joint_mit_ctrl.p_des, False)
            + nc.ConvertToList_8bit(
                ((joint_mit_ctrl.v_des >> 4) & 0xFF), False
            )
            + nc.ConvertToList_8bit(
                (
                    (((joint_mit_ctrl.v_des & 0xF) << 4) & 0xF0)
                    | ((joint_mit_ctrl.kp >> 8) & 0x0F)
                ),
                False,
            )
            + nc.ConvertToList_8bit(joint_mit_ctrl.kp & 0xFF, False)
            + nc.ConvertToList_8bit(
                ((joint_mit_ctrl.kd >> 4) & 0xFF), False
            )
            + nc.ConvertToList_8bit(
                (
                    (((joint_mit_ctrl.kd & 0xF) << 4) & 0xF0)
                    | ((joint_mit_ctrl.t_ff >> 8) & 0x0F)
                ),
                False,
            )
            + nc.ConvertToList_8bit(joint_mit_ctrl.t_ff & 0xFF, False)
        )
        return data


class Parser(DefaultParser):
    """v111+ parser using CodecV111."""

    def __init__(self, fps_manager, codec: Optional[Codec] = None):
        super().__init__(fps_manager, codec=codec or Codec())

    def _build_rx_map(
        self,
    ) -> Dict[int, Tuple[str, Type, Callable[[object, bytearray], None]]]:
        rx_map = super()._build_rx_map()
        rx_map[0x2A1] = (
            "arm_status",
            ArmMsgFeedbackStatusV111,
            self._codec.decode_2A1_status,
        )
        return rx_map

    def _build_tx_map(self) -> Dict[str, Tuple[int, Callable]]:
        tx_map = super()._build_tx_map()
        tx_map[ArmMsgModeCtrlV111.type_] = (
            0x151,
            self._codec.encode_151_mode_ctrl,
        )
        return tx_map
