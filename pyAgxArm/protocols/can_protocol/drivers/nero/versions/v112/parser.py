from typing import TYPE_CHECKING, Callable, Dict, Optional, Tuple, Type

from .......utiles.numeric_codec import NumericCodec as nc, DEG2RAD
from .....msgs.core.attritube_base import AttributeBase
from .....msgs.core.msg_abstract import MessageAbstract
from .....msgs.piper.default import (
    ArmMsgJointCtrl12,
    ArmMsgJointCtrl34,
    ArmMsgJointCtrl56,
    ArmMsgCPVSettingsAndQueries1,
    ArmMsgCPVSettingsAndQueries2,
    ArmMsgCPVSettingsAndQueries3,
    ArmMsgCPVSettingsAndQueries4,
    ArmMsgCPVSettingsAndQueries5,
    ArmMsgCPVSettingsAndQueries6,
    ArmMsgFeedbackCPVResponse1,
    ArmMsgFeedbackCPVResponse2,
    ArmMsgFeedbackCPVResponse3,
    ArmMsgFeedbackCPVResponse4,
    ArmMsgFeedbackCPVResponse5,
    ArmMsgFeedbackCPVResponse6,
)
from .....msgs.nero.default import (
    ArmMsgJointCtrl7,
    ArmMsgCPVSettingsAndQueries7,
    ArmMsgFeedbackCPVResponse7,
)
from .....msgs.nero.versions.v112 import (
    ArmMsgFeedbackIKJointStates12,
    ArmMsgFeedbackIKJointStates34,
    ArmMsgFeedbackIKJointStates56,
    ArmMsgFeedbackIKJointStates7,
)
from ....piper.default.parser import Parser as PiperParser
from ..v111.parser import (
    Codec as V111Codec,
    Parser as V111Parser,
    NeroV111DriverAPIProtoAdapter,
)
from ...default.parser import NeroDefaultDriverAPIOptions
from ....core.protocol_parser_abstract import DriverAPIOptions
from .....msgs.core import StrStruct
from .....msgs.nero.default import ArmMsgModeCtrl


class NeroV112DriverAPIOptions(DriverAPIOptions):
    class PAYLOAD(NeroDefaultDriverAPIOptions.PAYLOAD):
        pass

    class MOTION_MODE(StrStruct):
        P = "p"
        J = "j"
        L = "l"
        C = "c"
        MIT = "mit"
        JS = "js"
        CPV = "cpv"


class NeroV112DriverAPIProtoAdapter(NeroV111DriverAPIProtoAdapter):
    _MOVE_CODE = {
        **NeroV111DriverAPIProtoAdapter._MOVE_CODE,
        NeroV112DriverAPIOptions.MOTION_MODE.CPV: (
            ArmMsgModeCtrl.Enums.MotionMode.CPV
        ),
    }


class Codec(V111Codec):
    """Nero v112 codec."""

    def decode_170_joint_ctrl_7(self, m: ArmMsgJointCtrl7, d: bytearray) -> None:
        m.joint_7 = (
            nc.ConvertToNegative_32bit(nc.ConvertBytesToInt(d, 0, 4))
            * 1e-3
            * DEG2RAD
        )


class Parser(V111Parser):
    """Nero v112 parser."""

    _MSG_CPVSettingsAndQueriesByIndex: Dict[int, Type[AttributeBase]] = {
        **PiperParser._MSG_CPVSettingsAndQueriesByIndex,
        7: ArmMsgCPVSettingsAndQueries7,
    }

    if TYPE_CHECKING:
        leader_joint_12: Optional[MessageAbstract[ArmMsgJointCtrl12]]
        leader_joint_34: Optional[MessageAbstract[ArmMsgJointCtrl34]]
        leader_joint_56: Optional[MessageAbstract[ArmMsgJointCtrl56]]
        leader_joint_7: Optional[MessageAbstract[ArmMsgJointCtrl7]]

        cpv_response_7: Optional[MessageAbstract[ArmMsgFeedbackCPVResponse7]]

        ik_joint_12: Optional[MessageAbstract[ArmMsgFeedbackIKJointStates12]]
        ik_joint_34: Optional[MessageAbstract[ArmMsgFeedbackIKJointStates34]]
        ik_joint_56: Optional[MessageAbstract[ArmMsgFeedbackIKJointStates56]]
        ik_joint_7: Optional[MessageAbstract[ArmMsgFeedbackIKJointStates7]]

    def __init__(
        self,
        fps_manager,
        codec: Optional[Codec] = None,
        config: Optional[dict] = None,
    ):
        super().__init__(fps_manager, codec=codec or Codec(), config=config)
        self._codec = codec or Codec()

    def _build_rx_map(
        self,
    ) -> Dict[int, Tuple[str, Type, Callable[[object, bytearray], None]]]:
        rx = super()._build_rx_map()
        for can_id in (0x501, 0x502, 0x503, 0x504, 0x505, 0x506, 0x507):
            rx.pop(can_id, None)
        rx.update(
            {
                0x181: (
                    "cpv_response_1",
                    ArmMsgFeedbackCPVResponse1,
                    self._codec.decode_cpv_response,
                ),
                0x182: (
                    "cpv_response_2",
                    ArmMsgFeedbackCPVResponse2,
                    self._codec.decode_cpv_response,
                ),
                0x183: (
                    "cpv_response_3",
                    ArmMsgFeedbackCPVResponse3,
                    self._codec.decode_cpv_response,
                ),
                0x184: (
                    "cpv_response_4",
                    ArmMsgFeedbackCPVResponse4,
                    self._codec.decode_cpv_response,
                ),
                0x185: (
                    "cpv_response_5",
                    ArmMsgFeedbackCPVResponse5,
                    self._codec.decode_cpv_response,
                ),
                0x186: (
                    "cpv_response_6",
                    ArmMsgFeedbackCPVResponse6,
                    self._codec.decode_cpv_response,
                ),
                0x187: (
                    "cpv_response_7",
                    ArmMsgFeedbackCPVResponse7,
                    self._codec.decode_cpv_response,
                ),
                0x155: (
                    "leader_joint_12",
                    ArmMsgJointCtrl12,
                    self._codec.decode_155_joint_ctrl_12,
                ),
                0x156: (
                    "leader_joint_34",
                    ArmMsgJointCtrl34,
                    self._codec.decode_156_joint_ctrl_34,
                ),
                0x157: (
                    "leader_joint_56",
                    ArmMsgJointCtrl56,
                    self._codec.decode_157_joint_ctrl_56,
                ),
                0x170: (
                    "leader_joint_7",
                    ArmMsgJointCtrl7,
                    self._codec.decode_170_joint_ctrl_7,
                ),
                0x2AA: (
                    "ik_joint_12",
                    ArmMsgFeedbackIKJointStates12,
                    self._codec.decode_2A5_joint_12,
                ),
                0x2AB: (
                    "ik_joint_34",
                    ArmMsgFeedbackIKJointStates34,
                    self._codec.decode_2A6_joint_34,
                ),
                0x2AC: (
                    "ik_joint_56",
                    ArmMsgFeedbackIKJointStates56,
                    self._codec.decode_2A7_joint_56,
                ),
                0x2AD: (
                    "ik_joint_7",
                    ArmMsgFeedbackIKJointStates7,
                    self._codec.decode_2A9_joint_7,
                ),
            }
        )
        return rx

    def _build_tx_map(self) -> Dict[str, Tuple[int, Callable]]:
        tx = super()._build_tx_map()
        tx.update(
            {
                ArmMsgCPVSettingsAndQueries1.type_: (
                    0x181,
                    self._codec.encode_cpv_settings_and_queries,
                ),
                ArmMsgCPVSettingsAndQueries2.type_: (
                    0x182,
                    self._codec.encode_cpv_settings_and_queries,
                ),
                ArmMsgCPVSettingsAndQueries3.type_: (
                    0x183,
                    self._codec.encode_cpv_settings_and_queries,
                ),
                ArmMsgCPVSettingsAndQueries4.type_: (
                    0x184,
                    self._codec.encode_cpv_settings_and_queries,
                ),
                ArmMsgCPVSettingsAndQueries5.type_: (
                    0x185,
                    self._codec.encode_cpv_settings_and_queries,
                ),
                ArmMsgCPVSettingsAndQueries6.type_: (
                    0x186,
                    self._codec.encode_cpv_settings_and_queries,
                ),
                ArmMsgCPVSettingsAndQueries7.type_: (
                    0x187,
                    self._codec.encode_cpv_settings_and_queries,
                ),
            }
        )
        return tx
