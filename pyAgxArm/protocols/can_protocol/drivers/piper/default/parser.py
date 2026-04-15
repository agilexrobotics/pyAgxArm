from typing import TYPE_CHECKING, Callable, Optional, List, Dict, Tuple, Type

from typing_extensions import Final, Literal
from ......utiles.fps import FPSManager
from ......utiles.numeric_codec import (
    NumericCodec as nc,
    DEG2RAD
)
from ....msgs.core.attritube_base import AttributeBase
from ....msgs.core.msg_abstract import MessageAbstract
from ....msgs.piper.default import *
from ...core.protocol_parser_interface import ProtocolParserInterface
from ...core.protocol_parser_abstract import DriverAPIOptions, DriverAPIProtoAdapter
from ...core.table_driven import TableDriven
from ....msgs.core import StrStruct

class PiperDefaultDriverAPIOptions(DriverAPIOptions):
    class INSTALLATION_POS(StrStruct):
        HORIZONTAL: Final = "horizontal"
        LEFT: Final = "left"
        RIGHT: Final = "right"

    class PAYLOAD(StrStruct):
        EMPTY: Final = "empty"
        HALF: Final = "half"
        FULL: Final = "full"

    class MOTION_MODE(StrStruct):
        P: Final = "p"
        J: Final = "j"
        L: Final = "l"
        C: Final = "c"
        MIT: Final = "mit"
        JS: Final = "js"
        CPV: Final = "cpv"

class PiperDefaultDriverAPIProtoAdapter(DriverAPIProtoAdapter):
    _INSTALL_POS_CODE = {
        PiperDefaultDriverAPIOptions.INSTALLATION_POS.HORIZONTAL: ArmMsgModeCtrl.Enums.InstallationPos.HORIZONTAL,
        PiperDefaultDriverAPIOptions.INSTALLATION_POS.LEFT: ArmMsgModeCtrl.Enums.InstallationPos.LEFT,
        PiperDefaultDriverAPIOptions.INSTALLATION_POS.RIGHT: ArmMsgModeCtrl.Enums.InstallationPos.RIGHT,
    }

    _MOVE_CODE = {
        PiperDefaultDriverAPIOptions.MOTION_MODE.P: ArmMsgModeCtrl.Enums.MotionMode.P,
        PiperDefaultDriverAPIOptions.MOTION_MODE.J: ArmMsgModeCtrl.Enums.MotionMode.J,
        PiperDefaultDriverAPIOptions.MOTION_MODE.L: ArmMsgModeCtrl.Enums.MotionMode.L,
        PiperDefaultDriverAPIOptions.MOTION_MODE.C: ArmMsgModeCtrl.Enums.MotionMode.C,
        PiperDefaultDriverAPIOptions.MOTION_MODE.MIT: ArmMsgModeCtrl.Enums.MotionMode.MIT,
        PiperDefaultDriverAPIOptions.MOTION_MODE.JS: ArmMsgModeCtrl.Enums.MotionMode.J,
        PiperDefaultDriverAPIOptions.MOTION_MODE.CPV: ArmMsgModeCtrl.Enums.MotionMode.CPV,
    }

    _MIT_CODE = {
        PiperDefaultDriverAPIOptions.MOTION_MODE.MIT: ArmMsgModeCtrl.Enums.MitMode.MIT,
        PiperDefaultDriverAPIOptions.MOTION_MODE.JS: ArmMsgModeCtrl.Enums.MitMode.MIT,
    }

    _PAYLOAD_CODE = {
        PiperDefaultDriverAPIOptions.PAYLOAD.EMPTY: ArmMsgParamEnquiryAndConfig.Enums.SetPayLoadLevel.EMPTY,
        PiperDefaultDriverAPIOptions.PAYLOAD.HALF: ArmMsgParamEnquiryAndConfig.Enums.SetPayLoadLevel.HALF,
        PiperDefaultDriverAPIOptions.PAYLOAD.FULL: ArmMsgParamEnquiryAndConfig.Enums.SetPayLoadLevel.FULL,
    }

    @classmethod
    def installation_pos(cls, value: str) -> int:
        return cls._INSTALL_POS_CODE[value]

    @classmethod
    def motion_mode(cls, value: str) -> Tuple[int, int]:
        return cls._MOVE_CODE[value]
    
    @classmethod
    def mit_mode(cls, value: str) -> Tuple[int, int]:
        return cls._MIT_CODE.get(value, ArmMsgModeCtrl.Enums.MitMode.POS_VEL)

    @classmethod
    def payload(cls, value: str) -> int:
        return cls._PAYLOAD_CODE[value]

class Codec:
    """
    Piper 编解码器。

    - 目标：集中管理 decode/encode，便于继承、复用与覆写
    - 约定：decoder 形如 (msg, can_data) -> None；encoder 形如 (msg) -> data
    """
    # -------------------------
    # Common codec helpers
    # -------------------------
    def decode_high_spd(self, motor_state: ArmMsgFeedbackHighSpd, can_data: bytearray) -> None:
        """高速反馈通用解码：写入 velocity/current/position"""
        motor_state.velocity = (
            nc.ConvertToNegative_16bit(nc.ConvertBytesToInt(can_data, 0, 2)) * 1e-3
        )
        motor_state.current = (
            nc.ConvertToNegative_16bit(nc.ConvertBytesToInt(can_data, 2, 4)) * 1e-3
        )
        motor_state.position = (
            nc.ConvertToNegative_32bit(nc.ConvertBytesToInt(can_data, 4, 8)) * 1e-3
        )

    def decode_low_spd(self, driver_state: ArmMsgFeedbackLowSpd, can_data: bytearray) -> None:
        """低速反馈通用解码：写入 vol/foc_temp/motor_temp/foc_status_code/bus_current"""
        driver_state.vol = (
            nc.ConvertToNegative_16bit(nc.ConvertBytesToInt(can_data, 0, 2), False)
            * 1e-1
        )
        driver_state.foc_temp = (
            nc.ConvertToNegative_16bit(nc.ConvertBytesToInt(can_data, 2, 4)) * 1e-0
        )
        driver_state.motor_temp = (
            nc.ConvertToNegative_8bit(nc.ConvertBytesToInt(can_data, 4, 5)) * 1e-0
        )
        driver_state.foc_status_code = nc.ConvertToNegative_8bit(
            nc.ConvertBytesToInt(can_data, 5, 6), False
        )
        driver_state.bus_current = (
            nc.ConvertToNegative_16bit(nc.ConvertBytesToInt(can_data, 6, 8), False)
            * 1e-3
        )

    def pack_joint_mit_ctrl(self, joint_mit_ctrl: ArmMsgJointMitCtrl) -> bytearray:
        """
        MIT 控制通用打包（关节 1~6 结构一致）：返回 data（bytearray）。
        joint_mit_ctrl 需具备 p_des/v_des/kp/kd/t_ff/crc 字段。
        """
        data = bytearray(
            nc.ConvertToList_16bit(joint_mit_ctrl.p_des, False)
            + nc.ConvertToList_8bit(((joint_mit_ctrl.v_des >> 4) & 0xFF), False)
            + nc.ConvertToList_8bit(
                (
                    (((joint_mit_ctrl.v_des & 0xF) << 4) & 0xF0)
                    | ((joint_mit_ctrl.kp >> 8) & 0x0F)
                ),
                False,
            )
            + nc.ConvertToList_8bit(joint_mit_ctrl.kp & 0xFF, False)
            + nc.ConvertToList_8bit((joint_mit_ctrl.kd >> 4) & 0xFF, False)
            + nc.ConvertToList_8bit(
                (
                    (((joint_mit_ctrl.kd & 0xF) << 4) & 0xF0)
                    | ((joint_mit_ctrl.t_ff >> 4) & 0x0F)
                ),
                False,
            )
        )
        crc = (
            data[0] ^ data[1] ^ data[2] ^ data[3] ^ data[4] ^ data[5] ^ data[6]
        ) & 0x0F
        joint_mit_ctrl.crc = crc
        data.extend(
            nc.ConvertToList_8bit((((joint_mit_ctrl.t_ff << 4) & 0xF0) | crc), False)
        )
        return data

    # -------------------------
    # RX decoders (CAN -> msg)
    # -------------------------
    def decode_2A1_status(self, m: ArmMsgFeedbackStatus, d: bytearray) -> None:
        m.ctrl_mode = nc.ConvertToNegative_8bit(nc.ConvertBytesToInt(d, 0, 1), False)
        m.arm_status = nc.ConvertToNegative_8bit(nc.ConvertBytesToInt(d, 1, 2), False)
        m.mode_feedback = nc.ConvertToNegative_8bit(nc.ConvertBytesToInt(d, 2, 3), False)
        m.teach_status = nc.ConvertToNegative_8bit(nc.ConvertBytesToInt(d, 3, 4), False)
        m.motion_status = nc.ConvertToNegative_8bit(
            nc.ConvertBytesToInt(d, 4, 5), False
        )
        m.trajectory_num = nc.ConvertToNegative_8bit(
            nc.ConvertBytesToInt(d, 5, 6), False
        )
        m.err_code = nc.ConvertToNegative_16bit(nc.ConvertBytesToInt(d, 6, 8), False)

    def decode_2A2_end_pose_xy(self, m: ArmMsgFeedbackEndPoseXY, d: bytearray) -> None:
        m.X_axis = nc.ConvertToNegative_32bit(nc.ConvertBytesToInt(d, 0, 4)) * 1e-6
        m.Y_axis = nc.ConvertToNegative_32bit(nc.ConvertBytesToInt(d, 4, 8)) * 1e-6

    def decode_2A3_end_pose_zrx(
        self, m: ArmMsgFeedbackEndPoseZRX, d: bytearray
    ) -> None:
        m.Z_axis = nc.ConvertToNegative_32bit(nc.ConvertBytesToInt(d, 0, 4)) * 1e-6
        m.RX_axis = (
            nc.ConvertToNegative_32bit(nc.ConvertBytesToInt(d, 4, 8))
            * 1e-3
            * DEG2RAD
        )

    def decode_2A4_end_pose_ryrz(
        self, m: ArmMsgFeedbackEndPoseRYRZ, d: bytearray
    ) -> None:
        m.RY_axis = (
            nc.ConvertToNegative_32bit(nc.ConvertBytesToInt(d, 0, 4))
            * 1e-3
            * DEG2RAD
        )
        m.RZ_axis = (
            nc.ConvertToNegative_32bit(nc.ConvertBytesToInt(d, 4, 8))
            * 1e-3
            * DEG2RAD
        )

    def decode_2A5_joint_12(self, m: ArmMsgFeedbackJointStates12, d: bytearray) -> None:
        m.joint_1 = (
            nc.ConvertToNegative_32bit(nc.ConvertBytesToInt(d, 0, 4))
            * 1e-3
            * DEG2RAD
        )
        m.joint_2 = (
            nc.ConvertToNegative_32bit(nc.ConvertBytesToInt(d, 4, 8))
            * 1e-3
            * DEG2RAD
        )

    def decode_2A6_joint_34(self, m: ArmMsgFeedbackJointStates34, d: bytearray) -> None:
        m.joint_3 = (
            nc.ConvertToNegative_32bit(nc.ConvertBytesToInt(d, 0, 4))
            * 1e-3
            * DEG2RAD
        )
        m.joint_4 = (
            nc.ConvertToNegative_32bit(nc.ConvertBytesToInt(d, 4, 8))
            * 1e-3
            * DEG2RAD
        )

    def decode_2A7_joint_56(self, m: ArmMsgFeedbackJointStates56, d: bytearray) -> None:
        m.joint_5 = (
            nc.ConvertToNegative_32bit(nc.ConvertBytesToInt(d, 0, 4))
            * 1e-3
            * DEG2RAD
        )
        m.joint_6 = (
            nc.ConvertToNegative_32bit(nc.ConvertBytesToInt(d, 4, 8))
            * 1e-3
            * DEG2RAD
        )

    def decode_473_motor_angle_limit_max_spd(
        self, m: ArmMsgFeedbackAllCurrentMotorAngleLimitMaxSpd, d: bytearray
    ) -> None:
        m.joint_index = nc.ConvertToNegative_8bit(nc.ConvertBytesToInt(d, 0, 1), False)
        # Robustness: ignore corrupted/out-of-range joint index instead of crashing.
        if not (1 <= m.joint_index <= len(m.joints)):
            return
        m.joints[m.joint_index - 1].max_angle_limit = (
            nc.ConvertToNegative_16bit(nc.ConvertBytesToInt(d, 1, 3))
            * 1e-1
            * DEG2RAD
        )
        m.joints[m.joint_index - 1].min_angle_limit = (
            nc.ConvertToNegative_16bit(nc.ConvertBytesToInt(d, 3, 5))
            * 1e-1
            * DEG2RAD
        )
        m.joints[m.joint_index - 1].max_joint_spd = (
            nc.ConvertToNegative_16bit(nc.ConvertBytesToInt(d, 5, 7), False) * 1e-2
        )

    def decode_476_resp_set_instruction(
        self, m: ArmMsgFeedbackRespSetInstruction, d: bytearray
    ) -> None:
        m.instruction_index = nc.ConvertToNegative_8bit(
            nc.ConvertBytesToInt(d, 0, 1), False
        )
        m.is_set_zero_successfully = nc.ConvertToNegative_8bit(
            nc.ConvertBytesToInt(d, 1, 2), False
        )

    def decode_478_end_vel_acc_param(
        self, m: ArmMsgFeedbackCurrentEndVelAccParam, d: bytearray
    ) -> None:
        m.end_max_linear_vel = (
            nc.ConvertToNegative_16bit(nc.ConvertBytesToInt(d, 0, 2), False) * 1e-3
        )
        m.end_max_angular_vel = (
            nc.ConvertToNegative_16bit(nc.ConvertBytesToInt(d, 2, 4), False) * 1e-3
        )
        m.end_max_linear_acc = (
            nc.ConvertToNegative_16bit(nc.ConvertBytesToInt(d, 4, 6), False) * 1e-3
        )
        m.end_max_angular_acc = (
            nc.ConvertToNegative_16bit(nc.ConvertBytesToInt(d, 6, 8), False) * 1e-3
        )

    def decode_47B_crash_protection_rating(
        self, m: ArmMsgFeedbackCrashProtectionRating, d: bytearray
    ) -> None:
        m.joint_1 = nc.ConvertToNegative_8bit(nc.ConvertBytesToInt(d, 0, 1), False)
        m.joint_2 = nc.ConvertToNegative_8bit(nc.ConvertBytesToInt(d, 1, 2), False)
        m.joint_3 = nc.ConvertToNegative_8bit(nc.ConvertBytesToInt(d, 2, 3), False)
        m.joint_4 = nc.ConvertToNegative_8bit(nc.ConvertBytesToInt(d, 3, 4), False)
        m.joint_5 = nc.ConvertToNegative_8bit(nc.ConvertBytesToInt(d, 4, 5), False)
        m.joint_6 = nc.ConvertToNegative_8bit(nc.ConvertBytesToInt(d, 5, 6), False)
        # Nero 等 7 轴变体：若字段存在则解第 7 轴
        if hasattr(m, "joint_7"):
            m.joint_7 = nc.ConvertToNegative_8bit(nc.ConvertBytesToInt(d, 6, 7), False)

    def decode_488_joint_assistance_rating(
        self, m: ArmMsgFeedbackJointAssistanceRating, d: bytearray
    ) -> None:
        m.joint_1 = nc.ConvertToNegative_8bit(nc.ConvertBytesToInt(d, 0, 1), False)
        m.joint_2 = nc.ConvertToNegative_8bit(nc.ConvertBytesToInt(d, 1, 2), False)
        m.joint_3 = nc.ConvertToNegative_8bit(nc.ConvertBytesToInt(d, 2, 3), False)
        m.joint_4 = nc.ConvertToNegative_8bit(nc.ConvertBytesToInt(d, 3, 4), False)
        m.joint_5 = nc.ConvertToNegative_8bit(nc.ConvertBytesToInt(d, 4, 5), False)
        m.joint_6 = nc.ConvertToNegative_8bit(nc.ConvertBytesToInt(d, 5, 6), False)

    def decode_47C_motor_max_acc_limit(
        self, m: ArmMsgFeedbackAllCurrentMotorMaxAccLimit, d: bytearray
    ) -> None:
        m.joint_index = nc.ConvertToNegative_8bit(nc.ConvertBytesToInt(d, 0, 1), False)
        # Robustness: ignore corrupted/out-of-range joint index instead of crashing.
        if not (1 <= m.joint_index <= len(m.joints)):
            return
        m.joints[m.joint_index - 1].max_joint_acc = (
            nc.ConvertToNegative_16bit(nc.ConvertBytesToInt(d, 1, 3), False) * 1e-2
        )

    def decode_4AF_firmware_info(self, m: ArmMsgFeedbackFirmware, d: bytearray) -> None:
        if len(d) != 8 or d == bytearray(
            [0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        ):
            return
        m.data_seg += d
        if len(m.data_seg) > 8 * 11:
            m.clear()
            m.data_seg += d

    def decode_155_joint_ctrl_12(self, m: ArmMsgJointCtrl12, d: bytearray) -> None:
        m.joint_1 = (
            nc.ConvertToNegative_32bit(nc.ConvertBytesToInt(d, 0, 4))
            * 1e-3
            * DEG2RAD
        )
        m.joint_2 = (
            nc.ConvertToNegative_32bit(nc.ConvertBytesToInt(d, 4, 8))
            * 1e-3
            * DEG2RAD
        )

    def decode_156_joint_ctrl_34(self, m: ArmMsgJointCtrl34, d: bytearray) -> None:
        m.joint_3 = (
            nc.ConvertToNegative_32bit(nc.ConvertBytesToInt(d, 0, 4))
            * 1e-3
            * DEG2RAD
        )
        m.joint_4 = (
            nc.ConvertToNegative_32bit(nc.ConvertBytesToInt(d, 4, 8))
            * 1e-3
            * DEG2RAD
        )

    def decode_157_joint_ctrl_56(self, m: ArmMsgJointCtrl56, d: bytearray) -> None:
        m.joint_5 = (
            nc.ConvertToNegative_32bit(nc.ConvertBytesToInt(d, 0, 4))
            * 1e-3
            * DEG2RAD
        )
        m.joint_6 = (
            nc.ConvertToNegative_32bit(nc.ConvertBytesToInt(d, 4, 8))
            * 1e-3
            * DEG2RAD
        )

    def decode_cpv_response(self, m: ArmMsgFeedbackCPVResponse, d: bytearray) -> None:
        mode = chr(nc.ConvertToNegative_8bit(nc.ConvertBytesToInt(d, 0, 1), False))
        if mode != "a":
            return
        type_code = (
            chr(nc.ConvertToNegative_8bit(nc.ConvertBytesToInt(d, 1, 2), False))
            + chr(nc.ConvertToNegative_8bit(nc.ConvertBytesToInt(d, 2, 3), False))
        )
        value = nc.ConvertToNegative_32bit(nc.ConvertBytesToInt(d, 3, 7))

        m.mode = mode
        m.type = type_code
        m.value = value
        m.type_value[m.type] = m.value

    # -------------------------
    # TX encoders (msg -> data)
    # -------------------------
    def encode_150_motion_ctrl(self, msg: ArmMsgMotionCtrl) -> List[int]:
        return (
            nc.ConvertToList_8bit(msg.emergency_stop, False)
            + nc.ConvertToList_8bit(msg.track_ctrl, False)
            + nc.ConvertToList_8bit(msg.grag_teach_ctrl, False)
            + [0] * 5
        )

    def encode_151_mode_ctrl(self, msg: ArmMsgModeCtrl) -> List[int]:
        return (
            nc.ConvertToList_8bit(msg.ctrl_mode, False)
            + nc.ConvertToList_8bit(msg.move_mode, False)
            + nc.ConvertToList_8bit(msg.move_spd_rate_ctrl, False)
            + nc.ConvertToList_8bit(msg.mit_mode, False)
            + nc.ConvertToList_8bit(msg.residence_time, False)
            + nc.ConvertToList_8bit(msg.installation_pos, False)
            + [0] * 2
        )

    def encode_152_end_pose_ctrl_xy(self, msg: ArmMsgEndPoseCtrlXY) -> List[int]:
        return nc.ConvertToList_32bit(msg.X_axis) + nc.ConvertToList_32bit(msg.Y_axis)

    def encode_153_end_pose_ctrl_zrx(self, msg: ArmMsgEndPoseCtrlZRX) -> List[int]:
        return nc.ConvertToList_32bit(msg.Z_axis) + nc.ConvertToList_32bit(msg.RX_axis)

    def encode_154_end_pose_ctrl_ryrz(self, msg: ArmMsgEndPoseCtrlRYRZ) -> List[int]:
        return nc.ConvertToList_32bit(msg.RY_axis) + nc.ConvertToList_32bit(msg.RZ_axis)

    def encode_155_joint_ctrl_12(self, msg: ArmMsgJointCtrl12) -> List[int]:
        return nc.ConvertToList_32bit(msg.joint_1) + nc.ConvertToList_32bit(msg.joint_2)

    def encode_156_joint_ctrl_34(self, msg: ArmMsgJointCtrl34) -> List[int]:
        return nc.ConvertToList_32bit(msg.joint_3) + nc.ConvertToList_32bit(msg.joint_4)

    def encode_157_joint_ctrl_56(self, msg: ArmMsgJointCtrl56) -> List[int]:
        return nc.ConvertToList_32bit(msg.joint_5) + nc.ConvertToList_32bit(msg.joint_6)

    def encode_158_circular_pattern_coord_num_update_ctrl(
        self, msg: ArmMsgCircularPatternCoordNumUpdateCtrl
    ) -> List[int]:
        return nc.ConvertToList_8bit(msg.instruction_num, False) + [0] * 7

    def encode_191_leader_arm_move_to_home(
        self, msg: ArmMsgLeaderArmMoveToHome
    ) -> List[int]:
        """
        0x191 请求主导臂（Leader Arm）回零（V1.7-4 及以后）
        data 固定 8 字节，根据 mode 选择不同位：
          - 0: [0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
          - 1: [0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00]
          - 2: [0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00]
        """
        mode = getattr(msg, "mode", None)
        if mode == 0:
            return [0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        if mode == 1:
            return [0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00]
        if mode == 2:
            return [0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00]
        raise ValueError("mode should be 0, 1, or 2")

    def encode_470_leader_follower_mode_config(
        self, msg: ArmMsgLeaderFollowerModeConfig
    ) -> List[int]:
        return (
            nc.ConvertToList_8bit(msg.linkage_config, False)
            + nc.ConvertToList_8bit(msg.feedback_offset, False)
            + nc.ConvertToList_8bit(msg.ctrl_offset, False)
            + nc.ConvertToList_8bit(msg.linkage_offset, False)
            + [0] * 4
        )

    def encode_471_motor_enable_disable_config(
        self, msg: ArmMsgMotorEnableDisableConfig
    ) -> List[int]:
        return (
            nc.ConvertToList_8bit(msg.joint_index, False)
            + nc.ConvertToList_8bit(msg.enable_flag, False)
            + [0] * 6
        )

    def encode_472_search_motor_max_angle_spd_acc_limit(
        self, msg: ArmMsgSearchMotorMaxAngleSpdAccLimit
    ) -> List[int]:
        return (
            nc.ConvertToList_8bit(msg.joint_index, False)
            + nc.ConvertToList_8bit(msg.search_content, False)
            + [0] * 6
        )

    def encode_474_motor_angle_limit_max_spd_set(
        self, msg: ArmMsgMotorAngleLimitMaxSpdSet
    ) -> List[int]:
        return (
            nc.ConvertToList_8bit(msg.joint_index, False)
            + nc.ConvertToList_16bit(msg.max_angle_limit)
            + nc.ConvertToList_16bit(msg.min_angle_limit)
            + nc.ConvertToList_16bit(msg.max_joint_spd, False)
            + [0]
        )

    def encode_475_joint_config(self, msg: ArmMsgJointConfig) -> List[int]:
        return (
            nc.ConvertToList_8bit(msg.joint_index, False)
            + nc.ConvertToList_8bit(msg.set_motor_current_pos_as_zero, False)
            + nc.ConvertToList_8bit(msg.acc_param_config_is_effective_or_not, False)
            + nc.ConvertToList_16bit(msg.max_joint_acc, False)
            + nc.ConvertToList_8bit(msg.clear_joint_err, False)
            + [0] * 2
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

    def encode_479_end_vel_acc_param_config(
        self, msg: ArmMsgEndVelAccParamConfig
    ) -> List[int]:
        return (
            nc.ConvertToList_16bit(msg.end_max_linear_vel, False)
            + nc.ConvertToList_16bit(msg.end_max_angular_vel, False)
            + nc.ConvertToList_16bit(msg.end_max_linear_acc, False)
            + nc.ConvertToList_16bit(msg.end_max_angular_acc, False)
        )

    def encode_47A_crash_protection_rating_config(
        self, msg: ArmMsgCrashProtectionRatingConfig
    ) -> List[int]:
        data = (
            nc.ConvertToList_8bit(msg.joint_1, False)
            + nc.ConvertToList_8bit(msg.joint_2, False)
            + nc.ConvertToList_8bit(msg.joint_3, False)
            + nc.ConvertToList_8bit(msg.joint_4, False)
            + nc.ConvertToList_8bit(msg.joint_5, False)
            + nc.ConvertToList_8bit(msg.joint_6, False)
        )
        if hasattr(msg, "joint_7"):
            data += nc.ConvertToList_8bit(msg.joint_7, False)
        # pad to 8 bytes
        if len(data) < 8:
            data += [0] * (8 - len(data))
        return data[:8]

    def encode_487_joint_assistance_rating_config(
        self, msg: ArmMsgJointAssistanceRatingConfig
    ) -> List[int]:
        return (
            nc.ConvertToList_8bit(msg.joint_1, False)
            + nc.ConvertToList_8bit(msg.joint_2, False)
            + nc.ConvertToList_8bit(msg.joint_3, False)
            + nc.ConvertToList_8bit(msg.joint_4, False)
            + nc.ConvertToList_8bit(msg.joint_5, False)
            + nc.ConvertToList_8bit(msg.joint_6, False)
            + [0] * 2
        )

    def encode_4AF_req_firmware(self, _msg: ArmMsgReqFirmware) -> List[int]:
        # 历史行为：固定请求帧
        return [0x01]

    def encode_cpv_settings_and_queries(
        self, msg: ArmMsgCPVSettingsAndQueries
    ) -> List[int]:
        type_code = msg.type if isinstance(msg.type, str) else "".join(msg.type)
        if len(type_code) != 2:
            raise ValueError("CPV type code should contain 2 chars")
        return (
            nc.ConvertToList_8bit(ord(msg.mode), False)
            + nc.ConvertToList_8bit(ord(type_code[0]), False)
            + nc.ConvertToList_8bit(ord(type_code[1]), False)
            + nc.ConvertToList_32bit(msg.value)
        )


class Parser(TableDriven, ProtocolParserInterface):
    # Message classes used by driver-side message construction helpers.
    #
    # NOTE:
    # - These are class attributes on purpose, so Nero (or other variants) can
    #   override them while still inheriting helper methods safely.
    _MSG_EndPoseCtrlXY = ArmMsgEndPoseCtrlXY
    _MSG_EndPoseCtrlZRX = ArmMsgEndPoseCtrlZRX
    _MSG_EndPoseCtrlRYRZ = ArmMsgEndPoseCtrlRYRZ

    _MSG_JointCtrl12 = ArmMsgJointCtrl12
    _MSG_JointCtrl34 = ArmMsgJointCtrl34
    _MSG_JointCtrl56 = ArmMsgJointCtrl56
    _MSG_JointCtrl7 = None  # Nero overrides

    _MSG_CircularCoordNumUpdate = ArmMsgCircularPatternCoordNumUpdateCtrl

    _MSG_JointMitCtrlByIndex: Dict[int, Type[AttributeBase]] = {
        1: ArmMsgJointMitCtrl1,
        2: ArmMsgJointMitCtrl2,
        3: ArmMsgJointMitCtrl3,
        4: ArmMsgJointMitCtrl4,
        5: ArmMsgJointMitCtrl5,
        6: ArmMsgJointMitCtrl6,
    }

    _MSG_CPVSettingsAndQueriesByIndex: Dict[int, Type[AttributeBase]] = {
        1: ArmMsgCPVSettingsAndQueries1,
        2: ArmMsgCPVSettingsAndQueries2,
        3: ArmMsgCPVSettingsAndQueries3,
        4: ArmMsgCPVSettingsAndQueries4,
        5: ArmMsgCPVSettingsAndQueries5,
        6: ArmMsgCPVSettingsAndQueries6,
    }

    if TYPE_CHECKING:
        arm_status: Optional[MessageAbstract[ArmMsgFeedbackStatus]]
        end_pose_xy: Optional[MessageAbstract[ArmMsgFeedbackEndPoseXY]]
        end_pose_zrx: Optional[MessageAbstract[ArmMsgFeedbackEndPoseZRX]]
        end_pose_ryrz: Optional[MessageAbstract[ArmMsgFeedbackEndPoseRYRZ]]

        joint_12: Optional[MessageAbstract[ArmMsgFeedbackJointStates12]]
        joint_34: Optional[MessageAbstract[ArmMsgFeedbackJointStates34]]
        joint_56: Optional[MessageAbstract[ArmMsgFeedbackJointStates56]]

        motor_state_1: Optional[MessageAbstract[ArmMsgFeedbackHighSpd1]]
        motor_state_2: Optional[MessageAbstract[ArmMsgFeedbackHighSpd2]]
        motor_state_3: Optional[MessageAbstract[ArmMsgFeedbackHighSpd3]]
        motor_state_4: Optional[MessageAbstract[ArmMsgFeedbackHighSpd4]]
        motor_state_5: Optional[MessageAbstract[ArmMsgFeedbackHighSpd5]]
        motor_state_6: Optional[MessageAbstract[ArmMsgFeedbackHighSpd6]]

        driver_state_1: Optional[MessageAbstract[ArmMsgFeedbackLowSpd1]]
        driver_state_2: Optional[MessageAbstract[ArmMsgFeedbackLowSpd2]]
        driver_state_3: Optional[MessageAbstract[ArmMsgFeedbackLowSpd3]]
        driver_state_4: Optional[MessageAbstract[ArmMsgFeedbackLowSpd4]]
        driver_state_5: Optional[MessageAbstract[ArmMsgFeedbackLowSpd5]]
        driver_state_6: Optional[MessageAbstract[ArmMsgFeedbackLowSpd6]]

        motor_angle_limit_max_spd: Optional[
            MessageAbstract[ArmMsgFeedbackAllCurrentMotorAngleLimitMaxSpd]
        ]
        resp_set_instruction: Optional[
            MessageAbstract[ArmMsgFeedbackRespSetInstruction]
        ]
        end_vel_acc_param: Optional[
            MessageAbstract[ArmMsgFeedbackCurrentEndVelAccParam]
        ]
        crash_protection_rating: Optional[
            MessageAbstract[ArmMsgFeedbackCrashProtectionRating]
        ]
        joint_assistance_rating: Optional[
            MessageAbstract[ArmMsgFeedbackJointAssistanceRating]
        ]
        motor_max_acc_limit: Optional[
            MessageAbstract[ArmMsgFeedbackAllCurrentMotorMaxAccLimit]
        ]

        firmware_info: Optional[MessageAbstract[ArmMsgFeedbackFirmware]]

        leader_joint_12: Optional[MessageAbstract[ArmMsgJointCtrl12]]
        leader_joint_34: Optional[MessageAbstract[ArmMsgJointCtrl34]]
        leader_joint_56: Optional[MessageAbstract[ArmMsgJointCtrl56]]

        cpv_response_1: Optional[MessageAbstract[ArmMsgFeedbackCPVResponse1]]
        cpv_response_2: Optional[MessageAbstract[ArmMsgFeedbackCPVResponse2]]
        cpv_response_3: Optional[MessageAbstract[ArmMsgFeedbackCPVResponse3]]
        cpv_response_4: Optional[MessageAbstract[ArmMsgFeedbackCPVResponse4]]
        cpv_response_5: Optional[MessageAbstract[ArmMsgFeedbackCPVResponse5]]
        cpv_response_6: Optional[MessageAbstract[ArmMsgFeedbackCPVResponse6]]

    def __init__(self, fps_manager: FPSManager, codec: Optional[Codec] = None):
        super().__init__(fps_manager=fps_manager)
        self._codec = codec or Codec()        
        self._rx_map = self._build_rx_map()
        self._tx_map = self._build_tx_map()

    # -------------------------
    # Driver-side message builders (internal helpers)
    # -------------------------
    def _make_end_pose_ctrl_msgs(
        self,
        *,
        x_um: int,
        y_um: int,
        z_um: int,
        roll_mdeg: int,
        pitch_mdeg: int,
        yaw_mdeg: int,
    ) -> List[AttributeBase]:
        """Build 3 end-pose control messages (XY, ZRX, RYRZ) from protocol fields."""
        return [
            self._MSG_EndPoseCtrlXY(X_axis=x_um, Y_axis=y_um),
            self._MSG_EndPoseCtrlZRX(Z_axis=z_um, RX_axis=roll_mdeg),
            self._MSG_EndPoseCtrlRYRZ(RY_axis=pitch_mdeg, RZ_axis=yaw_mdeg),
        ]

    def _make_joint_ctrl_msgs(self, joints_mdeg: List[int]) -> List[AttributeBase]:
        """
        Build joint control messages from scaled protocol fields.

        Parameters
        ----------
        joints_mdeg:
            Joint angles scaled to milli-degrees (deg * 1e3), length 6 (Piper) or
            7 (Nero variant).
        """
        if len(joints_mdeg) not in (6, 7):
            raise ValueError("joints_mdeg length must be 6 (or 7 for variants)")

        msgs: List[AttributeBase] = [
            self._MSG_JointCtrl12(joint_1=joints_mdeg[0], joint_2=joints_mdeg[1]),
            self._MSG_JointCtrl34(joint_3=joints_mdeg[2], joint_4=joints_mdeg[3]),
            self._MSG_JointCtrl56(joint_5=joints_mdeg[4], joint_6=joints_mdeg[5]),
        ]

        if len(joints_mdeg) == 7:
            if self._MSG_JointCtrl7 is None:
                raise ValueError("7th joint control message is not supported")
            msgs.append(self._MSG_JointCtrl7(joint_7=joints_mdeg[6]))

        return msgs

    def _make_circular_coord_num_update_msg(self, instruction_num: int) -> AttributeBase:
        """Build a circular pattern coord-number update control message."""
        return self._MSG_CircularCoordNumUpdate(instruction_num=instruction_num)

    def _make_joint_mit_ctrl_msg(
        self,
        *,
        joint_index: int,
        p_des: int,
        v_des: int,
        kp: int,
        kd: int,
        t_ff: int,
    ) -> AttributeBase:
        """Build one MIT control message for a specific joint index."""
        cls = self._MSG_JointMitCtrlByIndex.get(joint_index)
        if cls is None:
            raise ValueError(f"Unsupported joint_index for MIT control: {joint_index}")
        return cls(p_des=p_des, v_des=v_des, kp=kp, kd=kd, t_ff=t_ff)

    def _make_cpv_settings_and_queries_msg(
        self,
        *,
        joint_index: int,
        mode: Literal["w", "r"],
        type_: Literal["po", "sp", "ac", "dc", "vv", "pp", "kp", "ki"],
        value: int,
    ) -> AttributeBase:
        """Build one CPV setting/query message for a specific joint index."""
        cls = self._MSG_CPVSettingsAndQueriesByIndex.get(joint_index)
        if cls is None:
            raise ValueError(
                f"Unsupported joint_index for CPV setting/query: {joint_index}"
            )
        return cls(mode=mode, type=type_, value=value)

    def _build_rx_map(
        self,
    ) -> Dict[int, Tuple[str, Type, Callable[[object, bytearray], None]]]:
        # 接收侧：can_id -> (attr_name, msg_cls, decoder)
        return {
            0x155: (
                "leader_joint_12",
                ArmMsgJointCtrl12,
                self._codec.decode_155_joint_ctrl_12
            ),
            0x156: (
                "leader_joint_34",
                ArmMsgJointCtrl34,
                self._codec.decode_156_joint_ctrl_34
            ),
            0x157: (
                "leader_joint_56",
                ArmMsgJointCtrl56,
                self._codec.decode_157_joint_ctrl_56
            ),
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
            0x251: (
                "motor_state_1",
                ArmMsgFeedbackHighSpd1,
                self._codec.decode_high_spd
            ),
            0x252: (
                "motor_state_2",
                ArmMsgFeedbackHighSpd2,
                self._codec.decode_high_spd
            ),
            0x253: (
                "motor_state_3",
                ArmMsgFeedbackHighSpd3,
                self._codec.decode_high_spd
            ),
            0x254: (
                "motor_state_4",
                ArmMsgFeedbackHighSpd4,
                self._codec.decode_high_spd
            ),
            0x255: (
                "motor_state_5",
                ArmMsgFeedbackHighSpd5,
                self._codec.decode_high_spd
            ),
            0x256: (
                "motor_state_6",
                ArmMsgFeedbackHighSpd6,
                self._codec.decode_high_spd
            ),
            0x261: (
                "driver_state_1",
                ArmMsgFeedbackLowSpd1,
                self._codec.decode_low_spd
            ),
            0x262: (
                "driver_state_2",
                ArmMsgFeedbackLowSpd2,
                self._codec.decode_low_spd
            ),
            0x263: (
                "driver_state_3",
                ArmMsgFeedbackLowSpd3,
                self._codec.decode_low_spd
            ),
            0x264: (
                "driver_state_4",
                ArmMsgFeedbackLowSpd4,
                self._codec.decode_low_spd
            ),
            0x265: (
                "driver_state_5",
                ArmMsgFeedbackLowSpd5,
                self._codec.decode_low_spd
            ),
            0x266: (
                "driver_state_6",
                ArmMsgFeedbackLowSpd6,
                self._codec.decode_low_spd
            ),
            0x2A1: (
                "arm_status",
                ArmMsgFeedbackStatus,
                self._codec.decode_2A1_status
            ),
            0x2A2: (
                "end_pose_xy",
                ArmMsgFeedbackEndPoseXY,
                self._codec.decode_2A2_end_pose_xy
            ),
            0x2A3: (
                "end_pose_zrx",
                ArmMsgFeedbackEndPoseZRX,
                self._codec.decode_2A3_end_pose_zrx
            ),
            0x2A4: (
                "end_pose_ryrz",
                ArmMsgFeedbackEndPoseRYRZ,
                self._codec.decode_2A4_end_pose_ryrz
            ),
            0x2A5: (
                "joint_12",
                ArmMsgFeedbackJointStates12,
                self._codec.decode_2A5_joint_12
            ),
            0x2A6: (
                "joint_34",
                ArmMsgFeedbackJointStates34,
                self._codec.decode_2A6_joint_34
            ),
            0x2A7: (
                "joint_56",
                ArmMsgFeedbackJointStates56,
                self._codec.decode_2A7_joint_56
            ),
            0x473: (
                "motor_angle_limit_max_spd",
                ArmMsgFeedbackAllCurrentMotorAngleLimitMaxSpd,
                self._codec.decode_473_motor_angle_limit_max_spd
            ),
            0x476: (
                "resp_set_instruction",
                ArmMsgFeedbackRespSetInstruction,
                self._codec.decode_476_resp_set_instruction
            ),
            0x478: (
                "end_vel_acc_param",
                ArmMsgFeedbackCurrentEndVelAccParam,
                self._codec.decode_478_end_vel_acc_param
            ),
            0x47B: (
                "crash_protection_rating",
                ArmMsgFeedbackCrashProtectionRating,
                self._codec.decode_47B_crash_protection_rating
            ),
            0x47C: (
                "motor_max_acc_limit",
                ArmMsgFeedbackAllCurrentMotorMaxAccLimit,
                self._codec.decode_47C_motor_max_acc_limit
            ),
            0x488: (
                "joint_assistance_rating",
                ArmMsgFeedbackJointAssistanceRating,
                self._codec.decode_488_joint_assistance_rating
            ),
            0x4AF: (
                "firmware_info",
                ArmMsgFeedbackFirmware,
                self._codec.decode_4AF_firmware_info
            ),
        }

    def _build_tx_map(self) -> Dict[str, Tuple[int, Callable]]:
        # 发送侧：msg.type_ -> (can_id, encoder)
        return {
            ArmMsgMotionCtrl.type_: (
                0x150,
                self._codec.encode_150_motion_ctrl
            ),
            ArmMsgModeCtrl.type_: (
                0x151,
                self._codec.encode_151_mode_ctrl
            ),
            ArmMsgEndPoseCtrlXY.type_: (
                0x152,
                self._codec.encode_152_end_pose_ctrl_xy
            ),
            ArmMsgEndPoseCtrlZRX.type_: (
                0x153,
                self._codec.encode_153_end_pose_ctrl_zrx
            ),
            ArmMsgEndPoseCtrlRYRZ.type_: (
                0x154,
                self._codec.encode_154_end_pose_ctrl_ryrz
            ),
            ArmMsgJointCtrl12.type_: (0x155, self._codec.encode_155_joint_ctrl_12),
            ArmMsgJointCtrl34.type_: (0x156, self._codec.encode_156_joint_ctrl_34),
            ArmMsgJointCtrl56.type_: (0x157, self._codec.encode_157_joint_ctrl_56),
            ArmMsgCircularPatternCoordNumUpdateCtrl.type_: (
                0x158,
                self._codec.encode_158_circular_pattern_coord_num_update_ctrl
            ),
            ArmMsgJointMitCtrl1.type_: (0x15A, self._codec.pack_joint_mit_ctrl),
            ArmMsgJointMitCtrl2.type_: (0x15B, self._codec.pack_joint_mit_ctrl),
            ArmMsgJointMitCtrl3.type_: (0x15C, self._codec.pack_joint_mit_ctrl),
            ArmMsgJointMitCtrl4.type_: (0x15D, self._codec.pack_joint_mit_ctrl),
            ArmMsgJointMitCtrl5.type_: (0x15E, self._codec.pack_joint_mit_ctrl),
            ArmMsgJointMitCtrl6.type_: (0x15F, self._codec.pack_joint_mit_ctrl),
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
            ArmMsgLeaderArmMoveToHome.type_: (
                0x191,
                self._codec.encode_191_leader_arm_move_to_home
            ),
            ArmMsgLeaderFollowerModeConfig.type_: (
                0x470,
                self._codec.encode_470_leader_follower_mode_config
            ),
            ArmMsgMotorEnableDisableConfig.type_: (
                0x471,
                self._codec.encode_471_motor_enable_disable_config
            ),
            ArmMsgSearchMotorMaxAngleSpdAccLimit.type_: (
                0x472,
                self._codec.encode_472_search_motor_max_angle_spd_acc_limit
            ),
            ArmMsgMotorAngleLimitMaxSpdSet.type_: (
                0x474,
                self._codec.encode_474_motor_angle_limit_max_spd_set
            ),
            ArmMsgJointConfig.type_: (
                0x475,
                self._codec.encode_475_joint_config
            ),
            ArmMsgParamEnquiryAndConfig.type_: (
                0x477,
                self._codec.encode_477_param_enquiry_and_config
            ),
            ArmMsgEndVelAccParamConfig.type_: (
                0x479,
                self._codec.encode_479_end_vel_acc_param_config
            ),
            ArmMsgCrashProtectionRatingConfig.type_: (
                0x47A,
                self._codec.encode_47A_crash_protection_rating_config
            ),
            ArmMsgJointAssistanceRatingConfig.type_: (
                0x487,
                self._codec.encode_487_joint_assistance_rating_config
            ),
            ArmMsgReqFirmware.type_: (
                0x4AF,
                self._codec.encode_4AF_req_firmware
            ),
        }
