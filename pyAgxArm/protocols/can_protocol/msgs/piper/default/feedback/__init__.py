# 导入 feedback 子模块的类
from .arm_feedback_crash_protection_rating import ArmMsgFeedbackCrashProtectionRating
from .arm_feedback_end_pose import ArmMsgFeedbackEndPose, ArmMsgFeedbackEndPoseXY, ArmMsgFeedbackEndPoseZRX, ArmMsgFeedbackEndPoseRYRZ
from .arm_feedback_current_motor_angle_limit_max_spd import ArmMsgFeedbackCurrentMotorAngleLimitMaxSpd, ArmMsgFeedbackAllCurrentMotorAngleLimitMaxSpd
from .arm_feedback_current_end_vel_acc_param import ArmMsgFeedbackCurrentEndVelAccParam
from .arm_feedback_current_motor_max_acc_limit import ArmMsgFeedbackCurrentMotorMaxAccLimit, ArmMsgFeedbackAllCurrentMotorMaxAccLimit
from .arm_feedback_high_spd import ArmMsgFeedbackHighSpd, ArmMsgFeedbackAllHighSpd, \
    ArmMsgFeedbackHighSpd1, ArmMsgFeedbackHighSpd2, ArmMsgFeedbackHighSpd3, \
    ArmMsgFeedbackHighSpd4, ArmMsgFeedbackHighSpd5, ArmMsgFeedbackHighSpd6
from .arm_feedback_joint_states import ArmMsgFeedbackJointStates, \
    ArmMsgFeedbackJointStates12, ArmMsgFeedbackJointStates34, ArmMsgFeedbackJointStates56
from .arm_feedback_low_spd import ArmMsgFeedbackLowSpd, ArmMsgFeedbackAllLowSpd, \
    ArmMsgFeedbackLowSpd1, ArmMsgFeedbackLowSpd2, ArmMsgFeedbackLowSpd3, \
    ArmMsgFeedbackLowSpd4, ArmMsgFeedbackLowSpd5, ArmMsgFeedbackLowSpd6
from .arm_feedback_status import ArmMsgFeedbackStatus, ArmMsgFeedbackStatusEnum
from .arm_feedback_set_instruction_response import ArmMsgFeedbackRespSetInstruction
from .arm_feedback_firmware import ArmMsgFeedbackFirmware
from .arm_feedback_joint_assistance_rating import ArmMsgFeedbackJointAssistanceRating
from .arm_feedback_cpv_response import (
    ArmMsgFeedbackCPVResponse,
    ArmMsgFeedbackCPVResponse1,
    ArmMsgFeedbackCPVResponse2,
    ArmMsgFeedbackCPVResponse3,
    ArmMsgFeedbackCPVResponse4,
    ArmMsgFeedbackCPVResponse5,
    ArmMsgFeedbackCPVResponse6,
)

__all__ = [
    # 反馈
    'ArmMsgFeedbackEndPose',
    'ArmMsgFeedbackEndPoseXY',
    'ArmMsgFeedbackEndPoseZRX',
    'ArmMsgFeedbackEndPoseRYRZ',
    'ArmMsgFeedbackJointStates',
    'ArmMsgFeedbackJointStates12',
    'ArmMsgFeedbackJointStates34',
    'ArmMsgFeedbackJointStates56',
    'ArmMsgFeedbackStatus',
    'ArmMsgFeedbackStatusEnum',
    'ArmMsgFeedbackCurrentMotorAngleLimitMaxSpd',
    'ArmMsgFeedbackCurrentEndVelAccParam',
    'ArmMsgFeedbackCurrentMotorMaxAccLimit',
    'ArmMsgFeedbackAllCurrentMotorAngleLimitMaxSpd',
    'ArmMsgFeedbackAllCurrentMotorMaxAccLimit',
    'ArmMsgFeedbackCrashProtectionRating',
    'ArmMsgFeedbackHighSpd',
    'ArmMsgFeedbackAllHighSpd',
    'ArmMsgFeedbackHighSpd1',
    'ArmMsgFeedbackHighSpd2',
    'ArmMsgFeedbackHighSpd3',
    'ArmMsgFeedbackHighSpd4',
    'ArmMsgFeedbackHighSpd5',
    'ArmMsgFeedbackHighSpd6',
    'ArmMsgFeedbackLowSpd',
    'ArmMsgFeedbackAllLowSpd',
    'ArmMsgFeedbackLowSpd1',
    'ArmMsgFeedbackLowSpd2',
    'ArmMsgFeedbackLowSpd3',
    'ArmMsgFeedbackLowSpd4',
    'ArmMsgFeedbackLowSpd5',
    'ArmMsgFeedbackLowSpd6',
    'ArmMsgFeedbackRespSetInstruction',
    'ArmMsgFeedbackFirmware',
    'ArmMsgFeedbackJointAssistanceRating',
    'ArmMsgFeedbackCPVResponse',
    'ArmMsgFeedbackCPVResponse1',
    'ArmMsgFeedbackCPVResponse2',
    'ArmMsgFeedbackCPVResponse3',
    'ArmMsgFeedbackCPVResponse4',
    'ArmMsgFeedbackCPVResponse5',
    'ArmMsgFeedbackCPVResponse6',
]
