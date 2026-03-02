# 导入 feedback 子模块的类
# 相同则沿用
from ....piper.default import (
    ArmMsgFeedbackLowSpd1,
    ArmMsgFeedbackLowSpd2,
    ArmMsgFeedbackLowSpd3,
    ArmMsgFeedbackLowSpd4,
    ArmMsgFeedbackLowSpd5,
    ArmMsgFeedbackLowSpd6,
    ArmMsgFeedbackEndPose,
    ArmMsgFeedbackEndPoseXY,
    ArmMsgFeedbackEndPoseZRX,
    ArmMsgFeedbackEndPoseRYRZ,
    ArmMsgFeedbackJointStates12,
    ArmMsgFeedbackJointStates34,
    ArmMsgFeedbackJointStates56,
    ArmMsgFeedbackRespSetInstruction,
    ArmMsgFeedbackCurrentEndVelAccParam,
    ArmMsgFeedbackFirmware,
)

# 不同则新增
from .arm_feedback_crash_protection_rating import ArmMsgFeedbackCrashProtectionRating
from .arm_feedback_current_motor_angle_limit_max_spd import ArmMsgFeedbackCurrentMotorAngleLimitMaxSpd, ArmMsgFeedbackAllCurrentMotorAngleLimitMaxSpd
from .arm_feedback_current_motor_max_acc_limit import ArmMsgFeedbackCurrentMotorMaxAccLimit, ArmMsgFeedbackAllCurrentMotorMaxAccLimit
from .arm_feedback_high_spd import ArmMsgFeedbackHighSpd, ArmMsgFeedbackAllHighSpd, \
    ArmMsgFeedbackHighSpd1, ArmMsgFeedbackHighSpd2, ArmMsgFeedbackHighSpd3, \
    ArmMsgFeedbackHighSpd4, ArmMsgFeedbackHighSpd5, ArmMsgFeedbackHighSpd6, \
    ArmMsgFeedbackHighSpd7
from .arm_feedback_joint_states import ArmMsgFeedbackJointStates, ArmMsgFeedbackJointStates7
from .arm_feedback_low_spd import ArmMsgFeedbackLowSpd, ArmMsgFeedbackAllLowSpd, ArmMsgFeedbackLowSpd7
from .arm_feedback_status import ArmMsgFeedbackStatus, ArmMsgFeedbackStatusEnum
from .arm_feedback_master_joint_states import (
    ArmMsgFeedbackMasterJointStates,
    ArmMsgFeedbackMasterJointStates1,
    ArmMsgFeedbackMasterJointStates2,
    ArmMsgFeedbackMasterJointStates3,
    ArmMsgFeedbackMasterJointStates4,
    ArmMsgFeedbackMasterJointStates5,
    ArmMsgFeedbackMasterJointStates6,
    ArmMsgFeedbackMasterJointStates7,
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
    'ArmMsgFeedbackJointStates7',
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
    'ArmMsgFeedbackHighSpd7',
    'ArmMsgFeedbackLowSpd',
    'ArmMsgFeedbackAllLowSpd',
    'ArmMsgFeedbackLowSpd1',
    'ArmMsgFeedbackLowSpd2',
    'ArmMsgFeedbackLowSpd3',
    'ArmMsgFeedbackLowSpd4',
    'ArmMsgFeedbackLowSpd5',
    'ArmMsgFeedbackLowSpd6',
    'ArmMsgFeedbackLowSpd7',
    'ArmMsgFeedbackRespSetInstruction',
    'ArmMsgFeedbackFirmware',
    'ArmMsgFeedbackMasterJointStates',
    'ArmMsgFeedbackMasterJointStates1',
    'ArmMsgFeedbackMasterJointStates2',
    'ArmMsgFeedbackMasterJointStates3',
    'ArmMsgFeedbackMasterJointStates4',
    'ArmMsgFeedbackMasterJointStates5',
    'ArmMsgFeedbackMasterJointStates6',
    'ArmMsgFeedbackMasterJointStates7',
]
