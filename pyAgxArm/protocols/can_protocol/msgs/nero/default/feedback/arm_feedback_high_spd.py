#!/usr/bin/env python3
# -*-coding:utf8-*-
from ....piper.default import (
    ArmMsgFeedbackHighSpd as ArmMsgFeedbackHighSpdBase,
    ArmMsgFeedbackAllHighSpd as ArmMsgFeedbackAllHighSpdBase,
)
class ArmMsgFeedbackHighSpd(ArmMsgFeedbackHighSpdBase):
    '''
    feedback

    驱动器信息高速反馈 0x5

    节点 ID:
        0x1~0x07

    CAN ID:
        0X251~0x257

    Args:
        velocity: 电机当前转速
        current: 电机当前电流
        position: 电机当前位置
        torque: 关节侧真实力矩（由电流线性拟合再按减速比折算）,单位 N·m

    位描述:

        Byte 0: 转速高八位, int16, 电机当前转速 单位: 0.001rad/s
        Byte 1: 转速低八位
        Byte 2: 电流高八位, uint16, 电机当前电流 单位: 0.001A
        Byte 3: 电流低八位
        Byte 4: 位置最高位, int32, 电机当前位置 单位: rad
        Byte 5: 位置次高位
        Byte 6: 位置次低位
        Byte 7: 位置最低位
    '''
    '''
    feedback

    High-Speed Feedback of Drive Information 0x5

    Node ID:
        0x1~0x07

    CAN ID:
        0x251~0x257

    Args:
        velocity: Motor Speed.
        current: Motor Current.
        position: Motor Position.
        torque: Joint-side true torque from current linear fit and gear ratio, N·m.

    Bit Description:

        Byte 0: Motor Speed (High Byte), int16, unit: 0.001rad/s
        Byte 1: Motor Speed (Low Byte)
        Byte 2: Motor Current (High Byte), uint16, unit: 0.001A
        Byte 3: Motor Current (Low Byte)
        Byte 4: Motor Position (Most Significant Byte), int32, unit: rad
        Byte 5: Motor Position (Second Most Significant Byte)
        Byte 6: Motor Position (Second Least Significant Byte)
        Byte 7: Motor Position (Least Significant Byte)
    '''
    pass
class ArmMsgFeedbackHighSpd1(ArmMsgFeedbackHighSpd):
    '''CAN ID:
        0x251'''
    _VALID_CAN_ID = 0x251

class ArmMsgFeedbackHighSpd2(ArmMsgFeedbackHighSpd):
    '''CAN ID:
        0x252'''
    _VALID_CAN_ID = 0x252

class ArmMsgFeedbackHighSpd3(ArmMsgFeedbackHighSpd):
    '''CAN ID:
        0x253'''
    _VALID_CAN_ID = 0x253

class ArmMsgFeedbackHighSpd4(ArmMsgFeedbackHighSpd):
    '''CAN ID:
        0x254'''
    _VALID_CAN_ID = 0x254

class ArmMsgFeedbackHighSpd5(ArmMsgFeedbackHighSpd):
    '''CAN ID:
        0x255'''
    _VALID_CAN_ID = 0x255

class ArmMsgFeedbackHighSpd6(ArmMsgFeedbackHighSpd):
    '''CAN ID:
        0x256'''
    _VALID_CAN_ID = 0x256

class ArmMsgFeedbackHighSpd7(ArmMsgFeedbackHighSpd):
    '''CAN ID:
        0x257'''
    _VALID_CAN_ID = 0x257

class ArmMsgFeedbackAllHighSpd(ArmMsgFeedbackAllHighSpdBase):
    '''CAN ID:
        0x251~0x257'''
    def __init__(self,
                 joint_1 = ArmMsgFeedbackHighSpd(),
                 joint_2 = ArmMsgFeedbackHighSpd(),
                 joint_3 = ArmMsgFeedbackHighSpd(),
                 joint_4 = ArmMsgFeedbackHighSpd(),
                 joint_5 = ArmMsgFeedbackHighSpd(),
                 joint_6 = ArmMsgFeedbackHighSpd(),
                 joint_7 = ArmMsgFeedbackHighSpd()):
        super().__init__(joint_1, joint_2, joint_3, joint_4, joint_5, joint_6)
        self.joint_7 = joint_7
