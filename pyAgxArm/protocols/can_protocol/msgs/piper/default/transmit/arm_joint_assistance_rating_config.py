#!/usr/bin/env python3
# -*-coding:utf8-*-
from ....core.attritube_base import AttributeBase

class ArmMsgJointAssistanceRatingConfig(AttributeBase):
    '''
    transmit

    关节助力等级设置指令

    CAN ID:
        0x487

    有效值 : 0~10

    等级 0 代表不助力； 6个关节可以独立设置

    Args:
        joint_1: 关节1的助力等级设定
        joint_2: 关节2的助力等级设定
        joint_3: 关节3的助力等级设定
        joint_4: 关节4的助力等级设定
        joint_5: 关节5的助力等级设定
        joint_6: 关节6的助力等级设定

    位描述:
        Byte 0: 1 号关节助力等级, uint8
        Byte 1: 2 号关节助力等级, uint8
        Byte 2: 3 号关节助力等级, uint8
        Byte 3: 4 号关节助力等级, uint8
        Byte 4: 5 号关节助力等级, uint8
        Byte 5: 6 号关节助力等级, uint8
        Byte 6: 保留
        Byte 7: 保留
    '''
    def __init__(self, 
                 joint_1: int = 0xFF,
                 joint_2: int = 0xFF,
                 joint_3: int = 0xFF,
                 joint_4: int = 0xFF,
                 joint_5: int = 0xFF,
                 joint_6: int = 0xFF
                 ):
        self.joint_1 = joint_1
        self.joint_2 = joint_2
        self.joint_3 = joint_3
        self.joint_4 = joint_4
        self.joint_5 = joint_5
        self.joint_6 = joint_6
