#!/usr/bin/env python3
# -*-coding:utf8-*-
from ....core.attritube_base import AttributeBase
from typing import Optional

class ArmMsgFeedbackJointAssistanceRating(AttributeBase):
    '''
    feedback

    关节助力等级反馈指令

    CAN ID: 
        0x488

    设定值 : 0~10

    等级 0 代表不助力； 6个关节可以独立设置

    Args:
        joint_1: 1号关节助力等级
        joint_2: 2号关节助力等级
        joint_3: 3号关节助力等级
        joint_4: 4号关节助力等级
        joint_5: 5号关节助力等级
        joint_6: 6号关节助力等级

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
                 joint_1: Optional[int] = None,
                 joint_2: Optional[int] = None,
                 joint_3: Optional[int] = None,
                 joint_4: Optional[int] = None,
                 joint_5: Optional[int] = None,
                 joint_6: Optional[int] = None
                 ):
        self.joint_1 = joint_1
        self.joint_2 = joint_2
        self.joint_3 = joint_3
        self.joint_4 = joint_4
        self.joint_5 = joint_5
        self.joint_6 = joint_6

    def clear(self):
        self.joint_1 = None
        self.joint_2 = None
        self.joint_3 = None
        self.joint_4 = None
        self.joint_5 = None
        self.joint_6 = None
