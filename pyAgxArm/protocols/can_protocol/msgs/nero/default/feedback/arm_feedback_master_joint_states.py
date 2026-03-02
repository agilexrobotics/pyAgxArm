#!/usr/bin/env python3
# -*-coding:utf8-*-
from ....core.attritube_base import AttributeBase
from typing import Union

class ArmMsgFeedbackMasterJointStates(AttributeBase):
    '''
    feedback
    
    主臂关节角度反馈,单位rad
    
    CAN ID: 
        0x500 + NUM
    
    Args:
        joint_1: 关节1反馈角度
        joint_2: 关节2反馈角度
        joint_3: 关节3反馈角度
        joint_4: 关节4反馈角度
        joint_5: 关节5反馈角度
        joint_6: 关节6反馈角度
        joint_7: 关节7反馈角度
    '''
    '''
    feedback
    
    Joint Angle Feedback for Robotic Arm, in Radians
    
    CAN ID: 
        0x500 + NUM
    
    Args:
        joint_1: Feedback angle of joint 1, in 0.001 degrees.
        joint_2: Feedback angle of joint 2, in 0.001 degrees.
        joint_3: Feedback angle of joint 3, in 0.001 degrees.
        joint_4: Feedback angle of joint 4, in 0.001 degrees.
        joint_5: Feedback angle of joint 5, in 0.001 degrees.
        joint_6: Feedback angle of joint 6, in 0.001 degrees.
        joint_7: Feedback angle of joint 7, in 0.001 degrees.
    '''
    def __init__(self,
                 joint_1: Union[int, float] = 0,
                 joint_2: Union[int, float] = 0,
                 joint_3: Union[int, float] = 0,
                 joint_4: Union[int, float] = 0,
                 joint_5: Union[int, float] = 0,
                 joint_6: Union[int, float] = 0,
                 joint_7: Union[int, float] = 0):
        self.joint_1 = joint_1
        self.joint_2 = joint_2
        self.joint_3 = joint_3
        self.joint_4 = joint_4
        self.joint_5 = joint_5
        self.joint_6 = joint_6
        self.joint_7 = joint_7

class ArmMsgFeedbackMasterJointStates1(AttributeBase):
    '''CAN ID:
        0x501'''
    def __init__(self, 
                 joint_1: int = 0):
        self.joint_1 = joint_1

class ArmMsgFeedbackMasterJointStates2(AttributeBase):
    '''CAN ID:
        0x502'''
    def __init__(self, 
                 joint_2: int = 0):
        self.joint_2 = joint_2

class ArmMsgFeedbackMasterJointStates3(AttributeBase):
    '''CAN ID:
        0x503'''
    def __init__(self, 
                 joint_3: int = 0):
        self.joint_3 = joint_3

class ArmMsgFeedbackMasterJointStates4(AttributeBase):
    '''CAN ID:
        0x504'''
    def __init__(self, 
                 joint_4: int = 0):
        self.joint_4 = joint_4

class ArmMsgFeedbackMasterJointStates5(AttributeBase):
    '''CAN ID:
        0x505'''
    def __init__(self, 
                 joint_5: int = 0):
        self.joint_5 = joint_5

class ArmMsgFeedbackMasterJointStates6(AttributeBase):
    '''CAN ID:
        0x506'''
    def __init__(self, 
                 joint_6: int = 0):
        self.joint_6 = joint_6

class ArmMsgFeedbackMasterJointStates7(AttributeBase):
    '''CAN ID:
        0x507'''
    def __init__(self, 
                 joint_7: int = 0):
        self.joint_7 = joint_7
