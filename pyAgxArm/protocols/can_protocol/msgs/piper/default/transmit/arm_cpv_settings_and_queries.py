#!/usr/bin/env python3
# -*-coding:utf8-*-
from typing_extensions import Literal

from ....core.attritube_base import AttributeBase


class ArmMsgCPVSettingsAndQueries(AttributeBase):
    '''
    transmit

    机械臂CPV设置与查询

    CAN ID:
        0x180 + NUM

    每个ID对应单个关节

    Args:
        mode: 模式 ('w': 写入模式; 'r': 读取模式)
        type: 参数类型 ('po': 位置; 'sp'：速度; 'ac': 加速度; 'dc': 减速度; 'vv': 轮廓速度; 'pp': 位置环Kp; 'kp': 速度环Kp; 'ki': 速度环Ki)
        value: 参数值 (位置/速度/加速度/减速度/轮廓速度/位置环Kp/速度环Kp/速度环Ki)

    位描述:

        Byte 0: mode 模式位, 'r'(0x72): 查询参数, 'w'(0x77): 设置参数, 'a'(0x61): 应答帧
        Byte 1: type 参数类型位 'p'(0x70), 's'(0x73), 'a'(0x61), 'd'(0x64), 'v'(0x76), 'k'(0x6B)
        Byte 2: type 参数类型位 'o'(0x6F), 'c'(0x63), 'i'(0x69)
        Byte 3: value 参数值高八位
        Byte 4: value 参数值低八位
        Byte 5: value 参数值高八位
        Byte 6: value 参数值低八位
    '''

    def __init__(
        self,
        mode: Literal['w', 'r'],
        type: Literal['po', 'sp', 'ac', 'dc', 'vv', 'pp', 'kp', 'ki'],
        value: int,
    ):
        if mode not in ['w', 'r']:
            raise ValueError("Invalid mode")
        if type not in ['po', 'sp', 'ac', 'dc', 'vv', 'pp', 'kp', 'ki']:
            raise ValueError("Invalid type")
        if not isinstance(value, int):
            raise ValueError("Invalid value, should be int")
        self.mode = mode
        self.type = [char for char in type]
        self.value = value


class ArmMsgCPVSettingsAndQueries1(ArmMsgCPVSettingsAndQueries):
    '''CAN ID:
        0x181'''


class ArmMsgCPVSettingsAndQueries2(ArmMsgCPVSettingsAndQueries):
    '''CAN ID:
        0x182'''


class ArmMsgCPVSettingsAndQueries3(ArmMsgCPVSettingsAndQueries):
    '''CAN ID:
        0x183'''


class ArmMsgCPVSettingsAndQueries4(ArmMsgCPVSettingsAndQueries):
    '''CAN ID:
        0x184'''


class ArmMsgCPVSettingsAndQueries5(ArmMsgCPVSettingsAndQueries):
    '''CAN ID:
        0x185'''


class ArmMsgCPVSettingsAndQueries6(ArmMsgCPVSettingsAndQueries):
    '''CAN ID:
        0x186'''
