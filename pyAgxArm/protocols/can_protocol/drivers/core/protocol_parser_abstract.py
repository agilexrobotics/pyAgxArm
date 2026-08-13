#!/usr/bin/env python3
# -*-coding:utf8-*-

from .protocol_parser_interface import ProtocolParserInterface
from typing_extensions import Literal, Final

class DriverAPIOptions:
    class EFFECTOR:
        """
        End-effector kind constants.

        Use:
            robot.init_effector(robot.OPTIONS.EFFECTOR.AGX_GRIPPER)
        """

        AGX_GRIPPER: Final[Literal["agx_gripper"]] = "agx_gripper"
        REVO2: Final[Literal["revo2"]] = "revo2"
        REVO2_PRO: Final[Literal["revo2_pro"]] = "revo2_pro"
        REVO2_TOUCH: Final[Literal["revo2_touch"]] = "revo2_touch"

class DriverAPIProtoAdapter:
    pass

class ProtocolParserInterface(ProtocolParserInterface):
    
    def parse_packet(self, **kwargs):
        ...

    def pack(self, **kwargs):
        ...

    