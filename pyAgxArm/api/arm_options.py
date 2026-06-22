#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from typing_extensions import Final, Literal


class ArmModel:
    """Robot arm model identifiers.

    Use::

        from pyAgxArm import ArmModel
        create_agx_arm_config(robot=ArmModel.PIPER, ...)
    """

    NERO: Final[Literal["nero"]] = "nero"
    PIPER: Final[Literal["piper"]] = "piper"
    PIPER_H: Final[Literal["piper_h"]] = "piper_h"
    PIPER_L: Final[Literal["piper_l"]] = "piper_l"
    PIPER_X: Final[Literal["piper_x"]] = "piper_x"


class PiperFW:
    """Piper series firmware versions (piper / piper_h / piper_l / piper_x).

    ======== ========================
    Constant Firmware range
    ======== ========================
    DEFAULT  <= S-V1.8-2
    V183     S-V1.8-3 ~ S-V1.8-7
    V188     S-V1.8-8
    V189     >= S-V1.8-9
    ======== ========================

    Use::

        from pyAgxArm import PiperFW
        create_agx_arm_config(..., firmeware_version=PiperFW.V188)
    """

    DEFAULT: Final[Literal["default"]] = "default"
    V183: Final[Literal["v183"]] = "v183"
    V188: Final[Literal["v188"]] = "v188"
    V189: Final[Literal["v189"]] = "v189"


class NeroFW:
    """Nero series firmware versions.

    ======== ========================
    Constant Firmware range
    ======== ========================
    DEFAULT  <= 1.10
    V111     1.11
    V112     1.12
    V120     >= 1.20
    ======== ========================

    Use::

        from pyAgxArm import NeroFW
        create_agx_arm_config(..., firmeware_version=NeroFW.V111)
    """

    DEFAULT: Final[Literal["default"]] = "default"
    V111: Final[Literal["v111"]] = "v111"
    V112: Final[Literal["v112"]] = "v112"
    V120: Final[Literal["v120"]] = "v120"
