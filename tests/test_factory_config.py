import pytest

from pyAgxArm import AgxArmFactory, ArmModel, NeroFW, PiperFW, create_agx_arm_config

from tests.conftest import new_virtual_channel

_LOAD_CLASS_CASES = [
    (ArmModel.PIPER, PiperFW.DEFAULT, "pyAgxArm.protocols.can_protocol.drivers.piper.default.driver"),
    (ArmModel.PIPER, PiperFW.V183, "pyAgxArm.protocols.can_protocol.drivers.piper.versions.v183.driver"),
    (ArmModel.PIPER, PiperFW.V188, "pyAgxArm.protocols.can_protocol.drivers.piper.versions.v188.driver"),
    (ArmModel.PIPER, PiperFW.V189, "pyAgxArm.protocols.can_protocol.drivers.piper.versions.v189.driver"),
    (ArmModel.NERO, NeroFW.DEFAULT, "pyAgxArm.protocols.can_protocol.drivers.nero.default.driver"),
    (ArmModel.NERO, NeroFW.V111, "pyAgxArm.protocols.can_protocol.drivers.nero.versions.v111.driver"),
    (ArmModel.NERO, NeroFW.V112, "pyAgxArm.protocols.can_protocol.drivers.nero.versions.v112.driver"),
    (ArmModel.NERO, NeroFW.V120, "pyAgxArm.protocols.can_protocol.drivers.nero.versions.v120.driver"),
    (ArmModel.PIPER_H, PiperFW.DEFAULT, "pyAgxArm.protocols.can_protocol.drivers.piper_h.default.driver"),
    (ArmModel.PIPER_H, PiperFW.V183, "pyAgxArm.protocols.can_protocol.drivers.piper_h.versions.v183.driver"),
    (ArmModel.PIPER_H, PiperFW.V188, "pyAgxArm.protocols.can_protocol.drivers.piper_h.versions.v188.driver"),
    (ArmModel.PIPER_H, PiperFW.V189, "pyAgxArm.protocols.can_protocol.drivers.piper_h.versions.v189.driver"),
    (ArmModel.PIPER_L, PiperFW.DEFAULT, "pyAgxArm.protocols.can_protocol.drivers.piper_l.default.driver"),
    (ArmModel.PIPER_L, PiperFW.V183, "pyAgxArm.protocols.can_protocol.drivers.piper_l.versions.v183.driver"),
    (ArmModel.PIPER_L, PiperFW.V188, "pyAgxArm.protocols.can_protocol.drivers.piper_l.versions.v188.driver"),
    (ArmModel.PIPER_L, PiperFW.V189, "pyAgxArm.protocols.can_protocol.drivers.piper_l.versions.v189.driver"),
    (ArmModel.PIPER_X, PiperFW.DEFAULT, "pyAgxArm.protocols.can_protocol.drivers.piper_x.default.driver"),
    (ArmModel.PIPER_X, PiperFW.V183, "pyAgxArm.protocols.can_protocol.drivers.piper_x.versions.v183.driver"),
    (ArmModel.PIPER_X, PiperFW.V188, "pyAgxArm.protocols.can_protocol.drivers.piper_x.versions.v188.driver"),
    (ArmModel.PIPER_X, PiperFW.V189, "pyAgxArm.protocols.can_protocol.drivers.piper_x.versions.v189.driver"),
]


@pytest.mark.parametrize("robot,fw,expected_module", _LOAD_CLASS_CASES)
def test_load_class_routes_to_expected_driver_module(robot, fw, expected_module):
    channel = new_virtual_channel("ci_factory")
    cfg = create_agx_arm_config(
        robot=robot,
        firmeware_version=fw,
        interface="virtual",
        channel=channel,
    )
    driver_cls = AgxArmFactory.load_class(cfg)
    assert driver_cls.__module__ == expected_module


@pytest.mark.parametrize(
    "fw,expect_cpv",
    [
        (NeroFW.DEFAULT, False),
        (NeroFW.V111, False),
        (NeroFW.V112, True),
        (NeroFW.V120, True),
    ],
)
def test_nero_motion_mode_options_cpv_on_v112_and_v120(fw, expect_cpv):
    channel = new_virtual_channel("ci_nero_opts")
    cfg = create_agx_arm_config(
        robot=ArmModel.NERO,
        firmeware_version=fw,
        interface="virtual",
        channel=channel,
    )
    arm = AgxArmFactory.create_arm(cfg)
    modes = arm.OPTIONS.MOTION_MODE.value_list()
    assert ("cpv" in modes) is expect_cpv
    if not expect_cpv:
        with pytest.raises(ValueError):
            arm.set_motion_mode("cpv")


@pytest.mark.parametrize("robot,fw", [(c[0], c[1]) for c in _LOAD_CLASS_CASES])
def test_create_arm_connect_disconnect_smoke(robot, fw):
    channel = new_virtual_channel("ci_factory_smoke")
    cfg = create_agx_arm_config(
        robot=robot,
        firmeware_version=fw,
        interface="virtual",
        channel=channel,
    )
    arm = AgxArmFactory.create_arm(cfg)
    arm.connect()
    arm.disconnect()
