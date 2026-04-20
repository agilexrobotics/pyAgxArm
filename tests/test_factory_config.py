import pytest

from pyAgxArm import AgxArmFactory, ArmModel, NeroFW, PiperFW, create_agx_arm_config

from tests.conftest import new_virtual_channel

_LOAD_CLASS_CASES = [
    (ArmModel.PIPER, PiperFW.DEFAULT, "pyAgxArm.protocols.can_protocol.drivers.piper.default.driver"),
    (ArmModel.PIPER, PiperFW.V183, "pyAgxArm.protocols.can_protocol.drivers.piper.versions.v183.driver"),
    (ArmModel.PIPER, PiperFW.V188, "pyAgxArm.protocols.can_protocol.drivers.piper.versions.v188.driver"),
    (ArmModel.NERO, NeroFW.DEFAULT, "pyAgxArm.protocols.can_protocol.drivers.nero.default.driver"),
    (ArmModel.NERO, NeroFW.V111, "pyAgxArm.protocols.can_protocol.drivers.nero.versions.v111.driver"),
    (ArmModel.PIPER_H, PiperFW.DEFAULT, "pyAgxArm.protocols.can_protocol.drivers.piper_h.default.driver"),
    (ArmModel.PIPER_H, PiperFW.V183, "pyAgxArm.protocols.can_protocol.drivers.piper_h.versions.v183.driver"),
    (ArmModel.PIPER_H, PiperFW.V188, "pyAgxArm.protocols.can_protocol.drivers.piper_h.versions.v188.driver"),
    (ArmModel.PIPER_L, PiperFW.DEFAULT, "pyAgxArm.protocols.can_protocol.drivers.piper_l.default.driver"),
    (ArmModel.PIPER_L, PiperFW.V183, "pyAgxArm.protocols.can_protocol.drivers.piper_l.versions.v183.driver"),
    (ArmModel.PIPER_L, PiperFW.V188, "pyAgxArm.protocols.can_protocol.drivers.piper_l.versions.v188.driver"),
    (ArmModel.PIPER_X, PiperFW.DEFAULT, "pyAgxArm.protocols.can_protocol.drivers.piper_x.default.driver"),
    (ArmModel.PIPER_X, PiperFW.V183, "pyAgxArm.protocols.can_protocol.drivers.piper_x.versions.v183.driver"),
    (ArmModel.PIPER_X, PiperFW.V188, "pyAgxArm.protocols.can_protocol.drivers.piper_x.versions.v188.driver"),
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


def _make_minimal_factory_config():
    return {
        "robot": ArmModel.PIPER,
        "firmeware_version": PiperFW.DEFAULT,
        "comm": {"type": "can"},
    }


def _patch_dummy_driver(monkeypatch):
    class DummyArm:
        def __init__(self, config, **kwargs):
            self.config = config
            self.kwargs = kwargs
            self.disconnect_calls = 0

        def disconnect(self):
            self.disconnect_calls += 1

    monkeypatch.setattr(
        AgxArmFactory,
        "load_class",
        classmethod(lambda cls, config: DummyArm),
    )
    with AgxArmFactory._cache_lock:
        AgxArmFactory._instance_cache.clear()
    AgxArmFactory.set_reuse_policy("replace")
    return DummyArm


def test_create_arm_default_policy_is_replace(monkeypatch):
    _patch_dummy_driver(monkeypatch)
    config = _make_minimal_factory_config()

    old_arm = AgxArmFactory.create_arm(config)
    new_arm = AgxArmFactory.create_arm(config)

    assert old_arm is not new_arm
    assert old_arm.disconnect_calls >= 1


def test_create_arm_policy_new_returns_new_instance(monkeypatch):
    _patch_dummy_driver(monkeypatch)
    config = _make_minimal_factory_config()
    AgxArmFactory.set_reuse_policy("new")

    arm1 = AgxArmFactory.create_arm(config)
    arm2 = AgxArmFactory.create_arm(config)

    assert arm1 is not arm2


def test_create_arm_policy_reuse_returns_cached_instance(monkeypatch):
    _patch_dummy_driver(monkeypatch)
    config = _make_minimal_factory_config()
    AgxArmFactory.set_reuse_policy("reuse")

    arm1 = AgxArmFactory.create_arm(config)
    arm2 = AgxArmFactory.create_arm(config)

    assert arm1 is arm2


def test_create_arm_policy_replace_disconnects_old_instance(monkeypatch):
    _patch_dummy_driver(monkeypatch)
    config = _make_minimal_factory_config()
    AgxArmFactory.set_reuse_policy("replace")

    old_arm = AgxArmFactory.create_arm(config)
    new_arm = AgxArmFactory.create_arm(config)

    assert old_arm is not new_arm
    assert old_arm.disconnect_calls >= 1


def test_agx_arm_factory_detect_can_configs_uses_python_can(monkeypatch):
    import can as python_can

    def _fake_detect(interfaces=None, timeout=5.0):
        return [{"interface": "virtual", "channel": "probe_test"}]

    monkeypatch.setattr(python_can, "detect_available_configs", _fake_detect)
    out = AgxArmFactory.detect_can_configs(interfaces=["virtual"], timeout=0.1)
    assert out == [{"interface": "virtual", "channel": "probe_test"}]
