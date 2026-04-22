import logging

from pyAgxArm import AgxArmFactory, ArmModel, Logger, PiperFW, create_agx_arm_config
from pyAgxArm.utiles.validator import Validator


def test_formats_presets_are_formatter_instances():
    assert isinstance(Logger.Formats.FULL, logging.Formatter)
    assert isinstance(Logger.Formats.PLAIN_TEXT, logging.Formatter)


def test_bridge_routes_records_and_default_is_plain_text():
    debug_msgs = []
    warn_msgs = []
    log = Logger("pyAgxArm.bridge.default")
    # Default bridge level is INFO; pass DEBUG here to exercise the debug sink.
    log.bridge_enable(
        debug=debug_msgs.append,
        warning=warn_msgs.append,
        level=Logger.Level.DEBUG,
        replace_handlers=True,
        propagate=False,
    )
    src = logging.getLogger("pyAgxArm.bridge.default")

    try:
        src.debug("debug-x")
        src.warning("warn-x")
    finally:
        log.bridge_disable()

    assert any(m == "debug-x" for m in debug_msgs)
    assert any(m == "warn-x" for m in warn_msgs)


def test_bridge_supports_custom_formatter():
    lines = []
    log = Logger("pyAgxArm.bridge.custom")
    log.bridge_enable(
        warning=lines.append,
        formatter=Logger.Formats.FULL,
        replace_handlers=True,
        propagate=False,
    )
    src = logging.getLogger("pyAgxArm.bridge.custom")

    try:
        src.warning("custom-fmt")
    finally:
        log.bridge_disable()

    assert lines
    assert "[WARNING]" in lines[-1]
    assert "custom-fmt" in lines[-1]


def test_validator_child_logs_route_to_parent_bridge():
    warn_msgs = []
    log = Logger("pyAgxArm.bridge.validator")
    log.bridge_enable(
        warning=warn_msgs.append,
        replace_handlers=True,
        propagate=False,
    )
    validator_logger = log.get_child("validator")

    try:
        Validator.clamp_joints([999.0] * 6, 6, logger_=validator_logger)
    finally:
        log.bridge_disable()

    assert warn_msgs
    assert any("joints[" in m for m in warn_msgs)


def test_console_enable_supports_custom_handler_and_format():
    lines = []

    class _ListHandler(logging.Handler):
        def emit(self, record):
            lines.append(self.format(record))

    log = Logger("pyAgxArm.console")
    log.console_enable(
        handler=_ListHandler(),
        formatter=Logger.Formats.PLAIN_TEXT,
        replace_handlers=True,
        propagate=False,
    )
    src = logging.getLogger("pyAgxArm.console")

    try:
        src.warning("console-check")
    finally:
        log.console_disable()

    assert lines
    assert "console-check" in lines[-1]


def test_handler_kinds_can_coexist_without_replacing_each_other():
    bridge_lines = []
    console_lines = []
    log = Logger("pyAgxArm.coexist")

    class _ListHandler(logging.Handler):
        def emit(self, record):
            console_lines.append(self.format(record))

    log.bridge_enable(
        warning=bridge_lines.append,
        replace_handlers=False,
        propagate=False,
    )
    log.console_enable(
        level=logging.WARNING,
        replace_handlers=False,
        propagate=False,
        handler=_ListHandler(),
        formatter=Logger.Formats.PLAIN_TEXT,
    )

    src = logging.getLogger("pyAgxArm.coexist")
    src.warning("coexist-check")
    log.bridge_disable()
    log.console_disable()

    assert bridge_lines and "coexist-check" in bridge_lines[-1]
    assert console_lines and "coexist-check" in console_lines[-1]


def test_logger_isolation_between_instances():
    lines = []
    log_a = Logger("pyAgxArm.instance.a")
    log_b = Logger("pyAgxArm.instance.b")
    log_a.bridge_enable(
        warning=lines.append,
        replace_handlers=True,
        propagate=False,
    )
    try:
        log_b.warning("isolated-check")
    finally:
        log_a.bridge_disable()

    assert not lines


def test_bridge_emit_min_interval_throttles_identical_records():
    lines = []
    log = Logger("pyAgxArm.bridge.throttle")
    log.bridge_enable(
        warning=lines.append,
        replace_handlers=True,
        propagate=False,
        emit_min_interval=1.0,
    )
    src = logging.getLogger("pyAgxArm.bridge.throttle")
    try:
        src.warning("dup")
        src.warning("dup")
    finally:
        log.bridge_disable()
    assert lines.count("dup") == 1


def test_bridge_emit_min_interval_zero_disables_throttle():
    lines = []
    log = Logger("pyAgxArm.bridge.nothrottle")
    log.bridge_enable(
        warning=lines.append,
        replace_handlers=True,
        propagate=False,
        emit_min_interval=0.0,
    )
    src = logging.getLogger("pyAgxArm.bridge.nothrottle")
    try:
        src.warning("dup")
        src.warning("dup")
    finally:
        log.bridge_disable()
    assert lines.count("dup") == 2


def test_console_emit_min_interval_throttles_identical_records():
    lines = []

    class _ListHandler(logging.Handler):
        def emit(self, record):
            lines.append(self.format(record))

    log = Logger("pyAgxArm.console.throttle")
    log.console_enable(
        handler=_ListHandler(),
        formatter=Logger.Formats.PLAIN_TEXT,
        replace_handlers=True,
        propagate=False,
        emit_min_interval=1.0,
    )
    src = logging.getLogger("pyAgxArm.console.throttle")
    try:
        src.warning("dup")
        src.warning("dup")
    finally:
        log.console_disable()
    assert len(lines) == 1


def test_factory_replace_cleans_old_instance_managed_handlers():
    prev_policy = AgxArmFactory.get_reuse_policy()
    AgxArmFactory.set_reuse_policy("replace")
    channel = "ci_logger_replace_handlers"
    cfg = create_agx_arm_config(
        robot=ArmModel.PIPER,
        firmeware_version=PiperFW.DEFAULT,
        interface="virtual",
        channel=channel,
    )

    try:
        arm1 = AgxArmFactory.create_arm(cfg)
        arm1.log.bridge_enable(
            warning=lambda _msg: None,
            replace_handlers=True,
            propagate=False,
        )
        old_logger_name = arm1.log.logger_name

        arm2 = AgxArmFactory.create_arm(cfg)
        assert arm2 is not arm1

        old_logger = logging.getLogger(old_logger_name)
        assert not any(getattr(h, "_managed_handler", False) for h in old_logger.handlers)
        assert any(isinstance(h, logging.NullHandler) for h in old_logger.handlers)

        arm2.disconnect()
    finally:
        AgxArmFactory.set_reuse_policy(prev_policy)
