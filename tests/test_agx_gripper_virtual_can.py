import pytest

from tests.conftest import hex_payloads, new_virtual_channel, wait_until
from tests.slaves.agx_gripper_can_slave import AgxGripperCanSlave

from pyAgxArm import AgxArmFactory, ArmModel, PiperFW, create_agx_arm_config


def test_agx_gripper_init_and_move_gripper_m_l2():
    channel = new_virtual_channel("ci_gripper")
    slave = AgxGripperCanSlave(channel=channel)
    slave.start()
    try:
        cfg = create_agx_arm_config(
            robot=ArmModel.PIPER,
            firmeware_version=PiperFW.DEFAULT,
            interface="virtual",
            channel=channel,
        )
        arm = AgxArmFactory.create_arm(cfg)
        arm.connect()
        gripper = arm.init_effector(arm.OPTIONS.EFFECTOR.AGX_GRIPPER)
        n0 = len(slave.host_frames)

        arm.set_motion_mode(arm.OPTIONS.MOTION_MODE.P)
        gripper.move_gripper_m(0.05, force=1.0)

        assert wait_until(lambda: len(slave.host_frames) > n0)
        host = slave.host_frames[n0:]
        assert host
        assert all(len(x) <= 16 for x in hex_payloads(host))
        arm.disconnect()
    finally:
        slave.stop()


def test_agx_gripper_read_and_control_apis_l2():
    channel = new_virtual_channel("ci_gripper_more")
    slave = AgxGripperCanSlave(channel=channel)
    slave.start()
    try:
        cfg = create_agx_arm_config(
            robot=ArmModel.PIPER,
            firmeware_version=PiperFW.DEFAULT,
            interface="virtual",
            channel=channel,
        )
        arm = AgxArmFactory.create_arm(cfg)
        arm.connect()
        gripper = arm.init_effector(arm.OPTIONS.EFFECTOR.AGX_GRIPPER)
        n0 = len(slave.host_frames)

        gripper.move_gripper_deg(5.0, force=1.0)
        with pytest.warns(DeprecationWarning):
            gripper.move_gripper(0.01, force=1.0)

        assert wait_until(lambda: len(slave.host_frames) > n0)
        # Slave replies with 0x2A8 then 0x159; the read thread handles one frame per
        # recv() tick, so status may appear before ctrl feedback without a short wait.
        assert wait_until(
            lambda: gripper.get_gripper_status() is not None
            and gripper.get_gripper_ctrl_states() is not None
        )
        assert isinstance(gripper.disable_gripper(), bool)
        assert isinstance(gripper.reset_gripper(), bool)
        assert isinstance(gripper.calibrate_gripper(), bool)

        p = gripper.get_gripper_teaching_pendant_param(timeout=1.0, min_interval=0.0)
        assert p is not None
        assert p.msg.teaching_range_per == 100
        assert abs(p.msg.max_range_config - 0.07) < 1e-9
        assert p.msg.teaching_friction == 1

        assert gripper.set_gripper_teaching_pendant_param(
            teaching_range_per=120,
            max_range_config=0.1,
            teaching_friction=3,
            timeout=1.0,
        )
        p2 = gripper.get_gripper_teaching_pendant_param(timeout=1.0, min_interval=0.0)
        assert p2 is not None
        assert p2.msg.teaching_range_per == 120
        assert abs(p2.msg.max_range_config - 0.1) < 1e-9
        assert p2.msg.teaching_friction == 3

        host_ids = {f.arbitration_id for f in slave.host_frames[n0:]}
        assert {0x159, 0x477, 0x47D}.issubset(host_ids)
        arm.disconnect()
    finally:
        slave.stop()
