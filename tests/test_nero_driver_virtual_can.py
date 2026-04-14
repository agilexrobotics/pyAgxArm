import pytest

from pyAgxArm import AgxArmFactory, ArmModel, NeroFW, create_agx_arm_config

from tests.conftest import hex_payloads, new_virtual_channel, wait_until
from tests.slaves.nero_can_slave import NeroCanSlave

def _make_nero_arm(fw, channel):
    cfg = create_agx_arm_config(
        robot=ArmModel.NERO,
        firmeware_version=fw,
        interface="virtual",
        channel=channel,
    )
    return AgxArmFactory.create_arm(cfg)


def _assert_send_only_flow(arm, device):
    arm.connect()
    joints = [0.0] * arm.joint_nums

    arm.set_speed_percent(100)
    arm.set_motion_mode(arm.OPTIONS.MOTION_MODE.J)
    arm.enable()
    arm.move_j(joints)
    arm.move_js(joints)
    arm.move_mit(1, p_des=0.0, v_des=0.0, kp=10.0, kd=0.8, t_ff=0.0)
    arm.disable()
    arm.disconnect()

    ok = wait_until(lambda: len(device.host_frames) >= 8)
    assert ok, "Timeout waiting for host command frames on virtual CAN"

    host_hex = hex_payloads(device.host_frames)
    assert host_hex
    assert all(len(x) <= 16 for x in host_hex)


@pytest.mark.parametrize("fw", [NeroFW.DEFAULT, NeroFW.V111])
def test_nero_driver_demo_style_api_with_virtual_device(fw):
    channel = new_virtual_channel("ci_nero")
    device = NeroCanSlave(channel=channel)
    device.start()
    try:
        arm = _make_nero_arm(fw, channel)
        _assert_send_only_flow(arm, device)
    finally:
        device.stop()


def test_nero_get_leader_joint_angles_virtual_slave():
    channel = new_virtual_channel("ci_nero_leader")
    device = NeroCanSlave(channel=channel)
    device.start()
    try:
        arm = _make_nero_arm(NeroFW.DEFAULT, channel)
        arm.connect()
        # 主动反馈类：由模拟臂主动发送测试 hex 帧，主机只校验解码值。
        device.emit_proactive_feedback_once()
        exp = [0.01] * 7

        def _leader_ok():
            m = arm.get_leader_joint_angles()
            if m is None:
                return False
            return all(abs(m.msg[i] - exp[i]) < 1e-5 for i in range(7))

        assert wait_until(_leader_ok, timeout=2.0)
        arm.disconnect()
    finally:
        device.stop()


def test_nero_driver_set_normal_mode_and_extended_motion_l2():
    channel = new_virtual_channel("ci_nero_ext")
    device = NeroCanSlave(channel=channel)
    device.start()
    try:
        arm = _make_nero_arm(NeroFW.DEFAULT, channel)
        arm.connect()
        arm.set_normal_mode()

        pose = [-0.45, -0.0, 0.45, -1.5708, 0.0, -3.14159]
        mid = [-0.45, 0.0, 0.5, -1.5708, 0.0, -3.14159]
        end = [-0.45, 0.2, 0.45, -1.5708, 0.0, -3.14159]

        arm.set_motion_mode(arm.OPTIONS.MOTION_MODE.P)
        arm.move_p(pose)

        arm.set_motion_mode(arm.OPTIONS.MOTION_MODE.L)
        arm.move_l(pose)

        arm.set_motion_mode(arm.OPTIONS.MOTION_MODE.C)
        arm.move_c(pose, mid, end)

        arm.electronic_emergency_stop()
        arm.reset()
        arm.disconnect()

        assert wait_until(lambda: len(device.host_frames) >= 12)
    finally:
        device.stop()


def test_nero_read_apis_with_virtual_feedback():
    channel = new_virtual_channel("ci_nero_read")
    device = NeroCanSlave(channel=channel)
    device.start()
    try:
        arm = _make_nero_arm(NeroFW.DEFAULT, channel)
        arm.connect()
        # 主动反馈类：直接注入测试帧，再校验读取类 API。
        device.emit_proactive_feedback_once()

        assert wait_until(lambda: arm.get_joint_angles() is not None)
        ja = arm.get_joint_angles()
        fp = arm.get_flange_pose()
        st = arm.get_arm_status()
        ms = arm.get_motor_states(1)
        ds = arm.get_driver_states(1)
        es = arm.get_joint_enable_status(1)
        es_all = arm.get_joints_enable_status_list()

        assert ja is not None and len(ja.msg) == 7
        assert fp is not None and len(fp.msg) == 6
        assert st is not None
        assert ms is not None
        assert ds is not None
        assert isinstance(es, bool)
        assert isinstance(es_all, list) and len(es_all) == 7
        arm.disconnect()
    finally:
        device.stop()


def test_nero_leader_follower_apis_send_expected_frames():
    channel = new_virtual_channel("ci_nero_lf")
    device = NeroCanSlave(channel=channel)
    device.start()
    try:
        arm = _make_nero_arm(NeroFW.DEFAULT, channel)
        arm.connect()
        n0 = len(device.host_frames)

        arm.set_leader_mode()
        arm.set_follower_mode()
        arm.set_normal_mode()

        assert wait_until(lambda: len(device.host_frames) > n0)
        ids = {f.arbitration_id for f in device.host_frames[n0:]}
        assert 0x470 in ids
        assert 0x151 in ids
        arm.disconnect()
    finally:
        device.stop()


def test_nero_get_firmware_with_realistic_hex():
    channel = new_virtual_channel("ci_nero_fw")
    device = NeroCanSlave(channel=channel)
    device.start()
    try:
        arm = _make_nero_arm(NeroFW.DEFAULT, channel)
        arm.connect()
        fw = arm.get_firmware(timeout=1.0, min_interval=0.0)
        assert fw is not None
        assert fw["software_version"] == "1.11"
        arm.disconnect()
    finally:
        device.stop()


def test_nero_driver_virtual_can_cpv_joint7_and_round_trip():
    channel = new_virtual_channel("ci_nero_cpv")
    device = NeroCanSlave(channel=channel)
    device.start()
    try:
        arm = _make_nero_arm(NeroFW.DEFAULT, channel)
        arm.connect()
        arm.set_motion_mode(arm.OPTIONS.MOTION_MODE.CPV)
        assert wait_until(
            lambda: any(
                f.arbitration_id == 0x151 and len(f.data) >= 2 and f.data[1] == 0x05
                for f in device.host_frames
            ),
            timeout=1.0,
        )

        pos7 = 0.087
        n0 = len(device.host_frames)
        arm.move_cpv_pos(7, pos7)
        assert wait_until(
            lambda: any(
                f.arbitration_id == 0x187 for f in device.host_frames[n0:]
            ),
            timeout=1.0,
        )

        got7 = arm.get_cpv_pos(7, timeout=1.0, min_interval=0.0)
        assert got7 is not None
        assert abs(got7 - pos7) < 1e-4

        arm.move_cpv_vel(1, 0.01)
        g1 = arm.get_cpv_vel(1, timeout=1.0, min_interval=0.0)
        assert g1 is not None and abs(g1 - 0.01) < 1e-6

        assert arm.set_cpv_cv(4, 0.5, timeout=1.0)
        cv = arm.get_cpv_cv(4, timeout=1.0, min_interval=0.0)
        assert cv is not None and abs(cv - 0.5) < 1e-6

        n_cov = len(device.host_frames)
        for ji in range(1, 8):
            arm.get_cpv_pp(ji, timeout=1.0, min_interval=0.0)
        cpv_tx = {
            f.arbitration_id
            for f in device.host_frames[n_cov:]
            if 0x181 <= f.arbitration_id <= 0x187
        }
        assert cpv_tx == {0x181 + i for i in range(7)}
        arm.disconnect()
    finally:
        device.stop()


def test_nero_driver_virtual_can_cpv_timeout_when_slave_mutes_cpv():
    channel = new_virtual_channel("ci_nero_cpv_to")
    device = NeroCanSlave(channel=channel)
    device.start()
    try:
        arm = _make_nero_arm(NeroFW.DEFAULT, channel)
        arm.connect()
        arm.set_motion_mode(arm.OPTIONS.MOTION_MODE.CPV)
        device._cpv_reply_enabled = False
        assert arm.get_cpv_kp(2, timeout=0.05, min_interval=0.0) is None
        arm.disconnect()
    finally:
        device.stop()


def test_nero_driver_virtual_can_cpv_each_public_api_once():
    """对 Nero 驱动文档中的 CPV 公开接口各做一次成功调用（含第 7 轴 0x187）。"""
    channel = new_virtual_channel("ci_nero_cpv_all")
    device = NeroCanSlave(channel=channel)
    device.start()
    try:
        arm = _make_nero_arm(NeroFW.DEFAULT, channel)
        arm.connect()
        arm.set_motion_mode("cpv")
        to = 1.0
        mi = 0.0

        arm.move_cpv_pos(7, 0.04)
        assert arm.get_cpv_pos(7, timeout=to, min_interval=mi) is not None

        arm.move_cpv_vel(6, 0.025)
        assert arm.get_cpv_vel(6, timeout=to, min_interval=mi) is not None

        assert arm.set_cpv_acc(1, 1.05, timeout=to)
        assert arm.get_cpv_acc(1, timeout=to, min_interval=mi) is not None

        assert arm.set_cpv_dcc(2, 1.06, timeout=to)
        assert arm.get_cpv_dcc(2, timeout=to, min_interval=mi) is not None

        assert arm.set_cpv_cv(3, 0.42, timeout=to)
        assert arm.get_cpv_cv(3, timeout=to, min_interval=mi) is not None

        assert arm.set_cpv_pp(4, 1.15, timeout=to)
        assert arm.get_cpv_pp(4, timeout=to, min_interval=mi) is not None

        assert arm.set_cpv_kp(5, 0.88, timeout=to)
        assert arm.get_cpv_kp(5, timeout=to, min_interval=mi) is not None

        assert arm.set_cpv_ki(7, 0.19, timeout=to)
        assert arm.get_cpv_ki(7, timeout=to, min_interval=mi) is not None

        arm.disconnect()
    finally:
        device.stop()
