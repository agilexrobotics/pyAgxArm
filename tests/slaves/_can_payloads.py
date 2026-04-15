# CAN 8 字节 payload 构造（基于真实抓包样本 hex；仅测试用）。

_HEX_251_HIGH_SPD = "0000000000000000"
_HEX_261_LOW_SPD = "01e0001919400000"
_HEX_2A1_ARM_STATUS = "0100010000000000"
_HEX_2A2_ENDPOSE_XY = "0000000000000000"
_HEX_2A3_ENDPOSE_ZRX = "0000000000000000"
_HEX_2A4_ENDPOSE_RYRZ = "0000000000000000"
_HEX_2A5_JOINT12 = "0000000000000000"
_HEX_2A8_GRIPPER_FB = "0000000000000000"
_HEX_1C0_HAND_STATUS = "0000000000000000"
_HEX_1C1_FINGER_POS = "0000000000000000"
_HEX_1C2_FINGER_SPD = "0000000000000000"
_HEX_1C3_FINGER_CURRENT = "0000000000000000"
_HEX_473_MOTOR_LIMIT_SPD = "0100000000000000"
_HEX_47B_CRASH_RATING = "0000000000000000"
_HEX_488_JOINT_ASSIST = "0000000000000000"
_HEX_47C_MOTOR_MAX_ACC = "0100000000000000"
_HEX_478_END_VEL_ACC = "03e803e803e803e8"
_HEX_501_LEADER_J1 = "0ad7233c00000000"
_HEX_2A9_JOINT7 = "0000000000000000"


def _from_hex(h: str) -> bytes:
    return bytes.fromhex(h)


def pack_leader_joint_rad(j_rad: float) -> bytes:
    """Nero 0x501–0x507：使用真实样本帧（忽略入参，仅保留兼容签名）。"""
    return _from_hex(_HEX_501_LEADER_J1)


def pack_feedback_high_spd(
    velocity_rad_s: float = 0.0,
    current_a: float = 0.0,
    position_rad: float = 0.0,
) -> bytes:
    return _from_hex(_HEX_251_HIGH_SPD)


def pack_feedback_low_spd(
    vol_v: float = 48.0,
    foc_temp_c: float = 25.0,
    motor_temp_c: int = 25,
    foc_status_byte: int = 0x40,
    bus_current_a: float = 0.0,
) -> bytes:
    d = bytearray(_from_hex(_HEX_261_LOW_SPD))
    # 只允许按需改 foc 状态位，以支持 enable/disable 相关行为断言。
    d[5] = foc_status_byte & 0xFF
    return bytes(d)


def pack_joint_pair_feedback(j1_rad: float, j2_rad: float) -> bytes:
    return _from_hex(_HEX_2A5_JOINT12)


def pack_arm_status(
    ctrl_mode: int = 0x01,
    arm_status: int = 0x00,
    mode_feedback: int = 0x01,
    teach_status: int = 0x00,
    motion_status: int = 0x00,
    trajectory_num: int = 0,
    err_code: int = 0,
) -> bytes:
    d = bytearray(_from_hex(_HEX_2A1_ARM_STATUS))
    d[0] = ctrl_mode & 0xFF
    d[2] = mode_feedback & 0xFF
    return bytes(d)


def pack_end_pose_xy_um(x_um: int = 0, y_um: int = 0) -> bytes:
    return _from_hex(_HEX_2A2_ENDPOSE_XY)


def pack_end_pose_zrx(z_um: int = 0, rx_mdeg: int = 0) -> bytes:
    return _from_hex(_HEX_2A3_ENDPOSE_ZRX)


def pack_end_pose_ryrz(ry_mdeg: int = 0, rz_mdeg: int = 0) -> bytes:
    return _from_hex(_HEX_2A4_ENDPOSE_RYRZ)


def pack_set_instruction_response(instruction_index: int, zero_or_ack_byte: int = 0) -> bytes:
    return bytes(
        [instruction_index & 0xFF, zero_or_ack_byte & 0xFF] + [0] * 6
    )


def pack_end_vel_acc_param_feedback(
    end_max_linear_vel: float = 1.0,
    end_max_angular_vel: float = 1.0,
    end_max_linear_acc: float = 1.0,
    end_max_angular_acc: float = 1.0,
) -> bytes:
    return _from_hex(_HEX_478_END_VEL_ACC)


def pack_feedback_473_motor_angle_limit_max_spd() -> bytes:
    return _from_hex(_HEX_473_MOTOR_LIMIT_SPD)


def pack_feedback_47b_crash_protection() -> bytes:
    return _from_hex(_HEX_47B_CRASH_RATING)


def pack_feedback_488_joint_assistance(joints=None) -> bytes:
    if joints is None:
        return _from_hex(_HEX_488_JOINT_ASSIST)
    if len(joints) != 6:
        raise ValueError("joints should contain 6 items")
    data = [int(v) & 0xFF for v in joints] + [0, 0]
    return bytes(data)


def pack_feedback_47c_motor_max_acc_limit() -> bytes:
    return _from_hex(_HEX_47C_MOTOR_MAX_ACC)


def pack_gripper_feedback_2a8(
    value_raw: int,
    force_raw: int,
    status_code: int = 0,
    mode_byte: int = 0x00,
) -> bytes:
    return _from_hex(_HEX_2A8_GRIPPER_FB)


def pack_revo2_finger_pos_1c1(
    thumb_tip: int = 0,
    thumb_base: int = 0,
    index_finger: int = 0,
    middle_finger: int = 0,
    ring_finger: int = 0,
    pinky_finger: int = 0,
) -> bytes:
    return _from_hex(_HEX_1C1_FINGER_POS)


def pack_revo2_hand_status_1c0(
    thumb_tip: int = 0,
    thumb_base: int = 0,
    index_finger: int = 0,
    middle_finger: int = 0,
    ring_finger: int = 0,
    pinky_finger: int = 0,
    left_or_right: int = 0,
) -> bytes:
    return _from_hex(_HEX_1C0_HAND_STATUS)


def pack_revo2_finger_spd_1c2() -> bytes:
    return _from_hex(_HEX_1C2_FINGER_SPD)


def pack_revo2_finger_current_1c3() -> bytes:
    return _from_hex(_HEX_1C3_FINGER_CURRENT)


def pack_gripper_teaching_pendant_param_47e(
    teaching_range_per: int = 100,
    max_range_config_mm: int = 70,
    teaching_friction: int = 1,
) -> bytes:
    return bytes(
        [
            teaching_range_per & 0xFF,
            max_range_config_mm & 0xFF,
            teaching_friction & 0xFF,
            0,
            0,
            0,
            0,
            0,
        ]
    )


def pack_joint7_only_feedback(j_rad: float = 0.0) -> bytes:
    return _from_hex(_HEX_2A9_JOINT7)


def pack_cpv_ack(type_str: str, value_i32: int) -> bytes:
    from pyAgxArm.utiles.numeric_codec import NumericCodec as nc

    if len(type_str) != 2:
        raise ValueError("type_str must be length 2")
    body = (
        [0x61, ord(type_str[0]), ord(type_str[1])]
        + nc.ConvertToList_32bit(value_i32, signed=True)
    )
    return bytes(body).ljust(8, b"\x00")
