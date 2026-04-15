import threading
from collections import deque
from typing import Deque, List

import can

from tests.slaves import _can_payloads as pl

DEVICE_FRAMES_MAX = 8192

# ArmMsgModeCtrl.Enums.MotionMode 数值 -> 0x2A1 mode_feedback（与 Piper 协议一致）
_MOTION_MODE_TO_FEEDBACK = {
    0x00: 0x00,
    0x01: 0x01,
    0x02: 0x02,
    0x03: 0x03,
    0x04: 0x04,
    0x05: 0x05,
    0x06: 0x06,
}


class PiperCanSlave:
    """模拟 Piper：主动状态反馈（251–266、2A1–2A7）只按需在总线上发 **一轮**，依赖上位机 Parser 缓存；查询/设置应答（476 等）仍按对应主机帧即时回复。"""

    def __init__(self, channel: str):
        self._bus = can.Bus(interface="virtual", channel=channel, receive_own_messages=False)
        self._host_frames: Deque[can.Message] = deque(maxlen=DEVICE_FRAMES_MAX)
        self._device_frames: Deque[can.Message] = deque(maxlen=DEVICE_FRAMES_MAX)
        self._lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._stop = threading.Event()
        self._th = threading.Thread(target=self._loop, daemon=True)

        self._joint_count = 6
        self._joints_enabled = [False] * self._joint_count
        self._ctrl_mode = 0x01
        self._mode_feedback = 0x01
        self._proactive_burst_sent = False
        self._proactive_refresh = False
        self._cpv_reply_enabled = True
        self._cpv_store: dict = {}
        self._joint_assistance = [0] * self._joint_count

    def start(self):
        self._th.start()

    def stop(self):
        self._stop.set()
        self._th.join(timeout=1.0)
        self._bus.shutdown()

    @property
    def host_frames(self):
        with self._lock:
            return list(self._host_frames)

    @property
    def device_frames(self):
        with self._lock:
            return list(self._device_frames)

    def _apply_host_command(self, aid: int, data: bytes) -> None:
        d = data.ljust(8, b"\x00")
        if aid == 0x151 and len(d) >= 2:
            nm = d[0]
            nf = _MOTION_MODE_TO_FEEDBACK.get(d[1], 0x01)
            with self._state_lock:
                if nm != self._ctrl_mode or nf != self._mode_feedback:
                    self._proactive_refresh = True
                self._ctrl_mode = nm
                self._mode_feedback = nf
        elif aid == 0x471:
            ji = d[0]
            en = d[1] == 0x02
            with self._state_lock:
                prev = list(self._joints_enabled)
                if ji in (7, 0xFF):
                    self._joints_enabled = [en] * self._joint_count
                elif 1 <= ji <= 6:
                    self._joints_enabled[ji - 1] = en
                if self._joints_enabled != prev:
                    self._proactive_refresh = True

    def _foc_byte(self, joint_zero_based: int) -> int:
        with self._state_lock:
            return 0x40 if self._joints_enabled[joint_zero_based] else 0x00

    def _high_joint_messages(self) -> List[can.Message]:
        frames: List[can.Message] = []
        for i in range(self._joint_count):
            frames.append(
                can.Message(
                    is_extended_id=False,
                    arbitration_id=0x251 + i,
                    data=pl.pack_feedback_high_spd(),
                )
            )
        return frames

    def _low_joint_messages(self) -> List[can.Message]:
        frames: List[can.Message] = []
        for i in range(self._joint_count):
            frames.append(
                can.Message(
                    is_extended_id=False,
                    arbitration_id=0x261 + i,
                    data=pl.pack_feedback_low_spd(foc_status_byte=self._foc_byte(i)),
                )
            )
        return frames

    def _master_feedback_messages(self) -> List[can.Message]:
        with self._state_lock:
            cm = self._ctrl_mode
            mf = self._mode_feedback
        return [
            can.Message(
                is_extended_id=False,
                arbitration_id=0x2A1,
                data=pl.pack_arm_status(ctrl_mode=cm, mode_feedback=mf),
            ),
            can.Message(
                is_extended_id=False,
                arbitration_id=0x2A2,
                data=pl.pack_end_pose_xy_um(),
            ),
            can.Message(
                is_extended_id=False,
                arbitration_id=0x2A3,
                data=pl.pack_end_pose_zrx(),
            ),
            can.Message(
                is_extended_id=False,
                arbitration_id=0x2A4,
                data=pl.pack_end_pose_ryrz(),
            ),
            can.Message(
                is_extended_id=False,
                arbitration_id=0x2A5,
                data=pl.pack_joint_pair_feedback(0.0, 0.0),
            ),
            can.Message(
                is_extended_id=False,
                arbitration_id=0x2A6,
                data=pl.pack_joint_pair_feedback(0.0, 0.0),
            ),
            can.Message(
                is_extended_id=False,
                arbitration_id=0x2A7,
                data=pl.pack_joint_pair_feedback(0.0, 0.0),
            ),
        ]

    def _standard_feedback_burst(self) -> List[can.Message]:
        return (
            self._high_joint_messages()
            + self._low_joint_messages()
            + self._master_feedback_messages()
        )

    def _set_instruction_replies(self, aid: int, data: bytes) -> List[can.Message]:
        d = data.ljust(8, b"\x00")
        out: List[can.Message] = []
        if aid == 0x472:
            if d[1] == 0x01:
                out.append(
                    can.Message(
                        is_extended_id=False,
                        arbitration_id=0x473,
                        data=pl.pack_feedback_473_motor_angle_limit_max_spd(),
                    )
                )
            elif d[1] == 0x02:
                out.append(
                    can.Message(
                        is_extended_id=False,
                        arbitration_id=0x47C,
                        data=pl.pack_feedback_47c_motor_max_acc_limit(),
                    )
                )
        if aid == 0x477:
            if d[3] == 0xAE:
                out.append(
                    can.Message(
                        is_extended_id=False,
                        arbitration_id=0x476,
                        data=pl.pack_set_instruction_response(0x77, 0),
                    )
                )
            if d[0] == 0x01:
                out.append(
                    can.Message(
                        is_extended_id=False,
                        arbitration_id=0x478,
                        data=pl.pack_end_vel_acc_param_feedback(),
                    )
                )
            if d[0] == 0x02:
                out.append(
                    can.Message(
                        is_extended_id=False,
                        arbitration_id=0x47B,
                        data=pl.pack_feedback_47b_crash_protection(),
                    )
                )
            if d[0] == 0x05:
                out.append(
                    can.Message(
                        is_extended_id=False,
                        arbitration_id=0x488,
                        data=pl.pack_feedback_488_joint_assistance(self._joint_assistance),
                    )
                )
            if d[2] in (0x01, 0x02):
                out.append(
                    can.Message(
                        is_extended_id=False,
                        arbitration_id=0x476,
                        data=pl.pack_set_instruction_response(0x77, 0),
                    )
                )
            if d[1] in (0x01, 0x02):
                out.append(
                    can.Message(
                        is_extended_id=False,
                        arbitration_id=0x476,
                        data=pl.pack_set_instruction_response(0x77, 0),
                    )
                )
        elif aid == 0x475:
            if d[1] == 0xAE:
                out.append(
                    can.Message(
                        is_extended_id=False,
                        arbitration_id=0x476,
                        data=pl.pack_set_instruction_response(0x75, 1),
                    )
                )
            elif d[2] == 0xAE or d[5] == 0xAE:
                out.append(
                    can.Message(
                        is_extended_id=False,
                        arbitration_id=0x476,
                        data=pl.pack_set_instruction_response(0x75, 0),
                    )
                )
        elif aid == 0x479:
            out.append(
                can.Message(
                    is_extended_id=False,
                    arbitration_id=0x476,
                    data=pl.pack_set_instruction_response(0x79, 0),
                )
            )
        elif aid == 0x474:
            out.append(
                can.Message(
                    is_extended_id=False,
                    arbitration_id=0x476,
                    data=pl.pack_set_instruction_response(0x74, 0),
                )
            )
        elif aid == 0x47A:
            out.append(
                can.Message(
                    is_extended_id=False,
                    arbitration_id=0x476,
                    data=pl.pack_set_instruction_response(0x7A, 0),
                )
            )
        elif aid == 0x487:
            self._joint_assistance = list(d[:6])
            out.append(
                can.Message(
                    is_extended_id=False,
                    arbitration_id=0x476,
                    data=pl.pack_set_instruction_response(0x87, 0),
                )
            )
            out.append(
                can.Message(
                    is_extended_id=False,
                    arbitration_id=0x488,
                    data=pl.pack_feedback_488_joint_assistance(self._joint_assistance),
                )
            )
        return out

    def _cpv_replies(self, aid: int, data: bytes) -> List[can.Message]:
        if not self._cpv_reply_enabled or not (0x181 <= aid <= 0x186):
            return []
        d = data.ljust(8, b"\x00")
        mode_byte = d[0]
        if mode_byte not in (0x72, 0x77):
            return []
        type_str = chr(d[1]) + chr(d[2])
        raw = int.from_bytes(bytes(d[3:7]), "big", signed=True)
        ji = aid - 0x180
        if mode_byte == 0x77:
            self._cpv_store[(ji, type_str)] = raw
            out_raw = raw
        else:
            out_raw = self._cpv_store.get((ji, type_str), 0)
        payload = pl.pack_cpv_ack(type_str, out_raw)
        return [
            can.Message(
                is_extended_id=False,
                arbitration_id=aid,
                data=payload,
            )
        ]

    def _firmware_replies(self, aid: int) -> List[can.Message]:
        if aid != 0x4AF:
            return []
        # 88 字节（11 帧 * 8 字节），字段位置与 Driver.get_firmware() 取值切片一致。
        seg = bytearray(b"\x00" * 88)
        seg[0:8] = b"H-V1.2-1"
        seg[16:18] = b"10"
        seg[32:38] = b"ARM_MC"
        seg[60:68] = b"S-V1.8-8"
        seg[68:74] = b"250925"
        seg[76:78] = b"15"
        return [
            can.Message(
                is_extended_id=False,
                arbitration_id=0x4AF,
                data=bytes(seg[i : i + 8]),
            )
            for i in range(0, 88, 8)
        ]

    def _send_and_record(self, msg: can.Message) -> None:
        self._bus.send(msg, timeout=0.2)
        with self._lock:
            self._device_frames.append(msg)

    def _loop(self) -> None:
        while not self._stop.is_set():
            frame = self._bus.recv(timeout=0.05)
            if frame is None:
                continue
            payload = bytes(frame.data)
            with self._lock:
                self._host_frames.append(frame)

            aid = frame.arbitration_id
            self._apply_host_command(aid, payload)

            with self._state_lock:
                need_proactive = (not self._proactive_burst_sent) or self._proactive_refresh
                if need_proactive:
                    self._proactive_burst_sent = True
                    self._proactive_refresh = False
            if need_proactive:
                for m in self._standard_feedback_burst():
                    self._send_and_record(m)

            for m in self._set_instruction_replies(aid, payload):
                self._send_and_record(m)
            for m in self._cpv_replies(aid, payload):
                self._send_and_record(m)
            for m in self._firmware_replies(aid):
                self._send_and_record(m)
