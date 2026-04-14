import threading
from collections import deque
from typing import Deque, List

import can

from tests.slaves import _can_payloads as pl

DEVICE_FRAMES_MAX = 8192

_MOTION_MODE_TO_FEEDBACK = {
    0x00: 0x00,
    0x01: 0x01,
    0x02: 0x02,
    0x03: 0x03,
    0x04: 0x04,
    0x05: 0x05,
    0x06: 0x06,
}


class NeroCanSlave:
    """模拟 Nero：主动反馈（251–267、2A1–2A9、501–507）只按需发 **一轮**；不测真实周期。"""

    def __init__(self, channel: str):
        self._bus = can.Bus(interface="virtual", channel=channel, receive_own_messages=False)
        self._host_frames: Deque[can.Message] = deque(maxlen=DEVICE_FRAMES_MAX)
        self._device_frames: Deque[can.Message] = deque(maxlen=DEVICE_FRAMES_MAX)
        self._lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._stop = threading.Event()
        self._th = threading.Thread(target=self._loop, daemon=True)

        self._joint_count = 7
        self._joints_enabled = [False] * self._joint_count
        self._ctrl_mode = 0x01
        self._mode_feedback = 0x01
        self._leader_rad = [0.01 * float(i + 1) for i in range(self._joint_count)]
        self._proactive_burst_sent = False
        self._proactive_refresh = False
        self._cpv_reply_enabled = True
        self._cpv_store: dict = {}

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

    def emit_proactive_feedback_once(self) -> None:
        """测试辅助：主动发送一轮标准反馈帧。"""
        for m in self._standard_feedback_burst():
            self._send_and_record(m)

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
                if ji == 8:
                    self._joints_enabled = [en] * self._joint_count
                elif 1 <= ji <= 7:
                    self._joints_enabled[ji - 1] = en
                if self._joints_enabled != prev:
                    self._proactive_refresh = True

    def _foc_byte(self, joint_zero_based: int) -> int:
        with self._state_lock:
            return 0x40 if self._joints_enabled[joint_zero_based] else 0x00

    def _high_joint_messages(self) -> List[can.Message]:
        frames: List[can.Message] = []
        for i in range(6):
            frames.append(
                can.Message(
                    is_extended_id=False,
                    arbitration_id=0x251 + i,
                    data=pl.pack_feedback_high_spd(),
                )
            )
        frames.append(
            can.Message(
                is_extended_id=False,
                arbitration_id=0x257,
                data=pl.pack_feedback_high_spd(),
            )
        )
        return frames

    def _low_joint_messages(self) -> List[can.Message]:
        frames: List[can.Message] = []
        for i in range(6):
            frames.append(
                can.Message(
                    is_extended_id=False,
                    arbitration_id=0x261 + i,
                    data=pl.pack_feedback_low_spd(foc_status_byte=self._foc_byte(i)),
                )
            )
        frames.append(
            can.Message(
                is_extended_id=False,
                arbitration_id=0x267,
                data=pl.pack_feedback_low_spd(foc_status_byte=self._foc_byte(6)),
            )
        )
        return frames

    def _master_feedback_messages(self) -> List[can.Message]:
        with self._state_lock:
            cm = self._ctrl_mode
            mf = self._mode_feedback
            j7 = self._leader_rad[6]
            lr = list(self._leader_rad)
        frames: List[can.Message] = [
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
            can.Message(
                is_extended_id=False,
                arbitration_id=0x2A9,
                data=pl.pack_joint7_only_feedback(j7),
            ),
        ]
        for i, lid in enumerate(range(0x501, 0x508)):
            frames.append(
                can.Message(
                    is_extended_id=False,
                    arbitration_id=lid,
                    data=pl.pack_leader_joint_rad(lr[i]),
                )
            )
        return frames

    def _standard_feedback_burst(self) -> List[can.Message]:
        return (
            self._high_joint_messages()
            + self._low_joint_messages()
            + self._master_feedback_messages()
        )

    def _send_and_record(self, msg: can.Message) -> None:
        self._bus.send(msg, timeout=0.2)
        with self._lock:
            self._device_frames.append(msg)

    def _cpv_replies(self, aid: int, data: bytes) -> List[can.Message]:
        if not self._cpv_reply_enabled or not (0x181 <= aid <= 0x187):
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
        # Nero get_firmware() 读取 data[6], data[7] -> "major.minor"
        return [
            can.Message(
                is_extended_id=False,
                arbitration_id=0x4AF,
                data=bytes([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x0B]),
            )
        ]

    def _loop(self) -> None:
        while not self._stop.is_set():
            frame = self._bus.recv(timeout=0.05)
            if frame is None:
                continue
            payload = bytes(frame.data)
            with self._lock:
                self._host_frames.append(frame)

            self._apply_host_command(frame.arbitration_id, payload)

            with self._state_lock:
                need_proactive = (not self._proactive_burst_sent) or self._proactive_refresh
                if need_proactive:
                    self._proactive_burst_sent = True
                    self._proactive_refresh = False
            if need_proactive:
                for m in self._standard_feedback_burst():
                    self._send_and_record(m)
            for m in self._cpv_replies(frame.arbitration_id, payload):
                self._send_and_record(m)
            for m in self._firmware_replies(frame.arbitration_id):
                self._send_and_record(m)
