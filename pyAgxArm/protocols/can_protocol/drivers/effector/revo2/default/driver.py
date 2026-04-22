import logging
from typing import Optional, TypeVar

from typing_extensions import Literal

from .parser import Parser
from .....msgs.core.attritube_base import AttributeBase
from ....core.driver_context import DriverContext
from ....core.effector_driver_context import EffectorDriverContext
from .....msgs.effector.revo2.default import (
    FeedbackFingerCurrent,
    FeedbackFingerPos,
    FeedbackFingerSpd,
    FeedbackHandStatus,
    FingerPosCtrl,
    FingerSpdCtrl,
    FingerCurrentCtrl,
    FingerPosTimeCtrl,
)
from .....msgs.core.msg_abstract import MessageAbstract


T = TypeVar("T")


class Driver:
    """
    revo2 end-effector driver.
    """

    def __init__(self, config: dict, ctx: DriverContext):
        self._config = config.copy()
        self._ctx = ctx
        self._logger = self._ctx.logger.getChild("effector.revo2")
        try:
            self._parser = Parser(
                self._ctx.fps,
                logger_=self._logger.getChild("parser"),
            )
            self._effector_ctx = EffectorDriverContext(
                config,
                self._ctx,
                self._parser,
            )
        except Exception as exc:
            self._logger.error(
                "Failed to initialize revo2 effector driver (%s: %s).",
                type(exc).__name__,
                exc,
                exc_info=True,
            )
            raise

    @property
    def logger(self) -> logging.Logger:
        return self._logger

    # -------------------------
    # Internal send helpers
    # -------------------------
    def _send_msg(self, msg: AttributeBase) -> None:
        if isinstance(msg, AttributeBase):
            data = self._parser.pack(msg)
            if data is not None:
                comm = self._ctx.get_comm()
                if comm is None:
                    self._logger.warning(
                        "Failed to send %s: comm is None (driver not connected).",
                        type(msg).__name__,
                    )
                    raise RuntimeError(
                        "Robot is not connected (comm is None). "
                        "Call `connect()` before sending effector commands."
                    )
                try:
                    comm.send(data)
                except Exception as exc:
                    self._logger.error(
                        "Failed to send %s on channel '%s' (%s: %s).",
                        type(msg).__name__,
                        comm.get_channel(),
                        type(exc).__name__,
                        exc,
                        exc_info=True,
                    )
                    raise RuntimeError(
                        f"Failed to send {type(msg).__name__} on channel '{comm.get_channel()}': {exc}"
                    ) from exc
        else:
            raise TypeError("msg must be AttributeBase")

    # -------------------------
    # Public APIs (gripper)
    # -------------------------
    def is_ok(self):
        return self._effector_ctx.is_ok()

    def get_fps(self):
        return self._effector_ctx.get_fps()

    def get_hand_status(self) -> Optional[MessageAbstract[FeedbackHandStatus]]:
        """
        Get the hand status.

        Returns
        -------
        MessageAbstract[FeedbackHandStatus] | None
            The hand status. If the hand status is not available, return None.

        Message
        -------
        `left_or_right`: Left/right hand indicator
        - 01 left hand; 02 right hand

        `thumb_tip`: Thumb tip
        - 0 motor idle; 1 motor running; 2 motor stall

        `thumb_base`: Thumb base
        - 0 motor idle; 1 motor running; 2 motor stall

        `index_finger`: Fore finger
        - 0 motor idle; 1 motor running; 2 motor stall

        `middle_finger`: Middle finger
        - 0 motor idle; 1 motor running; 2 motor stall

        `ring_finger`: Ring finger
        - 0 motor idle; 1 motor running; 2 motor stall

        `pinky_finger`: Little finger
        - 0 motor idle; 1 motor running; 2 motor stall

        Examples
        --------
        >>> hs = end_effector.get_hand_status()
        >>> if hs is not None:
        >>>     print(hs.msg)
        >>>     print(hs.hz, hs.timestamp)
        """
        hs: Optional[MessageAbstract[FeedbackHandStatus]] = getattr(
            self._parser, "hand_status", None
        )
        if hs is not None:
            hs.hz = self._ctx.fps.get_fps(hs.msg_type)
            return hs
        return None

    def get_finger_pos(self) -> Optional[MessageAbstract[FeedbackFingerPos]]:
        """
        Get the finger position.

        Returns
        -------
        MessageAbstract[FeedbackFingerPos] | None
            The finger position. If the finger position is not
            available, return None.

        Message
        -------
        `thumb_tip`: Thumb tip position, range: [0, 100]

        `thumb_base`: Thumb base position, range: [0, 100]

        `index_finger`: Fore finger position, range: [0, 100]

        `middle_finger`: Middle finger position, range: [0, 100]

        `ring_finger`: Ring finger position, range: [0, 100]

        `pinky_finger`: Little finger position, range: [0, 100]

        Examples
        --------
        >>> fp = end_effector.get_finger_pos()
        >>> if fp is not None:
        >>>     print(fp.msg)
        >>>     print(fp.hz, fp.timestamp)
        """
        fp: Optional[MessageAbstract[FeedbackFingerPos]] = getattr(
            self._parser, "finger_pos", None
        )
        if fp is not None:
            fp.hz = self._ctx.fps.get_fps(fp.msg_type)
            return fp
        return None

    def get_finger_spd(self) -> Optional[MessageAbstract[FeedbackFingerSpd]]:
        """
        Get the finger speed.

        Returns
        -------
        MessageAbstract[FeedbackFingerSpd] | None
            The finger speed. If the finger speed is not available, return None.

        Message
        -------
        `thumb_tip`: Thumb tip speed, range: [-100, 100]

        `thumb_base`: Thumb base speed, range: [-100, 100]

        `index_finger`: Fore finger speed, range: [-100, 100]

        `middle_finger`: Middle finger speed, range: [-100, 100]

        `ring_finger`: Ring finger speed, range: [-100, 100]

        `pinky_finger`: Little finger speed, range: [-100, 100]

        Examples
        --------
        >>> fs = end_effector.get_finger_spd()
        >>> if fs is not None:
        >>>     print(fs.msg)
        >>>     print(fs.hz, fs.timestamp)
        """
        fs: Optional[MessageAbstract[FeedbackFingerSpd]] = getattr(
            self._parser, "finger_spd", None
        )
        if fs is not None:
            fs.hz = self._ctx.fps.get_fps(fs.msg_type)
            return fs
        return None

    def get_finger_current(
            self) -> Optional[MessageAbstract[FeedbackFingerCurrent]]:
        """
        Get the finger current.

        Returns
        -------
        MessageAbstract[FeedbackFingerCurrent] | None
            The finger current. If the finger current is not available, return None.

        Message
        -------
        `thumb_tip`: Thumb tip current, range: [-100, 100]

        `thumb_base`: Thumb base current, range: [-100, 100]

        `index_finger`: Fore finger current, range: [-100, 100]

        `middle_finger`: Middle finger scurrentpeed, range: [-100, 100]

        `ring_finger`: Ring finger current, range: [-100, 100]

        `pinky_finger`: Little finger current, range: [-100, 100]

        Examples
        --------
        >>> fc = end_effector.get_finger_current()
        >>> if fc is not None:
        >>>     print(fc.msg)
        >>>     print(fc.hz, fc.timestamp)
        """
        fc: Optional[MessageAbstract[FeedbackFingerCurrent]] = getattr(
            self._parser, "finger_current", None
        )
        if fc is not None:
            fc.hz = self._ctx.fps.get_fps(fc.msg_type)
            return fc
        return None

    def position_ctrl(
        self,
        thumb_tip: int = 0,
        thumb_base: int = 0,
        index_finger: int = 0,
        middle_finger: int = 0,
        ring_finger: int = 0,
        pinky_finger: int = 0,
    ) -> None:
        """
        Control the finger position.

        Parameters
        ----------
        `thumb_tip`: Thumb tip position, range: [0, 100]

        `thumb_base`: Thumb base position, range: [0, 100]

        `index_finger`: Fore finger position, range: [0, 100]

        `middle_finger`: Middle finger position, range: [0, 100]

        `ring_finger`: Ring finger position, range: [0, 100]

        `pinky_finger`: Little finger position, range: [0, 100]

        Examples
        --------
        >>> end_effector.position_ctrl()
        """
        self._send_msg(
            FingerPosCtrl(
                thumb_tip=thumb_tip,
                thumb_base=thumb_base,
                index_finger=index_finger,
                middle_finger=middle_finger,
                ring_finger=ring_finger,
                pinky_finger=pinky_finger
            )
        )

    def speed_ctrl(
        self,
        thumb_tip: int = 0,
        thumb_base: int = 0,
        index_finger: int = 0,
        middle_finger: int = 0,
        ring_finger: int = 0,
        pinky_finger: int = 0,
    ) -> None:
        """
        Control the finger position.

        Parameters
        ----------
        `thumb_tip`: Thumb tip speed, range: [-100, 100]

        `thumb_base`: Thumb base speed, range: [-100, 100]

        `index_finger`: Fore finger speed, range: [-100, 100]

        `middle_finger`: Middle finger speed, range: [-100, 100]

        `ring_finger`: Ring finger speed, range: [-100, 100]

        `pinky_finger`: Little finger speed, range: [-100, 100]

        Examples
        --------
        >>> end_effector.speed_ctrl()
        """
        self._send_msg(
            FingerSpdCtrl(
                thumb_tip=thumb_tip,
                thumb_base=thumb_base,
                index_finger=index_finger,
                middle_finger=middle_finger,
                ring_finger=ring_finger,
                pinky_finger=pinky_finger
            )
        )

    def current_ctrl(
        self,
        thumb_tip: int = 0,
        thumb_base: int = 0,
        index_finger: int = 0,
        middle_finger: int = 0,
        ring_finger: int = 0,
        pinky_finger: int = 0,
    ) -> None:
        """
        Control the finger current.

        Parameters
        ----------
        `thumb_tip`: Thumb tip current, range: [-100, 100]

        `thumb_base`: Thumb base current, range: [-100, 100]

        `index_finger`: Fore finger current, range: [-100, 100]

        `middle_finger`: Middle finger current, range: [-100, 100]

        `ring_finger`: Ring finger current, range: [-100, 100]

        `pinky_finger`: Little finger current, range: [-100, 100]

        Examples
        --------
        >>> end_effector.current_ctrl()
        """
        self._send_msg(
            FingerCurrentCtrl(
                thumb_tip=thumb_tip,
                thumb_base=thumb_base,
                index_finger=index_finger,
                middle_finger=middle_finger,
                ring_finger=ring_finger,
                pinky_finger=pinky_finger
            )
        )

    def position_time_ctrl(
        self,
        mode: Literal['pos', 'time'] = 'pos',
        thumb_tip: int = 0,
        thumb_base: int = 0,
        index_finger: int = 0,
        middle_finger: int = 0,
        ring_finger: int = 0,
        pinky_finger: int = 0,
    ) -> None:
        """
        Control the finger position or time.

        Parameters
        ----------
        `mode`: Control mode, 'pos' or 'time'
        - `pos`: Control the finger position, range: [0, 100]
        - `time`: Control the finger time, unit: 10ms, range: [0, 255]
        - The interval between position and time control messages
            must not exceed 50 milliseconds.

        `thumb_tip`: Thumb tip position or time

        `thumb_base`: Thumb base position or time

        `index_finger`: Fore finger position or time

        `middle_finger`: Middle finger position or time

        `ring_finger`: Ring finger position or time

        `pinky_finger`: Little finger position or time

        Examples
        --------
        The tip of the thumb moves to position 100 in 2 seconds.
        >>> end_effector.position_time_ctrl(mode="pos", thumb_tip=100)
        >>> end_effector.position_time_ctrl(mode="time", thumb_tip=200)
        """
        mode = {'pos': 0x12, 'time': 0x22}[mode]
        self._send_msg(
            FingerPosTimeCtrl(
                mode=mode,
                thumb_tip=thumb_tip,
                thumb_base=thumb_base,
                index_finger=index_finger,
                middle_finger=middle_finger,
                ring_finger=ring_finger,
                pinky_finger=pinky_finger
            )
        )
