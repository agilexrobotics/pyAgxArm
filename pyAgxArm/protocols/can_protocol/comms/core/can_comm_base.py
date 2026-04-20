from typing import Any, Optional

from can.message import Message

from .comm_abstract import CommAbstract


class CanCommBase(CommAbstract):
    def __init__(self):
        super().__init__()
        self._config = {}
        self._type = ""
        self._channel = ""
        self._is_connected = False

    def connect(self, **kwargs) -> None:
        raise NotImplementedError

    def is_connected(self) -> bool:
        return self._is_connected

    def close(self) -> Any:
        raise NotImplementedError

    def get_status(self) -> Any:
        raise NotImplementedError

    def get_config(self) -> dict:
        return self._config.copy()

    def get_type(self) -> str:
        return self._type

    def get_channel(self) -> str:
        return self._channel

    def send(self, message: Message, **kwargs) -> None:
        raise NotImplementedError

    def recv(self, **kwargs) -> Optional[Message]:
        raise NotImplementedError
