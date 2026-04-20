from abc import ABC, abstractmethod
from typing import Any, Callable, Optional


class CommInterface(ABC):

    @abstractmethod
    def connect(self, **kwargs) -> Any:
        ...

    @abstractmethod
    def is_connected(self) -> bool:
        ...

    @abstractmethod
    def close(self) -> Any:
        ...

    @abstractmethod
    def get_status(self) -> Any:
        ...

    @abstractmethod
    def get_config(self) -> dict:
        ...

    @abstractmethod
    def get_type(self) -> str:
        ...

    @abstractmethod
    def get_channel(self) -> str:
        ...

    @abstractmethod
    def send(self, data: Any, **kwargs) -> Any:
        ...

    @abstractmethod
    def recv(self, **kwargs) -> Any:
        ...

    @abstractmethod
    def set_callback(self, cb: Callable[[Any], None]) -> None:
        ...

    @abstractmethod
    def get_callback(self) -> Optional[Callable[[Any], None]]:
        ...

    @abstractmethod
    def clear_callback(self) -> None:
        ...

    @abstractmethod
    def has_callback(self) -> bool:
        ...
