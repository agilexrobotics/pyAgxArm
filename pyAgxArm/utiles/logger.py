import logging
import threading
import time
from typing import Any, Callable, Optional, Tuple


LogWriter = Callable[[str], None]


def _throttle_key(record: logging.LogRecord) -> Tuple[Any, ...]:
    exc_t = record.exc_info[0] if record.exc_info else None
    return (record.levelno, record.name, record.msg, record.args, exc_t)


class _ThrottledHandler(logging.Handler):
    """Wraps a handler: same ``_throttle_key`` may emit at most once per ``min_interval``."""

    def __init__(self, target: logging.Handler, min_interval: float) -> None:
        super().__init__(level=target.level)
        self.target = target
        self._min_interval = float(min_interval)
        self._last: dict = {}
        self._lock = threading.Lock()
        if target.formatter is not None:
            self.formatter = target.formatter

    def setLevel(self, level: int) -> None:
        super().setLevel(level)
        self.target.setLevel(level)

    def setFormatter(self, fmt: logging.Formatter) -> None:
        super().setFormatter(fmt)
        self.target.setFormatter(fmt)

    def emit(self, record: logging.LogRecord) -> None:
        if self._min_interval <= 0:
            self.target.emit(record)
            return
        key = _throttle_key(record)
        with self._lock:
            now = time.monotonic()
            prev = self._last.get(key)
            if prev is not None and (now - prev) < self._min_interval:
                return
            self._last[key] = now
        try:
            self.target.emit(record)
        except Exception:
            self.handleError(record)

    def flush(self) -> None:
        self.target.flush()

    def close(self) -> None:
        self.target.close()
        super().close()


class _CallbackLogBridgeHandler(logging.Handler):
    """Forward formatted log lines to optional per-level callables.

    If a level has no dedicated writer, a coarser writer is used (e.g. ERROR
    falls back to WARNING when ``error`` is omitted).
    """

    def __init__(
        self,
        *,
        debug: Optional[LogWriter] = None,
        info: Optional[LogWriter] = None,
        warning: Optional[LogWriter] = None,
        error: Optional[LogWriter] = None,
        critical: Optional[LogWriter] = None,
        emit_min_interval: float = 1.0,
    ):
        super().__init__(level=logging.DEBUG)
        self._debug = debug
        self._info = info
        self._warning = warning
        self._error = error
        self._critical = critical
        self._emit_min_interval = float(emit_min_interval)
        self._throttle_last: dict = {}
        self._throttle_lock = threading.Lock()

    def _pick_writer(self, levelno: int) -> Optional[LogWriter]:
        if levelno >= logging.CRITICAL:
            return self._critical or self._error or self._warning or self._info or self._debug
        if levelno >= logging.ERROR:
            return self._error or self._warning or self._info or self._debug
        if levelno >= logging.WARNING:
            return self._warning or self._info or self._debug
        if levelno >= logging.INFO:
            return self._info or self._debug
        return self._debug

    def emit(self, record: logging.LogRecord) -> None:
        writer = self._pick_writer(record.levelno)
        if writer is None:
            return
        if self._emit_min_interval > 0:
            key = _throttle_key(record)
            with self._throttle_lock:
                now = time.monotonic()
                prev = self._throttle_last.get(key)
                if prev is not None and (now - prev) < self._emit_min_interval:
                    return
                self._throttle_last[key] = now
        try:
            writer(self.format(record))
        except Exception:
            self.handleError(record)


class Logger:
    """Configurable logging facade with per-instance handler boundaries.

    Notes
    -----
    - ``propagate`` defaults to ``False`` on the underlying logger so sibling
      instances do not share handlers unless you enable propagation explicitly.
    """

    class Kind:
        BRIDGE = "bridge"
        CONSOLE = "console"
        MANAGED_KINDS = frozenset({BRIDGE, CONSOLE})

    class Formats:
        FULL = logging.Formatter("[%(levelname)s] [%(created).9f] [%(name)s]: %(message)s")
        PLAIN_TEXT = logging.Formatter("%(message)s")

    class Level:
        NOTSET = logging.NOTSET
        DEBUG = logging.DEBUG
        INFO = logging.INFO
        WARNING = logging.WARNING
        WARN = logging.WARN
        ERROR = logging.ERROR
        CRITICAL = logging.CRITICAL

    def __init__(
        self,
        logger_name: str,
    ):
        """Create a facade for the given ``logging`` logger name.

        Parameters
        ----------
        `logger_name`: str
        - Distinct name passed to ``logging.getLogger``.
        """
        self._logger_name = logger_name
        self._logger = logging.getLogger(logger_name)
        self._formatter: logging.Formatter = self.Formats.FULL

        self._logger.propagate = False
        self._ensure_library_default()

    @property
    def logger(self) -> logging.Logger:
        """The underlying standard-library ``logging.Logger`` instance."""
        return self._logger

    @property
    def logger_name(self) -> str:
        """The name passed to ``logging.getLogger`` for this facade."""
        return self._logger_name

    def _remove_handlers(self, kind: Optional[str] = None, close: bool = False) -> None:
        for handler in list(self._logger.handlers):
            if kind is not None and getattr(handler, "_handler_kind", None) != kind:
                continue
            self._logger.removeHandler(handler)
            if close:
                try:
                    handler.close()
                except Exception:
                    pass

    def _ensure_library_default(self) -> None:
        if not self._logger.handlers:
            self._logger.addHandler(logging.NullHandler())

    def _drop_library_defaults(self) -> None:
        for handler in list(self._logger.handlers):
            if isinstance(handler, logging.NullHandler) and not getattr(
                handler, "_managed_handler", False
            ):
                self._logger.removeHandler(handler)

    def _set_logger_level_for(self, level: int) -> None:
        current = self._logger.level
        if current in (logging.NOTSET, 0) or current > level:
            self._logger.setLevel(level)

    def _bind_managed_handler(
        self,
        handler: logging.Handler,
        *,
        kind: str,
        level: int,
        formatter: Optional[logging.Formatter] = None,
    ) -> logging.Handler:
        setattr(handler, "_managed_handler", True)
        setattr(handler, "_handler_kind", kind)
        handler.setLevel(level)
        handler.setFormatter(formatter or self._formatter)
        return handler

    def _replace_kind(self, kind: str) -> None:
        if kind not in self.Kind.MANAGED_KINDS:
            raise ValueError(f"unknown handler kind: {kind}")
        self._remove_handlers(kind=kind, close=True)

    def configure(
        self,
        *,
        level: Optional[int] = None,
        propagate: Optional[bool] = None,
        formatter: Optional[logging.Formatter] = None,
        replace_handlers: bool = False,
    ) -> None:
        """Adjust logger level, propagation, formatter, and optionally clear handlers.

        Parameters
        ----------
        `level`: int | None
        - ``logging`` level for this logger (e.g. ``logging.INFO``), or
          ``None`` to leave unchanged.

        `propagate`: bool | None
        - Whether records propagate to ancestor loggers, or ``None`` to leave
          unchanged.

        `formatter`: logging.Formatter | None
        - Formatter applied to managed handlers, or ``None`` to leave
          unchanged.

        `replace_handlers`: bool
        - If ``True``, remove all handlers on this logger before applying other
          options; ensures a clean slate.
        """
        if replace_handlers:
            self._remove_handlers(close=True)

        if level is not None:
            self._logger.setLevel(level)
        if propagate is not None:
            self._logger.propagate = propagate
        if formatter is not None:
            self._formatter = formatter
            for handler in self._logger.handlers:
                if getattr(handler, "_managed_handler", False):
                    handler.setFormatter(self._formatter)

        self._ensure_library_default()

    def get_child(self, suffix: str) -> logging.Logger:
        """Return a child ``logging.Logger`` that inherits this logger's configuration.

        Parameters
        ----------
        `suffix`: str
        - Non-empty suffix for ``Logger.getChild``.

        Returns
        -------
        logging.Logger
        - Child logger with ``propagate`` set to ``True`` so records reach this
          logger's handlers.

        Raises
        ------
        ValueError
        - If ``suffix`` is empty or whitespace-only.
        """
        suffix_text = str(suffix).strip()
        if not suffix_text:
            raise ValueError("suffix must be a non-empty string")
        child = self._logger.getChild(suffix_text)
        child.propagate = True
        return child

    def bridge_enable(
        self,
        *,
        debug: Optional[LogWriter] = None,
        info: Optional[LogWriter] = None,
        warning: Optional[LogWriter] = None,
        error: Optional[LogWriter] = None,
        critical: Optional[LogWriter] = None,
        level: int = Level.INFO,
        emit_min_interval: float = 1.0,
        replace_handlers: bool = False,
        propagate: Optional[bool] = None,
        formatter: Optional[logging.Formatter] = None,
    ) -> _CallbackLogBridgeHandler:
        """Install a single bridge handler that forwards lines to callables.

        Parameters
        ----------
        `debug`, `info`, `warning`, `error`, `critical`: callable[[str], None] | None
        - Optional sinks receiving one formatted line per log record at that
          severity (after handler level filtering).

        `level`: int
        - Minimum level for the bridge handler (default ``Logger.Level.INFO``).

        `emit_min_interval`: float
        - Minimum seconds between emitting two records with the same throttle
          key for **this** bridge handler only. Default ``1.0``. ``<= 0`` disables
          throttling.

        `replace_handlers`: bool
        - If ``True``, remove all handlers first; otherwise only replace the
          existing bridge handler.

        `propagate`: bool | None
        - Sets ``self.logger.propagate`` when not ``None``.

        `formatter`: logging.Formatter | None
        - Formatter for this handler. Defaults to pure text
          (``Logger.Formats.PLAIN_TEXT``), useful when bridging to ROS where
          timestamp/level may already be injected by the target runtime.

        Returns
        -------
        _CallbackLogBridgeHandler
        - The installed bridge handler instance.
        """
        if replace_handlers:
            self._remove_handlers(close=True)
        else:
            self._replace_kind(self.Kind.BRIDGE)

        if propagate is not None:
            self._logger.propagate = propagate
        self._set_logger_level_for(level)
        self._drop_library_defaults()

        handler = self._bind_managed_handler(
            _CallbackLogBridgeHandler(
                debug=debug,
                info=info,
                warning=warning,
                error=error,
                critical=critical,
                emit_min_interval=emit_min_interval,
            ),
            kind=self.Kind.BRIDGE,
            level=level,
            formatter=formatter or self.Formats.PLAIN_TEXT,
        )
        self._logger.addHandler(handler)
        return handler

    def bridge_disable(self) -> None:
        """Remove and close the bridge handler; restore default NullHandler if needed."""
        self._remove_handlers(kind=self.Kind.BRIDGE, close=True)
        self._ensure_library_default()

    def console_enable(
        self,
        *,
        level: int = Level.INFO,
        emit_min_interval: float = 1.0,
        replace_handlers: bool = False,
        propagate: Optional[bool] = None,
        formatter: Optional[logging.Formatter] = None,
        handler: Optional[logging.Handler] = None,
        stream=None,
    ) -> logging.Handler:
        """Install a stream-oriented handler (default: ``StreamHandler`` to stderr).

        Parameters
        ----------
        `level`: int
        - Minimum level for the console handler (default ``Logger.Level.INFO``).

        `emit_min_interval`: float
        - Minimum seconds between emitting two records with the same throttle
          key for **this** console handler only. Default ``1.0``. ``<= 0`` disables
          throttling (underlying handler receives every record).

        `replace_handlers`: bool
        - If ``True``, remove all handlers first; otherwise only replace the
          existing console handler.

        `propagate`: bool | None
        - Sets ``self.logger.propagate`` when not ``None``.

        `formatter`: logging.Formatter | None
        - Formatter for this handler.

        `handler`: logging.Handler | None
        - Custom handler to wrap; if ``None``, a ``StreamHandler`` is created.

        `stream`
        - Passed to ``StreamHandler`` when ``handler`` is ``None``.

        Returns
        -------
        logging.Handler
        - The installed (managed) console handler (may be a throttle wrapper).
        """
        if replace_handlers:
            self._remove_handlers(close=True)
        else:
            self._replace_kind(self.Kind.CONSOLE)

        if propagate is not None:
            self._logger.propagate = propagate
        self._set_logger_level_for(level)
        self._drop_library_defaults()

        inner = handler if handler is not None else logging.StreamHandler(stream=stream)
        if emit_min_interval > 0:
            inner = _ThrottledHandler(inner, emit_min_interval)
        out_handler = self._bind_managed_handler(
            inner,
            kind=self.Kind.CONSOLE,
            level=level,
            formatter=formatter,
        )
        self._logger.addHandler(out_handler)
        return out_handler

    def console_disable(self) -> None:
        """Remove and close the console handler; restore default NullHandler if needed."""
        self._remove_handlers(kind=self.Kind.CONSOLE, close=True)
        self._ensure_library_default()

    def shutdown(self) -> None:
        """Close and remove all managed handlers for this logger instance.

        This is intended for lifecycle cleanup to avoid leaking file 
        descriptors or callback handlers when instances are replaced.
        A default ``NullHandler`` is restored afterwards.
        """
        self._remove_handlers(kind=self.Kind.BRIDGE, close=True)
        self._remove_handlers(kind=self.Kind.CONSOLE, close=True)
        self._ensure_library_default()

    def debug(self, msg, *args, **kwargs) -> None:
        """Log at ``DEBUG``."""
        self._logger.debug(msg, *args, **kwargs)

    def info(self, msg, *args, **kwargs) -> None:
        """Log at ``INFO``."""
        self._logger.info(msg, *args, **kwargs)

    def warning(self, msg, *args, **kwargs) -> None:
        """Log at ``WARNING``."""
        self._logger.warning(msg, *args, **kwargs)

    def error(self, msg, *args, **kwargs) -> None:
        """Log at ``ERROR``."""
        self._logger.error(msg, *args, **kwargs)

    def critical(self, msg, *args, **kwargs) -> None:
        """Log at ``CRITICAL``."""
        self._logger.critical(msg, *args, **kwargs)

    def exception(self, msg, *args, **kwargs) -> None:
        """Log at ``ERROR`` with exception info."""
        self._logger.exception(msg, *args, **kwargs)
