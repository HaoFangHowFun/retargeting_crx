"""Live Quest 3 acquisition through the integrated USB/WebXR session."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from teleoperation.inputs.quest3.common import _missing_sample, decode_quest3_sample
from teleoperation.types import BimanualSensorHandSample, SensorHandSample


class Quest3OnlineInput:
    """Own one Quest USB session and expose latest-only hand samples."""

    def __init__(
        self,
        *,
        hand_side: str = "right",
        port: int = 8765,
        adb: str | Path | None = None,
        serial: str | None = None,
        max_age_s: float = 0.15,
    ) -> None:
        if hand_side not in {"left", "right"}:
            raise ValueError("hand_side must be 'left' or 'right'.")
        if not 1 <= int(port) <= 65535:
            raise ValueError("port must be in [1, 65535].")
        if max_age_s <= 0.0:
            raise ValueError("max_age_s must be positive.")
        self.hand_side = hand_side
        self.port = int(port)
        self.adb = None if adb is None else Path(adb).expanduser()
        self.serial = serial
        self.max_age_s = float(max_age_s)
        self._session: Any | None = None

    @property
    def device_serial(self) -> str | None:
        """Return the selected ADB serial after the session starts."""
        return None if self._session is None else self._session.serial

    def open(self) -> None:
        """Start the integrated receiver, ADB bridge, and Quest Browser."""
        if self._session is not None:
            return
        # Keep aiohttp and the USB runtime optional for offline/core imports.
        from teleoperation.inputs.quest3.session import Quest3UsbSession

        self._session = Quest3UsbSession(
            port=self.port,
            adb=self.adb,
            serial=self.serial,
        ).start()

    def read(self) -> SensorHandSample:
        """Return the newest selected hand or an explicit missing sample."""
        if self._session is None:
            raise RuntimeError("Quest3OnlineInput must be opened before read().")
        frame = self._session.poll()
        if frame is None:
            return _missing_sample(None)
        if self._session.age_s() > self.max_age_s:
            return _missing_sample(frame)
        return decode_quest3_sample(frame, hand_side=self.hand_side)

    def reset(self) -> None:
        """Keep the live session open; no finite source cursor is retained."""

    def close(self) -> None:
        """Stop tracking and release all receiver and ADB resources."""
        session = self._session
        self._session = None
        if session is not None:
            session.close()


class Quest3BimanualOnlineInput:
    """Acquire both Quest hands from one synchronized USB/WebXR session."""

    def __init__(
        self,
        *,
        port: int = 8765,
        adb: str | Path | None = None,
        serial: str | None = None,
        max_age_s: float = 0.15,
    ) -> None:
        if not 1 <= int(port) <= 65535:
            raise ValueError("port must be in [1, 65535].")
        if max_age_s <= 0.0:
            raise ValueError("max_age_s must be positive.")
        self.port = int(port)
        self.adb = None if adb is None else Path(adb).expanduser()
        self.serial = serial
        self.max_age_s = float(max_age_s)
        self._session: Any | None = None

    @property
    def device_serial(self) -> str | None:
        return None if self._session is None else self._session.serial

    def open(self) -> None:
        if self._session is not None:
            return
        from teleoperation.inputs.quest3.session import Quest3UsbSession

        self._session = Quest3UsbSession(port=self.port, adb=self.adb, serial=self.serial).start()

    def read(self) -> BimanualSensorHandSample:
        if self._session is None:
            raise RuntimeError("Quest3BimanualOnlineInput must be opened before read().")
        frame = self._session.poll()
        if frame is None or self._session.age_s() > self.max_age_s:
            missing = _missing_sample(frame)
            return BimanualSensorHandSample(left=missing, right=missing)
        return BimanualSensorHandSample(
            left=decode_quest3_sample(frame, hand_side="left"),
            right=decode_quest3_sample(frame, hand_side="right"),
        )

    def reset(self) -> None:
        return None

    def close(self) -> None:
        session = self._session
        self._session = None
        if session is not None:
            session.close()


__all__ = ["Quest3OnlineInput", "Quest3BimanualOnlineInput"]
