"""Live single-hand and synchronized bimanual Quest acquisition."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any

from teleoperation.inputs.quest3.common import _missing_sample, decode_quest3_sample
from teleoperation.types import BimanualSensorHandSample, SensorHandSample


class _Quest3Input:
    """Share USB acquisition and freshness checks across hand selections."""

    def __init__(self, *, port=8765, adb=None, serial=None, max_age_s=0.15):
        if not 1 <= int(port) <= 65535:
            raise ValueError("port must be in [1, 65535].")
        if not math.isfinite(max_age_s) or max_age_s <= 0:
            raise ValueError("max_age_s must be positive and finite.")
        self.port = int(port)
        self.adb = None if adb is None else Path(adb).expanduser()
        self.serial = serial
        self.max_age_s = float(max_age_s)
        self._session: Any | None = None

    @property
    def device_serial(self) -> str | None:
        return None if self._session is None else self._session.serial

    @property
    def stats(self):
        return None if self._session is None else self._session.stats

    def open(self) -> None:
        if self._session is None:
            # Keep aiohttp and USB optional for offline/core imports.
            from teleoperation.inputs.quest3.session import Quest3UsbSession

            self._session = Quest3UsbSession(
                port=self.port, adb=self.adb, serial=self.serial,
            ).start()

    def _read_frame(self):
        if self._session is None:
            raise RuntimeError(f"{type(self).__name__} must be opened before read().")
        frame = self._session.poll()
        fresh = frame is not None and self._session.age_s() <= self.max_age_s
        return frame, fresh

    def reset(self) -> None:
        """A live input has no finite cursor to reset."""

    def close(self) -> None:
        session, self._session = self._session, None
        if session is not None:
            session.close()


class Quest3OnlineInput(_Quest3Input):
    """Expose the newest selected hand, or an explicit missing sample."""

    def __init__(self, *, hand_side="right", port=8765, adb=None, serial=None, max_age_s=0.15):
        if hand_side not in {"left", "right"}:
            raise ValueError("hand_side must be 'left' or 'right'.")
        super().__init__(port=port, adb=adb, serial=serial, max_age_s=max_age_s)
        self.hand_side = hand_side

    def read(self) -> SensorHandSample:
        frame, fresh = self._read_frame()
        return decode_quest3_sample(frame, hand_side=self.hand_side) if fresh else _missing_sample(frame)


class Quest3BimanualOnlineInput(_Quest3Input):
    """Decode both hands from one poll, preserving the shared frame sequence."""

    def read(self) -> BimanualSensorHandSample:
        frame, fresh = self._read_frame()
        if not fresh:
            missing = _missing_sample(frame)
            return BimanualSensorHandSample(left=missing, right=missing)
        return BimanualSensorHandSample(
            left=decode_quest3_sample(frame, hand_side="left"),
            right=decode_quest3_sample(frame, hand_side="right"),
        )


__all__ = ["Quest3OnlineInput", "Quest3BimanualOnlineInput"]
