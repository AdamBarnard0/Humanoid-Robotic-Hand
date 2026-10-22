"""One Euro Filter (Casiez, Roussel & Vogel, CHI 2012).

The brief specifies a One Euro filter (not a moving average) for smoothing
MediaPipe landmark / retargeted-angle noise, since it adapts: heavy
smoothing when the signal is nearly still (kills jitter), light smoothing
when it's moving fast (kills lag).

Reference: http://cristal.univ-lille.fr/~casiez/1euro/
"""

from __future__ import annotations

import math
from typing import Optional


class LowPassFilter:
    def __init__(self, alpha: float):
        self._alpha = alpha
        self._y: Optional[float] = None
        self._s: Optional[float] = None

    def set_alpha(self, alpha: float) -> None:
        self._alpha = alpha

    def filter(self, value: float) -> float:
        if self._y is None:
            self._s = value
        else:
            self._s = self._alpha * value + (1.0 - self._alpha) * self._s
        self._y = value
        return self._s

    def last_value(self) -> Optional[float]:
        return self._y

    def has_last_raw_value(self) -> bool:
        return self._y is not None


class OneEuroFilter:
    """Filters a single scalar signal sampled at irregular timestamps."""

    def __init__(
        self,
        mincutoff: float = 1.0,
        beta: float = 0.0,
        dcutoff: float = 1.0,
    ):
        self.mincutoff = mincutoff
        self.beta = beta
        self.dcutoff = dcutoff

        self._x_filter = LowPassFilter(1.0)
        self._dx_filter = LowPassFilter(1.0)
        self._last_time: Optional[float] = None

    @staticmethod
    def _alpha(cutoff: float, dt: float) -> float:
        tau = 1.0 / (2 * math.pi * cutoff)
        return 1.0 / (1.0 + tau / dt)

    def filter(self, value: float, timestamp: float) -> float:
        """timestamp is seconds (any monotonic clock); must increase."""
        if self._last_time is None:
            dt = 1.0 / 30.0  # assume ~30 Hz for the first sample
        else:
            dt = max(timestamp - self._last_time, 1e-6)
        self._last_time = timestamp

        if not self._x_filter.has_last_raw_value():
            dx = 0.0
        else:
            dx = (value - self._x_filter.last_value()) / dt

        self._dx_filter.set_alpha(self._alpha(self.dcutoff, dt))
        edx = self._dx_filter.filter(dx)

        cutoff = self.mincutoff + self.beta * abs(edx)
        self._x_filter.set_alpha(self._alpha(cutoff, dt))
        return self._x_filter.filter(value)


class VectorOneEuroFilter:
    """N independent OneEuroFilters, one per element of a fixed-length vector."""

    def __init__(
        self,
        size: int,
        mincutoff: float = 1.0,
        beta: float = 0.0,
        dcutoff: float = 1.0,
    ):
        self._filters = [
            OneEuroFilter(mincutoff=mincutoff, beta=beta, dcutoff=dcutoff)
            for _ in range(size)
        ]

    def filter(self, values, timestamp: float) -> list[float]:
        return [f.filter(v, timestamp) for f, v in zip(self._filters, values)]
