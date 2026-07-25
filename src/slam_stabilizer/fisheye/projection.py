from __future__ import annotations

from dataclasses import dataclass
from math import pi


THETA_TRANS_DEFAULT = 80.0 * pi / 180.0
THETA_MAX_DEFAULT = 110.0 * pi / 180.0


@dataclass(frozen=True)
class KannalaBrandt:
    """CPU reference for the fisheye polynomial used by Silver Bullet."""

    fx: float
    fy: float
    cx: float
    cy: float
    k: tuple[float, float, float, float, float] = (0.0, 0.0, 0.0, 0.0, 0.0)
    theta_trans: float = THETA_TRANS_DEFAULT
    theta_max: float = THETA_MAX_DEFAULT
    r_max: float | None = None

    def radius(self, theta: float) -> float:
        if theta <= self.theta_trans or self.r_max is None:
            return self._kb_forward(theta)
        return self._cubic_extension(theta)

    def _kb_forward(self, theta: float) -> float:
        t2 = theta * theta
        inner = 1.0
        power = t2
        for coeff in self.k:
            inner += coeff * power
            power *= t2
        return self.fx * theta * inner

    def _kb_derivative(self, theta: float) -> float:
        t2 = theta * theta
        total = 1.0
        power = t2
        for i, coeff in enumerate(self.k):
            total += (2 * i + 3) * coeff * power
            power *= t2
        return self.fx * total

    def _cubic_extension(self, theta: float) -> float:
        assert self.r_max is not None
        span = self.theta_max - self.theta_trans
        u = max(0.0, min(1.0, (theta - self.theta_trans) / span))
        r_trans = self._kb_forward(self.theta_trans)
        d_trans = self._kb_derivative(self.theta_trans)
        h00 = 2.0 * u**3 - 3.0 * u**2 + 1.0
        h10 = u**3 - 2.0 * u**2 + u
        h01 = -2.0 * u**3 + 3.0 * u**2
        return h00 * r_trans + h10 * span * d_trans + h01 * self.r_max

