"""
Differential-drive kinematics shared by the base nodes.

Sign conventions (verified on the Companio chassis):
  * The right motor is mounted mirrored, so its wheel target is negated.
  * The left encoder counts backwards relative to forward travel, so its
    delta is negated when integrating odometry.
"""

from math import cos, sin


class DifferentialKinematics:
    """Twist (v, w) -> (left, right) wheel velocity targets."""

    def __init__(self, base_width, invert_right=True, invert_left=False):
        self.base_width = float(base_width)
        self.right_sign = -1.0 if invert_right else 1.0
        self.left_sign = -1.0 if invert_left else 1.0

    def wheel_targets(self, linear_x, angular_z):
        half = self.base_width / 2.0
        left = self.left_sign * (linear_x - angular_z * half)
        right = self.right_sign * (linear_x + angular_z * half)
        return left, right


class WheelOdometry:
    """Integrates wrapped encoder counts into a planar pose (x, y, theta)."""

    def __init__(self, ticks_per_meter, base_width,
                 encoder_min=0, encoder_max=4294967295,
                 invert_left=True, invert_right=False):
        self.ticks_per_meter = float(ticks_per_meter)
        self.base_width = float(base_width)
        self.encoder_min = encoder_min
        self.encoder_max = encoder_max
        span = encoder_max - encoder_min
        self.low_wrap = span * 0.3 + encoder_min
        self.high_wrap = span * 0.7 + encoder_min
        self.left_sign = -1.0 if invert_left else 1.0
        self.right_sign = -1.0 if invert_right else 1.0

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v = 0.0
        self.w = 0.0

        self._left = 0.0
        self._right = 0.0
        self._lmult = 0
        self._rmult = 0
        self._prev_l = 0
        self._prev_r = 0
        self._last_left = None
        self._last_right = None

    # -- encoder ingestion (handles 32-bit counter wrap) ----------------------
    def _unwrap(self, enc, prev, mult):
        if enc < self.low_wrap and prev > self.high_wrap:
            mult += 1
        if enc > self.high_wrap and prev < self.low_wrap:
            mult -= 1
        value = 1.0 * (enc + mult * (self.encoder_max - self.encoder_min))
        return value, mult

    def update_left(self, enc):
        self._left, self._lmult = self._unwrap(enc, self._prev_l, self._lmult)
        self._prev_l = enc

    def update_right(self, enc):
        self._right, self._rmult = self._unwrap(enc, self._prev_r, self._rmult)
        self._prev_r = enc

    # -- integration -----------------------------------------------------------
    def step(self, dt):
        """Integrate motion since the previous step. dt in seconds (> 0)."""
        if self._last_left is None and self._last_right is None:
            d_left = 0.0
            d_right = 0.0
        else:
            d_left = self.left_sign * (self._left - self._last_left) / self.ticks_per_meter
            d_right = self.right_sign * (self._right - self._last_right) / self.ticks_per_meter
        self._last_left = self._left
        self._last_right = self._right

        d = (d_left + d_right) / 2.0
        dth = (d_right - d_left) / self.base_width
        self.v = d / dt if dt > 0 else 0.0
        self.w = dth / dt if dt > 0 else 0.0

        if d != 0.0:
            dx = cos(dth) * d
            dy = -sin(dth) * d
            self.x += cos(self.theta) * dx - sin(self.theta) * dy
            self.y += sin(self.theta) * dx + cos(self.theta) * dy
        if dth != 0.0:
            self.theta += dth
        return self.x, self.y, self.theta
