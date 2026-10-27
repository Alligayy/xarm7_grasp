"""Bounded position-based force regulation. Force units are calibrated newtons."""
from dataclasses import dataclass
import math


@dataclass
class ForceRule:
    target: float = 1.0       # per finger, not sum of both fingers
    band: float = 0.15
    contact: float = 0.2
    maximum: float = 2.0
    taxel_maximum: float = 0.6
    step: float = 2.0         # xArm native position units; smaller closes
    open_position: float = 850.0
    close_position: float = 100.0

    def __post_init__(self):
        if not all(math.isfinite(v) for v in vars(self).values()):
            raise ValueError('Force parameters must be finite')
        if not (0 < self.contact < self.target - self.band < self.target + self.band < self.maximum):
            raise ValueError('Require 0 < contact < target-band < target+band < maximum')
        if not (self.taxel_maximum > 0 and self.step > 0 and
                0 <= self.close_position < self.open_position <= 850):
            raise ValueError('Invalid taxel limit, position bounds or step')

    def decide(self, forces, peak, position):
        if not all(math.isfinite(v) for v in (*forces, peak, position)):
            raise ValueError('Nonfinite force/position')
        if max(forces) >= self.maximum or peak >= self.taxel_maximum:
            raise ValueError('Force safety limit exceeded')
        # Use maximum for compression protection, minimum for bilateral contact.
        if max(forces) > self.target + self.band:
            return min(self.open_position, position + self.step), False
        stable = min(forces) >= self.target - self.band
        if stable:
            return position, True
        # One side already at target: do not crush it to seek the other side.
        if max(forces) >= self.target - self.band:
            return position, False
        if position <= self.close_position:
            raise ValueError('Travel limit reached without bilateral grasp')
        return max(self.close_position, position - self.step), False
