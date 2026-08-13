from dataclasses import dataclass
from enum import Enum


class LongitudinalAction(Enum):
    THROTTLE_25 = "THROTTLE_25"
    THROTTLE_40 = "THROTTLE_40"
    THROTTLE_60 = "THROTTLE_60"
    THROTTLE_80 = "THROTTLE_80"
    THROTTLE_100 = "THROTTLE_100"
    LIFT = "LIFT"
    COAST = "COAST"
    BRAKE_20 = "BRAKE_20"
    BRAKE_40 = "BRAKE_40"
    BRAKE_60 = "BRAKE_60"
    BRAKE_80 = "BRAKE_80"
    BRAKE_100 = "BRAKE_100"


class GearAction(Enum):
    GEAR_UP = "GEAR_UP"
    GEAR_HOLD = "GEAR_HOLD"
    GEAR_DOWN = "GEAR_DOWN"


@dataclass
class AIAction:
    longitudinal: LongitudinalAction = LongitudinalAction.COAST
    gear: GearAction = GearAction.GEAR_HOLD
    reason: str = ""

