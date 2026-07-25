from enum import Enum
from dataclasses import dataclass

class CornerType(Enum):
    STRAIGHT="STRAIGHT"
    GENTLE="GENTLE"
    MEDIUM="MEDIUM"
    SHARP="SHARP"
    HAIRPIN="HAIRPIN"

@dataclass
class CornerProfile:
    corner_type: CornerType = CornerType.STRAIGHT
    turn_angle:float = 0.0
    severity:float=0.0
    