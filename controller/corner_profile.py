from enum import Enum
from dataclasses import dataclass

class CornerType(Enum):
    STRAIGHT="STRAIGHT"
    GENTLE_LEFT="GENTLE_LEFT"
    GENTLE_RIGHT="GENTLE_RIGHT"
    MEDIUM_LEFT="MEDIUM_LEFT"
    MEDIUM_RIGHT="MEDIUM_RIGHT"
    HAIRPIN_LEFT="HAIRPIN_LEFT"
    HAIRPIN_RIGHT="HAIRPIN_RIGHT"
    CHICANE="CHICANE"
    EXIT="EXIT"

@dataclass
class CornerProfile:
    corner_type: CornerType = CornerType.STRAIGHT
    turn_angle:float = 0.0
    severity:float=0.0
    direction: str = "STRAIGHT"
    
