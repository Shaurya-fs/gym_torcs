from enum import Enum, auto


class DrivingState(Enum):
    from enum import Enum, auto


class DrivingState(Enum):
    START = auto()
    # Straight
    FULL_THROTTLE = auto()
    # Corner entry
    APPROACH_CORNER = auto()
    BRAKING = auto()
    TRAIL_BRAKE = auto()
    TURN_IN = auto()
    # Corner
    MID_CORNER = auto()
    # Exit
    THROTTLE_APPLICATION = auto()
    CORNER_EXIT = auto()
    # Emergency
    RECOVER = auto()
    # Misc
    PIT = auto()
    STOP = auto()


class DrivingStateMachine:

    def __init__(self):
        self.current_state = DrivingState.START
        self.previous_state = DrivingState.START
        self.time_in_state = 0

    def update(self, vehicle_state, planner):
        pass