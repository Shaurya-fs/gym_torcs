from enum import Enum, auto


class DrivingState(Enum):
    START = auto()
    FULL_THROTTLE = auto()
    APPROACH_CORNER = auto()
    BRAKING = auto()
    TRAIL_BRAKE = auto()
    TURN_IN = auto()
    MID_CORNER = auto()
    CORNER_EXIT = auto()
    RECOVER = auto()
    PIT = auto()


class DrivingStateMachine:

    def __init__(self):
        self.current_state = DrivingState.START
        self.previous_state = DrivingState.START
        self.time_in_state = 0

    def update(self, vehicle_state, planner):
        pass