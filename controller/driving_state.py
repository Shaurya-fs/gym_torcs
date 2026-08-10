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
        self.state_history = [self.current_state]

        self.valid_transitions = {
            DrivingState.START: {
                DrivingState.FULL_THROTTLE,
                DrivingState.STOP,
            },
            DrivingState.FULL_THROTTLE: {
                DrivingState.APPROACH_CORNER,
                DrivingState.RECOVER,
                DrivingState.PIT,
                DrivingState.STOP,
            },
            DrivingState.APPROACH_CORNER: {
                DrivingState.BRAKING,
                DrivingState.FULL_THROTTLE,
                DrivingState.RECOVER,
            },
            DrivingState.BRAKING: {
                DrivingState.TRAIL_BRAKE,
                DrivingState.TURN_IN,
                DrivingState.RECOVER,
            },
            DrivingState.TRAIL_BRAKE: {
                DrivingState.TURN_IN,
                DrivingState.MID_CORNER,
                DrivingState.RECOVER,
            },
            DrivingState.TURN_IN: {
                DrivingState.MID_CORNER,
                DrivingState.RECOVER,
            },
            DrivingState.MID_CORNER: {
                DrivingState.THROTTLE_APPLICATION,
                DrivingState.CORNER_EXIT,
                DrivingState.RECOVER,
            },
            DrivingState.THROTTLE_APPLICATION: {
                DrivingState.CORNER_EXIT,
                DrivingState.RECOVER,
            },
            DrivingState.CORNER_EXIT: {
                DrivingState.FULL_THROTTLE,
                DrivingState.APPROACH_CORNER,
                DrivingState.RECOVER,
            },
            DrivingState.RECOVER: {
                DrivingState.FULL_THROTTLE,
                DrivingState.APPROACH_CORNER,
                DrivingState.STOP,
            },
            DrivingState.PIT: {
                DrivingState.FULL_THROTTLE,
                DrivingState.STOP,
            },
            DrivingState.STOP: set(),
        }

    def update(self, vehicle_state, planner):
        """Evaluate planner decisions and advance the driving state."""
        self.time_in_state += 1

        if planner is None:
            return self.current_state

        next_state = self._get_next_state(vehicle_state, planner)

        if next_state is not None and next_state != self.current_state:
            self.change_state(next_state)

        return self.current_state

    def change_state(self, new_state):
        """Change state while enforcing the FSM transition table."""
        if not isinstance(new_state, DrivingState):
            raise TypeError("new_state must be a DrivingState")

        if new_state == self.current_state:
            return

        if new_state not in self.valid_transitions[self.current_state]:
            raise ValueError(
                f"Invalid FSM transition: "
                f"{self.current_state.name} -> {new_state.name}"
            )

        self.previous_state = self.current_state
        self.current_state = new_state
        self.time_in_state = 0
        self.state_history.append(new_state)

        print(f"[FSM] {self.previous_state.name} -> {self.current_state.name}")

    def _get_next_state(self, vehicle_state, planner):
        """Map Planner decisions to the next driving phase."""
        state = self.current_state

        if self._planner_decision(planner, "race_finished", vehicle_state):
            return DrivingState.STOP

        if state == DrivingState.START:
            return DrivingState.FULL_THROTTLE

        if state == DrivingState.FULL_THROTTLE:
            if self._planner_decision(planner, "should_lift", vehicle_state):
                return DrivingState.APPROACH_CORNER
            return None

        if state == DrivingState.APPROACH_CORNER:
            if self._planner_decision(planner, "should_brake", vehicle_state):
                return DrivingState.BRAKING
            if self._planner_decision(planner, "full_throttle_ready", vehicle_state):
                return DrivingState.FULL_THROTTLE
            return None

        if state == DrivingState.BRAKING:
            if self._planner_decision(planner, "should_trail_brake", vehicle_state):
                return DrivingState.TRAIL_BRAKE
            if self._planner_decision(planner, "should_turn_in", vehicle_state):
                return DrivingState.TURN_IN
            return None

        if state == DrivingState.TRAIL_BRAKE:
            if self._planner_decision(planner, "should_turn_in", vehicle_state):
                return DrivingState.TURN_IN
            if self._planner_decision(planner, "at_apex", vehicle_state):
                return DrivingState.MID_CORNER
            return None

        if state == DrivingState.TURN_IN:
            if self._planner_decision(planner, "at_apex", vehicle_state):
                return DrivingState.MID_CORNER
            return None

        if state == DrivingState.MID_CORNER:
            if self._planner_decision(planner, "should_apply_throttle", vehicle_state):
                return DrivingState.THROTTLE_APPLICATION
            if self._planner_decision(planner, "corner_exit_ready", vehicle_state):
                return DrivingState.CORNER_EXIT
            return None

        if state == DrivingState.THROTTLE_APPLICATION:
            if self._planner_decision(planner, "corner_exit_ready", vehicle_state):
                return DrivingState.CORNER_EXIT
            return None

        if state == DrivingState.CORNER_EXIT:
            if self._planner_decision(planner, "full_throttle_ready", vehicle_state):
                return DrivingState.FULL_THROTTLE
            if self._planner_decision(planner, "should_lift", vehicle_state):
                return DrivingState.APPROACH_CORNER
            return None

        if state == DrivingState.RECOVER:
            if self._planner_decision(planner, "recovery_complete", vehicle_state):
                return DrivingState.FULL_THROTTLE
            return None

        if state == DrivingState.PIT:
            return None

        if state == DrivingState.STOP:
            return None

        return None

    @staticmethod
    def _planner_decision(planner, method_name, vehicle_state):
        """Safely call a Planner decision method if it exists."""
        method = getattr(planner, method_name, None)
        if method is None or not callable(method):
            return False

        try:
            return bool(method(vehicle_state))
        except TypeError:
            return bool(method())