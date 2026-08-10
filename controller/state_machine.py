from enum import Enum, auto


class DrivingState(Enum):
    START = auto()
    FULL_THROTTLE = auto()
    LIFT = auto()
    BRAKING = auto()
    TRAIL_BRAKE = auto()
    TURN_IN = auto()
    APEX = auto()
    THROTTLE_APPLICATION = auto()
    CORNER_EXIT = auto()
    RECOVER = auto()
    PIT = auto()
    FINISHED = auto()

class PlannerEvent(Enum):
    NONE = auto()
    REACH_BRAKE_POINT = auto()
    REACH_TURN_IN = auto()
    REACH_APEX = auto()
    REACH_EXIT = auto()
    STRAIGHT = auto()
    SPIN = auto()
    PIT_REQUEST = auto()
    FINISH = auto()


class DrivingStateMachine:
    VALID_TRANSITIONS = {
        DrivingState.START: {DrivingState.FULL_THROTTLE},

        DrivingState.FULL_THROTTLE: {
            DrivingState.LIFT,
            DrivingState.RECOVER,
            DrivingState.PIT,
            DrivingState.FINISHED,
        },

        DrivingState.LIFT: {
            DrivingState.BRAKING,
            DrivingState.TURN_IN,
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
            DrivingState.APEX,
            DrivingState.RECOVER,
        },

        DrivingState.TURN_IN: {
            DrivingState.APEX,
            DrivingState.RECOVER,
        },

        DrivingState.APEX: {
            DrivingState.THROTTLE_APPLICATION,
            DrivingState.RECOVER,
        },

        DrivingState.THROTTLE_APPLICATION: {
            DrivingState.CORNER_EXIT,
            DrivingState.RECOVER,
        },

        DrivingState.CORNER_EXIT: {
            DrivingState.FULL_THROTTLE,
            DrivingState.LIFT,
            DrivingState.RECOVER,
        },

        DrivingState.RECOVER: {
            DrivingState.FULL_THROTTLE,
            DrivingState.PIT,
        },

        DrivingState.PIT: {
            DrivingState.FINISHED,
        },

        DrivingState.FINISHED: set(),
    }

    GLOBAL_TRANSITIONS = {
        DrivingState.RECOVER,
        DrivingState.PIT,
        DrivingState.FINISHED,
    }

    def __init__(self):
        self.current_state = DrivingState.START
        self.previous_state = None
        self.time_in_state = 0
        self.state_history = [self.current_state]

        self._transition_handlers = {
            DrivingState.START: self._transition_from_start,
            DrivingState.FULL_THROTTLE: self._transition_from_full_throttle,
            DrivingState.LIFT: self._transition_from_lift,
            DrivingState.BRAKING: self._transition_from_braking,
            DrivingState.TRAIL_BRAKE: self._transition_from_trail_brake,
            DrivingState.TURN_IN: self._transition_from_turn_in,
            DrivingState.APEX: self._transition_from_apex,
            DrivingState.THROTTLE_APPLICATION: self._transition_from_throttle_application,
            DrivingState.CORNER_EXIT: self._transition_from_corner_exit,
            DrivingState.RECOVER: self._transition_from_recover,
            DrivingState.PIT: self._transition_from_pit,
            DrivingState.FINISHED: self._transition_from_finished,
        }

    def update(self, vehicle_state=None, planner=None):
        self.time_in_state += 1

        global_transition = self._evaluate_global_transitions(vehicle_state, planner)
        if global_transition is not None:
            self.change_state(global_transition)
            return self.current_state

        handler = self._transition_handlers.get(self.current_state)
        if handler is None:
            return self.current_state

        next_state = handler(vehicle_state, planner)
        if next_state is not None:
            self.change_state(next_state)

        return self.current_state

    def change_state(self, new_state):
        if new_state == self.current_state:
            return

        if not isinstance(new_state, DrivingState):
            raise TypeError("new_state must be a DrivingState")

        if not self._is_valid_transition(new_state):
            raise ValueError(
                f"Invalid FSM transition: {self.current_state.name} -> {new_state.name}"
            )

        old_state = self.current_state
        self.previous_state = old_state
        self.current_state = new_state
        self.time_in_state = 0
        self.state_history.append(new_state)

        print(f"[FSM] {old_state.name} -> {new_state.name}")

    def _is_valid_transition(self, new_state):
        if new_state in self.GLOBAL_TRANSITIONS:
            return True
        return new_state in self.VALID_TRANSITIONS[self.current_state]

    def _transition_from_start(self, vehicle_state, planner):
        return DrivingState.FULL_THROTTLE

    def _transition_from_full_throttle(self, vehicle_state, planner):
        if planner and planner.should_lift(vehicle_state):
            return DrivingState.LIFT
        return None

    def _transition_from_lift(self, vehicle_state, planner):
        if planner and planner.should_brake(vehicle_state):
            return DrivingState.BRAKING
        return None

    def _transition_from_braking(self, vehicle_state, planner):
        if planner and planner.should_trail_brake(vehicle_state):
            return DrivingState.TRAIL_BRAKE
        if planner and planner.should_turn_in(vehicle_state):
            return DrivingState.TURN_IN
        return None

    def _transition_from_trail_brake(self, vehicle_state, planner):
        if planner and planner.should_turn_in(vehicle_state):
            return DrivingState.TURN_IN
        if planner and planner.at_apex(vehicle_state):
            return DrivingState.APEX
        return None

    def _transition_from_turn_in(self, vehicle_state, planner):
        if planner and planner.at_apex(vehicle_state):
            return DrivingState.APEX
        return None

    def _transition_from_apex(self, vehicle_state, planner):
        if planner and planner.should_apply_throttle(vehicle_state):
            return DrivingState.THROTTLE_APPLICATION
        return None

    def _transition_from_throttle_application(self, vehicle_state, planner):
        if planner and planner.corner_exit_ready(vehicle_state):
            return DrivingState.CORNER_EXIT
        return None

    def _transition_from_corner_exit(self, vehicle_state, planner):
        if planner and planner.full_throttle_ready(vehicle_state):
            return DrivingState.FULL_THROTTLE
        return None

    def _transition_from_recover(self, vehicle_state, planner):
        if planner and planner.recovery_complete(vehicle_state):
            return DrivingState.FULL_THROTTLE
        return None

    def _transition_from_pit(self, vehicle_state, planner):
        if planner and planner.race_finished(vehicle_state):
            return DrivingState.FINISHED
        return None

    def _transition_from_finished(self, vehicle_state, planner):
        return None

    def _evaluate_global_transitions(self, vehicle_state, planner):
        # This method was called in update but was not defined in the original code.
        # Providing a placeholder implementation that always returns None.
        return None
