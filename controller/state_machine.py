from enum import Enum, auto


class DrivingState(Enum):
    START = auto()
    FULL_THROTTLE = auto()
    LIFT_OFF = auto()
    BRAKING = auto()
    TURN_IN = auto()
    MID_CORNER = auto()
    THROTTLE_APPLICATION = auto()
    CORNER_EXIT = auto()
    RECOVER = auto()
    PIT = auto()


class DrivingStateMachine:
    VALID_TRANSITIONS = {
        DrivingState.START: {DrivingState.FULL_THROTTLE},
        DrivingState.FULL_THROTTLE: {DrivingState.LIFT_OFF},
        DrivingState.LIFT_OFF: {DrivingState.BRAKING},
        DrivingState.BRAKING: {DrivingState.TURN_IN},
        DrivingState.TURN_IN: {DrivingState.MID_CORNER},
        DrivingState.MID_CORNER: {DrivingState.THROTTLE_APPLICATION},
        DrivingState.THROTTLE_APPLICATION: {DrivingState.CORNER_EXIT},
        DrivingState.CORNER_EXIT: {DrivingState.FULL_THROTTLE},
        DrivingState.RECOVER: {DrivingState.FULL_THROTTLE},
        DrivingState.PIT: set(),
    }

    GLOBAL_TRANSITIONS = {DrivingState.RECOVER, DrivingState.PIT}

    def __init__(self):
        self.current_state = DrivingState.START
        self.previous_state = None
        self.time_in_state = 0
        self.state_history = [self.current_state]
        self._transition_handlers = {
            DrivingState.START: self._transition_from_start,
            DrivingState.FULL_THROTTLE: self._transition_from_full_throttle,
            DrivingState.LIFT_OFF: self._transition_from_lift_off,
            DrivingState.BRAKING: self._transition_from_braking,
            DrivingState.TURN_IN: self._transition_from_turn_in,
            DrivingState.MID_CORNER: self._transition_from_mid_corner,
            DrivingState.THROTTLE_APPLICATION: self._transition_from_throttle_application,
            DrivingState.CORNER_EXIT: self._transition_from_corner_exit,
            DrivingState.RECOVER: self._transition_from_recover,
            DrivingState.PIT: self._transition_from_pit,
        }

    def update(self, vehicle_state=None, planner=None):
        self.time_in_state += 1

        global_transition = self._evaluate_global_transitions(vehicle_state, planner)
        if global_transition is not None:
            self.change_state(global_transition)
            return self.current_state

        transition_handler = self._transition_handlers[self.current_state]
        next_state = transition_handler(vehicle_state, planner)
        if next_state is not None:
            self.change_state(next_state)

        return self.current_state

    def change_state(self, new_state):
        if not isinstance(new_state, DrivingState):
            raise TypeError("new_state must be a DrivingState")

        if new_state == self.current_state:
            return

        if not self._is_valid_transition(new_state):
            raise ValueError(
                f"Invalid FSM transition: {self.current_state.name} -> {new_state.name}"
            )

        old_state = self.current_state
        self.previous_state = old_state
        self.current_state = new_state
        self.time_in_state = 0
        self.state_history.append(new_state)

        print("[FSM]")
        print(old_state.name)
        print(" ->")
        print(new_state.name)

    def _is_valid_transition(self, new_state):
        if new_state in self.GLOBAL_TRANSITIONS:
            return True

        valid_next_states = self.VALID_TRANSITIONS[self.current_state]
        return new_state in valid_next_states

    def _evaluate_global_transitions(self, vehicle_state, planner):
        # TODO:
        # Transition to RECOVER when validated recovery conditions are active.
        #
        # TODO:
        # Transition to PIT when strategy or damage/fuel logic requests pit entry.
        return None

    def _transition_from_start(self, vehicle_state, planner):
        # START has one legal next state. This initializes the normal driving loop.
        return DrivingState.FULL_THROTTLE

    def _transition_from_full_throttle(self, vehicle_state, planner):
        # TODO:
        # Transition to LIFT_OFF when planner indicates an approaching braking zone.
        return None

    def _transition_from_lift_off(self, vehicle_state, planner):
        # TODO:
        # Transition to BRAKING when planner indicates the braking point is reached.
        return None

    def _transition_from_braking(self, vehicle_state, planner):
        # TODO:
        # Transition to TURN_IN when target entry speed and turn-in point are reached.
        return None

    def _transition_from_turn_in(self, vehicle_state, planner):
        # TODO:
        # Transition to MID_CORNER when the car is committed to the corner arc.
        return None

    def _transition_from_mid_corner(self, vehicle_state, planner):
        # TODO:
        # Transition to THROTTLE_APPLICATION when apex or minimum-speed phase is reached.
        return None

    def _transition_from_throttle_application(self, vehicle_state, planner):
        # TODO:
        # Transition to CORNER_EXIT when throttle can increase toward exit acceleration.
        return None

    def _transition_from_corner_exit(self, vehicle_state, planner):
        # TODO:
        # Transition to FULL_THROTTLE when the car is stable and aligned for the next straight.
        return None

    def _transition_from_recover(self, vehicle_state, planner):
        # TODO:
        # Transition to FULL_THROTTLE when recovery is complete and normal driving is safe.
        return None

    def _transition_from_pit(self, vehicle_state, planner):
        # TODO:
        # PIT is terminal for now. Add pit-exit transitions only after pit behavior exists.
        return None
