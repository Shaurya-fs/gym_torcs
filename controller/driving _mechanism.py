from torcs.gym_torcs.controller.state_machine import DrivingStateMachine
from torcs.gym_torcs.controller.driving_state import DrivingState
from torcs.gym_torcs.controller.planner import Planner
from torcs.gym_torcs.controller.vehicle_state import VehicleState


class DrivingMechanism:
    """Convert the current FSM driving state and planner output into car controls."""

    def compute_action(self, vehicle_state: VehicleState, driving_state: DrivingState, planner: Planner):
        if driving_state == DrivingState.FULL_THROTTLE:
            throttle = 1.0
            brake = 0.0

        elif driving_state == DrivingState.LIFT:
            throttle = 0.0
            brake = 0.0

        elif driving_state == DrivingState.BRAKING:
            throttle = 0.0
            brake = planner.get_brake(vehicle_state)

        elif driving_state == DrivingState.TRAIL_BRAKE:
            throttle = 0.0
            brake = planner.get_trail_brake(vehicle_state)

        elif driving_state == DrivingState.TURN_IN:
            throttle = 0.0
            brake = planner.get_turn_in_brake(vehicle_state)

        elif driving_state == DrivingState.APEX:
            throttle = planner.get_apex_throttle(vehicle_state)
            brake = 0.0

        elif driving_state == DrivingState.THROTTLE_APPLICATION:
            throttle = planner.get_throttle_application(vehicle_state)
            brake = 0.0

        elif driving_state == DrivingState.CORNER_EXIT:
            throttle = planner.get_exit_throttle(vehicle_state)
            brake = 0.0

        elif driving_state == DrivingState.RECOVER:
            throttle = planner.get_recovery_throttle(vehicle_state)
            brake = planner.get_recovery_brake(vehicle_state)

        elif driving_state == DrivingState.PIT:
            throttle = 0.0
            brake = planner.get_pit_brake(vehicle_state)

        elif driving_state == DrivingState.FINISHED:
            throttle = 0.0
            brake = 1.0

        else:
            throttle = 0.0
            brake = 0.0

        steering = planner.get_steering(vehicle_state, driving_state)
        gear = planner.get_recommended_gear(vehicle_state, driving_state)

        return {
            "throttle": throttle,
            "brake": brake,
            "steering": steering,
            "gear": gear,
        }