from dataclasses import dataclass


@dataclass
class DrivingContext:
    speed_x: float
    speed_y: float
    speed_z: float
    target_speed: float
    current_gear: int
    target_gear: int
    corner_type: str
    corner_severity: float
    corner_direction: str
    distance_to_corner: float
    distance_to_apex: float
    track_position: float
    steering_angle: float
    wheel_slip: float
    vehicle_stability: float
    fsm_state: str
    previous_fsm_state: str
    previous_action: str
    time_in_state: int
    speed_error: float
    previous_speed_error: float
    previous_throttle: float
    previous_brake: float
    recovery_active: bool

