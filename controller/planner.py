from dataclasses import dataclass

from .corner_profile import CornerProfile, CornerType
from .road_perception import RoadGeometry
from .vehicle_state import VehicleState


@dataclass
class DrivingPlan:
    """
    High-level driving plan produced by the Planner.
    The Controller converts this plan into low-level vehicle commands.
    """
    target_speed: float = 0.0
    target_gear: int = 1
    steering_gain: float = 0.0
    brake_intensity: float = 0.0
    acceleration_limit: float = 0.0
    brake_point: float = 0.0
    turn_in_point: float = 0.0
    apex: float = 0.0
    exit_point: float = 0.0
    target_track_pos: float = 0.0
    trail_brake: float = 0.0


class Planner:
    """
    Generates a driving plan based on the current road geometry,
    vehicle state, and classified corner.
    """

    # Target Speeds (km/h)
    STRAIGHT_SPEED = 180.0
    GENTLE_SPEED = 130.0
    MEDIUM_SPEED = 95.0
    SHARP_SPEED = 65.0
    HAIRPIN_SPEED = 40.0
    DEFAULT_SPEED = 80.0

    # Steering Gains
    STRAIGHT_STEERING = 12.0
    GENTLE_STEERING = 16.0
    MEDIUM_STEERING = 20.0
    SHARP_STEERING = 24.0
    HAIRPIN_STEERING = 28.0
    DEFAULT_STEERING = 15.0

    # Base Brake Intensities
    STRAIGHT_BRAKE = 0.0
    GENTLE_BRAKE = 0.05
    MEDIUM_BRAKE = 0.15
    SHARP_BRAKE = 0.35
    HAIRPIN_BRAKE = 0.55
    DEFAULT_BRAKE = 0.10

    # Throttle Limits
    STRAIGHT_THROTTLE = 1.0
    GENTLE_THROTTLE = 0.8
    MEDIUM_THROTTLE = 0.6
    SHARP_THROTTLE = 0.35
    HAIRPIN_THROTTLE = 0.2
    DEFAULT_THROTTLE = 0.5

    def plan(
        self,
        road: RoadGeometry,
        vehicle: VehicleState,
        corner: CornerProfile,
    ) -> DrivingPlan:

        plan_table = {
            CornerType.STRAIGHT: (170.0, 6, 4.5, 0.00, 1.00),
            CornerType.GENTLE_LEFT: (130.0, 5, 5.5, 0.10, 0.78),
            CornerType.GENTLE_RIGHT: (130.0, 5, 5.5, 0.10, 0.78),
            CornerType.MEDIUM_LEFT: (92.0, 4, 7.0, 0.45, 0.52),
            CornerType.MEDIUM_RIGHT: (92.0, 4, 7.0, 0.45, 0.52),
            CornerType.HAIRPIN_LEFT: (48.0, 2, 9.0, 0.80, 0.28),
            CornerType.HAIRPIN_RIGHT: (48.0, 2, 9.0, 0.80, 0.28),
            CornerType.CHICANE: (70.0, 3, 8.0, 0.58, 0.38),
            CornerType.EXIT: (120.0, 4, 5.0, 0.05, 0.88),
        }

        target_speed, target_gear, steering_gain, brake_intensity, acceleration_limit = plan_table.get(
            corner.corner_type,
            (self.DEFAULT_SPEED, 3, self.DEFAULT_STEERING, self.DEFAULT_BRAKE, self.DEFAULT_THROTTLE),
        )

        # -----------------------------------------------------
        # Apply severity adjustment to ALL corner types
        # -----------------------------------------------------
        severity = max(0.0, min(1.0, corner.severity))

        target_speed *= (1.0 - 0.35 * severity)
        brake_intensity += 0.25 * severity
        acceleration_limit *= (1.0 - 0.50 * severity)

        brake_intensity = max(0.0, min(1.0, brake_intensity))
        acceleration_limit = max(0.0, min(1.0, acceleration_limit))
        signed_curvature = road.signed_curvature

        if signed_curvature > 0.04:
            entry_position = 0.55
            apex = -0.45
            exit_position = 0.35
        elif signed_curvature < -0.04:
            entry_position = -0.55
            apex = 0.45
            exit_position = -0.35
        else:
            entry_position = 0.0
            apex = 0.0
            exit_position = 0.0

        if corner.corner_type == CornerType.EXIT:
            target_track_pos = exit_position
            brake_intensity *= 0.25
            acceleration_limit = max(acceleration_limit, 0.75)
        elif corner.corner_type == CornerType.CHICANE:
            target_track_pos = entry_position * 0.45
            acceleration_limit = min(acceleration_limit, 0.42)
        else:
            target_track_pos = entry_position

        brake_point = max(18.0, vehicle.speed_x * (0.32 + 0.42 * severity))
        turn_in_point = max(8.0, brake_point * 0.45)
        trail_brake = brake_intensity * (0.55 if abs(signed_curvature) > 0.08 else 0.15)

        return DrivingPlan(
            target_speed=target_speed,
            target_gear=target_gear,
            steering_gain=steering_gain,
            brake_intensity=brake_intensity,
            acceleration_limit=acceleration_limit,
            brake_point=brake_point,
            turn_in_point=turn_in_point,
            apex=apex,
            exit_point=exit_position,
            target_track_pos=max(-0.8, min(0.8, target_track_pos)),
            trail_brake=max(0.0, min(1.0, trail_brake)),
        )
