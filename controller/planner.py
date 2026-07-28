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
    steering_gain: float = 0.0
    brake_intensity: float = 0.0
    acceleration_limit: float = 0.0


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

        if corner.corner_type == CornerType.STRAIGHT:
            target_speed = self.STRAIGHT_SPEED
            steering_gain = self.STRAIGHT_STEERING
            brake_intensity = self.STRAIGHT_BRAKE
            acceleration_limit = self.STRAIGHT_THROTTLE

        elif corner.corner_type == CornerType.GENTLE:
            target_speed = self.GENTLE_SPEED
            steering_gain = self.GENTLE_STEERING
            brake_intensity = self.GENTLE_BRAKE
            acceleration_limit = self.GENTLE_THROTTLE

        elif corner.corner_type == CornerType.MEDIUM:
            target_speed = self.MEDIUM_SPEED
            steering_gain = self.MEDIUM_STEERING
            brake_intensity = self.MEDIUM_BRAKE
            acceleration_limit = self.MEDIUM_THROTTLE

        elif corner.corner_type == CornerType.SHARP:
            target_speed = self.SHARP_SPEED
            steering_gain = self.SHARP_STEERING
            brake_intensity = self.SHARP_BRAKE
            acceleration_limit = self.SHARP_THROTTLE

        elif corner.corner_type == CornerType.HAIRPIN:
            target_speed = self.HAIRPIN_SPEED
            steering_gain = self.HAIRPIN_STEERING
            brake_intensity = self.HAIRPIN_BRAKE
            acceleration_limit = self.HAIRPIN_THROTTLE

        else:
            target_speed = self.DEFAULT_SPEED
            steering_gain = self.DEFAULT_STEERING
            brake_intensity = self.DEFAULT_BRAKE
            acceleration_limit = self.DEFAULT_THROTTLE

        # -----------------------------------------------------
        # Apply severity adjustment to ALL corner types
        # -----------------------------------------------------
        severity = max(0.0, min(1.0, corner.severity))

        target_speed *= (1.0 - 0.35 * severity)
        brake_intensity += 0.25 * severity
        acceleration_limit *= (1.0 - 0.50 * severity)

        brake_intensity = max(0.0, min(1.0, brake_intensity))
        acceleration_limit = max(0.0, min(1.0, acceleration_limit))

        return DrivingPlan(
            target_speed=target_speed,
            steering_gain=steering_gain,
            brake_intensity=brake_intensity,
            acceleration_limit=acceleration_limit,
        )