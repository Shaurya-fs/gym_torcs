from .road_perception import RoadPerception, RoadGeometry 
from .vehicle_state import VehicleState, VehiclePerception
import math
from typing import Optional
from .control_command import ControlCommand
from .corner_classifier import CornerClassifier
from .planner import Planner, DrivingPlan


class RacingController:
    """Coordinates the perception, planning, and control pipeline."""

    CENTERING_GAIN = 0.50

    UPSHIFT_RPM = 7000
    DOWNSHIFT_RPM = 2500

    def __init__(self):
        self.road_perception = RoadPerception()
        self.vehicle_perception = VehiclePerception()
        self.corner_classifier = CornerClassifier()
        self.planner = Planner()

    def update(self,sensor_data:dict,previous_command: Optional[ControlCommand]= None)->ControlCommand:
         if previous_command is None:
            previous_command = ControlCommand()
         road= self.road_perception.update(sensor_data['track'], sensor_data['angle'])
         vehicle = self.vehicle_perception.update(
            track_pos=sensor_data["trackPos"],
            steering_angle=sensor_data["angle"],
            speed_x=sensor_data["speedX"],
            gear=int(sensor_data["gear"]),
            rpm=sensor_data["rpm"],
            steering=previous_command.steering,
         )

         corner = self.corner_classifier.classify(
            road=road,
            vehicle=vehicle,
         )

         plan = self.planner.plan(
            road=road,
            vehicle=vehicle,
            corner=corner,
         )

         command = self._create_control_command(
            road=road,
            vehicle=vehicle,
            plan=plan,
         )

         return command

    def _create_control_command(
        self,
        road: RoadGeometry,
        vehicle: VehicleState,
        plan: DrivingPlan,
    ) -> ControlCommand:
        """
        Convert a high-level DrivingPlan into actual car commands.
        """

        steering = self._calculate_steering(
            vehicle=vehicle,
            plan=plan,
        )

        acceleration, brake = self._calculate_speed_control(
            vehicle=vehicle,
            plan=plan,
        )

        gear = self._calculate_gear(
            rpm=vehicle.rpm,
            current_gear=vehicle.gear,
        )

        return ControlCommand(
            steering=steering,
            acceleration=acceleration,
            brake=brake,
            gear=gear,
        )

    def _calculate_steering(
        self,
        vehicle: VehicleState,
        plan: DrivingPlan,
    ) -> float:
        """
        Calculate steering from heading angle and track position.
        """

        steering = vehicle.steering_angle * plan.steering_gain / math.pi

        steering -= vehicle.track_pos * self.CENTERING_GAIN

        return max(-1.0, min(1.0, steering))

    def _calculate_speed_control(
        self,
        vehicle: VehicleState,
        plan: DrivingPlan,
    ) -> tuple[float, float]:
        """
        Calculate throttle and brake from current speed and target speed.
        """

        speed_error = plan.target_speed - vehicle.speed_x

        if speed_error > 5.0:
            acceleration = plan.acceleration_limit
            brake = 0.0

        elif speed_error < -5.0:
            acceleration = 0.0
            brake = plan.brake_intensity

        else:
            acceleration = 0.20 * plan.acceleration_limit
            brake = 0.0

        return acceleration, brake

    def _calculate_gear(self, rpm: float, current_gear: int) -> int:
        """Automatic gear selection based on engine RPM."""

        if rpm > self.UPSHIFT_RPM and current_gear < 6:
            return current_gear + 1

        if rpm < self.DOWNSHIFT_RPM and current_gear > 1:
            return current_gear - 1

        return current_gear