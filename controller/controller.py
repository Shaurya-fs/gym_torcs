from .road_perception import RoadPerception, RoadGeometry 
from .vehicle_state import VehicleState, VehiclePerception
import math
import os
from dataclasses import asdict
from typing import Optional
from .control_command import ControlCommand
from .corner_classifier import CornerClassifier
from .planner import Planner, DrivingPlan
from .diagnostics import DEBUG, print_warnings, validate_sensor_frame
from .state_machine import DrivingStateMachine
from .driving_state import DrivingState


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
        self.state_machine = DrivingStateMachine()
        self.current_state = DrivingState.START
        self.previous_steering = 0.0
        self.previous_brake = 0.0
        self.previous_speed_error = 0.0
        self.shift_cooldown = 0
        self.frame = 0
        self.stuck_frames = 0
        self.telemetry_path = os.path.join(os.getcwd(), "controller_telemetry.csv")
        self.telemetry_initialized = False
        self.last_telemetry_context = {}
        self.last_recovery_active = False

    def update(self,sensor_data:dict,previous_command: Optional[ControlCommand]= None)->ControlCommand:
         if previous_command is None:
            previous_command = ControlCommand()
         print_warnings(validate_sensor_frame(sensor_data, "controller.sensor"))
         road= self.road_perception.update(sensor_data['track'], sensor_data['angle'])
         vehicle = self.vehicle_perception.update(
            track_pos=sensor_data["trackPos"],
            steering_angle=sensor_data["angle"],
            speed_x=sensor_data["speedX"],
            speed_y=sensor_data.get("speedY", 0.0),
            speed_z=sensor_data.get("speedZ", 0.0),
            gear=int(sensor_data["gear"]),
            rpm=sensor_data["rpm"],
            steering=previous_command.steering,
            wheel_spin_vel=sensor_data.get("wheelSpinVel", [0.0, 0.0, 0.0, 0.0]),
         )
         if DEBUG:
            print(
                f"[RAW SENSOR] speedX={sensor_data['speedX']} rpm={sensor_data['rpm']} "
                f"gear={sensor_data['gear']} trackPos={sensor_data['trackPos']} "
                f"wheelSpin={sensor_data.get('wheelSpinVel')}"
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
         self.current_state = self.state_machine.update(
            vehicle_state=vehicle,
            planner=self.planner,
         )
         if DEBUG:
             print(
                 f"[FSM STATE] {self.current_state.name} "
                 f"(time_in_state={self.state_machine.time_in_state})"
             )

         command = self._create_control_command(
                road=road,
                vehicle=vehicle,
                plan=plan,
                state=self.current_state,
        )

         self.last_telemetry_context = {
            "frame": self.frame,
            "sensor_data": sensor_data,
            "road": self._road_to_dict(road),
            "vehicle": asdict(vehicle),
            "corner": {
                "corner_type": corner.corner_type.value,
                "turn_angle": corner.turn_angle,
                "severity": corner.severity,
                "direction": corner.direction,
            },
            "plan": asdict(plan),
            "command": asdict(command),
            "recovery_active": self.last_recovery_active,
         }
         self.previous_steering = command.steering
         self.previous_brake = command.brake
         self.frame += 1
         return command

    def _create_control_command(
        self,
        road: RoadGeometry,
        vehicle: VehicleState,
        plan: DrivingPlan,
        state: DrivingState,
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
            state=state,
        )

        # FSM driving-phase filter. Steering remains planner/controller driven;
        # the FSM only constrains longitudinal behavior for now.
        if state in (DrivingState.START, DrivingState.FULL_THROTTLE):
            brake = 0.0
            acceleration = max(acceleration, min(1.0, plan.acceleration_limit))

        elif state == DrivingState.LIFT:
            acceleration = min(acceleration, 0.15)
            brake = 0.0

        elif state in (DrivingState.BRAKING, DrivingState.TRAIL_BRAKE):
            acceleration = 0.0
            brake = max(brake, plan.brake_intensity)

        elif state == DrivingState.TURN_IN:
            acceleration = 0.0
            brake = max(brake, plan.trail_brake)

        elif state == DrivingState.APEX:
            acceleration = min(acceleration, 0.50)
            brake = min(brake, 0.15)

        elif state == DrivingState.THROTTLE_APPLICATION:
            acceleration = max(acceleration, min(0.50, plan.acceleration_limit))
            brake = 0.0

        elif state == DrivingState.CORNER_EXIT:
            acceleration = max(acceleration, min(0.70, plan.acceleration_limit))
            brake = 0.0

        elif state == DrivingState.RECOVER:
            # Recovery is handled by _recovery_command below.
            pass

        elif state in (DrivingState.PIT, DrivingState.FINISHED):
            acceleration = 0.0
            brake = max(brake, 0.25)

        recovery_command = self._recovery_command(road=road, vehicle=vehicle)
        if recovery_command is not None:
            self.last_recovery_active = True
            return recovery_command
        self.last_recovery_active = False

        gear = self._calculate_gear(
            rpm=vehicle.rpm,
            current_gear=vehicle.gear,
            vehicle=vehicle,
            plan=plan,
            brake=brake,
        )

        if DEBUG:
            print(
                f"[GEAR] State={state.name} "
                f"Speed={vehicle.speed_x:.1f} "
                f"RPM={vehicle.rpm:.0f} "
                f"Observed={vehicle.gear} "
                f"Calculated={gear} "
                f"Target={getattr(plan, 'target_gear', 'N/A')}"
            )
            print(
                f"[CONTROL] State={state.name} "
                f"Throttle={acceleration:.2f} "
                f"Brake={brake:.2f} "
                f"Gear={gear} "
                f"Steer={steering:.3f}"
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

        target_track_pos = getattr(plan, "target_track_pos", 0.0)
        cross_track_error = vehicle.track_pos - target_track_pos
        speed = max(20.0, abs(vehicle.speed_x))
        lookahead = max(12.0, min(70.0, 0.45 * speed))

        heading_term = vehicle.steering_angle * plan.steering_gain / math.pi
        cross_track_term = -0.50 * cross_track_error / (1.0 + lookahead / 35.0)
        raw_steering = heading_term + cross_track_term

        damping = -0.65 * (raw_steering - self.previous_steering)
        steering = raw_steering + damping

        max_rate = 0.035 + 0.20 / (1.0 + speed / 80.0)
        delta = max(-max_rate, min(max_rate, steering - self.previous_steering))
        steering = self.previous_steering + delta

        return max(-1.0, min(1.0, steering))

    def _calculate_speed_control(
        self,
        vehicle: VehicleState,
        plan: DrivingPlan,
        state: DrivingState,
     ) -> tuple[float, float]:
        """
        Calculate throttle and brake from current speed and target speed.
        """

        speed_error = plan.target_speed - vehicle.speed_x
        steering_load = min(1.0, abs(self.previous_steering))
        lateral_load = min(1.0, abs(vehicle.speed_y) / 25.0)
        slip_load = min(1.0, vehicle.wheel_slip)

        if speed_error < -2.0:
            overspeed = min(1.0, abs(speed_error) / 55.0)
            brake = plan.brake_intensity * (0.30 + 0.70 * overspeed)
            if steering_load > 0.16 and vehicle.speed_x > plan.target_speed * 0.80:
                brake = max(brake, plan.trail_brake * (1.0 - min(1.0, max(speed_error, 0.0) / 20.0)))
            acceleration = 0.0
        else:
            brake = 0.0
            throttle_base = max(0.0, min(1.0, speed_error / 45.0))
            corner_reduction = max(0.20, 1.0 - 0.65 * steering_load - 0.25 * lateral_load)
            slip_reduction = max(0.20, 1.0 - 0.75 * slip_load)
            acceleration = plan.acceleration_limit * throttle_base * corner_reduction * slip_reduction
            if vehicle.speed_x < 25.0:
                acceleration = min(acceleration, 0.65)

        if brake < self.previous_brake:
            brake = max(brake, self.previous_brake - 0.08)

        self.previous_speed_error = speed_error
        return max(0.0, min(1.0, acceleration)), max(0.0, min(1.0, brake))

    def _calculate_gear(
        self,
        rpm: float,
        current_gear: int,
        vehicle: VehicleState,
        plan: DrivingPlan,
        brake: float = 0.0,
    ) -> int:
        """Corner-aware automatic gear selection with hysteresis."""

        current_gear = max(1, min(6, int(current_gear)))
        speed = max(0.0, vehicle.speed_x)

        if speed < 8.0:
            self.shift_cooldown = max(self.shift_cooldown - 1, 0)
            return 1

        if self.shift_cooldown > 0:
            self.shift_cooldown -= 1
            return current_gear

        upshift_rpm = [0, 6500, 6800, 7000, 7200, 7400, 99999]
        downshift_rpm = [0, 0, 3600, 3900, 4200, 4500, 4800]
        min_speed_for_gear = [0, 0, 20, 45, 80, 115, 150]
        max_speed_for_gear = [0, 55, 85, 125, 165, 210, 999]
        target_gear = max(1, min(6, int(getattr(plan, "target_gear", current_gear))))

        if brake > 0.15 and current_gear > target_gear:
            if rpm < 6500 or speed < max_speed_for_gear[current_gear - 1]:
                self.shift_cooldown = 8
                return current_gear - 1

        if current_gear > 1 and rpm < downshift_rpm[current_gear]:
            if speed < max_speed_for_gear[current_gear - 1]:
                self.shift_cooldown = 10
                return current_gear - 1

        if current_gear < 6 and rpm > upshift_rpm[current_gear]:
            if speed > min_speed_for_gear[current_gear + 1] and (current_gear < target_gear or brake < 0.05):
                self.shift_cooldown = 10
                return current_gear + 1

        return current_gear

    def _recovery_command(self, road: RoadGeometry, vehicle: VehicleState) -> Optional[ControlCommand]:
        if abs(vehicle.track_pos) < 0.92 and math.cos(vehicle.steering_angle) > 0.15:
            self.stuck_frames = 0
            return None

        if abs(vehicle.speed_x) < 3.0:
            self.stuck_frames += 1
        else:
            self.stuck_frames = 0

        steer_to_center = -vehicle.track_pos * 0.85 - vehicle.steering_angle * 0.45
        steer_to_center = max(-0.65, min(0.65, steer_to_center))

        if math.cos(vehicle.steering_angle) <= 0.15:
            return ControlCommand(steering=steer_to_center, acceleration=0.0, brake=0.25, gear=1)

        if self.stuck_frames > 80:
            return ControlCommand(steering=steer_to_center, acceleration=0.25, brake=0.0, gear=-1)

        return ControlCommand(steering=steer_to_center, acceleration=0.35, brake=0.0, gear=1)

    def _road_to_dict(self, road: RoadGeometry) -> dict:
        data = asdict(road)
        data["direction"] = road.direction.value
        return data
