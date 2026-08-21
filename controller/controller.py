from .road_perception import RoadPerception, RoadGeometry 
from .vehicle_state import VehicleState, VehiclePerception
import math
import os
from dataclasses import asdict
from typing import Optional
from .ai_action import AIAction, GearAction, LongitudinalAction
from .ai_brain import AIBrain
from .control_command import ControlCommand
from .corner_classifier import CornerClassifier
from .driving_context import DrivingContext
from .planner import Planner, DrivingPlan
from .diagnostics import DEBUG, print_warnings, validate_sensor_frame
from .driving_state import DrivingStateMachine, DrivingState


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
        self.ai_brain = AIBrain()
        self.state_machine = DrivingStateMachine()
        self.current_state = DrivingState.START
        self.previous_steering = 0.0
        self.previous_brake = 0.0
        self.previous_throttle = 0.0
        self.previous_speed_error = 0.0
        self.last_ai_action = AIAction()
        self.last_ai_context: Optional[DrivingContext] = None
        self.ai_decision_interval = 5
        self.ai_action_frames = 0
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
         driving_context = self._build_driving_context(
            vehicle=vehicle,
            plan=plan,
            corner=corner,
         )
         ai_action = self._get_ai_action(driving_context)
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
                ai_action=ai_action,
        )
         self._maybe_print_ai_diagnostics(driving_context, ai_action, command)

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
            "driving_context": asdict(driving_context),
            "ai_action": {
                "longitudinal": ai_action.longitudinal.value,
                "gear": ai_action.gear.value,
                "reason": ai_action.reason,
            },
            "command": asdict(command),
            "recovery_active": self.last_recovery_active,
         }
         self.previous_steering = command.steering
         self.previous_brake = command.brake
         self.previous_throttle = command.acceleration
         self.frame += 1
         return command

    def _create_control_command(
        self,
        road: RoadGeometry,
        vehicle: VehicleState,
        plan: DrivingPlan,
        state: DrivingState,
        ai_action: Optional[AIAction] = None,
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

        elif state == DrivingState.APPROACH_CORNER:
            acceleration = min(acceleration, 0.15)
            brake = 0.0

        elif state in (DrivingState.BRAKING, DrivingState.TRAIL_BRAKE):
            acceleration = 0.0
            brake = max(brake, plan.brake_intensity)

        elif state == DrivingState.TURN_IN:
            acceleration = 0.0
            brake = max(brake, plan.trail_brake)

        elif state == DrivingState.MID_CORNER:
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

        elif state in (DrivingState.PIT, DrivingState.STOP):
            acceleration = 0.0
            brake = max(brake, 0.25)

        if ai_action is not None:
            acceleration, brake = self._apply_ai_longitudinal_action(
                ai_action=ai_action,
                state=state,
                plan=plan,
                acceleration=acceleration,
                brake=brake,
            )

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
            ai_gear_action=ai_action.gear if ai_action is not None else None,
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

    def _build_driving_context(
        self,
        vehicle: VehicleState,
        plan: DrivingPlan,
        corner,
    ) -> DrivingContext:
        speed_error = plan.target_speed - vehicle.speed_x
        distance_to_corner = plan.brake_point if plan.brake_intensity > 0.12 else plan.turn_in_point
        distance_to_apex = abs(vehicle.track_pos - plan.apex)
        stability = 1.0
        stability -= min(0.35, abs(vehicle.speed_y) / 70.0)
        stability -= min(0.35, vehicle.wheel_slip * 0.35)
        stability -= min(0.30, abs(vehicle.steering_angle) / math.pi)

        previous_action = self.last_ai_action.longitudinal.value if self.last_ai_action else "COAST"

        return DrivingContext(
            speed_x=vehicle.speed_x,
            speed_y=vehicle.speed_y,
            speed_z=vehicle.speed_z,
            target_speed=plan.target_speed,
            current_gear=vehicle.gear,
            target_gear=plan.target_gear,
            corner_type=corner.corner_type.value,
            corner_severity=corner.severity,
            corner_direction=corner.direction,
            distance_to_corner=distance_to_corner,
            distance_to_apex=distance_to_apex,
            track_position=vehicle.track_pos,
            steering_angle=vehicle.steering_angle,
            wheel_slip=vehicle.wheel_slip,
            vehicle_stability=max(0.0, min(1.0, stability)),
            fsm_state=self.current_state.name,
            previous_fsm_state=self.state_machine.previous_state.name,
            previous_action=previous_action,
            time_in_state=self.state_machine.time_in_state,
            speed_error=speed_error,
            previous_speed_error=self.previous_speed_error,
            previous_throttle=self.previous_throttle,
            previous_brake=self.previous_brake,
            recovery_active=self.last_recovery_active,
        )

    def _get_ai_action(self, context: DrivingContext) -> AIAction:
        should_decide = self.frame % self.ai_decision_interval == 0
        if should_decide:
            try:
                next_action = self.ai_brain.decide(context)
            except Exception as error:
                if DEBUG:
                    print(f"[AI] decision failed: {error}")
                next_action = AIAction(LongitudinalAction.COAST, GearAction.GEAR_HOLD, "AI fallback")

            if next_action.longitudinal == self.last_ai_action.longitudinal and next_action.gear == self.last_ai_action.gear:
                self.ai_action_frames += self.ai_decision_interval
            else:
                self.ai_action_frames = 0
            self.last_ai_action = next_action
            self.last_ai_context = context

        return self.last_ai_action

    def _apply_ai_longitudinal_action(
        self,
        ai_action: AIAction,
        state: DrivingState,
        plan: DrivingPlan,
        acceleration: float,
        brake: float,
    ) -> tuple[float, float]:
        requested_acceleration, requested_brake = self._ai_longitudinal_values(ai_action.longitudinal)

        if state in (DrivingState.START, DrivingState.FULL_THROTTLE):
            acceleration = min(requested_acceleration, plan.acceleration_limit)
            brake = min(requested_brake, 0.20)

        elif state == DrivingState.APPROACH_CORNER:
            acceleration = min(requested_acceleration, 0.40, plan.acceleration_limit)
            brake = min(requested_brake, max(0.25, plan.brake_intensity * 0.50))

        elif state in (DrivingState.BRAKING, DrivingState.TRAIL_BRAKE):
            acceleration = 0.0
            brake = max(brake, requested_brake, plan.brake_intensity)

        elif state == DrivingState.TURN_IN:
            acceleration = 0.0
            brake = max(brake, min(requested_brake, max(0.20, plan.trail_brake)))

        elif state == DrivingState.MID_CORNER:
            acceleration = min(requested_acceleration, 0.50, plan.acceleration_limit)
            brake = min(requested_brake, 0.15)

        elif state == DrivingState.THROTTLE_APPLICATION:
            acceleration = min(requested_acceleration, 0.80, plan.acceleration_limit)
            brake = 0.0

        elif state == DrivingState.CORNER_EXIT:
            acceleration = min(requested_acceleration, plan.acceleration_limit)
            brake = 0.0

        elif state == DrivingState.RECOVER:
            acceleration = 0.0
            brake = max(brake, 0.20)

        elif state in (DrivingState.PIT, DrivingState.STOP):
            acceleration = 0.0
            brake = max(brake, requested_brake, 0.25)

        return max(0.0, min(1.0, acceleration)), max(0.0, min(1.0, brake))

    def _ai_longitudinal_values(self, action: LongitudinalAction) -> tuple[float, float]:
        throttle_values = {
            LongitudinalAction.THROTTLE_25: 0.25,
            LongitudinalAction.THROTTLE_40: 0.40,
            LongitudinalAction.THROTTLE_60: 0.60,
            LongitudinalAction.THROTTLE_80: 0.80,
            LongitudinalAction.THROTTLE_100: 1.00,
        }
        brake_values = {
            LongitudinalAction.BRAKE_20: 0.20,
            LongitudinalAction.BRAKE_40: 0.40,
            LongitudinalAction.BRAKE_60: 0.60,
            LongitudinalAction.BRAKE_80: 0.80,
            LongitudinalAction.BRAKE_100: 1.00,
        }
        if action in throttle_values:
            return throttle_values[action], 0.0
        if action in brake_values:
            return 0.0, brake_values[action]
        return 0.0, 0.0

    def _apply_ai_gear_action(
        self,
        ai_action: AIAction,
        gear: int,
        vehicle: VehicleState,
        plan: DrivingPlan,
        brake: float,
    ) -> int:
        return self._calculate_gear(
            rpm=vehicle.rpm,
            current_gear=vehicle.gear,
            vehicle=vehicle,
            plan=plan,
            brake=brake,
            ai_gear_action=ai_action.gear if ai_action is not None else None,
        )

    def _maybe_print_ai_diagnostics(
        self,
        context: DrivingContext,
        ai_action: AIAction,
        command: ControlCommand,
    ) -> None:
        if not DEBUG:
            return
        if self.frame % 25 != 0 and ai_action.longitudinal == self.last_ai_action.longitudinal:
            return
        print(
            "[AI] "
            f"state={context.fsm_state} "
            f"speed={context.speed_x:.1f}/{context.target_speed:.1f} "
            f"corner={context.corner_type} severity={context.corner_severity:.2f} "
            f"action={ai_action.longitudinal.value}/{ai_action.gear.value} "
            f"command=thr:{command.acceleration:.2f} brk:{command.brake:.2f} "
            f"gear:{command.gear} steer:{command.steering:.3f}"
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
        ai_gear_action: Optional[GearAction] = None,
    ) -> int:
        """Corner-aware automatic gear selection with hysteresis and AI recommendation validation."""

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

        if current_gear > 1:
            if rpm < downshift_rpm[current_gear] or (ai_gear_action == GearAction.GEAR_DOWN and current_gear > target_gear):
                if speed < max_speed_for_gear[current_gear - 1]:
                    self.shift_cooldown = 10
                    return current_gear - 1

        if current_gear < 6 and rpm > upshift_rpm[current_gear]:
            if speed > min_speed_for_gear[current_gear + 1] and (current_gear < target_gear or ai_gear_action == GearAction.GEAR_UP or brake < 0.05):
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
