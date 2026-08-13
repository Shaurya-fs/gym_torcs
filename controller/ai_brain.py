from .ai_action import AIAction, GearAction, LongitudinalAction
from .driving_context import DrivingContext


class AIBrain:
    """Fast deterministic placeholder for the constrained AI decision layer."""

    def decide(self, context: DrivingContext) -> AIAction:
        speed_error = context.speed_error
        state = context.fsm_state

        if context.recovery_active:
            return AIAction(LongitudinalAction.COAST, GearAction.GEAR_HOLD, "recovery fallback")

        if state in ("BRAKING", "TRAIL_BRAKE"):
            if speed_error < -35.0:
                longitudinal = LongitudinalAction.BRAKE_80
            elif speed_error < -22.0:
                longitudinal = LongitudinalAction.BRAKE_60
            elif speed_error < -10.0:
                longitudinal = LongitudinalAction.BRAKE_40
            else:
                longitudinal = LongitudinalAction.BRAKE_20
            return AIAction(longitudinal, self._gear_action(context), "braking to target speed")

        if state == "TURN_IN":
            if speed_error < -8.0:
                return AIAction(LongitudinalAction.BRAKE_20, self._gear_action(context), "settle turn in")
            return AIAction(LongitudinalAction.COAST, self._gear_action(context), "coast at turn in")

        if state == "APPROACH_CORNER":
            if speed_error < -12.0:
                return AIAction(LongitudinalAction.BRAKE_20, self._gear_action(context), "approach overspeed")
            if context.corner_severity > 0.45:
                return AIAction(LongitudinalAction.LIFT, self._gear_action(context), "lift for severe corner")
            return AIAction(LongitudinalAction.THROTTLE_25, self._gear_action(context), "reduced approach throttle")

        if state == "MID_CORNER":
            if context.vehicle_stability < 0.55 or context.wheel_slip > 0.30:
                return AIAction(LongitudinalAction.COAST, self._gear_action(context), "stability limited")
            return AIAction(LongitudinalAction.THROTTLE_40, self._gear_action(context), "balanced mid corner")

        if state == "THROTTLE_APPLICATION":
            return AIAction(self._progressive_throttle(context, 0.80), self._gear_action(context), "progressive throttle")

        if state == "CORNER_EXIT":
            return AIAction(self._progressive_throttle(context, 1.00), self._gear_action(context), "corner exit drive")

        if state in ("START", "FULL_THROTTLE"):
            return AIAction(self._progressive_throttle(context, 1.00), self._gear_action(context), "straight acceleration")

        if state in ("PIT", "STOP"):
            return AIAction(LongitudinalAction.BRAKE_40, GearAction.GEAR_HOLD, "stop constraint")

        return AIAction(LongitudinalAction.COAST, self._gear_action(context), "default safe coast")

    def _progressive_throttle(self, context: DrivingContext, limit: float) -> LongitudinalAction:
        if context.wheel_slip > 0.35 or context.vehicle_stability < 0.45:
            return LongitudinalAction.THROTTLE_25
        if context.speed_error <= 0.0:
            return LongitudinalAction.COAST

        previous = context.previous_action
        if previous in ("THROTTLE_25", "THROTTLE_40", "THROTTLE_60", "THROTTLE_80"):
            if limit >= 1.0 and previous == "THROTTLE_80":
                return LongitudinalAction.THROTTLE_100
            if limit >= 0.80 and previous == "THROTTLE_60":
                return LongitudinalAction.THROTTLE_80
            if previous == "THROTTLE_40":
                return LongitudinalAction.THROTTLE_60
            return LongitudinalAction.THROTTLE_40

        if context.speed_error > 45.0 and limit >= 1.0:
            return LongitudinalAction.THROTTLE_100
        if context.speed_error > 25.0 and limit >= 0.80:
            return LongitudinalAction.THROTTLE_80
        if context.speed_error > 12.0:
            return LongitudinalAction.THROTTLE_60
        return LongitudinalAction.THROTTLE_40

    def _gear_action(self, context: DrivingContext) -> GearAction:
        if context.current_gear < context.target_gear and context.speed_error > 5.0:
            return GearAction.GEAR_UP
        if context.current_gear > context.target_gear and context.speed_error < 0.0:
            return GearAction.GEAR_DOWN
        return GearAction.GEAR_HOLD

