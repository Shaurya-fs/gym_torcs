import math
import os
import time


DEBUG = os.environ.get("TORCS_DEBUG", "0") == "1"

UNITS = {
    "speedX": "km/h",
    "speedY": "km/h",
    "speedZ": "km/h",
    "rpm": "rev/min",
    "gear": "integer [-1, 0, 1..6]",
    "angle": "rad",
    "trackPos": "normalized lane offset, roughly [-1, 1] on track",
    "track": "m",
    "wheelSpinVel": "rad/s",
    "fuel": "l",
    "damage": "TORCS damage units",
    "curLapTime": "s",
    "distFromStart": "m",
    "distRaced": "m",
}


def now_timestamp() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%S%z")


def is_finite(value) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def finite_float(value, default=0.0) -> float:
    try:
        value = float(value)
    except (TypeError, ValueError):
        return default
    return value if math.isfinite(value) else default


def validate_scalar(name: str, value, warnings: list[str]) -> None:
    if not is_finite(value):
        warnings.append(f"{name} is not finite: {value!r}")


def validate_sensor_frame(sensor_data: dict, source: str = "sensor") -> list[str]:
    warnings: list[str] = []

    for name in ("speedX", "speedY", "speedZ", "rpm", "gear", "angle", "trackPos", "fuel", "damage"):
        if name in sensor_data:
            validate_scalar(f"{source}.{name}", sensor_data[name], warnings)

    if finite_float(sensor_data.get("speedX", 0.0)) < 0:
        warnings.append(f"{source}.speedX is negative: {sensor_data.get('speedX')!r}")
    if finite_float(sensor_data.get("rpm", 0.0)) < 0:
        warnings.append(f"{source}.rpm is negative: {sensor_data.get('rpm')!r}")

    gear = int(finite_float(sensor_data.get("gear", 1), 1))
    if gear not in (-1, 0, 1, 2, 3, 4, 5, 6):
        warnings.append(f"{source}.gear outside valid range: {gear!r}")

    track = sensor_data.get("track")
    if track is not None:
        if len(track) != 19:
            warnings.append(f"{source}.track expected 19 sensors, got {len(track)}")
        for idx, value in enumerate(track):
            if not is_finite(value):
                warnings.append(f"{source}.track[{idx}] is not finite: {value!r}")
            elif not 0.0 <= float(value) <= 200.0:
                warnings.append(f"{source}.track[{idx}] outside [0, 200] m: {value!r}")

    wheel_spin = sensor_data.get("wheelSpinVel")
    if wheel_spin is not None:
        if len(wheel_spin) != 4:
            warnings.append(f"{source}.wheelSpinVel expected 4 values, got {len(wheel_spin)}")
        for idx, value in enumerate(wheel_spin):
            if not is_finite(value):
                warnings.append(f"{source}.wheelSpinVel[{idx}] is not finite: {value!r}")

    return warnings


def validate_command(command: dict, source: str = "command") -> list[str]:
    warnings: list[str] = []
    for name in ("steer", "accel", "brake", "gear"):
        if name in command:
            validate_scalar(f"{source}.{name}", command[name], warnings)

    steer = finite_float(command.get("steer", 0.0))
    accel = finite_float(command.get("accel", 0.0))
    brake = finite_float(command.get("brake", 0.0))
    gear = int(finite_float(command.get("gear", 1), 1))

    if not -1.0 <= steer <= 1.0:
        warnings.append(f"{source}.steer outside [-1, 1]: {steer!r}")
    if not 0.0 <= accel <= 1.0:
        warnings.append(f"{source}.accel outside [0, 1]: {accel!r}")
    if not 0.0 <= brake <= 1.0:
        warnings.append(f"{source}.brake outside [0, 1]: {brake!r}")
    if gear not in (-1, 0, 1, 2, 3, 4, 5, 6):
        warnings.append(f"{source}.gear outside valid range: {gear!r}")
    if accel > 0.9 and brake > 0.9:
        warnings.append(f"{source}.accel and {source}.brake both > 0.9")

    return warnings


def print_warnings(warnings: list[str]) -> None:
    for warning in warnings:
        print(f"[VALIDATION WARNING] {warning}")


def compact_frame_report(context: dict, packet: dict | None = None) -> None:
    if not DEBUG:
        return

    sensor = context.get("sensor_data", {})
    road = context.get("road", {})
    corner = context.get("corner", {})
    plan = context.get("plan", {})
    command = context.get("command", {})
    frame = context.get("frame", "?")

    print(f"\nFRAME {frame}")
    print(
        "RAW "
        f"speed={sensor.get('speedX')} km/h "
        f"gear={sensor.get('gear')} rpm={sensor.get('rpm')} "
        f"trackPos={sensor.get('trackPos')}"
    )
    print(
        "ROAD "
        f"corner={corner.get('corner_type')} "
        f"visibility={road.get('forward_visibility')} m "
        f"curvature={road.get('signed_curvature')}"
    )
    print(
        "PLAN "
        f"target_speed={plan.get('target_speed')} km/h "
        f"target_steering_gain={plan.get('steering_gain')} "
        f"target_lane={plan.get('target_track_pos')}"
    )
    print(
        "CONTROL "
        f"steering={command.get('steering')} throttle={command.get('acceleration')} "
        f"brake={command.get('brake')} gear={command.get('gear')} "
        f"recovery={context.get('recovery_active')}"
    )
    if packet is not None:
        print(
            "PACKET "
            f"steer={packet.get('steer')} accel={packet.get('accel')} "
            f"brake={packet.get('brake')} gear={packet.get('gear')} meta={packet.get('meta')}"
        )
