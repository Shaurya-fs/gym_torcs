from dataclasses import dataclass


@dataclass
class VehicleState:
    speed_x: float = 0.0
    track_pos: float = 0.0
    steering_angle: float = 0.0
    steering: float = 0.0
    gear: int = 1
    rpm: float = 0.0

class VehiclePerception:
    def update(self, track_pos: float, steering_angle: float, speed_x: float, gear: int, rpm: float, steering: float) -> VehicleState:
        return VehicleState(
            track_pos=track_pos,
            steering_angle=steering_angle,
            speed_x=speed_x,
            gear=gear,
            rpm=rpm,
            steering=steering,
            )