from dataclasses import dataclass

@dataclass
class  VehicleState:
    track_pos: float=0.0
    steering_angle: float=0.0
    speed_x: float=0.0
    gear: int=1
    rpm: float=0.0
    steering: float=0.0

