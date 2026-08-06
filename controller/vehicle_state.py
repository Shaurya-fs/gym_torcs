from dataclasses import dataclass


@dataclass
class VehicleState:
    speed_x: float = 0.0
    speed_y: float = 0.0
    speed_z: float = 0.0
    track_pos: float = 0.0
    steering_angle: float = 0.0
    steering: float = 0.0
    gear: int = 1
    rpm: float = 0.0
    wheel_spin_vel: list[float] = None
    wheel_slip: float = 0.0

class VehiclePerception:
    def update(
        self,
        track_pos: float,
        steering_angle: float,
        speed_x: float,
        gear: int,
        rpm: float,
        steering: float,
        speed_y: float = 0.0,
        speed_z: float = 0.0,
        wheel_spin_vel: list[float] | None = None,
     ) -> VehicleState:
        wheel_spin_vel = (
            wheel_spin_vel
            if wheel_spin_vel is not None
            else [0.0, 0.0, 0.0, 0.0]
    )
        speed_x = float(speed_x)
        speed_y = float(speed_y)
        speed_z = float(speed_z)

        front_spin = (abs(wheel_spin_vel[0]) + abs(wheel_spin_vel[1])) * 0.5
        rear_spin = (abs(wheel_spin_vel[2]) + abs(wheel_spin_vel[3])) * 0.5
        wheel_slip = 0.0
        if abs(speed_x) > 8.0 and front_spin > 1.0:
            wheel_slip = max(0.0, min(1.0, (rear_spin - front_spin) / front_spin))

        return VehicleState(
            track_pos=track_pos,
            steering_angle=steering_angle,
            speed_x=speed_x,
            speed_y=speed_y,
            speed_z=speed_z,
            gear=gear,
            rpm=rpm,
            steering=steering,
            wheel_spin_vel=wheel_spin_vel,
            wheel_slip=wheel_slip,
        )
