from dataclasses import dataclass


@dataclass
class ControlCommand:
    """
    Represents a control command for the vehicle.
    This is the interface between the planning layer and the control layer.
    """
    steering: float = 0.0  # Steering angle [-1.0, 1.0]
    acceleration: float = 0.0  # Throttle/acceleration [0.0, 1.0]
    brake: float = 0.0  # Brake force [0.0, 1.0]
    gear: int = 1  # Gear selection [1-6, -1 for reverse]
    
    def __post_init__(self):
        """Validate control command values."""
        self.steering = max(-1.0, min(1.0, self.steering))
        self.acceleration = max(0.0, min(1.0, self.acceleration))
        self.brake = max(0.0, min(1.0, self.brake))
        self.gear = max(-1, min(6, self.gear))
