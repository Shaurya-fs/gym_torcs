"""
autom_agent.py

Bridge between Gym-TORCS and the modular RacingController.
"""

from controller.controller import RacingController
from controller.control_command import ControlCommand


class AutonomousAgent:
    def __init__(self):
        self.controller = RacingController()
        self.previous_command = ControlCommand()

    def act(self, ob, reward, done, vision_on=False):
        """Convert a Gym-TORCS observation into a controller command."""

        sensor_data = {
            "track": ob.track,
            "angle": ob.angle,
            "trackPos": ob.trackPos,
            "speedX": ob.speedX,
            "rpm": ob.rpm,
            # Gym observation does not expose gear. Use the previously commanded gear.
            "gear": self.previous_command.gear,
        }

        command = self.controller.update(
            sensor_data=sensor_data,
            previous_command=self.previous_command,
        )

        self.previous_command = command

        return [
            command.steering,
            command.acceleration,
            command.gear,
        ]