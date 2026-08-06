"""
autom_agent.py

Bridge between Gym-TORCS and the modular RacingController.
"""

from controller.controller import RacingController
from controller.control_command import ControlCommand


class ActionWithTelemetry(list):
    def __init__(self, values, telemetry_context=None):
        super().__init__(values)
        self.telemetry_context = telemetry_context or {}


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
            "speedY": ob.speedY,
            "speedZ": ob.speedZ,
            "rpm": ob.rpm,
            "wheelSpinVel": ob.wheelSpinVel,
            "gear": ob.gear,
            "fuel": ob.fuel,
            "damage": ob.damage,
            "curLapTime": getattr(ob, "curLapTime", 0.0),
            "distFromStart": getattr(ob, "distFromStart", 0.0),
            "distRaced": getattr(ob, "distRaced", 0.0),
        }

        command = self.controller.update(
            sensor_data=sensor_data,
            previous_command=self.previous_command,
        )

        self.previous_command = command

        return ActionWithTelemetry(
            [
                command.steering,
                command.acceleration,
                command.brake,
                command.gear,
            ],
            telemetry_context=self.controller.last_telemetry_context,
        )
