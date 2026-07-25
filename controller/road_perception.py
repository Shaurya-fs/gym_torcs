from dataclasses import dataclass
from typing import List
from enum import Enum


"""
road_perception.py

Converts raw TORCS track sensor data into a high-level description
of the road ahead.

This module only understands the road. It does not store vehicle state,
classify corners, plan speed, steer, brake, or change gears.
"""
class RoadDirection(Enum):
    STRAIGHT = "STRAIGHT"
    LEFT = "LEFT"
    RIGHT = "RIGHT"
    UNKNOWN = "UNKNOWN"


@dataclass
class RoadGeometry:
    direction: RoadDirection = RoadDirection.UNKNOWN
    curvature: float = 0.0
    forward_visibility: float = 0.0
    left_opening: float = 0.0
    right_opening: float = 0.0
    track_width: float = 0.0



class RoadPerception:
    """
    Converts raw TORCS sensor readings into RoadGeometry.
    """

    def __init__(self):
        self.geometry = RoadGeometry()

    def update(self, track: List[float], angle: float) -> RoadGeometry:
        """
        Update the road geometry using TORCS road sensors.

        Parameters
        ----------
        track:
            The 19 TORCS track sensor readings.

        angle:
            Car heading angle relative to the track axis.
        """

        return self.geometry
