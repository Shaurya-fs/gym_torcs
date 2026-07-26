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
    """Converts raw TORCS sensor readings into RoadGeometry."""

    def __init__(self):
        self.left_weights = [0.2, 0.3, 0.4, 0.6, 0.8, 1.0, 1.3, 1.6, 2.0]
        self.right_weights = [2.0, 1.6, 1.3, 1.0, 0.8, 0.6, 0.4, 0.3, 0.2]

        self.left_weight_sum = sum(self.left_weights)
        self.right_weight_sum = sum(self.right_weights)

    def update(self, track: List[float], angle: float) -> RoadGeometry:
        if len(track) != 19:
            raise ValueError("Expected 19 TORCS track sensor readings.")

        forward_visibility = track[9]

        left_sum = 0.0
        for sensor, weight in zip(track[:9], self.left_weights):
            left_sum += sensor * weight
        left_opening = left_sum / self.left_weight_sum

        right_sum = 0.0
        for sensor, weight in zip(track[10:], self.right_weights):
            right_sum += sensor * weight
        right_opening = right_sum / self.right_weight_sum

        track_width = max(track)

        threshold = 2.0
        difference = left_opening - right_opening

        if difference > threshold:
            direction = RoadDirection.LEFT
        elif difference < -threshold:
            direction = RoadDirection.RIGHT
        else:
            direction = RoadDirection.STRAIGHT

        curvature = abs(difference)

        return RoadGeometry(
            direction=direction,
            curvature=curvature,
            forward_visibility=forward_visibility,
            left_opening=left_opening,
            right_opening=right_opening,
            track_width=track_width,
        )
