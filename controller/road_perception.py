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
    signed_curvature: float = 0.0
    forward_visibility: float = 0.0
    left_opening: float = 0.0
    right_opening: float = 0.0
    track_width: float = 0.0
    near_bias: float = 0.0
    far_bias: float = 0.0
    left_near: float = 0.0
    right_near: float = 0.0
    left_far: float = 0.0
    right_far: float = 0.0
    is_chicane: bool = False
    is_exit: bool = False
    confidence: float = 0.0


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

        clean_track = [max(0.0, min(200.0, float(sensor))) for sensor in track]

        forward_visibility = clean_track[9]

        left_sum = 0.0
        for sensor, weight in zip(clean_track[:9], self.left_weights):
            left_sum += sensor * weight
        left_opening = left_sum / self.left_weight_sum

        right_sum = 0.0
        for sensor, weight in zip(clean_track[10:], self.right_weights):
            right_sum += sensor * weight
        right_opening = right_sum / self.right_weight_sum

        left_near = sum(clean_track[5:9]) / 4.0
        right_near = sum(clean_track[10:14]) / 4.0
        left_far = sum(clean_track[0:5]) / 5.0
        right_far = sum(clean_track[14:19]) / 5.0

        track_width = max(clean_track)

        near_bias = (left_near - right_near) / max(left_near + right_near, 1.0)
        far_bias = (left_far - right_far) / max(left_far + right_far, 1.0)
        opening_bias = (left_opening - right_opening) / max(left_opening + right_opening, 1.0)

        signed_curvature = 0.55 * near_bias + 0.30 * opening_bias + 0.15 * far_bias
        signed_curvature += max(-0.20, min(0.20, float(angle))) * 0.12

        if signed_curvature > 0.04:
            direction = RoadDirection.LEFT
        elif signed_curvature < -0.04:
            direction = RoadDirection.RIGHT
        else:
            direction = RoadDirection.STRAIGHT

        curvature = abs(signed_curvature)
        is_chicane = near_bias * far_bias < -0.015 and abs(far_bias) > 0.08
        is_exit = abs(near_bias) > 0.06 and abs(far_bias) < abs(near_bias) * 0.45
        confidence = min(1.0, forward_visibility / 120.0)

        return RoadGeometry(
            direction=direction,
            curvature=curvature,
            signed_curvature=signed_curvature,
            forward_visibility=forward_visibility,
            left_opening=left_opening,
            right_opening=right_opening,
            track_width=track_width,
            near_bias=near_bias,
            far_bias=far_bias,
            left_near=left_near,
            right_near=right_near,
            left_far=left_far,
            right_far=right_far,
            is_chicane=is_chicane,
            is_exit=is_exit,
            confidence=confidence,
        )
