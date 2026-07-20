"""
This module converts raw TORCS sensor data intoma high level 
description of the road ahead.

Responsibilities (Version 1):
- Receive raw track sensor data
- Store road geometry information
- Provide a clean interface for the rest of the controller

NOTE:
No steering, braking, gear selection or corner classification
belongs in this module.
"""
from dataclasses import dataclass
from typing import List 

@dataclass
class RoadGeometry:

    direction: str="unknown"
    curvature: float=0.0
    forward_visibility: float=0.0
    left_opening: float=0.0
    right_opening: float=0.0
    track_width: float=0.0
    confidence: float=0.0

class RoadPerception:

    """
    Converts raw TORCS sensor readings into RoadGeometry.
    """

    def __init__(self):

        self.geometry = RoadGeometry()

    def update(
        self,
        track: List[float],
        track_pos: float,
        angle: float,
        speed_x: float,
        gear: int,
        rpm: float,
        steering_angle: float,):

  
        self.geometry
        self.vehicle_state

        return self.geometry,self.vehicle_state