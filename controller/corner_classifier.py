from .road_perception import RoadGeometry, RoadDirection
from .vehicle_state import VehicleState
from .corner_profile import CornerProfile, CornerType


class CornerClassifier:
 

    # ---------------------------------------------------------
    # Corner classification thresholds (degrees)
    # ---------------------------------------------------------

    STRAIGHT_MAX = 10.0
    GENTLE_MAX = 25.0
    MEDIUM_MAX = 45.0
    SHARP_MAX = 70.0
    HAIRPIN_MAX = 180.0

    # ---------------------------------------------------------
    # Turn-angle estimation weights
    # (Easy to tune after testing)
    # ---------------------------------------------------------

    CURVATURE_WEIGHT = 70.0
    VISIBILITY_WEIGHT = 25.0
    OPENING_WEIGHT = 15.0

    # ---------------------------------------------------------
    # Severity weights
    # ---------------------------------------------------------

    ANGLE_WEIGHT = 0.45
    SPEED_WEIGHT = 0.30
    POSITION_WEIGHT = 0.15
    VISIBILITY_SEVERITY_WEIGHT = 0.10

    # ---------------------------------------------------------

    def classify(
        self,
        road: RoadGeometry,
        vehicle: VehicleState,
    ) -> CornerProfile:

        angle = self._estimate_turn_angle(road)

        corner = self._classify_corner_type(angle)

        severity = self._calculate_severity(
            road,
            vehicle,
            angle,
        )

        return CornerProfile(
            corner_type=corner,
            turn_angle=angle,
            severity=severity,
        )

    # =========================================================

    def _estimate_turn_angle(self, road: RoadGeometry) -> float:
        """
        Estimate turn angle (0-180°).

        Uses:
            • road curvature
            • forward visibility
            • left/right opening

        Direction is NOT encoded here.
        RoadGeometry.direction already stores LEFT / RIGHT.
        """

        angle = 0.0

        # -------------------------------------------------
        # 1. Curvature contribution
        # -------------------------------------------------

        angle += abs(road.curvature) * self.CURVATURE_WEIGHT

        # -------------------------------------------------
        # 2. Visibility contribution
        # -------------------------------------------------

        if road.forward_visibility < 80:
            visibility_score = (
                (80 - road.forward_visibility) / 80
            ) * self.VISIBILITY_WEIGHT

            angle += max(0.0, visibility_score)

        # -------------------------------------------------
        # 3. Opening contribution
        # -------------------------------------------------

        if road.track_width > 0:

            opening_difference = abs(
                road.left_opening
                - road.right_opening
            )

            opening_ratio = min(
                opening_difference / road.track_width,
                1.0,
            )

            angle += (
                opening_ratio
                * self.OPENING_WEIGHT
            )

        return min(angle, self.HAIRPIN_MAX)

    # =========================================================

    def _classify_corner_type(
        self,
        angle: float,
    ) -> CornerType:

        if angle <= self.STRAIGHT_MAX:
            return CornerType.STRAIGHT

        if angle <= self.GENTLE_MAX:
            return CornerType.GENTLE

        if angle <= self.MEDIUM_MAX:
            return CornerType.MEDIUM

        if angle <= self.SHARP_MAX:
            return CornerType.SHARP

        return CornerType.HAIRPIN

    # =========================================================

    def _calculate_severity(
        self,
        road: RoadGeometry,
        vehicle: VehicleState,
        angle: float,
    ) -> float:

        severity = 0.0

      
        # Angle

        angle_score = min(
            angle / self.HAIRPIN_MAX,
            1.0,
        )

        severity += (
            angle_score
            * self.ANGLE_WEIGHT
        )

       
        # Vehicle speed

        speed = abs(vehicle.speed_x)

        speed_score = min(
            speed / 150.0,
            1.0,
        )

        severity += (
            speed_score
            * self.SPEED_WEIGHT
        )

      
        # Track position

        position_score = min(
            abs(vehicle.track_pos),
            1.0,
        )

        severity += (
            position_score
            * self.POSITION_WEIGHT
        )

      
        # Visibility
        
        if road.forward_visibility < 80:

            visibility_score = (
                (80 - road.forward_visibility)
                / 80
            )

            severity += (
                visibility_score
                * self.VISIBILITY_SEVERITY_WEIGHT
            )

        return min(max(severity, 0.0), 1.0)
