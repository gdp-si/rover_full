import math

import pytest

from roast.vision import ThreatTracking


def create_points_of_circle(radius_1, radius_2, num_points):
    """Create x, y points on circle."""
    points = []
    for i in range(num_points):
        x1 = radius_1 * math.cos(2 * math.pi * i / num_points)
        y1 = radius_1 * math.sin(2 * math.pi * i / num_points)
        x2 = radius_2 * math.cos(2 * math.pi * i / num_points)
        y2 = radius_2 * math.sin(2 * math.pi * i / num_points)
        points.append(((x1, y1), (x2, y2)))

    return points


@pytest.mark.skip("Not implemented")
class TestThreatTracker:
    @pytest.mark.parametrize(
        "data",
        create_points_of_circle(2, 1, 50),
        ids=["point_{}".format(i) for i in range(50)],
    )
    def test_safe_points(self, data):
        """Test the safe points."""
        tracker = ThreatTracking()
        tracker._safety_perimeter_radius = 1
        actual_data = tracker.get_safety_coordinate({"x": data[0][0], "y": data[0][1]})

        assert actual_data["x"] == pytest.approx(data[1][0], abs=0.1)
        assert actual_data["y"] == pytest.approx(data[1][1], abs=0.1)
