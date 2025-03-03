import math

import pytest

from roast import RobotProfile
from roast.vision.pose_estimator import DepthEstimator, PoseFilter


def map_pixel_to_angle(image_width):
    """Map the pixel to the angle."""
    pixel_to_angle = 0.1875

    output = []
    for i in range(0, image_width, 120):
        result = math.fmod(
            (image_width - i) * pixel_to_angle + RobotProfile.CAMERA_OFFSET - 90, 360
        )

        # Change always to anti-clockwise and from 0 to 360
        result = math.fmod(result if result > 0 else result + 360, 360)
        output.append(({"xcenter": i}, result))

    return output


@pytest.mark.skip("Not implemented")
class TestDepthEstimator:
    @pytest.mark.parametrize("bbox, output", map_pixel_to_angle(image_width=480))
    def test_depth_estimator_single(self, bbox, output):
        """Test the depth estimator."""
        tmp = RobotProfile.OAKD_ARRAY_LIST
        RobotProfile.OAKD_ARRAY_LIST = ["front_oakd_camera"]
        estimator = DepthEstimator()

        result = estimator.estimate_angle(bbox)
        assert result == output, f"Expected {output}, got {result} for {bbox}"
        RobotProfile.OAKD_ARRAY_LIST = tmp

    @pytest.mark.parametrize("bbox, output", map_pixel_to_angle(image_width=480 * 2))
    def test_depth_estimator_double(self, bbox, output):
        """Test the depth estimator."""
        tmp = RobotProfile.OAKD_ARRAY_LIST
        RobotProfile.OAKD_ARRAY_LIST = ["front_oakd_camera", "left_oakd_camera"]
        estimator = DepthEstimator()

        result = estimator.estimate_angle(bbox)
        assert result == output, f"Expected {output}, got {result} for {bbox}"

        RobotProfile.OAKD_ARRAY_LIST = tmp

    @pytest.mark.parametrize("bbox, output", map_pixel_to_angle(image_width=480 * 2))
    def test_depth_estimator_double_2(self, bbox, output):
        """Test the depth estimator."""
        tmp = RobotProfile.OAKD_ARRAY_LIST
        RobotProfile.OAKD_ARRAY_LIST = ["front_oakd_camera", "right_oakd_camera"]
        estimator = DepthEstimator()

        result = estimator.estimate_angle(bbox)
        assert result == output, f"Expected {output}, got {result} for {bbox}"

        RobotProfile.OAKD_ARRAY_LIST = tmp

    @pytest.mark.parametrize("bbox, output", map_pixel_to_angle(image_width=480 * 3))
    def test_depth_estimator_triple(self, bbox, output):
        """Test the depth estimator."""
        tmp = RobotProfile.OAKD_ARRAY_LIST
        RobotProfile.OAKD_ARRAY_LIST = [
            "front_oakd_camera",
            "left_oakd_camera",
            "right_oakd_camera",
        ]
        estimator = DepthEstimator()

        result = estimator.estimate_angle(bbox)
        assert result == output, f"Expected {output}, got {result} for {bbox}"

        RobotProfile.OAKD_ARRAY_LIST = tmp

    @pytest.mark.parametrize("bbox, output", map_pixel_to_angle(image_width=480 * 4))
    def test_depth_estimator_fourth(self, bbox, output):
        """Test the depth estimator."""
        tmp = RobotProfile.OAKD_ARRAY_LIST
        RobotProfile.OAKD_ARRAY_LIST = [
            "front_oakd_camera",
            "left_oakd_camera",
            "right_oakd_camera",
            "back_oakd_camera",
        ]
        estimator = DepthEstimator()

        result = estimator.estimate_angle(bbox)
        assert result == output, f"Expected {output}, got {result} for {bbox}"

        RobotProfile.OAKD_ARRAY_LIST = tmp


class TestPoseFilter:
    @pytest.mark.parametrize("filter_size", range(1, 10))
    def test_pose_filter(self, filter_size):
        f = PoseFilter(filter_size=filter_size)

        inputs = [1] * 100

        for i in inputs:
            f.update(i)
            assert f.get() == 1, f"Expected {1}, got {f.get()}"
        assert f.get() == i, f"Expected {i}, got {f.get()}"

    @pytest.mark.parametrize("filter_size", range(1, 10))
    def test_pose_filter_2(self, filter_size):
        f = PoseFilter(filter_size=filter_size)

        inputs = [1] * 50 + [2] * 50

        for i in inputs:
            f.update(i)
            assert 1 <= f.get() <= 2, f"Expected {1} < {f.get()} < {2}"

        assert f.get() == 2, f"Expected {2}, got {f.get()}"

    @pytest.mark.parametrize("filter_size", range(1, 10))
    def test_pose_filter_3(self, filter_size):
        f = PoseFilter(filter_size=filter_size)

        inputs = [1] * 50 + [2] * 10 + [3] * 50

        for i in inputs:
            f.update(i)
            assert 1 <= f.get() <= 3, f"Expected {1} < {f.get()} < {3}"

        assert f.get() == 3, f"Expected {3}, got {f.get()}"
