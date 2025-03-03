"""Utility checker for ROS2 and python processes."""
import argparse

from roast.perf import UtilityChecker
from roast.perf.utils import get_python_process_list, get_ros2_process_list


def main():
    """Main function"""
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--ros2",
        help="Check ROS2 processes",
        action="store_true",
    )
    parser.add_argument(
        "--python",
        help="Check python processes",
        action="store_true",
    )

    args = parser.parse_args()

    utility = UtilityChecker()
    if args.ros2:
        utility.pid_list = get_ros2_process_list()
    elif args.python:
        utility.pid_list = get_python_process_list()
    else:
        utility.pid_list = get_python_process_list() + get_ros2_process_list()

    utility.run()


if __name__ == "__main__":
    main()
