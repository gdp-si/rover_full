"""Plotting library for the roast project"""
import matplotlib.pyplot as plt


def plot_waypoints(waypoints: list):
    """Plot 2d figure of the waypoints given the format (x, y)"""

    # Check if the waypoints are in tuples(x, y) format
    for waypoint in waypoints:
        if not isinstance(waypoint, tuple):
            raise ValueError("Waypoints are not in the format of (x, y)")
        if not len(waypoint) == 2:
            raise ValueError("Waypoints are not in the format of (x, y)")

    plt.plot(
        [waypoint[0] for waypoint in waypoints],
        [waypoint[1] for waypoint in waypoints],
        "r*",
    )
    plt.show()
