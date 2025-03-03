"""2D Visualization plotter for skid-steering robot
"""
import math

import matplotlib.pyplot as plt
import numpy as np
from matplotlib import patches

################### Initialize Constants ##################
# Set the robot vehicle's dimensions
# Robot is 2 feet in width and 4 feet in length
ROBOT_WIDTH = 2.0
ROBOT_LENGTH = 4.0
MAX_SPEED = 15.0
WHEEL_RADIUS = 0.5

# Length and width of the viewable area on screen
# The grid is 30 feet long by 15 feet wide
ENV_LENGTH = 30.0
ENV_WIDTH = 15.0

# The distance between ticks on the x and y-axis in feet
# Each square on the grid is 6inch by 6inch
X_TICK_SPACING = 1.0
Y_TICK_SPACING = 1.0

# Boundary in feet. Viewable area will stay fixed until
# such time as the vehicle reference point travels within
# 3 feet of the viewable edge.
BOUNDARY = 3.0

################### Global Variables ######################
# The robot vehicle object (which will be a rectangle)


class Plotter:
    """2D Plotter object which uses matplotlib"""

    # Initiate the simulation environment (i.e. grid)
    plt.ion()  # Turn on interactive mode
    fig = plt.figure()  # Create a new blank canvas
    plt.xlabel("X-axis")
    plt.ylabel("Y-axis")
    plt.axis("equal")
    plt.title("Simulation Environment")
    ax = fig.gca()  # Get the current axes

    rect = None

    # The grid specifications
    x_axis_min = None
    x_axis_max = None
    y_axis_min = None
    y_axis_max = None

    def is_close_to_edge(self, x, y):
        """
        The viewable area will stay fixed until such time as
        the vehicle reference point travels within 3 feet of
        the viewable edge. Check if the viewable area needs to
        be repositioned.
        @param x: x-coordinate of the vehicle reference point
        @param y: y-coordinate of the vehicle reference point
        @return bool
        """
        if (x - self.x_axis_max) > -BOUNDARY:
            return True
        if (x - self.x_axis_min) < BOUNDARY:
            return True
        if (y - self.y_axis_max) > -BOUNDARY:
            return True
        if (y - self.y_axis_min) < BOUNDARY:
            return True

        return False

    def plot_arrow(self, x, y, orientation):
        """
        Plot the arrow on the top of the robot. Arrow points
        to +y-direction of the robot (i.e. towards the front
        center part of the robot). It is centered on the
        vehicle reference point.
        @param x: x-coordinate of the vehicle reference point
        @param y: y-coordinate of the vehicle reference point
        @param orientation: orientation of the vehicle
            reference point in radians
        """
        # Clear datapoints if they exist
        try:
            for datapoints in self.ax.get_lines():
                datapoints.remove()
        except Exception:
            pass

        # Plot the arrow
        plt.plot(
            x, y, marker=(3, 0, math.degrees(orientation)), markersize=30, color="black"
        )

    def plot_grid(self, x, y):
        """
        Plot the grid.
        @param x: x-coordinate of the center of the grid.
        @param y: y-coordinate of the center of the grid
        """
        # Set the x and y limits of the grid.
        self.x_axis_max = x + (ENV_WIDTH / 2.0) + X_TICK_SPACING
        self.x_axis_min = x - (ENV_WIDTH / 2.0)
        self.y_axis_max = y + (ENV_LENGTH / 2.0) + Y_TICK_SPACING
        self.y_axis_min = y - (ENV_LENGTH / 2.0)
        self.ax.set(
            xlim=(self.x_axis_min, self.x_axis_max),
            ylim=(self.y_axis_min, self.y_axis_max),
        )

        # Each square on the grid is 6inch by
        # 6inch (6inch = 0.5 feet)
        self.ax.set_xticks(np.arange(self.x_axis_min, self.x_axis_max, X_TICK_SPACING))
        self.ax.set_yticks(np.arange(self.y_axis_min, self.y_axis_max, Y_TICK_SPACING))
        self.ax.grid(True)

        self.turn_off_tick_labels()

    @classmethod
    def plot_line(cls, x1, y1, direction):
        """
        Show the user defined path as a red line
        @param x1: x-coordinate of the start point of the line
        @param y1: y-coordinate of the start point of the line
        @direction: Direction of travel of the vehicle
            reference point in radians
        """
        # Arbitrary-chosen line length
        line_length = 2.0 * math.sqrt(2.0)

        x2 = line_length * math.cos(direction)
        x2 = x1 + x2
        y2 = line_length * math.sin(direction)
        y2 = y1 + y2

        plt.plot([x1, x2], [y1, y2], color="red", linestyle="-", linewidth=2)

    @classmethod
    def plot_path_traveled(cls, x_values, y_values):
        """
        Show the path traveled by the robot.
        @param x_values list: List of x values
        @param y_values list: List of y values
        """
        plt.plot(x_values, y_values, color="green", linestyle="-", linewidth=2)

    def plot_robot(self, x, y, orientation):
        """
        Plot the robot on the grid.
        Rotate lower left coordinate of the robot based on
        vehicle reference point's orientation.
        This equation gives the lower left coordinate's new
            position when rotated around the origin (x=0,y=0):
            X = x*cos(θ) - y*sin(θ)
            Y = x*sin(θ) + y*cos(θ)

        :                +------------------+
        :                |                  |
        :              height               |
        :                |                  |
        :               (xy)---- width -----+

        @param x: x-coordinate of the vehicle reference point
        @param y: y-coordinate of the vehicle reference point
        @param orientation: orientation of the vehicle
            reference point in radians
        """

        # Remove the existing rectangle if it exists
        try:
            self.rect.remove()
        except Exception:
            pass

        rect_x_pos = ((-ROBOT_WIDTH / 2.0) * math.cos(orientation)) - (
            (-ROBOT_LENGTH / 2.0) * math.sin(orientation)
        )
        rect_y_pos = ((-ROBOT_WIDTH / 2.0) * math.sin(orientation)) + (
            (-ROBOT_LENGTH / 2.0) * math.cos(orientation)
        )

        # Translate lower left coordinate of the robot so it
        #   is relative to the vehicle reference point
        rect_x_pos = rect_x_pos + x
        rect_y_pos = rect_y_pos + y

        # Update the robot's position and orientation
        self.rect = patches.Rectangle(
            (rect_x_pos, rect_y_pos),
            width=ROBOT_WIDTH,
            height=ROBOT_LENGTH,
            angle=math.degrees(orientation),
            lw=1,
            ec="black",
            fc="orange",
        )

        # Add the rectangle to the Axes
        self.ax.add_patch(self.rect)

    def turn_off_tick_labels(self):
        """
        Turn off the tick labels if desired.
        """
        self.ax.set_yticklabels([])
        self.ax.set_xticklabels([])

    @classmethod
    def plot_path(cls, x: list, y: list):
        """Plot path to be travelled

        Args:
            x (list): List containing initial and final x-coordinate
            y (list): List containing initial and final y-coordinate
        """

        plt.plot(
            x,
            y,
            color="red",
            linestyle="-",
            linewidth=2,
        )
