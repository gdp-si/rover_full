"""Utility functions to filter out unwanted content from the sensor data."""

import numpy as np


def filter_outliers(data, window_size):
    """Filter out outliers from the data using a median filter.

    data: list of data points
    window_size: size of the window to use for filtering

    Returns a list of data points with outliers removed.

    Example:
    filter_outliers([1, 2, 3, 4, 5, 6, 7, 8, 9, 10], 3)
    [2, 3, 4, 5, 6, 7, 8, 9]
    """
    filtered_data = []
    if len(data) < window_size:
        return data

    for i in range(len(data) - window_size + 1):
        window = data[i : i + window_size]
        median = np.median(window)
        deviation = np.median(np.abs(window - median))
        threshold = 3.0 * deviation
        outliers = [x for x in window if abs(x - median) > threshold]
        if len(outliers) == window_size:
            # If all data points in the window are outliers, remove all but the last one
            filtered_data.append(window[-1])
        else:
            # Otherwise, remove the outliers and take the median of the remaining data points
            filtered_window = [x for x in window if abs(x - median) <= threshold]
            filtered_data.append(np.median(filtered_window))

    return filtered_data
