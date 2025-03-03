"""Ultity functions to write CSV files from the data."""
from __future__ import unicode_literals

import csv
import datetime
import os
from typing import Optional

from roast.glogging import DEFAULT_LOGGER


def write_csv(
    data: Optional[list] = None,
    headers: Optional[list] = None,
    file_name: Optional[str] = None,
) -> None:
    """Write CSV file from the data."""

    if data is None:
        raise ValueError("Data is required to write CSV file.")

    if headers is not None:
        assert len(data) == len(headers), "Data and headers must have the same length."

    file_name = (
        f"{file_name.split('.')[0]}_{datetime.datetime.now().strftime('%Y%m%d%H%M%S')}.csv"
        if file_name is not None
        else f"output_{datetime.datetime.now().strftime('%Y%m%d%H%M%S')}.csv"
    )

    dirpath = os.path.join(os.path.expanduser("~"), "roast_logs")
    os.mkdir(dirpath)

    with open(os.path.join(dirpath, file_name), "w", newline="") as csv_file:
        writer = csv.writer(csv_file, delimiter=",")
        if headers is not None:
            writer.writerow(headers)
        writer.writerows(data)

    DEFAULT_LOGGER.INFO(f"CSV file {file_name} written successfully.")


def read_csv(file_name: Optional[str] = None) -> list:
    """Read CSV file and return the data."""

    if file_name is None:
        raise ValueError("File name is required to read CSV file.")

    data = []
    with open(file_name, "r") as csv_file:
        reader = csv.reader(csv_file, delimiter=",")
        for row in reader:
            data.append(row)

    DEFAULT_LOGGER.INFO(f"CSV file {file_name} read successfully.")
    return data
