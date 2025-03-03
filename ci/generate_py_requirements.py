#!/usr/bin/python3
"""Parse pyproject.toml file and compile the requiements.txt for dev dependencies and normal dependencies."""
import argparse

import toml

args = argparse.ArgumentParser()
args.add_argument("--pyproject", type=str, default="pyproject.toml")
args.add_argument("--requirements", type=str, default="requirements.txt")

args = args.parse_args()


def get_requirements(pyproject_path: str = "pyproject.toml"):
    """Get the requirements from pyproject.toml file."""
    with open(pyproject_path, "r") as file:
        data = toml.load(file)
    return (
        data["build-system"]["requires"]
        + data["project"]["dependencies"]
        + data["project"]["optional-dependencies"]["dev"]
    )


def write_requirements(requirements_path: str = "requirements.txt"):
    """Write the requirements to requirements.txt file."""
    with open(requirements_path, "w") as file:
        file.write("\n".join(get_requirements()))


get_requirements(args.pyproject)
write_requirements(args.requirements)
