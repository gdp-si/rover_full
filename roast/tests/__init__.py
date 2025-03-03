"""Test cases for the path module."""
import os

import pytest


class SkipIfHardwareNotAvailable(object):
    """Skip a test if the hardware is not available."""

    def __init__(self):
        self.enviroment = os.environ.get("TEST_HARDWARE", "false")

    def __call__(self, test_fun):
        if self.enviroment.lower() == "false":
            return pytest.mark.skip(
                f"{test_fun.__name__}: Hardware tests are skipped if not informed"
            )(test_fun)

        return test_fun


class SkipIfCudaNotAvailable(object):
    """Skip a test if the CUDA is not available."""

    def __init__(self):
        self.enviroment = os.environ.get("TEST_CUDA", "false")

    def __call__(self, test_fun):
        if self.enviroment.lower() == "false":
            return pytest.mark.skip(
                f"{test_fun.__name__}: CUDA tests are skipped if not informed"
            )(test_fun)

        return test_fun
