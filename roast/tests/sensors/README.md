# Handling test cases

The test cases are for the module's functionality.

## Pre-requisites

- pytest
- pytest-cov
- unittest

## Test cases

The test cases are for the module's functionality. The available test cases for the module `sensors` are:

- test_imu
- test_range
- test_motor_drive
- test_battery
- test_all_sensors

## Usage

We can simply run the following command:

If you want to run the test cases without the hardware, in project's root directory, you can use the following command:

```
TEST_HARDWARE=false pytest -v
```

or with hardware:

```
TEST_HARDWARE=true pytest -v
```

One can test individual sensors by running the following command:

Example:
```
TEST_HARDWARE=false pytest -v -k test_imu
```
