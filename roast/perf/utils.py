"""Util functions for performance testing."""
try:
    import psutil  # pylint: disable=import-error
except ImportError:
    raise Warning("psutil not installed. Please install psutil to use this tool.")


def get_python_process_list():
    """Get the python process list."""
    pid = []
    for p in psutil.process_iter():
        if "python" in p.name().lower():
            name = p.cmdline()[1].split("/")[-1]
            pid.append((p.pid, name))
    return pid


def get_ros2_process_list():
    """Get the ros2 process list."""
    pid = []
    for p in psutil.process_iter():
        cmd_line = p.cmdline()
        if cmd_line == []:
            continue

        # Assumes the path is either from /opt/ros/<version>/bin or /ros_ws/install/bin
        # This is a ugly way to do this, but it works for now.
        if "ros" in cmd_line[0].lower():
            name = p.cmdline()[0].split("/")[-1]
            pid.append((p.pid, name))

        if len(cmd_line) > 1:
            if "python" not in cmd_line[0].lower():
                continue
            if "ros" in cmd_line[1].lower() or "roast" in cmd_line[1].lower():
                name = cmd_line[1].split("/")[-1]
                pid.append((p.pid, name))

    return pid
