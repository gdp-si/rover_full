"""Check CPU and Memory utilization for a set of processes."""
import curses
import subprocess
from typing import Dict, List, Tuple

import numpy as np

try:
    import psutil  # pylint: disable=import-error
except ImportError:
    from warnings import warn

    warn("psutil not installed. Please install psutil to use this tool.")


class UtilityChecker:
    """CPU and Memory utilization Checker."""

    _pid_list: List[Tuple[str, int]] = []
    _utility_data: Dict[str, Dict[str, List[float]]] = dict()
    _instance = None

    def __new__(cls, **kwargs):
        """Singleton patterned class."""
        if not cls._instance:
            cls._instance = super().__new__(cls, **kwargs)
        return cls._instance

    @staticmethod
    def _check_process(pid):
        """Check if thr process is still active."""
        try:
            _ = psutil.Process(pid)
            return True
        except psutil.NoSuchProcess:
            return False

    def _check_utility(self, pid):
        """Check CPU and Memory utilization for a process."""
        try:
            p = psutil.Process(pid)
            cpu = p.cpu_percent(interval=0.1)
            mem = p.memory_percent()
            cpu = round(cpu, 2)
            mem = round(mem, 2)
            gpu = self._get_gpu_usage()
            return cpu, mem, gpu
        except psutil.NoSuchProcess:
            return None, None, None

    @property
    def pid_list(self) -> List[Tuple[str, int]]:
        """Get the pid list from the input."""
        return self._pid_list

    @pid_list.setter
    def pid_list(self, pid_list):
        """Set the pid list from the input."""
        self._pid_list.extend(pid_list)

        # Remove duplicates
        self._pid_list = list(set(self._pid_list))

    def _setup_utility_data(self):
        """Setup utility data"""

        for pid in self.pid_list:
            if not self._check_process(pid[0]):
                continue

            self._utility_data[pid] = {"cpu": [], "mem": [], "gpu": []}

    def _get_gpu_usage(self):
        """Get GPU usage using nvidia-smi and psutil."""
        try:
            output = subprocess.check_output(
                ["nvidia-smi", "--query-gpu=utilization.gpu", "--format=csv"]
            )
            gpu_usage = [
                int(x.strip().split()[0])
                for x in output.decode().strip().split("\n")[1:]
            ]
            gpu_usage = np.mean(gpu_usage)
            return gpu_usage
        except (subprocess.CalledProcessError, FileNotFoundError):
            return None

    def run(self):
        """Run the utility checker."""

        # Setup utility data
        self._setup_utility_data()

        # Setup curses screen
        stdscr = curses.initscr()
        curses.noecho()
        curses.cbreak()

        curses.start_color()
        curses.init_pair(1, curses.COLOR_GREEN, curses.COLOR_BLACK)
        curses.init_pair(2, curses.COLOR_RED, curses.COLOR_BLACK)
        curses.init_pair(3, curses.COLOR_YELLOW, curses.COLOR_BLACK)

        for i, pid in enumerate(self.pid_list):
            if not self._check_process(pid[0]):
                stdscr.addstr(i, 5, f"`{pid[1]}` is not running", curses.color_pair(2))
            else:
                stdscr.addstr(i, 5, f"`{pid[1]}` is running", curses.color_pair(1))
        stdscr.addstr(len(self.pid_list) + 1, 0, "Press Ctrl+C to exit")
        stdscr.addstr(len(self.pid_list) + 2, 0, "Processing...")
        stdscr.refresh()

        try:
            while True:
                stdscr.erase()
                stdscr.addstr(
                    0,
                    0,
                    f'{"PROCESS":<40} {"%CPU":<10} {"%MEM":<10} {"%GPU":<10} {"%CPU_AVG":<10} {"%CPU_MAX":<10} {"%MEM_AVG":<10} {"%MEM_MAX":<10} {"%GPU_AVG":<10} {"%GPU_MAX":<10}',  # pylint: disable=line-too-long, # noqa: E501
                    curses.color_pair(3),
                )
                for i, (pid, value) in enumerate(self._utility_data.items()):
                    cpu, mem, gpu = self._check_utility(pid[0])
                    if cpu is None or mem is None or gpu is None:
                        stdscr.addstr(
                            len(self.pid_list) + 2,
                            0,
                            f"Unable to get utility data for {pid[1]}",
                            curses.color_pair(2),
                        )
                        stdscr.refresh()
                        continue
                    else:
                        stdscr.addstr(
                            len(self.pid_list) + 2,
                            0,
                            "No issues detected...",
                            curses.color_pair(1) | curses.A_BLINK,
                        )
                    value["cpu"].append(cpu)
                    value["mem"].append(mem)
                    value["gpu"].append(gpu)

                    color = curses.color_pair(1)
                    if cpu > 80:
                        color = curses.color_pair(2)
                    if mem > 80:
                        color = curses.color_pair(2)
                    if gpu > 80:
                        color = curses.color_pair(2) | curses.A_BOLD
                    if cpu > 80 and mem > 80:
                        color = curses.color_pair(2) | curses.A_BOLD
                    stdscr.addstr(
                        i + 1,
                        0,
                        f"{pid[1]:<40} {value['cpu'][-1]:<10} {value['mem'][-1]:<10} {value['gpu'][-1]:<10} {round(np.mean(value['cpu']), 2):<10} {round(np.max(value['cpu']), 2):<10} {round(np.mean(value['mem']), 2):<10} {round(np.max(value['mem']), 2):<10} {round(np.mean(value['gpu']), 2):<10} {round(np.max(value['gpu']), 2):<10}",  # pylint: disable=line-too-long # noqa: E501
                        color,
                    )

                stdscr.addstr(len(self.pid_list) + 3, 0, "Press Ctrl+C to exit")
                stdscr.refresh()
        except KeyboardInterrupt:
            pass

    def __del__(self):
        """Destructor."""
        curses.nocbreak()
        curses.echo()
        curses.endwin()
