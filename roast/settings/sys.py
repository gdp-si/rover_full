"""System related settings for Roast."""
import os

# File system paths
FILE_PATH_PREFIX = os.path.join(os.path.expanduser("~"), ".roast")
DATA_PATH = os.path.join(FILE_PATH_PREFIX, "data")
LOG_PATH = os.path.join(FILE_PATH_PREFIX, "logs")
IPC_PATH = os.path.join(FILE_PATH_PREFIX, "ipc")
CACHE_PATH = os.path.join(FILE_PATH_PREFIX, "caches")
MEDIA_PATH = os.path.join(FILE_PATH_PREFIX, "media")

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


dirs = [
    ROOT_DIR,
    FILE_PATH_PREFIX,
    DATA_PATH,
    LOG_PATH,
    IPC_PATH,
    CACHE_PATH,
    MEDIA_PATH,
]
for d in dirs:
    if not os.path.exists(d):
        os.makedirs(d)
