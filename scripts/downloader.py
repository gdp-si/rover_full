#!/usr/bin/env python3

import os
import subprocess

from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Replace with your remote host, username, and password
host = os.getenv("SERVER_IP")
username = os.getenv("SERVER_USER")
password = os.getenv("SERVER_PASSWORD")
arch = os.getenv("ARCH")


def download_ai_models(
    host=host, username=username, password=password, remote_file="", local_dir=""
):
    # Replace with the path to the remote file you want to copy
    remote_file = os.path.expanduser(f"~/roast/ai_models/{arch}/yolov8_grayscale_v0.pt")

    # Replace with the path to the local directory where you want to save the file
    local_dir_prefix = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    local_dir = os.path.join(local_dir_prefix, "models")

    # Construct the `scp` command
    command = f"scp {username}@{host}:{remote_file} {local_dir}"

    # Run the `scp` command
    return subprocess.run(
        command, shell=True, check=True, input=f"{password}\n", encoding="ascii"
    )


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--host", type=str, default="")
    parser.add_argument("--username", type=str, default="")
    parser.add_argument("--password", type=str, default="")
    parser.add_argument("--remote_file", type=str, default="")
    parser.add_argument("--local_dir", type=str, default="")

    parser.set_defaults(host=host, username=username, password=password)

    args = parser.parse_args()

    result = download_ai_models(
        host=args.host,
        username=args.username,
        password=args.password,
        remote_file=args.remote_file,
        local_dir=args.local_dir,
    )

    exit(result.returncode)
