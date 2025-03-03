#!/usr/bin/env python3
import os


def update_desktop_entry(executable_path):
    # Get the path to the desktop entry file
    desktop_entry_path = os.path.expanduser("~/.local/share/applications/robot.desktop")

    # Read the desktop entry file
    with open(desktop_entry_path, "r") as desktop_entry_file:
        desktop_entry_content = desktop_entry_file.read()

    # Replace the EXEC_PLACEHOLDER with the actual executable path
    updated_entry_content = desktop_entry_content.replace(
        "{{ EXECUTABLE_PATH }}", executable_path
    )

    # Write the updated content back to the desktop entry file
    with open(desktop_entry_path, "w") as desktop_entry_file:
        desktop_entry_file.write(updated_entry_content)


def main():
    # Determine the path to the script's directory
    script_dir = os.path.dirname(os.path.abspath(__file__))

    # Update the desktop entry with the correct executable path
    update_desktop_entry(script_dir)


if __name__ == "__main__":
    main()
