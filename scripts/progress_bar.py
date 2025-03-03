loglist = []
value = 0
errorname = ""
import glob
import os
import subprocess
import threading
import time
import tkinter as tk
from datetime import datetime
from tkinter import *
from tkinter import ttk

window = tk.Tk()
window.attributes("-fullscreen", True)


# one click function
def botCommand():
    # button222 = tk.Label(frame, text="Processing...", font=("Arial", 15, "bold"),fg="green")
    # button222.pack(side=tk.LEFT, padx=10)
    subprocess.run(["ros2 launch roast_bringup robot_bringup.launch.py"], shell=True)


# Create a Tkinter window


# def navigationFunc():
# Make the window full screen
def close_window():
    print("clse thte eons")
    window.destroy()


def bot_function():
    oneclickthred = threading.Thread(target=botCommand)
    print("1")
    # closewndw = threading.Thread(target=close_window)
    # one click start

    oneclickthred.start()
    print("2")
    # closewndw.start()
    print(4)
    # one clcik threads to complete
    print("3")

    time.sleep(7)

    window.destroy()
    # subprocess.run("open -a Terminal")

    # Run command in Terminal
    # subprocess.run("python progress_bar2.py", shell=True)
    # oneclickthred.join()
    # closewndw.join()

    # return True


# Create a frame to hold the buttons

frame = tk.Frame(window)
# title
title12 = tk.Label(
    window,
    text="Ajai Robotics",
    font=("Arial", 50, "bold"),
    fg="yellow",
    width=30,
    height=1,
)
title12.pack(side=tk.TOP, padx=10)

# Create a toggle button
button1 = tk.Button(
    frame,
    text="One Click Start",
    command=bot_function,
    font=("Arial", 30, "bold"),
    fg="green",
    width=40,
    height=3,
)
button1.pack(side=tk.LEFT, padx=10)


# windo close button showing "X" symbol
close_button = tk.Button(window, text="X", bg="red", command=close_window)
close_button.pack(side=tk.TOP, anchor=tk.NE, padx=10, pady=10)
# Calculate the center position of the screen
screen_width = window.winfo_screenwidth()
screen_height = window.winfo_screenheight()
center_x = int(screen_width / 2)

# Calculate the position for the frame to be centered
frame_width = 200  # Adjust this value as needed
frame_height = 200  # Adjust this value as needed
frame_x = center_x - int(frame_width / 2)

# Position the frame in the center of the screen
frame.place(x=frame_x, rely=0.5, anchor=tk.CENTER)


# Start the Tkinter event loop
window.mainloop()
file_status = True
while file_status:
    # time.sleep(10)
    # Set the directory path
    dir_path = "/home/orin005/.ros/log/"

    # Get a list of all subdirectories in the directory
    all_items = glob.glob(os.path.join(dir_path, "*"))

    # Filter out the files and keep only the subdirectories
    all_folders = [item for item in all_items if os.path.isdir(item)]

    # Sort the list of subdirectories by modification time (newest first)
    all_folders.sort(key=os.path.getmtime, reverse=True)

    # Get the name of the most recently modified subdirectory
    latest_folder = os.path.basename(all_folders[0])

    print("Latest folder:", latest_folder)

    current_time = datetime.today()
    # print(current_time)
    convert_string = datetime.strftime(current_time, "%Y-%m-%d-%H-%M")
    # print(convert_string)
    filename = str(dir_path) + str(latest_folder) + "/launch.log"
    with open(filename, "r") as logfile:
        logfile = logfile.read()
        if "Managed nodes are active" in logfile:
            file_status = False
# filename = 'launch.log'
# print(filename)
with open(filename, "r") as logfile:
    # print(logfile)
    for i in logfile:
        # progress['value'] = value
        if "Starting managed nodes bringup" in i:
            # print('i')
            value += 5
            package_split = i.split("[1mStarting ")[1].split("\x1b[0m\x1b[0m\n")[0]

            loglist.append(
                {
                    "packageName": "Starting " + str(package_split),
                    "package_percentage": value,
                    "status": "Started",
                }
            )
        elif "Configuring" in i:
            value += 5
            package_split = i.split("[1mConfiguring ")[1].split("\x1b[0m\x1b[0m\n")[0]
            print(package_split)
            loglist.append(
                {
                    "packageName": str(package_split),
                    "package_percentage": value,
                    "status": "Configured",
                }
            )
        elif "Activating" in i:
            package_split = i.split("[1mActivating ")[1].split("\x1b[0m\x1b[0m\n")[0]
            value += 5
            print(package_split)
            loglist.append(
                {
                    "packageName": str(package_split),
                    "package_percentage": value,
                    "status": "Activating",
                }
            )
        elif "Managed nodes are active" in i:
            value += 5
            package_split = i.split("[1mManaged ")[1].split("\x1b[0m\x1b[0m\n")[0]
            print(package_split)
            loglist.append(
                {
                    "packageName": str(package_split),
                    "package_percentage": value,
                    "status": "Completed",
                }
            )

    else:
        errorname = i.split("] ")
        print("=========================", errorname[1], type(errorname))


package_list = [
    "velodyne",
    "component",
    "pointcloud",
    "sms",
    "environment",
    "gas",
    "sound",
    "led",
    "imu",
    "robot_state",
    "robot_control",
    "odom",
    "lifecycle",
]
print("package_list:", len(package_list), "loglist:", len(loglist))

root = tk.Tk()
# Make the window full screen
root.attributes("-fullscreen", True)

# Create a frame to hold the buttons
frame = tk.Frame(root)

title12 = tk.Label(
    root,
    text="Ajai Robotics",
    font=("Arial", 50, "bold"),
    fg="yellow",
    width=30,
    height=1,
)
title12.pack(side=tk.TOP, padx=10)

progress = ttk.Progressbar(
    root, orient="horizontal", length=550, mode="determinate", maximum=100
)
progress.pack(side="top", padx=40, pady=40)

pack_frame = tk.Frame(root)
pack_frame.pack(side="right")


# label_loading = tk.Label(root, text="Loading...", font=("Arial", 15, "bold"),fg="green")
# label_loading.pack()
# here we taking log file from current directory for checking all packages are coming or not
grid_frame = tk.Frame(root)
grid_frame.pack(side="left")


packagename = tk.Label(
    root, text="Install Packages", font=("Arial", 15, "bold"), fg="blue"
)
packagename.pack(anchor="w")  # Position labels on separate lines with left alignment

for num, data in enumerate(loglist, 1):
    time.sleep(2)
    packagename22 = data["packageName"].replace("_", " ")
    progress["value"] = data["package_percentage"]
    # packagename["text"] = f"Packages : {packagename22} Activated"

    rows = num
    num = tk.Label(
        root,
        text=packagename22 + " " + data["status"],
        font=("Arial", 15, "bold"),
        fg="green",
    )
    num.pack(anchor="w")  # Position labels on separate lines with left alignment
    root.update()


# label_loading.config(text = "All packages are completed")
time.sleep(2)
root.destroy()

# root.mainloop()


import subprocess


def Navigation():
    subprocess.run(
        ["bash ~/ajai/roast/ros_ws/src/roast_bringup/scripts/waypoint.sh"],
        shell=True,
    )  # ROAST_LOGLEVEL=INFO ros2 launch roast_navigation navigation.launch.py

    print("Button 1 clicked")


def Patrolling():
    subprocess.run(
        [
            "ROAST_LOG_LEVEL=INFO ros2 launch roast_behavior_tree behavior_tree_engine.launch.py"
        ],
        shell=True,
    )

    print("Button 2 clicked")


def on(devicestatus):  # device status is devicename/status ex:sufi1234node/0
    response = session.get(
        "http://localhost:5000/emergency_stop/" + devicestatus
    )  # http://localhost:5000/emergency_stop/sufi1234node/1
    print(response)
    return response


def off(devicestatus):  # device status is devicename/status ex: sufi1234node/1
    response = session.get(
        "http://localhost:5000/emergency_stop/" + devicestatus
    )  # http://localhost:5000/emergency_stop/sufi1234node/1
    print(response)
    return response


# Create a Tkinter window
def toggle():
    print(button3)
    if button3["text"] == "ON":
        print("bot stop function calling")
        button3["text"] = "OFF"
        button3["bg"] = "red"

        devicename = "sufi1234node"
        status = "0"
        off(devicename + "/" + status)

        # v = emergency_stop.emergency_stop_function(devicename+"|"+status)
        # print("result",result)
        # print("result",result.text())
    else:
        print("bot start function calling")
        button3["text"] = "ON"
        button3["bg"] = "green"
        # result = on()
        devicename = "sufi1234node"
        status = "1"
        on(devicename + "/" + status)

        # print("result",result)
        # import emergency_stop
        # v = emergency_stop.emergency_stop_function(devicename+"|"+status)
        # print("result",result.text())


# Create a Tkinter window
window = tk.Tk()

# def navigationFunc():
# Make the window full screen
window.attributes("-fullscreen", True)

# Create a frame to hold the buttons
frame = tk.Frame(window)

title11 = tk.Label(
    window,
    text="Ajai Robotics",
    font=("Arial", 50, "bold"),
    fg="yellow",
    width=30,
    height=1,
)
title11.pack(side=tk.TOP, padx=10)

# Create Button 1
button1 = tk.Button(
    frame,
    text="Navigation",
    command=Navigation,
    font=("Arial", 30, "bold"),
    fg="red",
    width=40,
    height=3,
)
button1.pack(side=tk.LEFT, padx=10)

# Create Button 2
button2 = tk.Button(
    frame,
    text="Patrolling",
    command=Patrolling,
    font=("Arial", 30, "bold"),
    fg="green",
    width=40,
    height=3,
)
button2.pack(side=tk.RIGHT, padx=10)


# Calculate the center position of the screen
screen_width = window.winfo_screenwidth()
screen_height = window.winfo_screenheight()
center_x = int(screen_width / 2)

# Calculate the position for the frame to be centered
frame_width = 200  # Adjust this value as needed
frame_height = 200  # Adjust this value as needed
frame_x = center_x - int(frame_width / 2)

# Position the frame in the center of the screen
frame.place(x=frame_x, rely=0.5, anchor=tk.CENTER)


# close button function
def close_window():
    window.destroy()


# windo close button showing "X" symbol
close_button = tk.Button(
    window, text="X", font=("Arial", 30, "bold"), bg="red", command=close_window
)
close_button.pack(side=tk.TOP, anchor=tk.NE, padx=10, pady=10)
# emergency stop lable name
emrgnLable = tk.Label(
    window, text="Emergency Stop", font=("Arial", 30, "bold"), width=16, height=1
)
emrgnLable.pack(side=tk.TOP, anchor=tk.NE, padx=10, pady=10)


button3 = tk.Button(
    window,
    text="ON",
    command=toggle,
    font=("Arial", 15, "bold"),
    fg="blue",
    bg="green",
    width=20,
    height=2,
)
button3.pack(side=tk.RIGHT, anchor=tk.NE, padx=10, pady=10)


window.mainloop()
# Start the Tkinter event loop
