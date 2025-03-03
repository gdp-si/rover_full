loglist = []
value = 0
errorname = ""
import subprocess
import threading
import time
import tkinter as tk
from tkinter import *


# one click function
def botCommand():
    subprocess.run(["ros2 launch roast_bringup robot_bringup.launch.py"], shell=True)
    pass


# Create a Tkinter window
window = tk.Tk()
# def navigationFunc():
# Make the window full screen
window.attributes("-fullscreen", True)


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
    subprocess.run("python progress_bar2.py", shell=True)
    # oneclickthred.join()
    # closewndw.join()

    return True


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
