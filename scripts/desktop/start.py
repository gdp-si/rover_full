import getpass
import time

import paramiko


def ssh_execute_command(host, username, password, command):
    try:
        # Create an SSH client
        ssh_client = paramiko.SSHClient()
        ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())

        # Connect to the remote server
        ssh_client.connect(host, username=username, password=password)

        # Start an interactive SSH shell session
        ssh_shell = ssh_client.invoke_shell()
        time.sleep(1)
        # Send the command to the remote server
        ssh_shell.send(command + "\n")

        time.sleep(1)

        # Read and display the command's output in real-time
        while True:
            output = ssh_shell.recv(1024).decode("utf-8")
            if not output:
                break
            print(output, end="")

    except KeyboardInterrupt:
        print("\n\nProgram execution aborted.")
        ssh_shell.close()
        ssh_client.close()
    except paramiko.AuthenticationException:
        print("Authentication failed, please check your credentials.")
    except paramiko.SSHException as e:
        print("SSH error:", str(e))
    except Exception as e:
        print("An error occurred:", str(e))


if __name__ == "__main__":
    host = input("Enter robot IP: ")
    username = input(f"Enter robot name for {host}: ")
    password = getpass.getpass(f"Enter password for {username}@{host}: ")

    # Ask if the user wants to record ROS bags
    record_rosbags = input("Do you want to record ROS bags? (yes or no): ").lower()

    if record_rosbags == "yes":
        ros_bag_record = "1"
    else:
        ros_bag_record = "0"

    # Set the ROS_BAG_RECORD environment variable
    command = (
        f"export ROS_BAG_RECORD={ros_bag_record} && "
        f"ros2 launch roast_bringup robot_bringup.launch.py | tee -a /home/{username}/.roast/logs/$(date +%Y-%m-%d-%H-%M-%S).log"
    )

    ssh_execute_command(host, username, password, command)
