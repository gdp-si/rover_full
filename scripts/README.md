## Scripts

### Startup script

The startup scripts are accessed by the service manager. The service manager is a daemon that runs in the background and manages the startup of services. The service manager is called systemd. The startup scripts are called unit files. The unit files are stored in the `/etc/systemd/user` directory.

Please refer to this link for How To Use Systemctl to Manage Systemd Services and Units:

https://www.digitalocean.com/community/tutorials/how-to-use-systemctl-to-manage-systemd-services-and-units

#### Usage

To setup the service manager, run the following command:

```bash
sudo cp scripts/robot-startup.service /etc/systemd/user/
systemctl --user enable robot-startup.service
systemctl --user start robot-startup.service
```

# Isaac ROS Dev Build Scripts

For Jetson or x86_64:
  `run_dev.sh` creates a dev environment with ROS 2 installed. By default, the directory `/workspaces/isaac_ros-dev` in the container is mapped from `~/workspaces/isaac_ros-dev` on the host machine if it exists OR the current working directory from where the script was invoked otherwise. The host directory the container maps to can be explicitly set by running the script with the desired path as the first argument:
  `run_dev.sh <path to workspace>`
