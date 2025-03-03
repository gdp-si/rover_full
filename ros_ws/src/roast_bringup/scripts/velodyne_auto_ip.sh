#!/bin/bash

echo "==========================================================================="
echo "This script is for Auto assigning ip address to Velodyne VLP-16 3D Lidar..."
echo "==========================================================================="
# Set the network interface name and IP address
interface="eth0"
ip_address="192.168.1.3/24"

# Set the IP address for the interface
sudo ip addr add $ip_address dev $interface

# Display the new IP address
echo "New IP address for Velodyne with Interface name $interface is:"
ip addr show $interface | grep -w inet | awk '{print $2}'
