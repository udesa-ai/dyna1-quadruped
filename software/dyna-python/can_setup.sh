#!/bin/bash
sudo modprobe can
sudo modprobe can_raw
sudo modprobe mttcan
sudo ip link set can0 type can bitrate 1000000
sleep 1
sudo ifconfig can0 up
sudo ifconfig can0 txqueuelen 1000
echo "Done!"
