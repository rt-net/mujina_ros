#!/usr/bin/bash
# Bring up can0 (front legs: FL/FR, motor IDs 1-6). See get_can_device().
sudo ip link set can0 type can bitrate 1000000
#sudo slcand -o -c -s8 /dev/usb_can can0
sudo ip link set up can0
