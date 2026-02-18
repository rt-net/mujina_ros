#!/usr/bin/bash
sudo ip link set can0 type can bitrate 1000000
#sudo slcand -o -c -s8 /dev/usb_can can0
sudo ip link set up can0
