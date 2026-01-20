#!/usr/bin/bash
sudo slcand -o -c -s8 /dev/usb_can can0
sudo ip link set up can0
