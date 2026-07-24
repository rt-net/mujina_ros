#!/usr/bin/bash
# Bring up can1 (rear legs: RL/RR, motor IDs 7-12). See get_can_device().
sudo ip link set can1 type can bitrate 1000000
sudo ip link set up can1
