#!/bin/zsh

control_c() {
    sudo pkill -f IMU_collect.py
    echo "--------------------------------------Process killed-----------"
}

trap control_c SIGINT
sudo python3 IMU_collect.py run1 &
sudo python3 gps_collect.py run1
