#!/bin/bash

echo "remap the device serial port(ttyUSBX) to  agv"
echo "rplidar usb connection as /dev/agv , check it using the command : ls -l /dev|grep ttyUSB"
echo "start copy rules to  /etc/udev/rules.d/"
echo "/home/baron/robot/src/agv_driver/scripts/agv.rules hokuyo.rules"
sudo cp /home/baron/robot/src/agv_driver/scripts/agv.rules  /etc/udev/rules.d
sudo cp /home/baron/robot/src/agv_driver/scripts/50-hokuyo.rules  /etc/udev/rules.d
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
echo "finish "
