#!/bin/bash
# cd infantry_7_25
source /opt/intel/openvino_2021/bin/setupvars.sh
BaudRate=115200
while true
do
    name=`ls /dev/| grep ACM`
    if ! `ps aux | grep -v grep | grep /home/csy/Desktop/RM/infantry_9_28/build/infantry_new`
    then
            /home/csy/Desktop/RM/infantry_9_28/build/infantry_new /dev/$name $BaudRate
    fi
    sleep 0.2
done