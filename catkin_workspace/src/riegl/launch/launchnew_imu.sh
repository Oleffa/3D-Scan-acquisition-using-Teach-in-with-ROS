roslaunch riegl backpack_imu.launch &
sleep 5
rosservice call --wait /setContinuous &
rosservice call --wait /startMeasuring &
echo "done, started measuring"
rosbag record -a
#sleep 20
#rosservice call --wait /stopMeasuring &

#rosservice list
#/bin/correction -d 500 -N 300 -S 300 -i 10 -I 10 -s 0 -e 13000 -r 10 -O 1 /home/visionlab/data3/scan3d/
