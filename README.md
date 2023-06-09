# navigine_car_driving

Install ROS2 (foxy or humble)

Clone the repository recursively:

```
sudo apt-get install ros-$ROS_DISTRO-nmea-msgs

mkdir -p /ros2_ws/src
cd /ros2_ws/src
git clone --recursive https://github.com/HekmatTaherinejad95/navigine_car_driving.git
cd ~/ros2_ws
colcon build
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash

```
Connect all the necessary sensors. 

Run:

```
ros2 launch ~/ros2_ws/src/all_package.launch.py

```


