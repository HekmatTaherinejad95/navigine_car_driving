# navigine_car_driving

Install ROS2 (foxy or humble)

clone the repository recursively:

```
mkdir -p /ros2_ws/src
cd /ros2_ws/src
git clone --recursive https://github.com/HekmatTaherinejad95/navigine_car_driving.git
cd ~/ros2_ws
colcon build
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash

```
Connect all the necassary sensors. 

run:

```
ros2 launch all_package.launch.py

```


