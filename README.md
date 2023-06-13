# navigine_car_driving

Install ROS2 (tested on humble devision)

### Install dependencies:
- pyserial: `pip3 install pyserial`
- nmea_msgs: `sudo apt-get install ros-humble-nmea-msgs`
- install pylon 5.1 from the [link](https://www.baslerweb.com/en/downloads/software-downloads/pylon-5-1-0-linux-x86-64-bit-debian/) 

clone the repository recursively:

```
mkdir -p /ros2_ws/src
cd /ros2_ws/src
git clone --recursive https://github.com/HekmatTaherinejad95/navigine_car_driving.git
cd ~/ros2_ws
colcon build
source /opt/ros/humble/setup.bash
source install/setup.bash

```
Connect all the necassary sensors. 

run:

```
ros2 launch all_package.launch.py

```


