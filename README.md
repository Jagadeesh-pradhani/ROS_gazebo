# ROS_gazebo
## Ros installation
open a terminal and type, 
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```
```
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```
```
sudo apt update
```
```
sudo apt install ros-noetic-desktop-full
```
this will install Ros noetic full version (it wil take time to install).
**Setup Environment**
use this command every time you open a new terminal to get access to ROS
```
source /opt/ros/noetic/setup.bash
```
*Install Dependencies*
```
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
```
*refer below page for more info*
http://wiki.ros.org/noetic/Installation/Ubuntu

## Installing Ardupilot Directory
https://github.com/punkypankaj/Installing-ArduPilot-directory/blob/main/docs.md

## Installing Ardupilot plugin for gazebo
https://github.com/punkypankaj/Installing-Gazebo-and-ArduPilot-Plugin/blob/main/Docs.md
or
https://github.com/dronedojo/ardupilot_gazebo


## Run simulation

**Open first terminal**, and copy the following commands to open gazebo simulator
make sure you have the world named as irish_world in gazebo_ros file.
```
source /opt/ros/noetic/setup.bash
```
```
roslaunch gazebo_ros iris_world.launch
```
if no world is present clone the following git,
```
git clone https://github.com/dronedojo/ardupilot_gazebo -b master
```

**open second terminal** , and copy the following to connect to the vehicle in gazebo world.
```
source /opt/ros/noetic/setup.bash
```
```
cd ~/ardupilot/ArduCopter/
sim_vehicle.py -v ArduCopter -f gazebo-iris --console
```
once you run the command you will be connected to the drone, you can now see drone's parameter in console and controll it through terminal.

**Open third terminal**, to autonomously simulate drone using dronekit code
```
python3 test.py
```


