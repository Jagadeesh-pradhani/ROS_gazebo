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
