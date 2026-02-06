### Installing ROS2 Humble ###
```
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main"
sudo apt update
```

to remove the old
```
sudo rm /etc/apt/trusted.gpg.d/ros-archive-keyring.gpg
```
then add it with
```
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo gpg --dearmor -o /etc/apt/trusted.gpg.d/ros-archive-keyring.gpg
```
then 
```
sudo apt update
```
finally
```
sudo apt install ros-humble-desktop-full
```
then source
```
source /opt/ros/humble/setup.bash
```
check installation using
```
echo $ROS_DISTRO
```
should return humble

### Installing dependencies
```
sudo apt-get install ros-humble-navigation2
sudo apt-get install ros-humble-slam-toolbox
```

### Installing GZ Harmonic ###

```
sudo apt-get install ros-humble-ros-gzharmonic
```

### Setup GZ models ###
add these lines to ~/.bashrc
```
export GZ_SIM_RESOURCE_PATH="$HOME/.gazebo_models"  # ROS2
export IGN_GAZEBO_RESOURCE_PATH="$HOME/.gazebo_models"  # ROS2
export GZ_VERSION=harmonic
```

### Running SLAM ###
```
ros2 launch mako_bringup slam_bringup.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```