# ROS2 Publisher/Subscriber

# Dependencies and Assumptions:

* System configuration: Ubuntu 22.04
* ROS2 Humble should be installed, if not you can follow the instructions below
* ament_cmake
* rclcpp
* std_msgs
  
# Setting up the dependencies, skip to next step if ROS2 Humble and Clangd already installed 
## Installing ROS2 Humble for Ubuntu 22.04 (http://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) 
You should follow the link for more updated instructions

```bash
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

```bash
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop
sudo apt install ros-humble-ros-base
sudo apt install ros-dev-tools
```

Source the setup file to start ROS2, add the following line in the .bashrc file of your system:
```bash
source /opt/ros/humble/setup.bash
```

## Install Clang

```bash
sudo apt install clang
export CC=clang
export CXX=clang++
colcon build --cmake-force-configure
```

# Adding beginners tutorial package in ros2

## Create ros2 workspace:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

## Clone this package

```bash
git clone -b Week10_HW  https://github.com/sanchit2843/beginner_tutorials
```

## Compile the package

```bash
colcon build --packages-select beginner_tutorials
```

## Source the package

You will need to source the package for every terminal you open

```bash
. install/setup.bash
```

# Running the publisher and subscriber

```bash
ros2 run beginner_tutorials talker

```

Create a new terminal and source the package again in this terminal after navigating to the ROS2 workspace created in first step.

```bash
ros2 run beginner_tutorials listener

```

# Running publisher and subscriber together:

```bash
ros2 launch beginner_tutorials launch.yaml pubfreq:=2.0 
```
### Change publisher frequency to custom value

# Invoke RQT console GUI

```bash
ros2 run rqt_console rqt_console
```
