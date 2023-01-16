# ros_shc: ROS2 Shared Memory Zero Copy

Installation: 
```
# Dashing
sudo apt update && sudo apt install curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-dashing-desktop

# Eloquent (suggested)
sudo apt update && sudo apt install curl gnupg2 lsb-release
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
sudo apt update
sudo apt install ros-eloquent-desktop

sudo apt install python3-colcon-common-extensions
```

Setup: 
```
# ROS2 Dashing
source /opt/ros/dashing/setup.bash
# ROS2 Eloquent
source /opt/ros/eloquent/setup.bash
```

## Create workspace:

```
mdkir ros_ws
cd ros_ws
mkdir src
cd src
git clone https://github.com/PARCO-LAB/ros_shc.git
cd ..
```

## Build and install package:

```
colcon build
colcon build --packages-select ros_shc
. install/setup.bash
```

## Benchmark

Standard copy: open 4 terminals
```
cd ros_ws && . install/setup.bash
ros2 run ros_shc sc_producer 1 256
ros2 run ros_shc sc_mat_mul_cpu 1
ros2 run ros_shc sc_mat_mul_gpu 1
ros2 run ros_shc sc_checker 1
```

Zero copy: single terminal
```
cd ros_ws && . install/setup.bash
ros2 run ros_shc zc_cluster 1 256
```
