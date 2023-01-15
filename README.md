# ros_shc: ROS2 Shared Memory Zero Copy

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
ros2 run ros_shc sc_producer 1 256
ros2 run ros_shc sc_mat_mul_cpu 1
ros2 run ros_shc sc_mat_mul_gpu 1
ros2 run ros_shc sc_checker 1
```

Zero copy: single terminal
```
ros2 run ros_shc zc_cluster 1 256
```
