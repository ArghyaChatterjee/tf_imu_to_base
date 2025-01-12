# Transform IMU to Baselink
This repository is about transforming imu data from imu frame to base link frame. This code is the ros2 version of this package.

# Requirements
This repository is tested with Ubuntu 22.04 and ROS Humble.

# Setup the repo
Create a ros2 colcon workspace and clone the repo inside the workspace:
```
mkdir -p ros2_ws/src
cd ros2_ws/src
git clone https://github.com/ArghyaChatterjee/tf_imu_to_base.git
cd tf_imu_to_base
git checkout feature/ros2
```
# Build the package
This requires mostly standard ros messages which are already installed at the time of installing ros humble. No additional packages are needed to be installed.
```
cd ~/ros2_ws
colcon build --packages-select tf_imu_to_base
source install/setup.bash
```

# Launch the transform node
Modify the `imu_transform.launch` file according to the `topics` and `frame name` that you want to publish the imu data. Then launch the node like this:
```bash
ros2 launch tf_imu_to_base imu_transform.launch.py
```
There is a separate launch file for zed camera. If you want to test it, launch it like this:
```bash
ros2 launch tf_imu_to_base zedm_imu_transform.launch.py
```
<div align="center">
   <img src="media/zed_imu_frame_transformed.gif"/>
</div>

For zed, the imu is publishing at 200 Hz. So, the transformed imu is also publishing at 200 hz. Here is the original imu data:
```
$ ros2 topic echo /zed/zed_node/imu/data
header: 
  seq: 30944
  stamp: 
    secs: 1736642323
    nsecs: 254978301
  frame_id: "zed_imu_link"
orientation: 
  x: 0.02223959006369114
  y: 0.00240450631827116
  z: 0.015558099374175072
  w: 0.9996287226676941
orientation_covariance: [1.7157596524839715e-10, -1.4698134780952678e-11, 3.057317139502141e-12, -1.4698133698734142e-11, 1.7962691618281004e-09, 4.94149726772295e-11, 3.057317139502141e-12, 4.9414968348355365e-11, 1.7787162005708141e-10]
angular_velocity: 
  x: -0.0006701409443013445
  y: -0.004481651869735539
  z: 0.0039126409596081674
angular_velocity_covariance: [1.6047202907882846e-09, 0.0, 0.0, 0.0, 1.581128827161396e-09, 0.0, 0.0, -0.0, 2.2308874323062063e-09]
linear_acceleration: 
  x: -0.10108465701341629
  y: 0.4545670747756958
  z: 9.754082679748535
linear_acceleration_covariance: [0.08261747658252716, 0.0, 0.0, 0.0, 0.08829377591609955, 0.0, 0.0, -0.0, 0.07581201940774918]

```
Here is the transformed imu data:
```
$ ros2 topic echo /zed/zed_node/imu/data_transformed
header: 
  seq: 5272
  stamp: 
    secs: 1736642194
    nsecs: 894398855
  frame_id: "zed_camera_link"
orientation: 
  x: 0.01986186491454831
  y: 0.0016617271223713034
  z: 0.010022339920807439
  w: 0.9997511335006893
orientation_covariance: [1.7189533226653974e-10, -6.3721525079510296e-12, 3.1814465230887293e-12, -6.3721552134973636e-12, 1.3196644138691882e-09, 3.063968021722298e-11, 3.181447064197996e-12, 3.063967805278591e-11, 1.7723875597412251e-10]
angular_velocity: 
  x: -0.000932026797172692
  y: -0.0006295253428333462
  z: -0.0023160180470354432
angular_velocity_covariance: [1.6047202907882846e-09, 0.0, 0.0, 0.0, 1.581128827161396e-09, 0.0, 0.0, -0.0, 2.2308874323062063e-09]
linear_acceleration: 
  x: -0.11459180870129143
  y: 0.47052105792639143
  z: 9.771776648942483
linear_acceleration_covariance: [0.08261747658252716, 0.0, 0.0, 0.0, 0.08829377591609955, 0.0, 0.0, -0.0, 0.07581201940774918]

```

