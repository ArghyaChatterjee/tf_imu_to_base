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


For zedmini, the imu is publishing at 200 Hz. So, the transformed imu is also publishing at 200 hz. Here is the original imu data:
```
$ ros2 topic echo /zed/zed_node/imu/data
header:
  stamp:
    sec: 1736650455
    nanosec: 378236523
  frame_id: zed_imu_link
orientation:
  x: 0.022179877385497093
  y: 0.0034157882910221815
  z: 0.14410610496997833
  w: 0.9893077611923218
orientation_covariance:
- 1.8508830344210603e-10
- -7.84383421990087e-11
- -8.845624788251505e-13
- -7.843837683000176e-11
- 1.2501102806605326e-09
- 1.5606025873626166e-10
- -8.845664695059932e-13
- 1.560602933672547e-10
- 2.085234081352584e-10
angular_velocity:
  x: -0.0006683777053465509
  y: -0.003925196048002675
  z: 0.0024423036996239875
angular_velocity_covariance:
- 1.6047202907882846e-09
- 0.0
- 0.0
- 0.0
- 1.581128827161396e-09
- 0.0
- 0.0
- -0.0
- 2.2308874323062063e-09
linear_acceleration:
  x: -0.03268469497561455
  y: 0.4052990972995758
  z: 9.761780738830566
linear_acceleration_covariance:
- 0.08261747658252716
- 0.0
- 0.0
- 0.0
- 0.08829377591609955
- 0.0
- 0.0
- -0.0
- 0.07581201940774918

```
Here is the transformed imu data:
```
$ ros2 topic echo /zed/zed_node/imu/data_transformed
header:
  stamp:
    sec: 1736650226
    nanosec: 611998149
  frame_id: zed_camera_link
orientation:
  x: 0.01975296422536684
  y: 0.0035145321644079117
  z: 0.15273977933441205
  w: 0.9880627603455336
orientation_covariance:
- 1.8700150997006051e-10
- -8.331717381898663e-11
- -1.5059674232022264e-12
- -8.331716516123836e-11
- 1.2416835903774486e-09
- 1.6375005343318772e-10
- -1.5059676937568596e-12
- 1.6375005343318772e-10
- 2.1186955857919582e-10
angular_velocity:
  x: 0.0009975657821477367
  y: -0.004158945196398707
  z: 0.002234799401954925
angular_velocity_covariance:
- 1.6047202907882846e-09
- 0.0
- 0.0
- 0.0
- 1.581128827161396e-09
- 0.0
- 0.0
- -0.0
- 2.2308874323062063e-09
linear_acceleration:
  x: -0.0013648316140784898
  y: 0.4655013204236458
  z: 9.77867291603649
linear_acceleration_covariance:
- 0.08261747658252716
- 0.0
- 0.0
- 0.0
- 0.08829377591609955
- 0.0
- 0.0
- -0.0
- 0.07581201940774918
```

