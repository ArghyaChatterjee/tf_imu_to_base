# Transform IMU to Baselink
This repository is about transforming imu from it's original frame to base link frame.

# Requirements
This repository is tested with Ubuntu 20.04 and ROS noetic.

# Setup the repo
Create a ros1 catkin workspace and clone the repo inside the workspace:
```
mkdir -p ros1_ws/src
git clone https://github.com/ArghyaChatterjee/tf_imu_to_base.git
```
# Build the package
This requires mostly standard ros messages which are already installed at the time of installing ros noetic. No additional packages are needed to be installed.
```
cd ~/ros1_ws
catkin build tf_imu_to_base
source devel/setup.bash
```

# Launch the transform node
Modify the `imu_transform.launch` file according to the `topics` and `frame name` that you want to publish the imu data. Then launch the node like this:
```bash
roslaunch tf_imu_to_base imu_transform.launch
```
There is a separate launch file for zed camera. If you want to test it, launch it like this:
```bash
roslaunch tf_imu_to_base zedm_imu_transform.launch
```

