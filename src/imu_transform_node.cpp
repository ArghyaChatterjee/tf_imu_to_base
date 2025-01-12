#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

class IMUTransformNode
{
public:
  IMUTransformNode()
  {
    // Publisher for transformed IMU data
    transformed_imu_pub_ = nh_.advertise<sensor_msgs::Imu>("output/imu/data", 10);

    // Subscriber for incoming IMU data
    imu_sub_ = nh_.subscribe("input/imu/data", 10, &IMUTransformNode::imuCallback, this);

    // Get the target frame parameter (default: base_link)
    ros::NodeHandle private_nh("~");
    private_nh.param<std::string>("target_frame", target_frame_, "base_link");

    // ROS Info for initialization
    ROS_INFO("IMU Transform Node initialized with:");
    ROS_INFO("Input Topic: %s", imu_sub_.getTopic().c_str());
    ROS_INFO("Output Topic: %s", transformed_imu_pub_.getTopic().c_str());
    ROS_INFO("Target Frame: %s", target_frame_.c_str());

    first_message_received_ = false;  // To track if the first IMU message is received
  }

private:
  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
  {
    if (!first_message_received_)
    {
      ROS_INFO("First IMU message received. Transform process started.");
      first_message_received_ = true;
    }

    try
    {
      // Lookup the transform from the IMU frame to the target frame
      tf::StampedTransform transform;
      tf_listener_.lookupTransform(target_frame_, msg->header.frame_id, ros::Time(0), transform);

      // Transform linear acceleration
      tf::Vector3 linear_accel(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
      linear_accel = transform * linear_accel;

      // Transform angular velocity
      tf::Vector3 angular_vel(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
      angular_vel = transform.getBasis() * angular_vel;

      // Transform orientation
      tf::Quaternion orientation;
      tf::quaternionMsgToTF(msg->orientation, orientation);
      orientation = transform.getRotation() * orientation;

      // Create transformed IMU message
      sensor_msgs::Imu transformed_msg = *msg;
      transformed_msg.header.frame_id = target_frame_;  // Update frame_id
      transformed_msg.linear_acceleration.x = linear_accel.x();
      transformed_msg.linear_acceleration.y = linear_accel.y();
      transformed_msg.linear_acceleration.z = linear_accel.z();
      transformed_msg.angular_velocity.x = angular_vel.x();
      transformed_msg.angular_velocity.y = angular_vel.y();
      transformed_msg.angular_velocity.z = angular_vel.z();
      tf::quaternionTFToMsg(orientation, transformed_msg.orientation);

      // Publish the transformed IMU data
      transformed_imu_pub_.publish(transformed_msg);
    }
    catch (tf::TransformException& ex)
    {
      ROS_WARN("Could not transform IMU data: %s", ex.what());
    }
  }

  ros::NodeHandle nh_;
  ros::Subscriber imu_sub_;
  ros::Publisher transformed_imu_pub_;
  tf::TransformListener tf_listener_;

  // Parameters
  std::string target_frame_;
  bool first_message_received_;  // To track the first IMU message
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "imu_transform_node");
  IMUTransformNode node;
  ros::spin();
  return 0;
}
