#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class IMUTransformNode : public rclcpp::Node
{
public:
    IMUTransformNode()
        : Node("imu_transform_node"),
          tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
          tf_listener_(*tf_buffer_)
    {
        // Declare and get the target frame parameter (default: base_link)
        this->declare_parameter<std::string>("target_frame", "base_link");
        target_frame_ = this->get_parameter("target_frame").as_string();

        // Create publisher for transformed IMU data
        transformed_imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("output/imu/data", 10);

        // Create subscription for raw IMU data
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "input/imu/data", 10,
            std::bind(&IMUTransformNode::imuCallback, this, std::placeholders::_1));

        // Log initialization info
        RCLCPP_INFO(this->get_logger(), "IMU Transform Node initialized with:");
        RCLCPP_INFO(this->get_logger(), "Input Topic: %s", imu_sub_->get_topic_name());
        RCLCPP_INFO(this->get_logger(), "Output Topic: %s", transformed_imu_pub_->get_topic_name());
        RCLCPP_INFO(this->get_logger(), "Target Frame: %s", target_frame_.c_str());

        first_message_received_ = false; // To track if the first IMU message is received
    }

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        if (!first_message_received_)
        {
            RCLCPP_INFO(this->get_logger(), "First IMU message received. Transform process started.");
            first_message_received_ = true;
        }

        try
        {
            // Lookup the transform from the IMU frame to the target frame
            geometry_msgs::msg::TransformStamped transform_stamped;
            transform_stamped = tf_buffer_->lookupTransform(target_frame_, msg->header.frame_id, tf2::TimePointZero);

            // Transform linear acceleration
            tf2::Vector3 linear_accel;
            tf2::fromMsg(msg->linear_acceleration, linear_accel);
            tf2::Transform tf2_transform;
            tf2::fromMsg(transform_stamped.transform, tf2_transform);
            linear_accel = tf2_transform * linear_accel;

            // Transform angular velocity
            tf2::Vector3 angular_vel;
            tf2::fromMsg(msg->angular_velocity, angular_vel);
            angular_vel = tf2_transform.getBasis() * angular_vel;

            // Transform orientation
            tf2::Quaternion orientation;
            tf2::fromMsg(msg->orientation, orientation);
            orientation = tf2_transform.getRotation() * orientation;

            // Create transformed IMU message
            auto transformed_msg = std::make_shared<sensor_msgs::msg::Imu>();
            transformed_msg->header = msg->header;
            transformed_msg->header.frame_id = target_frame_;  // Update frame_id

            // Populate the transformed fields
            transformed_msg->linear_acceleration.x = linear_accel.x();
            transformed_msg->linear_acceleration.y = linear_accel.y();
            transformed_msg->linear_acceleration.z = linear_accel.z();

            transformed_msg->angular_velocity.x = angular_vel.x();
            transformed_msg->angular_velocity.y = angular_vel.y();
            transformed_msg->angular_velocity.z = angular_vel.z();

            geometry_msgs::msg::Quaternion quat_msg;
            quat_msg.x = orientation.x();
            quat_msg.y = orientation.y();
            quat_msg.z = orientation.z();
            quat_msg.w = orientation.w();
            transformed_msg->orientation = quat_msg;

            // Copy covariances from the original message
            transformed_msg->orientation_covariance = msg->orientation_covariance;
            transformed_msg->angular_velocity_covariance = msg->angular_velocity_covariance;
            transformed_msg->linear_acceleration_covariance = msg->linear_acceleration_covariance;

            // Publish the transformed IMU data
            transformed_imu_pub_->publish(*transformed_msg);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform IMU data: %s", ex.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr transformed_imu_pub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // Parameters
    std::string target_frame_;
    bool first_message_received_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IMUTransformNode>());
    rclcpp::shutdown();
    return 0;
}
