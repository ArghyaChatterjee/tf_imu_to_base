from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf_imu_to_base',
            executable='imu_transform_node',
            name='imu_transform_node',
            output='screen',
            parameters=[
                {'target_frame': 'zed_camera_link'}  # Frame to transform IMU data into
            ],
            remappings=[
                ('input/imu/data', '/zed/zed_node/imu/data'),  # Input topic
                ('output/imu/data', '/zed/zed_node/imu/data_transformed')  # Output topic
            ]
        )
    ])
