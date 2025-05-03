# File: dcar_platform/src/examples/launch/sensor_fusion_no_rviz.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # RPLIDAR Node
        Node(
            package='rplidar_ros',
            executable='rplidar_node',  # ✅ Correct executable
            name='rplidar_node',
            output='screen',
            parameters=[
                {'channel_type': 'serial'},
                {'serial_port': '/dev/ttyUSB0'},   # double check this matches your device
                {'serial_baudrate': 460800},       # based on your RPLidar model
                {'frame_id': 'laser'},
                {'inverted': False},
                {'angle_compensate': True},
                {'scan_mode': 'Standard'},
            ]
        ),

        # Raspberry Pi Camera Node
        Node(
            package='pi_camera_driver',
            executable='camera_node',  # Adjust if needed
            name='pi_camera_node',
            output='screen',
            parameters=[
                {'frame_id': 'camera_frame'}
            ]
        ),

        # Static TF between lidar and camera
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_lidar_camera',
            arguments=[
                '0.2', '0.0', '0.1',   # Translation: 20cm forward, 10cm up (example)
                '0', '0', '0',         # No rotation
                'laser_frame',
                'camera_frame'
            ],
            output='screen'
        ),
    ])
