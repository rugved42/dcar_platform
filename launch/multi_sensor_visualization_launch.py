from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # RPLIDAR Node
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            output='screen',
            parameters=[
                {'channel_type': 'serial'},
                {'serial_port': '/dev/ttyUSB0'},
                {'serial_baudrate': 460800},   
                {'frame_id': 'laser'},
                {'inverted': False},
                {'angle_compensate': True},
                {'scan_mode': 'Standard'},
            ]
        ),

        # Raspberry Pi Camera Node
        Node(
            package='pi_camera_driver',
            executable='camera_node',
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
                '0.2', '0.0', '0.1',  
                '0', '0', '0',
                'laser_frame',
                'camera_frame'
            ],
            output='screen'
        ),
    ])
