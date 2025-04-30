from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pi_camera_driver',
            executable='camera_node',
            name='pi_camera_node',
            output='screen'
        ),
        # ExecuteProcess(
        #     cmd=['rviz2'],
        #     output='screen'
        # )
    ])
