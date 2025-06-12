from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_package',
            executable='test1',
            name='motors',
            output='screen'
        ),
        Node(
            package='my_robot_package',
            executable='motor_controller',
            name='motor_controller',
            output='screen'
        )
    ]) 