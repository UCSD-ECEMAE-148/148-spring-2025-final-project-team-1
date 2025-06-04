from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='autonomous_tow_car',
            executable='detect_damsel',
            name='detect_damsel',
            output='screen'
        ),
        Node(
            package='autonomous_tow_car',
            executable='three_point_turn',
            name='three_point_turn',
            output='screen'
        ),
        Node(
            package='autonomous_tow_car',
            executable='hook_and_vanish',
            name='hook_and_vanish',
            output='screen'
        )
    ])
