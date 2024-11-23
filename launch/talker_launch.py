from launch LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_cv_cpp',
            executable='talker',
            name='my_talker',
            output='screen',
            parameters=[{
                ''
            }]
        )
    ])