from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([
        # Include the Gazebo world launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(['/home/sriram/ros2_ws/src/project_clearpath/launch/empty_world.launch.py'])
        ),
        # Spawn the TurtleBot Waffle
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'turtlebot3_waffle',
                '-x', '0', '-y', '0', '-z', '0.1',
                '-file', '/opt/ros/ros2-humbele/share/turtlebot3_description/urdf/turtlebot3_waffle.urdf'
            ],
            output='screen'
        ),
    ])
