from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '/home/sriram/ros2_ws/src/project_clearpath/worlds/empty_world.sdf'],
            output='screen'
        ),
    ])
