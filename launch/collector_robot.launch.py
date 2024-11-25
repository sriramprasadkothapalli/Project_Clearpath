
import os
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    set_turtlebot3_model = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle_pi')
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    tbot3_launch_dir = os.path.join(get_package_share_directory('project_clearpath'), 'launch')
    trash_launch_dir = os.path.join(get_package_share_directory('project_clearpath'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='2.5')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    yaw = LaunchConfiguration('yaw', default='-3.14159')

    collection_world = os.path.join(get_package_share_directory('project_clearpath'),
                                              'worlds', 'empty_world.world')

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': collection_world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tbot3_launch_dir, 'spawn_tbot3_custom.py')),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose,
            'yaw':    yaw
        }.items()
    )
    trash_ld = IncludeLaunchDescription(PythonLaunchDescriptionSource([
              trash_launch_dir,
              '/spawn_trash.launch.py']))
    collector_node = Node(
        package='project_clearpath',
        executable='collector_node',
        name='collector_node'
    )

    ld = LaunchDescription()
    ld.add_action(set_turtlebot3_model)
    # Add the commands to the launch description
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)
    ld.add_action(trash_ld)
    ld.add_action(collector_node)

    return ld
