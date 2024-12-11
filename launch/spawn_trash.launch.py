import os
import random

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_block(i, model_name):
    model_path = os.path.join(get_package_share_directory('project_clearpath'), 'models')

    # Define random x and y positions within a range
    x_pos = random.uniform(-4.0, 4.0)  
    y_pos = random.uniform(-4.0, 4.0)  

    return Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', f'{model_name}_{i}',
                   '-file', os.path.join(model_path, model_name, 'model.sdf'),
                   '-x', str(x_pos),
                   '-y', str(y_pos),
                   '-z', '0.0',
                   '-Y', '0.0'],
        output='screen'
    )

def generate_launch_description():
    objects_to_spawn = ['trash_block', 'beer', 'coke_can', 'wooden_peg', 'cricket_ball'] 
    blocks = []
    
    for i, model_name in enumerate(objects_to_spawn):
        blocks.append(generate_block(i, model_name))

    return LaunchDescription(blocks)

