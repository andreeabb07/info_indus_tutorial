"""Lance le nœud cinématique avec le fichier de paramètres du package."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('pentographe_kinematics'),
        'config', 'pentographe_kinematics.yaml')
    return LaunchDescription([
        Node(
            package='pentographe_kinematics',
            executable='kinematics_node',
            name='pentographe_kinematics',
            parameters=[params],
            output='screen',
        ),
    ])
