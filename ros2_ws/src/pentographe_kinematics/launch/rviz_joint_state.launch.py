"""Lance le nœud RViz (tous les angles) avec les dimensions du modèle Creo."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('pentographe_kinematics'),
        'config', 'pentographe_creo.yaml')
    return LaunchDescription([
        Node(
            package='pentographe_kinematics',
            executable='rviz_joint_state_node',
            name='pentographe_rviz_joint_state',
            parameters=[params],
            output='screen',
        ),
    ])
