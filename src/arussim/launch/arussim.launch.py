# @file arussim.launch.py
# @brief Launch file for ARUSim simulator and RViz visualization.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

# @brief Generates the launch description for the ARUSim simulation and RViz visualization.
# @return A LaunchDescription containing the configuration for nodes and arguments.
def generate_launch_description():

    # Get the package directory
    # @var package_name The name of the package where ARUSim is located.
    package_name = 'arussim'  # Replace with your package name
    
    # Define the path to the RViz configuration file
    # @var rviz_config_dir Full path to the RViz configuration file for ARUSim visualization.
    rviz_config_dir = os.path.join(get_package_share_directory(package_name), 
                                   'config', 
                                   'arussim_rviz_config.rviz')

    # Launch configuration variables
    # @var rviz_config_file A LaunchConfiguration object for RViz config file, set by a launch argument.
    rviz_config_file = LaunchConfiguration('rviz_config_file', default=rviz_config_dir)

    # Define the path to the parameters file (YAML)
    # @var config_file Full path to the parameters file used to configure ARUSim.
    config_file = os.path.join(get_package_share_directory(package_name), 
                               'config', 
                               'params.yaml')

    return LaunchDescription([
        # Declare the launch argument for the RViz config file
        DeclareLaunchArgument(
            'rviz_config_file',
            default_value=rviz_config_file,
            description='Full path to the RViz config file to use'
        ),

        # Launch the RViz node with the specified config file
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        ),

        # Launch the ARUSSim
        Node(
            package='arussim',
            executable='arussim_exec',
            name='arussim',
            output='screen',
            parameters=[config_file]
        )
    ])

