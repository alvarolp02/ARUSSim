import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Get the package directory
    package_name = 'arussim'  # Replace with your package name
    rviz_config_dir = os.path.join(
        get_package_share_directory(package_name),
        'config',  # The directory inside the package where RViz configs are stored
        'arrussim_rviz_config.rviz'  # Replace with your RViz config filename
    )

    # Launch configuration variables
    rviz_config_file = LaunchConfiguration('rviz_config_file', default=rviz_config_dir)

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
            arguments=None
        )
    ])

