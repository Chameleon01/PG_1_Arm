from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    # Create Node action for micro_ros_agent
    micro_ros_agent_node = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent_node',
        arguments=['serial', '--dev', '/dev/ttyACM0', 'baudrate=115200']
    )
    
    # Return the LaunchDescription object
    return LaunchDescription([
        micro_ros_agent_node,
    ])
