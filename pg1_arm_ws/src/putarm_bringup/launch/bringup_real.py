from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Define the path to the demo.launch.py file in putarm_moveit_config package
    putarm_moveit_config_dir = get_package_share_directory('putarm_moveit_config')
    putarm_moveit_config_launch_path = os.path.join(putarm_moveit_config_dir, 'launch', 'demo.launch.py')
    
    # Create IncludeLaunchDescription action
    putarm_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(putarm_moveit_config_launch_path),
        launch_arguments={'some_argument': 'some_value'}.items()
    )
    
    # Create Node action for mcu_pub
    mcu_pub_node = Node(
        package='mcu_pub',
        executable='mcu_pub',
        name='mcu_pub_node'
    )
    
    # Create Node action for micro_ros_agent
    micro_ros_agent_node = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent_node',
        arguments=['serial', '--dev', '/dev/ttyACM0', 'baudrate=115200']
    )
    
    # Return the LaunchDescription object
    return LaunchDescription([
        mcu_pub_node,
        putarm_moveit_launch
    ])
