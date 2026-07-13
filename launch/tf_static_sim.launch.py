import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),

        Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['18', '1', ' 1.5', '0','0', '0', 'map', 'goal1']),
        Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['0.5', '1', ' 1.5',  '0', '0', '0', 'map', 'goal2']),
            
    	Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['8', '1', ' 1.5',  '0', '0', '0', 'map', 'goal3']), 
            
    	Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['6.5', '1', ' 1.5',  '0', '0', '0', 'map', 'goal4']), 
    	Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['12', '1', ' 1.5',  '0', '0', '0', 'map', 'goal5']), 
        Node(
           package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['16.5', '1', ' 1.5',  '0', '0', '0', 'map', 'goal6']), 
        Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['15', '1', ' 1.5',  '0', '0', '0', 'map', 'goal7']), 
        
        Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['0', '1', '0', '0', '0', '0', '1', 'map', 'drone/map']), 
            
        Node(
            package='babyk_drone_manager', executable='vio_aligner_node', output='screen'),
    ])