import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    """
    TF static publisher for the sewer inspection scenario.

    Frame relationships (mirrors exploration.yml / corridor setup):
      global  -- identity -->  map         (OpenVINS global = map at startup)
      map     -- (0,1,0) -->   drone/map   (drone takes off in drone/map, goals in map)
      map     -- (1.5,0,6.0) --> sewer_entry (goal above sewer entrance;
                                 sewer tube is at Gazebo world X=3.0, drone spawns at world X=1.5
                                 → sewer is 1.5m ahead in MAP frame)

    The drone spawns at world (1.5, 0, 5.2). Drone is 1.5m closer to the checkerboard
    wall (X=5.0) so camera_up sees it at ~30° elevation for better VIO initialization.
    """

    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),



        # drone/map is IDENTITY with map for the sewer world (no Y offset needed).
        # In the corridor, drone/map is at Y=+1 in map because all goals have Y=1.
        # In the sewer, there is no such constraint: the drone starts at (0,0,0) in world
        # and all goals are expressed in map, so map == drone/map keeps things simple.
        # drone/map is the drone's start pose. Drone spawns at Gazebo world Z=5.8.
        # By setting drone/map at Z=5.8 in map, we force map Z=0 to exactly match Gazebo Z=0!
        Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['0', '0', '5.8', '0', '0', '0', 'map', 'drone/map']),

        # Sewer entry: fixed relative to the drone's takeoff position (odom frame).
        # It is 4.0m forward and 0.7m higher than the takeoff point.
        Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['4.0', '0', '7.3', '0', '0', '0', 'odom', 'sewer_entry']),

        # Modalità dinamica:
        # vio_aligner_node aspetterà l'inizializzazione di OpenVINS e calcolerà
        # l'offset dinamicamente, allineando "odom" con "drone/map" a prescindere
        # dall'angolo (yaw) casuale con cui OpenVINS si inizializza.
        Node(
            package='babyk_drone_manager', executable='vio_aligner_node', output='screen',
            parameters=[{'gt_topic': '/model/babyk_sewer_0/odometry'}]),

    ])
