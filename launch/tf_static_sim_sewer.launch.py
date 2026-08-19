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
            arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'drone/map']),

        # Sewer entry: sewer at world X=4.0, drone at world X=0.0 → map X=4.0.
        # Tube top is at world Z=5.91. Hover target world Z=6.2.
        # Since map frame now perfectly matches Gazebo world, we can use exact world coordinates!
        Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['4.0', '0', '6.2', '0', '0', '0', '1', 'map', 'sewer_entry']),

        # VIO aligner in hardware mode: by passing use_sim_time=False the node
        # immediately publishes identity TFs for drone/map→global and global→odom
        # without waiting for OpenVINS odometry. In the sewer the drone always
        # starts at yaw=0 so the offset is 0 — identical to what the dynamic
        # aligner would compute. This mirrors the corridor behaviour exactly.
        # NOTE: global→imu is still published only by OpenVINS once it initialises.
        Node(
            package='babyk_drone_manager', executable='vio_aligner_node', output='screen',
            parameters=[{'use_sim_time': False}]),

        # global -> imu (identity bootstrap): bridges the two disconnected TF trees
        # (map->...->global and imu->base_link) so that map->base_link is available
        # immediately from t=2s. OpenVINS overwrites this with the real dynamic
        # pose estimate once it initialises (~20 s). Safe because the drone sits
        # at the VIO origin (0,0,0 in global frame) during the ZUPT phase.
        # There is NO conflict: in TF2, dynamic transforms on /tf always take
        # precedence over static ones on /tf_static for recent timestamps.
        Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            # Bootstrap: global→imu at identity (Z=0).
            # The drone/map→global chain already accounts for the 5.2m elevation
            # (map→drone/map at Z=5.2). Adding height here would double-count it.
            arguments=['0', '0', '0', '0', '0', '0', '1', 'global', 'imu']),
    ])
