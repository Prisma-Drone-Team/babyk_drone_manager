from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    simulation_arg = DeclareLaunchArgument(
        'simulation',
        default_value='true',
        description='Whether to use simulation time'
    )

    # Note: the parameters are loaded directly inside the node constructor 
    # since we used get_parameter() with default values in the code.
    # If we wanted an external config file, we could add it here.
    
    # Create the test node
    test_node = Node(
        package='babyk_drone_manager',
        executable='sewer_autonomous_test_node',
        name='sewer_autonomous_test_node',
        parameters=[
            {'use_sim_time': LaunchConfiguration('simulation')}
        ],
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([
        simulation_arg,
        test_node
    ])
