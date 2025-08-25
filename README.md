# Drone Manager

**Centralized management system for drone control and component coordination.**

## Overview

The `babyk_drone_manager` is the main package that manages the entire drone system, coordinating all components and providing a unified interface for control. It centralizes launch files, configurations, and TMUX files for simplified system management.

## Architecture

```
babyk_drone_manager/
├── src/
│   └── move_manager_node.cpp      # Movement management node
├── include/babyk_drone_manager/
│   └── move_manager_node.h        # Move manager header
├── launch/                        # Centralized launch files
│   ├── move_manager.launch.py     # Movement management
│   ├── full_system.launch.py      # Complete system
│   ├── rtabmap_sim.launch.py      # SLAM simulation
│   ├── tf_static_sim.launch.py    # Static TF simulation
│   └── px4_tf_pub_simulation.launch.py # PX4 TF simulation
├── config/                        # Centralized configurations
│   ├── move_manager_params.yaml   # Real flight parameters
│   └── move_manager_simulation.yaml # Simulation parameters
├── rviz/
│   └── leo.rviz                   # RViz configuration
├── simulation.yml                 # TMUX simulation
└── flight.yml                    # TMUX real flight
```

## Main Components

### Move Manager Node
**Node**: `move_manager_node`  
**Description**: Coordinates drone movements and manages command interface.

**Main Topics**:
- `/move_manager/command` (input) - Movement commands
- `/move_manager/status` (output) - System status
- `/move_base_simple/goal` (output) - Goals for path planner
- `/trajectory_path` (output) - Trajectories for interpolator

**Supported Commands**:
```bash
# Takeoff (maintains current yaw, single waypoint)
ros2 topic pub /move_manager/command std_msgs/msg/String "{data: 'takeoff'}" --once

# Direct movement (without path planning)
ros2 topic pub /move_manager/command std_msgs/msg/String "{data: 'go(x,y,z)'}" --once

# Movement with path planning
ros2 topic pub /move_manager/command std_msgs/msg/String "{data: 'flyto(frame_name)'}" --once

# Landing (maintains current yaw, single waypoint)
ros2 topic pub /move_manager/command std_msgs/msg/String "{data: 'land'}" --once

# Emergency stop
ros2 topic pub /move_manager/command std_msgs/msg/String "{data: 'stop'}" --once

# Command a direct tilting pitch setpoint (TODO create smooth traj for pitch-tilting)
ros2 topic pub --once /fmu/in/tilting_mc_desired_angles px4_msgs/msg/TiltingMcDesiredAngles "{timestamp: $(($(date +%s%N)/1000)), roll_body: 0.1, pitch_body: -0.1}" --once

# Teleop control (activates joystick control)
ros2 topic pub /move_manager/command std_msgs/msg/String "{data: 'teleop'}" --once
```

### Teleop Integration

The move manager includes automatic joystick detection and teleop coordination:

**Joystick Detection**:
- Automatically detects when a joystick is connected to `/joy` topic
- Publishes joystick status on `/move_manager/joystick_connected` (Bool)
- When joystick detected, teleop mode can be activated

**Teleop Command**:
- `teleop` command activates joystick control mode
- Stops any current path execution
- Publishes teleop activation flag on `/move_manager/teleop_active` (Bool)
- Seamlessly integrates with the enhanced_teleop node and trajectory interpolator

**Teleop Integration Flow**:
1. Move manager detects joystick presence
2. User sends `teleop` command to activate teleop mode
3. Move manager publishes `teleop_active: true` flag
4. Enhanced_teleop node responds to flag and begins velocity control
5. Trajectory interpolator integrates velocity increments for smooth control

**Related Topics**:
- `/joy` (input) - Joystick messages for detection
- `/move_manager/joystick_connected` (output) - Joystick status
- `/move_manager/teleop_active` (output) - Teleop mode activation flag
- `/teleop/velocity_increments` (input) - Velocity commands from teleop

## Launch Files

### Complete System
```bash
ros2 launch babyk_drone_manager full_system.launch.py
```
Launches all components: move_manager + path_planner.

### Move Manager
```bash
ros2 launch babyk_drone_manager move_manager.launch.py config_file:=config/move_manager_params.yaml simulation:=false
```

### RTABMap Simulation
```bash
ros2 launch babyk_drone_manager rtabmap_sim.launch.py use_sim_time:=true
```

## TMUX Configurations

### Complete Simulation
```bash
tmuxp load simulation.yml
```

**System started**:
- PX4 SITL + Gazebo
- MicroXRCE Agent
- Gazebo-ROS Bridge
- RTABMap SLAM
- RViz
- TF Publishers
- Move Manager
- Path Planner
- Trajectory Interpolator
- PlotJuggler

### Real Flight
```bash
tmuxp load flight.yml
```

**System started**:
- Move Manager
- Path Planner  
- Trajectory Interpolator
- SLAM (Leonardo)
- RViz

## Configuration Parameters

### move_manager_params.yaml (Real Flight)
```yaml
move_manager_node:
  ros__parameters:
    command_topic: "/move_manager/command"
    status_topic: "/move_manager/status"
    takeoff_altitude: 1.5
    simulation: false
```

### move_manager_simulation.yaml (Simulation)
```yaml
move_manager_node:
  ros__parameters:
    command_topic: "/move_manager/command"
    status_topic: "/move_manager/status"
    takeoff_altitude: 1.5
    simulation: true  # Enables TF publishing
```

## System Integration

The `babyk_drone_manager` coordinates:

1. **Path Planner** (`path_planner`) - Path planning with OMPL+FCL
2. **Trajectory Interpolator** (`traj_interp`) - Interpolation and trajectory resampling
3. **Drone Odometry** (`drone_odometry2`) - TF publishing and PX4 odometry
4. **RTABMap** - Visual SLAM for environmental mapping

## System States

- `IDLE` - System ready
- `PLANNING_PATH` - Path planning in progress
- `EXECUTING_TRAJECTORY` - Trajectory execution
- `TAKING_OFF` - Takeoff in progress
- `LANDING` - Landing in progress
- `STOPPED` - System stopped
- `ERROR_*` - Various error states

## Troubleshooting

### Common Issues

1. **Move Manager not found**:
   ```bash
   colcon build --packages-select babyk_drone_manager
   source install/setup.bash
   ```

2. **TF not published in simulation**:
   - Verify `simulation: true` in parameters
   - Check that px4_tf_pub_simulation.launch.py is active

3. **Commands not responding**:
   - Verify topic: `ros2 topic echo /move_manager/status`
   - Check odometry: `ros2 topic echo /px4/odometry/out`

4. **Path planning fails**:
   - Verify octomap: `ros2 topic echo /octomap_binary`
   - Check workspace limits in path_planner config

## Dependencies

**ROS 2 Packages**:
- `rclcpp`, `nav_msgs`, `geometry_msgs`, `std_msgs`
- `tf2`, `tf2_ros`, `tf2_geometry_msgs`
- `rtabmap_ros`, `rtabmap_util`, `sensor_msgs`
- `image_transport`, `visualization_msgs`

**Custom Packages**:
- `path_planner` - Path planning
- `traj_interp` - Trajectory interpolation  
- `drone_odometry2` - TF and PX4 odometry

## Build and Installation

```bash
# Build
cd ~/ros2_ws
colcon build --packages-select babyk_drone_manager

# Source
source install/setup.bash

# Verify installation
ros2 launch babyk_drone_manager move_manager.launch.py --help
```

## Development Notes

- **Simulation Mode**: Enables automatic TF publishing for Gazebo integrations
- **Real Mode**: Disables TF publishing, uses real hardware
- **Replan Logic**: Maximum 5 attempts for path planning
- **Emergency Stop**: `stop` command immediately interrupts any movement
