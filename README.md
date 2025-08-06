# Drone Manager

**Sistema di gestione centralizzato per il controllo del drone e coordinamento dei componenti.**

## Panoramica

Il `drone_menager` è il pacchetto principale che gestisce l'intero sistema del drone, coordinando tutti i componenti e fornendo un'interfaccia unificata per il controllo. Centralizza launch files, configurazioni e file TMUX per una gestione semplificata del sistema.

## Architettura

```
drone_menager/
├── src/
│   └── move_manager_node.cpp      # Nodo di gestione movimenti
├── include/drone_menager/
│   └── move_manager_node.h        # Header del move manager
├── launch/                        # Launch files centralizzati
│   ├── move_manager.launch.py     # Gestione movimenti
│   ├── full_system.launch.py      # Sistema completo
│   ├── rtabmap_sim.launch.py      # SLAM simulazione
│   ├── tf_static_sim.launch.py    # TF statici simulazione
│   └── px4_tf_pub_simulation.launch.py # TF PX4 simulazione
├── config/                        # Configurazioni centralizzate
│   ├── move_manager_params.yaml   # Parametri volo reale
│   └── move_manager_simulation.yaml # Parametri simulazione
├── rviz/
│   └── leo.rviz                   # Configurazione RViz
├── simulation.yml                 # TMUX simulazione
└── flight.yml                    # TMUX volo reale
```

## Componenti Principali

### Move Manager Node
**Nodo**: `move_manager_node`  
**Descrizione**: Coordina i movimenti del drone e gestisce l'interfaccia di comando.

**Topics Principali**:
- `/move_manager/command` (input) - Comandi di movimento
- `/move_manager/status` (output) - Stato del sistema
- `/move_base_simple/goal` (output) - Goal per path planner
- `/trajectory_path` (output) - Traiettorie per interpolator

**Comandi Supportati**:
```bash
# Decollo
ros2 topic pub /move_manager/command std_msgs/msg/String "{data: 'takeoff'}" --once

# Movimento diretto (senza path planning)
ros2 topic pub /move_manager/command std_msgs/msg/String "{data: 'go(x,y,z)'}" --once

# Movimento con path planning
ros2 topic pub /move_manager/command std_msgs/msg/String "{data: 'flyto(frame_name)'}" --once

# Atterraggio
ros2 topic pub /move_manager/command std_msgs/msg/String "{data: 'land'}" --once

# Stop emergenza
ros2 topic pub /move_manager/command std_msgs/msg/String "{data: 'stop'}" --once
```

## Launch Files

### Sistema Completo
```bash
ros2 launch drone_menager full_system.launch.py
```
Lancia tutti i componenti: move_manager + path_planner.

### Move Manager
```bash
ros2 launch drone_menager move_manager.launch.py config_file:=config/move_manager_params.yaml simulation:=false
```

### Simulazione RTABMap
```bash
ros2 launch drone_menager rtabmap_sim.launch.py use_sim_time:=true
```

## Configurazioni TMUX

### Simulazione Completa
```bash
tmuxinator start -p simulation.yml
```

**Sistema avviato**:
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

### Volo Reale
```bash
tmuxinator start -p flight.yml
```

**Sistema avviato**:
- Move Manager
- Path Planner  
- Trajectory Interpolator
- SLAM (Leonardo)
- RViz

## Parametri di Configurazione

### move_manager_params.yaml (Volo Reale)
```yaml
move_manager_node:
  ros__parameters:
    command_topic: "/move_manager/command"
    status_topic: "/move_manager/status"
    takeoff_altitude: 1.5
    simulation: false
```

### move_manager_simulation.yaml (Simulazione)
```yaml
move_manager_node:
  ros__parameters:
    command_topic: "/move_manager/command"
    status_topic: "/move_manager/status"
    takeoff_altitude: 1.5
    simulation: true  # Abilita pubblicazione TF
```

## Integrazione Sistema

Il `drone_menager` coordina:

1. **Path Planner** (`path_planner`) - Pianificazione percorsi con OMPL+FCL
2. **Trajectory Interpolator** (`traj_interp`) - Interpolazione e resampling traiettorie
3. **Drone Odometry** (`drone_odometry2`) - Pubblicazione TF e odometria PX4
4. **RTABMap** - SLAM visivo per mapping ambientale

## Stati del Sistema

- `IDLE` - Sistema pronto
- `PLANNING_PATH` - Pianificazione percorso in corso
- `EXECUTING_TRAJECTORY` - Esecuzione traiettoria
- `TAKING_OFF` - Decollo in corso
- `LANDING` - Atterraggio in corso
- `STOPPED` - Sistema fermato
- `ERROR_*` - Stati di errore vari

## Troubleshooting

### Problemi Comuni

1. **Move Manager non trovato**:
   ```bash
   colcon build --packages-select drone_menager
   source install/setup.bash
   ```

2. **TF non pubblicati in simulazione**:
   - Verificare `simulation: true` nei parametri
   - Controllare che px4_tf_pub_simulation.launch.py sia attivo

3. **Comandi non risponde**:
   - Verificare topic: `ros2 topic echo /move_manager/status`
   - Controllare odometria: `ros2 topic echo /px4/odometry/out`

4. **Path planning fallisce**:
   - Verificare octomap: `ros2 topic echo /octomap_binary`
   - Controllare limiti workspace in path_planner config

## Dipendenze

**Pacchetti ROS 2**:
- `rclcpp`, `nav_msgs`, `geometry_msgs`, `std_msgs`
- `tf2`, `tf2_ros`, `tf2_geometry_msgs`
- `rtabmap_ros`, `rtabmap_util`, `sensor_msgs`
- `image_transport`, `visualization_msgs`

**Pacchetti Custom**:
- `path_planner` - Pianificazione percorsi
- `traj_interp` - Interpolazione traiettorie  
- `drone_odometry2` - TF e odometria PX4

## Build e Installazione

```bash
# Build
cd ~/ros2_ws
colcon build --packages-select drone_menager

# Source
source install/setup.bash

# Verifica installazione
ros2 launch drone_menager move_manager.launch.py --help
```

## Note Sviluppo

- **Modalità Simulazione**: Abilita pubblicazione TF automatica per integrazioni Gazebo
- **Modalità Reale**: Disabilita TF publishing, usa hardware reale
- **Replan Logic**: Limite massimo 5 tentativi per path planning
- **Emergency Stop**: Comando `stop` interrompe immediatamente ogni movimento
