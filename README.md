# Lattice Planner Package

Clean local path planner using lattice-based planning for autonomous vehicles.

## Launch Instructions

### Basic Launch
```bash
ros2 launch lattice_planner_pkg lattice_planner.launch.py
```

### With Parameters

#### Simulation Mode
```bash
ros2 launch lattice_planner_pkg lattice_planner.launch.py sim_mode:=true use_sim_time:=true
```

#### Real Car Mode (default)
```bash
ros2 launch lattice_planner_pkg lattice_planner.launch.py sim_mode:=false use_sim_time:=false
```

### Launch Parameters

- `sim_mode`: Use simulation mode (`ego_racecar/odom`) if true, real car mode (`/pf/pose/odom`) if false (default: false)
- `use_sim_time`: Use simulation time if true (default: false)

## Node Information

- **Node Name**: `local_planner`
- **Executable**: `local_planner_node`
- **Config File**: `config/planner_config.yaml`

## Topics

### Subscribed Topics
- `/odom`: Odometry data (remapped based on mode)
- `/map`: Map data (remapped to `/updated_map`)

### Published Topics
- Local path planning outputs (see node implementation for details)