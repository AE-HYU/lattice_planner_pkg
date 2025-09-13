# Lattice Planner Package

Lattice-based local path planner for autonomous racing with real-time obstacle avoidance.

## Quick Start

```bash
# Build
colcon build --packages-select lattice_planner_pkg
source install/setup.bash

# Real car
ros2 launch lattice_planner_pkg lattice_planner.launch.py

# Simulation
ros2 launch lattice_planner_pkg lattice_planner.launch.py sim_mode:=true
```

## Launch Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `sim_mode` | `false` | Use simulation topics if true |
| `use_sim_time` | `false` | Use simulation time |

## Topics

### Input
- `/pf/pose/odom` (real) / `/ego_racecar/odom` (sim) - Vehicle pose
- `/updated_map` - Occupancy grid for collision checking
- `/detected_obstacles` - Dynamic obstacles (optional)

### Output
- `/planned_waypoints` - Path for controller (`ae_hyu_msgs/WpntArray`)
- `/path_candidates` - Visualization markers
- `/reference_path` - Reference raceline

## Key Configuration

Edit `config/planner_config.yaml`:

```yaml
# Planning
planning_frequency: 20.0      # Update rate (Hz)
planning_horizon: 3.0         # Lookahead distance (m)
lateral_step: 0.3             # Path sampling resolution (m)
max_lateral_offset: 1.5       # Max deviation from reference (m)

# Vehicle
wheelbase: 0.32               # Vehicle wheelbase (m)
max_velocity: 5.0             # Max speed (m/s)
safety_margin: 0.2            # Obstacle buffer (m)

# Reference path file
reference_path_file: "sibal1.csv"
```

## Reference Path

Place CSV files in `config/reference_paths/` with format:
```
x_m, y_m, s_m, psi_rad, kappa_radpm, vx_mps, width_left, width_right
```

Available paths: `sibal1.csv`, `Spielberg_map.csv`, `levine.csv`

## Prerequisites

- Monte Carlo Localization running
- Reference path loaded
- Map server (optional)
- Obstacle detection (optional)