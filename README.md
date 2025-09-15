# Lattice Planner

Lattice-based local path planner for F1TENTH with real-time obstacle avoidance and optimized Frenet transformations.

## Quick Start

```bash
# Build
colcon build --packages-select lattice_planner_pkg
source install/setup.bash

# Real hardware
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

### Real Hardware Mode
- **Input**: `/pf/pose/odom` - Localized pose from MCL
- **Output**: `/local_waypoints`, `/global_waypoints` - Planned paths
- **Frenet**: `/car_state/frenet/odom` - Vehicle state in Frenet coordinates

### Simulation Mode
- **Input**: `/ego_racecar/odom` - Simulation pose data
- **Output**: `/local_waypoints`, `/global_waypoints` - Planned paths
- **Frenet**: `/car_state/frenet/odom` - Vehicle state in Frenet coordinates

### Visualization
- `/path_candidates` - Path sampling visualization
- `/reference_path` - Reference raceline markers

## Key Configuration

Edit `config/planner_config.yaml`:

```yaml
# Core Planning
planning_frequency: 20.0      # Update rate (Hz)
planning_horizon: 3.0         # Lookahead distance (m)
lateral_step: 0.3             # Path sampling resolution (m)
max_lateral_offset: 1.5       # Max deviation from reference (m)

# Vehicle parameters (auto-set by sim_mode)
wheelbase: 0.32               # Vehicle wheelbase (m)
max_velocity: 5.0             # Max speed (m/s)
safety_margin: 0.2            # Obstacle buffer (m)
```

## Reference Paths

Place CSV files in `config/reference_paths/` with format:
```
x_m, y_m, s_m, psi_rad, kappa_radpm, vx_mps, width_left, width_right
```

Available paths:
- `sibal1.csv` (default racing circuit)
- `Spielberg_map.csv` (F1 Austria GP)
- `levine.csv` (multi-floor building)

## Algorithm

Lattice-based planning with Frenet coordinate system:
- **Path sampling**: Generate lateral offset candidates from reference line
- **Collision checking**: Dynamic obstacle avoidance with safety margins
- **Trajectory optimization**: Speed-aware path selection with curvature constraints

## Prerequisites

- Monte Carlo Localization running
- Reference path file loaded
- Map server (optional for collision checking)
- Obstacle detection (optional for dynamic avoidance)