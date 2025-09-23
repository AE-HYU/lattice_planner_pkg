# Lattice Planner

Lattice-based local path planner for F1TENTH with real-time obstacle avoidance and optimized Frenet transformations.

## Quick Start

```bash
# Build
colcon build --packages-select lattice_planner_pkg
source install/setup.bash

# Real hardware
ros2 launch lattice_planner_pkg lattice_planner.launch.py mod:=real

# Simulation (using simulator odometry)
ros2 launch lattice_planner_pkg lattice_planner.launch.py mod:=sim

# Simulation with MCL (using particle filter localization)
ros2 launch lattice_planner_pkg lattice_planner.launch.py mod:=sim_pf

# Custom raceline
ros2 launch lattice_planner_pkg lattice_planner.launch.py mod:=real race_line:=Spielberg_global
```

## Launch Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `mod` | `real` | Launch mode: `real`, `sim`, or `sim_pf` |
| `race_line` | `fmap_5.0_0.3_7.0_7.0` | Raceline CSV filename (without extension) |

## Topics

### Real Mode (`mod:=real`)
- **Input**: `/pf/pose/odom` - Localized pose from MCL
- **Output**: `/local_waypoints`, `/global_waypoints` - Planned paths
- **Frenet**: `/car_state/frenet/odom` - Vehicle state in Frenet coordinates
- **Timing**: Real time

### Simulation Mode (`mod:=sim`)
- **Input**: `/ego_racecar/odom` - Direct simulator odometry
- **Output**: `/local_waypoints`, `/global_waypoints` - Planned paths
- **Frenet**: `/car_state/frenet/odom` - Vehicle state in Frenet coordinates
- **Timing**: Simulation time

### Simulation + MCL Mode (`mod:=sim_pf`)
- **Input**: `/pf/pose/odom` - MCL localization in simulation
- **Output**: `/local_waypoints`, `/global_waypoints` - Planned paths
- **Frenet**: `/car_state/frenet/odom` - Vehicle state in Frenet coordinates
- **Timing**: Simulation time

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

Available paths (use with `race_line:=<filename>`):
- `fmap_5.0_0.3_7.0_7.0` (default)
- `Spielberg_global` (F1 Austria GP)
- `icra_2`, `first_map_raceline`, `slamtoolbox_raceline`
- `sibal1_*ms` (various speed profiles)
- And more in `config/reference_paths/`

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