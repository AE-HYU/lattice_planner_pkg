from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('lattice_planner_pkg')
    
    # Config file path
    config_file = os.path.join(pkg_dir, 'config', 'planner_config.yaml')
    
    # Launch arguments
    mod_arg = DeclareLaunchArgument(
        'mod',
        default_value='real',
        description='Launch mode: real (use /pf/pose/odom), sim (use /ego_racecar/odom), sim_pf (use /pf/pose/odom)'
    )
    
    race_line_arg = DeclareLaunchArgument(
        'race_line',
        default_value='fmap_5.0_0.3_7.0_7.0',
        description='Raceline CSV filename (without extension) from config/reference_paths/'
    )
    
    
    # Lattice planner node
    lattice_planner_node = Node(
        package='lattice_planner_pkg',
        executable='lattice_planner_node',
        name='lattice_planner',
        output='screen',
        parameters=[
            config_file,
            {
                'use_sim_time': PythonExpression([
                    "'true' if '", LaunchConfiguration('mod'), "' in ['sim', 'sim_pf'] else 'false'"
                ]),
                'reference_path_file': LaunchConfiguration('race_line')
            }
        ],
        remappings=[
            ('/odom', PythonExpression([
                "'/ego_racecar/odom' if '", LaunchConfiguration('mod'), "' == 'sim' else '/pf/pose/odom'"
            ])),
            ('/map', '/updated_map'),
        ]
    )
    
    return LaunchDescription([
        mod_arg,
        race_line_arg,
        lattice_planner_node,
    ])