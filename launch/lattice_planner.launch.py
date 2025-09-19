from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('lattice_planner_pkg')
    
    # Config file path
    config_file = os.path.join(pkg_dir, 'config', 'planner_config.yaml')
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    sim_mode_arg = DeclareLaunchArgument(
        'sim_mode',
        default_value='false',
        description='Use simulation mode if true, real car mode if false'
    )
    
    odom_mode_arg = DeclareLaunchArgument(
        'odom_mode',
        default_value='/ego_racecar/odom',
        description='Odometry topic to use in sim mode'
    )
    
    race_line_arg = DeclareLaunchArgument(
        'race_line',
        default_value='fmap_5.0_0.3_7.0_7.0',
        description='Raceline CSV filename (without extension) from config/reference_paths/'
    )
    
    
    # Lattice planner node for simulation mode
    lattice_planner_sim_node = Node(
        package='lattice_planner_pkg',
        executable='lattice_planner_node',
        name='lattice_planner',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time'),
             'reference_path_file': LaunchConfiguration('race_line')}
        ],
        remappings=[
            ('/odom', LaunchConfiguration('odom_mode')),
            ('/map', '/updated_map'),
        ],
        condition=IfCondition(LaunchConfiguration('sim_mode'))
    )
    
    # Lattice planner node for real car mode
    lattice_planner_real_node = Node(
        package='lattice_planner_pkg',
        executable='lattice_planner_node',
        name='lattice_planner',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time'),
             'reference_path_file': LaunchConfiguration('race_line')}
        ],
        remappings=[
            ('/odom', '/pf/pose/odom'),
            ('/map', '/updated_map'),
        ],
        condition=UnlessCondition(LaunchConfiguration('sim_mode'))
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        sim_mode_arg,
        odom_mode_arg,
        race_line_arg,
        lattice_planner_sim_node,
        lattice_planner_real_node,
    ])