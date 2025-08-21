from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('local_planner_pkg')
    
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
        description='Use simulation mode (ego_racecar/odom) if true, real car mode (/pf/pose/odom) if false'
    )
    
    
    # Local planner node for simulation mode
    local_planner_sim_node = Node(
        package='local_planner_pkg',
        executable='local_planner_node',
        name='local_planner',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            # ('/odom', '/ego_racecar/odom'),
            ('/odom', '/pf/pose/odom'),
            ('/map', '/updated_map'),
        ],
        condition=IfCondition(LaunchConfiguration('sim_mode'))
    )
    
    # Local planner node for real car mode
    local_planner_real_node = Node(
        package='local_planner_pkg',
        executable='local_planner_node',
        name='local_planner',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
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
        local_planner_sim_node,
        local_planner_real_node,
    ])