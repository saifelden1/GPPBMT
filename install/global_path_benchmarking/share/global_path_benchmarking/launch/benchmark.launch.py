import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    # Retrieve configuration paths
    pkg_share = get_package_share_directory('global_path_benchmarking')
    default_config = os.path.join(pkg_share, 'config', 'scenarios.yaml')
    
    config_file = LaunchConfiguration('config').perform(context)
    scenario_id = LaunchConfiguration('scenario_id').perform(context)
    verify_str = LaunchConfiguration('verify').perform(context).lower()
    use_astar_str = LaunchConfiguration('use_astar').perform(context).lower()
    clean_str = LaunchConfiguration('clean').perform(context).lower()
    
    # Run cleanup synchronously if requested
    if clean_str == 'true':
        print("[INFO] Launch cleanup: killing old processes before startup...")
        import subprocess
        targets = ["planner_server", "controller_server", "bt_navigator", "waypoint_follower", "nav2_container", "gzserver", "gzclient", "gazebo", "mock_planner"]
        for target in targets:
            subprocess.run(["pkill", "-f", "-9", target], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        import time
        time.sleep(1.0)
        
    # Resolve full path to config if it is relative
    if not os.path.isabs(config_file) and not os.path.exists(config_file):
        config_file = os.path.join(pkg_share, config_file)
        
    # Build CLI arguments list for the benchmarker node
    benchmarker_args = ['--config', config_file]
    if scenario_id:
        benchmarker_args.extend(['--scenario_id', scenario_id])
    if verify_str == 'true':
        benchmarker_args.append('--verify')
        
    # Mock Planner Node
    mock_planner = Node(
        package='global_path_benchmarking',
        executable='mock_planner',
        name='mock_planner',
        parameters=[{
            'use_astar': (use_astar_str == 'true')
        }],
        output='screen'
    )
    
    # Benchmarker Node
    benchmarker = Node(
        package='global_path_benchmarking',
        executable='benchmarker',
        name='global_path_benchmarker',
        arguments=benchmarker_args,
        output='screen'
    )
    
    # If in verify mode, we don't need the mock planner running
    if verify_str == 'true':
        return [benchmarker]
    else:
        return [mock_planner, benchmarker]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value='config/scenarios.yaml',
            description='Path to the scenarios YAML file'
        ),
        DeclareLaunchArgument(
            'scenario_id',
            default_value='',
            description='Specific scenario ID to run (runs all if empty)'
        ),
        DeclareLaunchArgument(
            'verify',
            default_value='false',
            description='Set to true to generate verification images and exit'
        ),
        DeclareLaunchArgument(
            'use_astar',
            default_value='true',
            description='Toggle A* vs Straight-Line for the mock planner'
        ),
        DeclareLaunchArgument(
            'clean',
            default_value='false',
            description='Kill old navigation/Gazebo processes before launch'
        ),
        OpaqueFunction(function=launch_setup)
    ])

