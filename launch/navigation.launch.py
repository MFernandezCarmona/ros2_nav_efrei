from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    rviz_config_dir = os.path.join(
        get_package_share_directory('ros2_nav_efrei'),
        'rviz',
        'nav_view.rviz')
    
    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            get_package_share_directory('ros2_nav_efrei'),
            'params',
            'nav_param.yaml'))

    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory('ros2_nav_efrei'),
            'map',
            'cave_world.yaml'))

    ld = LaunchDescription()
    # simulated world
    stage_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare("stage_ros2"), '/launch', '/stage.launch.py']))      
    ld.add_action(stage_launch)
    
    # navigation 2
    nav2_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': param_dir}.items(),
        )
    ld.add_action(nav2_launch)

    # rviz node
    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen')
    ld.add_action(rviz_node)

    # launch arguments
    map_arg = DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load')
    ld.add_action(map_arg)

    nav_param_arg = DeclareLaunchArgument(
            'params_file',
            default_value=param_dir,
            description='Full path to param file to load')
    ld.add_action(nav_param_arg)

    use_sim_time_arg = DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true')
    ld.add_action(use_sim_time_arg)

    return ld