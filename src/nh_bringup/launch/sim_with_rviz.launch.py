from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_config = LaunchConfiguration('rviz_config')
    # 1. Define the world configuration
    world = LaunchConfiguration('world')

    clear_path_gz_share = get_package_share_directory('clearpath_gz')
    sim_launch_path = os.path.join(clear_path_gz_share, 'launch', 'simulation.launch.py')

    # 2. Pass 'world' into the simulation launch arguments
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_launch_path),
        launch_arguments={
            'namespace': namespace, 
            'use_sim_time': use_sim_time,
            'world': world
        }.items()
    )
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', rviz_config],
        remappings=[
            ('tf', '/j100_0000/tf'),
            ('tf_static', '/j100_0000/tf_static'),
        ]
    )

    default_rviz = os.path.join(get_package_share_directory('nh_bringup'), 'rviz', 'new_horizon.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='j100_0000'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz),
        # 3. Declare the argument (default is usually 'warehouse' in clearpath_gz)
        DeclareLaunchArgument('world', default_value='warehouse'),
        sim,
        rviz
    ])