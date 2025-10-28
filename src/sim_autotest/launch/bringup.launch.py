from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    targets = LaunchConfiguration('targets', default='5')
    duration = LaunchConfiguration('duration', default='60.0')
    noise = LaunchConfiguration('noise', default='0.25')
    use_tracker = LaunchConfiguration('use_tracker', default='false')

    # Launch Gazebo with floor world
    gazebo = ExecuteProcess(cmd=['gazebo', '--verbose', os.path.join('worlds', 'floor.world')], output='screen')

    nodes = [
        Node(package='sim_autotest', executable='spawner', output='screen', parameters=[{'targets': targets}]),
        Node(package='sim_autotest', executable='mover', output='screen', parameters=[{'targets': targets}]),
        Node(package='sim_autotest', executable='detector', output='screen', parameters=[{'noise': noise}]),
        Node(package='sim_autotest', executable='evaluator', output='screen', parameters=[{'duration': duration}]),
    ]

    # Optionally start EKF-JPDA tracker if present in the workspace
    # ros2 launch ekf_jpda tracker.launch.py
    tracker = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(['share/ekf_jpda/launch/tracker.launch.py']),
        condition=None,
    )

    return LaunchDescription([
        DeclareLaunchArgument('targets', default_value='5'),
        DeclareLaunchArgument('duration', default_value='60.0'),
        DeclareLaunchArgument('noise', default_value='0.25'),
        DeclareLaunchArgument('use_tracker', default_value='false'),
        gazebo,
        *nodes,
        # Users can run tracker separately or extend this to conditionally include
    ])
