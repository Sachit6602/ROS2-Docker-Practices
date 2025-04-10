from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get paths to necessary files in the package
    pkg_share = FindPackageShare('maze_solve')  # Replace with your actual package name
    world_path = PathJoinSubstitution([pkg_share, 'worlds', 'mazeworld.world'])  # Path to your world file
    urdf_path = PathJoinSubstitution([pkg_share, 'urdf', 'camera.urdf'])  # Path to your URDF file

    # Launch Gazebo with your custom world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])
        ]),
        launch_arguments={'world': world_path}.items()
    )

    # Spawn the camera URDF in Gazebo at a height, looking down at the maze/robot
    spawn_camera = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'overhead_camera',
            '-file', urdf_path,
            '-x', '0.0', '-y', '0.0', '-z', '10.0',  # Adjust z for top view (5 meters above the ground)
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,        # Launch Gazebo with your world
        spawn_camera   # Spawn the camera URDF in Gazebo
    ])
