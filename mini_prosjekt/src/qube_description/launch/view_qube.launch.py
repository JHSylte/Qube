from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package_name = 'qube_description'  # Erstatt med riktig pakkenavn hvis nødvendig
    urdf_file = 'qube.urdf.xacro'
    
    urdf_path = PathJoinSubstitution([
        FindPackageShare(package_name),
        'urdf',
        urdf_file
    ])
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', urdf_path])}]
    )
    
    rviz_config_path = PathJoinSubstitution([
        FindPackageShare(package_name),
        'rviz',
        #'view_qube.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )
    
    return LaunchDescription([
        robot_state_publisher,
        rviz_node
    ])

