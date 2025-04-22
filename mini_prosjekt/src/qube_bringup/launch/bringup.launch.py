from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package_name = 'qube_bringup'
    driver_package = 'qube_driver'
    urdf_file = 'controlled_qube.urdf.xacro'

    urdf_path = PathJoinSubstitution([
        FindPackageShare(package_name),
        'urdf',
        urdf_file
    ])

    # Include Qube-driver launch file
    qube_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare(driver_package),
                'launch',
                'qube_driver.launch.py'
            ])
        ),
        launch_arguments={
            'use_sim': LaunchConfiguration('use_sim'),
            'baudrate': LaunchConfiguration('baudrate'),
            'device': LaunchConfiguration('device')
        }.items()
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command([
                'xacro ', urdf_path,
                ' simulation:=', LaunchConfiguration('use_sim'),
                ' baud_rate:=', LaunchConfiguration('baudrate'),
                ' device:=', LaunchConfiguration('device')
            ]),
            'use_sim_time': LaunchConfiguration('use_sim')
        }]
    )

    # Controller node
    qube_controller = Node(
        package='qube_controller',
        executable='qube_controller',
        name='qube_controller',
    )

    # RViz node
    rviz_config_path = PathJoinSubstitution([
        FindPackageShare(package_name),
        'rviz',
        'view_qube.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim',
            default_value='true',
            description='Whether to run in simulation mode or not'
        ),
        DeclareLaunchArgument(
            'baudrate',
            default_value='115200',
            description='Baudrate for serial communication'
        ),
        DeclareLaunchArgument(
            'device',
            default_value='/dev/ttyACM0',
            description='Device path for serial communication'
        ),
        qube_driver_launch,
        robot_state_publisher,
        qube_controller,
        rviz_node
    ])

