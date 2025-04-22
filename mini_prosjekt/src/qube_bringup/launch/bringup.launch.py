def generate_launch_description():
    # Definerer navnene på pakkene som brukes i oppsettet, samt URDF-filens navn.
    package_name = 'qube_bringup'
    driver_package = 'qube_driver'
    urdf_file = 'controlled_qube.urdf.xacro'

    # Genererer stien til URDF-filen som skal brukes for robotbeskrivelsen.
    urdf_path = PathJoinSubstitution([
        FindPackageShare(package_name),
        'urdf',
        urdf_file
    ])

    # Inkluderer launch-filen for å starte Qube-driveren, og setter relevante launch-argumenter som
    # simulering, baudrate og enhet for seriell kommunikasjon.
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

    # Starter robot_state_publisher-noden som tar URDF-data og publiserer robotens tilstand (posisjon, orientering)
    # til andre noder. Her kjøres xacro på URDF-filen, og nødvendige parametere som simuleringstid og baudrate 
    # blir sendt inn.
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

    # Setter opp og starter Qube-controlleren, som håndterer kontrollen av roboten basert på input og tilbakemeldinger.
    qube_controller = Node(
        package='qube_controller',
        executable='qube_controller',
        name='qube_controller',
    )

    # Angir konfigurasjonen for RViz, et verktøy for visualisering av robotens tilstand og 3D-modell.
    rviz_config_path = PathJoinSubstitution([
        FindPackageShare(package_name),
        'rviz',
        'view_qube.rviz'
    ])

    # Starter RViz for å vise robotens 3D-modell.
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    # Returnerer LaunchDescription som setter opp og starter alle nodene som er nødvendige for oppsettet:
    # Qube-driveren, robot_state_publisher, qube_controller og RViz.
    return LaunchDescription([
        # Deklarerer launch-argumenter som kan konfigureres ved oppstart (simulering, baudrate, enhet)
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
        # Inkluderer Qube-driveren, robot_state_publisher, Qube-controlleren og RViz i launch-prosessen.
        qube_driver_launch,
        robot_state_publisher,
        qube_controller,
        rviz_node
    ])


