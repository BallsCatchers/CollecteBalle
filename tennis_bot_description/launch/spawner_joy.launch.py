import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
import ament_index_python.packages

ROS_DISTRO_ELOQUENT = "eloquent"
ROS_DISTRO_FOXY = "foxy"
ROS_DISTRO = os.environ.get("ROS_DISTRO")

def generate_launch_description():
    executable = "executable" if ROS_DISTRO == ROS_DISTRO_FOXY else "node_executable"

    pkg_share = launch_ros.substitutions.FindPackageShare(package='tennis_bot_description').find('tennis_bot_description')
    default_model_path = os.path.join(pkg_share, 'src/description/tennis_bot_description_2.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')

    pkg_share = launch_ros.substitutions.FindPackageShare(package='tennis_court').find('tennis_court')
    court_path = os.path.join(pkg_share, 'launch/tennis_court.launch.py')


    load_joint_state_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_joint_trajectory_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'joint_trajectory_controller'],
        output='screen'
    )

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )

    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        # condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )


    spawn_entity = launch_ros.actions.Node( # fait spawn le robot
    	package='gazebo_ros', 
    	executable='spawn_entity.py',
        arguments=['-entity', 'tennis_bot', '-x', '0', '-y', '6', '-z', '1', '-topic', 'robot_description'], # nom du robot
        output='screen'
    )

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    # Joy node
    config_directory = os.path.join(
    ament_index_python.packages.get_package_share_directory('joy'),
    'config')
    params = os.path.join(config_directory, 'joy-params.yaml')
    joy_node = launch_ros.actions.Node(
            package='joy',
            node_executable='joy_node',
            output='screen',
            # parameters=[params]
            parameters=[{'dev': '/dev/input/js0'}]
        )
    joy_gz_node = launch_ros.actions.Node(
        package="bot_control",
        condition=launch.conditions.IfCondition(LaunchConfiguration("joy")),
        parameters=[{"use_sim_time": True}],
        output="screen",
        emulate_tty=True,
        **{executable: "joy_to_gazebo.py"}
    )

    # Arm control 
    arm_control = launch_ros.actions.Node(
        package="bot_control",
        condition=launch.conditions.IfCondition(LaunchConfiguration("control")),
        parameters=[{"use_sim_time": True}],
        output="screen",
        emulate_tty=True,
        **{executable: "arm_control.py"}
    )

    # camera node
    camera_node = launch_ros.actions.Node(
        package="bot_control",
        condition=launch.conditions.IfCondition(LaunchConfiguration("control")),
        parameters=[{"use_sim_time": True}],
        output="screen",
        emulate_tty=True,
        **{executable: "cameraV2.py"}
    )

    return launch.LaunchDescription([
        launch.actions.IncludeLaunchDescription( # lance le gazebo
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                court_path)
        ),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        launch.actions.DeclareLaunchArgument(name="joy", default_value="true"),
        launch.actions.DeclareLaunchArgument(name="control", default_value="true"),
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_controller],
            )
        ),
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_joint_trajectory_controller],
            )
        ),
        launch.actions.RegisterEventHandler(
                                         event_handler=launch.event_handlers.OnProcessExit(
                                             target_action=joy_node,
                                             on_exit=[launch.actions.EmitEvent(
                                                 event=launch.events.Shutdown())],
                                         )),
        robot_state_publisher_node,
        joint_state_publisher_node,
        # joint_state_publisher_gui_node,
        spawn_entity,
        joy_node,
        joy_gz_node,
        arm_control,
        camera_node,
        rviz_node
    ])

