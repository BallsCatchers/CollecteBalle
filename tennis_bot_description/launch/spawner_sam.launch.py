import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='tennis_court').find('tennis_court')
    court_path = os.path.join(pkg_share, 'launch/tennis_court.launch.py')

    pkg_share = launch_ros.substitutions.FindPackageShare(package='sam_bot_description').find('sam_bot_description')
    default_model_path = os.path.join(pkg_share, 'src/description/sam_bot_description.urdf')
    rviz_path = os.path.join(pkg_share, 'rviz/gazebo.rviz.')

    robot_state_publisher_node = launch_ros.actions.Node( # robot model
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )

    spawn_entity = launch_ros.actions.Node( # fait spawn le robot
    	package='gazebo_ros', 
    	executable='spawn_entity.py',
        arguments=['-entity', 'sam_bot', '-topic', 'robot_description'], # nom du robot
        output='screen'
    )

    rviz_node = launch_ros.actions.Node( # Rviz
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', str(rviz_path)]
    )

    # rqt robot steering = 

    return launch.LaunchDescription([
        launch.actions.IncludeLaunchDescription( # lance le gazebo
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                court_path)
        ),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                    description='Absolute path to robot urdf file'),
        robot_state_publisher_node,
        spawn_entity,
        rviz_node
    ])