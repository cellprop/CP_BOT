
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'CP_BOT'

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    )

    # Set the initial pose of the bot to the first waypoint (example coordinates, adjust as needed)
    initial_pose = {'x': '-2.295368', 'y': '8.692272', 'z': '0.2', 'R': '0', 'P': '0.000004', 'Y': '-0.048323'} 

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'my_bot',
                                   '-x', initial_pose['x'],
                                   '-y', initial_pose['y'],
                                   '-z', initial_pose['z'],
                                   '-R', initial_pose['R'],
                                   '-P', initial_pose['P'],
                                   '-Y', initial_pose['Y']],
                        output='screen')

    mqtt_controller = Node(
        package=package_name,
        executable='mqtt_robot_controller.py',
        name='mqtt_robot_controller',
        output='screen'
    )

    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        mqtt_controller,
    ])
