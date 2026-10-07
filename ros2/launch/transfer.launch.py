from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, TimerAction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share=Path(get_package_share_directory('desktop_arm'))
    task=Node(package='desktop_arm',executable='desktop-arm-ros',arguments=['task'],output='screen')
    return LaunchDescription([
        DeclareLaunchArgument('rviz',default_value='false'),
        DeclareLaunchArgument('output',default_value=str(Path.cwd()/'results/ros2')),
        SetEnvironmentVariable('DESKTOP_ARM_ROOT',str(share)),
        Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[{'robot_description':(share/'assets/desktop_arm.urdf').read_text()}]),
        Node(package='desktop_arm',executable='desktop-arm-ros',arguments=['server','--output',LaunchConfiguration('output')],output='screen'),
        TimerAction(period=2.0,actions=[task]),
        RegisterEventHandler(OnProcessExit(target_action=task,on_exit=[EmitEvent(event=Shutdown(reason='Transfer task finished'))])),
        Node(package='rviz2',executable='rviz2',arguments=['-d',str(share/'ros2/desktop_arm.rviz')],condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
