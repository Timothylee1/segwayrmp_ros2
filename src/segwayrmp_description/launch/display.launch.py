import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = get_package_share_directory('segwayrmp_description')
    default_model_path = os.path.join(pkg_share, 'urdf/segwayrmp.urdf.xacro')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/default_view.rviz')

    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument(name='model', default_value=default_model_path,
                              description='Absolute path to robot urdf file'),
        DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                              description='Absolute path to rviz config file'),
        Node(package='robot_state_publisher',
             executable='robot_state_publisher',
             name='robot_state_publisher',
             output='screen',
             parameters=[{'robot_description': robot_description}]),
        Node(package='joint_state_publisher_gui',
             executable='joint_state_publisher_gui',
             name='joint_state_publisher_gui',
             output='screen'),
        Node(package='rviz2',
             executable='rviz2',
             name='rviz2',
             output='screen',
             arguments=['-d', LaunchConfiguration('rvizconfig')])
    ])
