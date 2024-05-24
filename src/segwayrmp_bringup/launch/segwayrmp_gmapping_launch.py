import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():
    # Launch configuration variables
    use_sim_time = LaunchConfiguration("use_sim_time", default="True")

    # Include SLAM toolbox launch file from slam_toolbox package
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    FindPackageShare("slam_toolbox").find("slam_toolbox"),
                    "launch",
                    "online_async_launch.py",
                )
            ]
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # Launch RViz with a specific configuration file
    rviz2_launch = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "rviz2",
            "rviz2",
            "-d",
            os.path.join(
                os.path.expanduser("~"),
                "segwayrmp_ros2",
                "src",
                "segwayrmp_bringup",
                "rviz",
                "default_view.rviz",
            ),
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="True",
                description="Use simulation (Gazebo) clock if true",
            ),
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="False",
                description="Launch RViz if set to true",
            ),
            slam_toolbox_launch,
            rviz2_launch,
        ]
    )
