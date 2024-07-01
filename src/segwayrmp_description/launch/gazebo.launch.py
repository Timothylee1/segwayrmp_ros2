import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():
    # Launch configuration variables
    use_sim_time = LaunchConfiguration("use_sim_time", default="True")
    launch_rviz = LaunchConfiguration("launch_rviz", default="False")

    # Include SLAM toolbox launch file from slam_toolbox package
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                FindPackageShare("slam_toolbox").find("slam_toolbox"),
                "launch",
                "online_async_launch.py"
            )
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # Get the source directory of the package
    segwayrmp_description_src = os.path.join(
        FindPackageShare("segwayrmp_description").find("segwayrmp_description"),
        os.pardir,
        os.pardir,
        "src",
        "segwayrmp_description"
    )

    # Define the path to the RViz configuration file
    rviz_config_file_full_path = os.path.join(segwayrmp_description_src, "rviz", "default_view.rviz")

    # Launch RViz with the specific configuration file
    rviz2_launch = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "rviz2",
            "rviz2",
            "-d",
            rviz_config_file_full_path,
        ],
        output="screen",
        condition=IfCondition(launch_rviz),
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
