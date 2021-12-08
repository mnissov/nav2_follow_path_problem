import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
import launch
from launch.actions import DeclareLaunchArgument
from launch.actions.execute_process import ExecuteProcess
from launch.actions.include_launch_description import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    tractor_navigation_dir = get_package_share_directory("tractor_navigation")

    # Create the launch configuration variables
    autostart = LaunchConfiguration("autostart")
    default_bt_xml_filename = LaunchConfiguration("default_bt_xml_filename")
    map_file = LaunchConfiguration("map_file")
    params_file = LaunchConfiguration("params_file")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    use_rviz = LaunchConfiguration("use_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # declare launch arguments
    declare_autostart = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the nav2 stack",
    )
    declare_bt_xml_file = DeclareLaunchArgument(
        "default_bt_xml_filename",
        default_value=os.path.join(
            get_package_share_directory("nav2_bt_navigator"),
            "behavior_trees",
            "navigate_w_replanning_and_recovery.xml",
        ),
        # default_value=os.path.join(
        #     tractor_gazebo_dir,
        #     "config",
        #     "navigate_through_poses_w_replanning_and_recovery.xml",
        # ),
        description="Full path to behavior tree xml file to use",
    )
    declare_map_file = DeclareLaunchArgument(
        "map_file",
        default_value=os.path.join(tractor_navigation_dir, "map", "default.yaml"),
        description="Full path to the nav2 map yaml file to use",
    )
    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(tractor_navigation_dir, "config", "nav2.yaml"),
        description="Full path to the ROS2 parameters file to use",
    )
    declare_rviz_config_file = DeclareLaunchArgument(
        "rviz_config_file",
        default_value=os.path.join(tractor_navigation_dir, "rviz", "nav2.rviz"),
        description="Default rviz config file",
    )
    declare_use_rviz = DeclareLaunchArgument(
        "use_rviz", default_value="false", description="Whether to use rviz"
    )
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Flag to enable use_sim_time",
    )

    # from localization_launch.py, not including amcl
    nav2_map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[params_file, {"yaml_filename": map_file}],
    )
    nav2_map_lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "autostart": autostart,
                "node_names": ["map_server"],
            }
        ],
    )

    # calling navigation_launch.py
    nav2_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "autostart": autostart,
            "params_file": params_file,
            "default_bt_xml_filename": default_bt_xml_filename,
            "use_lifecycle_mgr": "false",
            "map_subscribe_transient_local": "true",
        }.items(),
    )

    nav2_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, "launch", "rviz_launch.py")
        ),
        condition=IfCondition(use_rviz),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "rviz_config": rviz_config_file,
        }.items(),
    )

    return LaunchDescription(
        [
            declare_autostart,
            declare_bt_xml_file,
            declare_map_file,
            declare_params_file,
            declare_rviz_config_file,
            declare_use_rviz,
            declare_use_sim_time,
            nav2_map_server,
            nav2_map_lifecycle,
            nav2_navigation,
            nav2_rviz,
        ]
    )
