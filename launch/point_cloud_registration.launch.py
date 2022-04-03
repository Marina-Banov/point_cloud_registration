import os
import sys
import xacro
import tempfile
import launch
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


sys.path.append(os.path.dirname(os.path.realpath(__file__)))


PKG_THIS = get_package_share_directory("point_cloud_registration")
PKG_GAZEBO_ROS = get_package_share_directory("gazebo_ros")
SIMULATE_MODEL = "turtlebot3_burger.urdf.xacro"


def to_urdf(xacro_path, parameters=None):
    urdf_path = tempfile.mktemp(prefix="%s_" % os.path.basename(xacro_path))
    doc = xacro.process_file(xacro_path, mappings=parameters)
    out = xacro.open_output(urdf_path)
    out.write(doc.toprettyxml(indent="  "))
    return urdf_path


octomap_params = {
    "resolution": 0.15,
    "frame_id": "odom",
    "base_frame_id": "camera_depth_optical_frame",
    "height_map": True,
    "colored_map": True,
    "color_factor": 0.8,
    "filter_ground": False,
    "filter_speckles": False,
    "ground_filter/distance": 0.04,
    "ground_filter/angle": 0.15,
    "ground_filter/plane_distance": 0.07,
    "compress_map": True,
    "incremental_2D_projection": False,
    "sensor_model/max_range": -1.0,
    "sensor_model/hit": 0.7,
    "sensor_model/miss": 0.4,
    "sensor_model/min": 0.12,
    "sensor_model/max": 0.97,
    "color/r": 0.0,
    "color/g": 0.0,
    "color/b": 1.0,
    "color/a": 1.0,
    "color_free/r": 0.0,
    "color_free/g": 0.0,
    "color_free/b": 1.0,
    "color_free/a": 1.0,
    "publish_free_space": False,
}


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="True")
    world = LaunchConfiguration("world", default="env3")

    rviz_config_dir = os.path.join(PKG_THIS, "rviz", "rviz.rviz")
    rviz_node = Node(
        name="rviz2",
        package="rviz2",
        executable="rviz2",
        output = "screen",
        arguments=["-d", rviz_config_dir],
        parameters=[{"use_sim_time": False}],
    )

    urdf = to_urdf(
        os.path.join(PKG_THIS, "urdf", SIMULATE_MODEL),
        { "use_nominal_extrinsics": "true", "add_plug": "true", "use_sim_time": use_sim_time },
    )
    robot_state_publisher = Node(
        name="robot_state_publisher",
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="",
        output="screen",
        arguments = [urdf],
    )

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(PKG_GAZEBO_ROS, "launch", "gzserver.launch.py")),
        launch_arguments={ "world": [PKG_THIS, "/sdf/worlds/", world, "_world.sdf"] }.items(),
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(PKG_GAZEBO_ROS, "launch", "gzclient.launch.py")),
    )

    gazebo_sim_time = ExecuteProcess(
        cmd=["ros2", "param", "set", "/gazebo", "use_sim_time", use_sim_time],
        output="screen",
    )

    octomap_server_node = Node(
        package="octomap_server2",
        executable="octomap_server",
        output="screen",
        remappings=[("cloud_in", "/depth/color/points")],
        parameters=[octomap_params],
    )

    return launch.LaunchDescription([
        # rviz_node,
        robot_state_publisher,
        gazebo_server,
        gazebo_client,
        gazebo_sim_time,
        # octomap_server_node,
    ])

