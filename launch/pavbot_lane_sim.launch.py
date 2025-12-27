import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory("pavbot_sim_gz")

    world_path = os.path.join(pkg_share, "worlds", "pavbot_test_world.sdf")

    # Add your model directory so `model://pavbot_cam_test` resolves.
    # This assumes you placed models under: pavbot_sim_gz/models
    models_path = os.path.join(os.path.dirname(pkg_share), "pavbot_sim_gz", "models")

    # Gazebo topics (from your working setup)
    gz_image_topic = "/world/default/model/pavbot_cam_test/link/base_link/sensor/rgb_cam/image"
    gz_info_topic  = "/world/default/model/pavbot_cam_test/link/base_link/sensor/rgb_cam/camera_info"

    # This ensures Gazebo can find your models regardless of shell env.
    set_gz_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=models_path + ":" + os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    )

    # Start Gazebo (GUI + real-time). Do NOT use -s.
    gz_sim = ExecuteProcess(
        cmd=["gz", "sim", "-r", world_path, "-v", "4"],
        output="screen"
    )

    # Bridge image + camera_info (Gazebo -> ROS)
    bridge = ExecuteProcess(
        cmd=[
            "ros2", "run", "ros_gz_bridge", "parameter_bridge",
            f"{gz_image_topic}@sensor_msgs/msg/Image[gz.msgs.Image",
            f"{gz_info_topic}@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        ],
        output="screen"
    )

    # Run lane detector with remaps to the bridged Gazebo topics
    lane_detector = Node(
        package="lane_lab",
        executable="lane_detector",
        name="lane_detector",
        output="screen",
        parameters=[],
        remappings=[
            ("/camera/image_raw", gz_image_topic),
            ("/camera/camera_info", gz_info_topic),
        ],
    )

    return LaunchDescription([
        set_gz_path,
        gz_sim,
        bridge,
        lane_detector,
    ])
