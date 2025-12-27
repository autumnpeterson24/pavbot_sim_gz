"""
RUN THIS:
    colcon build --packages-select pavbot_sim_gz
    ros2 launch pavbot_sim_gz sim_bringup.launch.py


"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, EnvironmentVariable, PythonExpression
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg = FindPackageShare("pavbot_sim_gz")

    world = PathJoinSubstitution([pkg, "worlds", "pavbot_test_world.sdf"])
    rviz_cfg = PathJoinSubstitution([pkg, "rviz", "pavbot_sim.rviz"])
    models_path = PathJoinSubstitution([pkg, "models"])

    # Append our models path to whatever GZ_SIM_RESOURCE_PATH already is.
    # If the env var is empty, just use models_path.
    gz_resource_path = PythonExpression([
        "'%s:%s' % ('", EnvironmentVariable("GZ_SIM_RESOURCE_PATH"), "', '", models_path, "') "
        "if '", EnvironmentVariable("GZ_SIM_RESOURCE_PATH"), "' != '' "
        "else '", models_path, "'"
    ])

    # Choose ONE camera (recommend camera_link)
    gz_image = "/world/default/model/pavbot_test/link/camera_link/sensor/rgb_cam/image"
    gz_info  = "/world/default/model/pavbot_test/link/camera_link/sensor/rgb_cam/camera_info"

    return LaunchDescription([
        SetEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH",
            value=gz_resource_path
        ),

        ExecuteProcess(
            cmd=["gz", "sim", "-r", world],
            output="screen"
        ),

        # Bridge Gazebo -> ROS topics (use the long gz topics directly)
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
                "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                f"{gz_image}@sensor_msgs/msg/Image[gz.msgs.Image",
                f"{gz_info}@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            ],
            output="screen"
        ),

        # Remap lane detector input to the bridged Gazebo image topic
        Node(
            package="lane_lab",
            executable="lane_detector",
            name="lane_detector",
            output="screen",
            remappings=[
                ("/camera/image_raw", gz_image),
            ]
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_cfg],
            output="screen"
        )
    ])
