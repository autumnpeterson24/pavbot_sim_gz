from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, EnvironmentVariable, PythonExpression, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterFile

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    lane_params  = LaunchConfiguration("lane_params")

    WORLD_NAME = "pavbot_test_world_oval"
    MODEL_NAME = "pavbot_test"
    pkg = FindPackageShare("pavbot_sim_gz")

    world = PathJoinSubstitution([pkg, "worlds", "pavbot_test_world_oval.sdf"])
    models_path = PathJoinSubstitution([pkg, "models"])

    gz_resource_path = PythonExpression([
        "'%s:%s' % ('", EnvironmentVariable("GZ_SIM_RESOURCE_PATH"), "', '", models_path, "') "
        "if '", EnvironmentVariable("GZ_SIM_RESOURCE_PATH"), "' != '' "
        "else '", models_path, "'"
    ])

    gz_left_image  = f"/world/{WORLD_NAME}/model/{MODEL_NAME}/link/left_camera_link/sensor/left_cam/image"
    gz_left_info   = f"/world/{WORLD_NAME}/model/{MODEL_NAME}/link/left_camera_link/sensor/left_cam/camera_info"
    gz_right_image = f"/world/{WORLD_NAME}/model/{MODEL_NAME}/link/right_camera_link/sensor/right_cam/image"
    gz_right_info  = f"/world/{WORLD_NAME}/model/{MODEL_NAME}/link/right_camera_link/sensor/right_cam/camera_info"

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="True",
            description="Use simulation time (/clock)"
        ),

        DeclareLaunchArgument(
            "lane_params",
            default_value=PathJoinSubstitution([
                FindPackageShare("pavbot_vision"),
                "config",
                "lane_detector_dual_sim.yaml"
            ]),
            description="YAML params file for lane_detector_dual"
        ),

        SetEnvironmentVariable(name="GZ_SIM_RESOURCE_PATH", value=gz_resource_path),

        ExecuteProcess(
            cmd=["gz", "sim", "-r", world],
            output="screen"
        ),

        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="ros_gz_bridge",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
            arguments=[
                "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
                "/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry",
                f"{gz_left_image}@sensor_msgs/msg/Image[gz.msgs.Image",
                f"{gz_left_info}@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                f"{gz_right_image}@sensor_msgs/msg/Image[gz.msgs.Image",
                f"{gz_right_info}@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            ],
            remappings=[
                (gz_left_image,  "/left_cam/image_raw"),
                (gz_left_info,   "/left_cam/camera_info"),
                (gz_right_image, "/right_cam/image_raw"),
                (gz_right_info,  "/right_cam/camera_info"),
            ],
        ),

        Node(
            package="pavbot_vision",
            executable="lane_detector_dual",
            name="lane_detector_dual",
            output="screen",
            parameters=[
                ParameterFile(lane_params, allow_substs=True),
                {"use_sim_time": use_sim_time},   # force it
            ],
        ),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0.6","0.65","0.4","0","0.61","0.524",
                       "pavbot_test/base_link","pavbot_test/left_camera_link/left_cam"],
            output="screen",
        ),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0.6","-0.65","0.4","0","0.61","-0.524",
                       "pavbot_test/base_link","pavbot_test/right_camera_link/right_cam"],
            output="screen",
        ),
    ])
