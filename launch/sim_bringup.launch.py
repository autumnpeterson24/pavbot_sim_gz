from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, EnvironmentVariable, PythonExpression, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

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

    gz_left_image = f"/world/{WORLD_NAME}/model/{MODEL_NAME}/link/left_camera_link/sensor/left_cam/image"
    gz_left_info  = f"/world/{WORLD_NAME}/model/{MODEL_NAME}/link/left_camera_link/sensor/left_cam/camera_info"
    gz_right_image = f"/world/{WORLD_NAME}/model/{MODEL_NAME}/link/right_camera_link/sensor/right_cam/image"
    gz_right_info  = f"/world/{WORLD_NAME}/model/{MODEL_NAME}/link/right_camera_link/sensor/right_cam/camera_info"

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="True",
            description="Use simulation time (/clock)"),
        
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
            parameters=[{
                "use_sim_time": use_sim_time,
                "left_camera_topic": "/left_cam/image_raw",
                "right_camera_topic": "/right_cam/image_raw",
                "path_frame": "base_link",
                "sync_slop_sec": 0.05,

                "use_sync_gate":True,

                "min_lane_confidence": 0.35,
                "lane_conf_hysteresis": 0.07,
                "centerline_smooth_alpha": 0.9,
                "auto_learn_half_width": True,
                "half_width_learn_alpha": 0.90,
                "half_width_min_m": 0.25,
                "half_width_max_m": 1.25,
            }]
        ),
    ])
