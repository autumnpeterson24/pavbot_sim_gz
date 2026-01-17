from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, EnvironmentVariable, PythonExpression
from launch_ros.substitutions import FindPackageShare

# START PAV @ x=-15, y+-15, yaw=0 in world
def generate_launch_description():
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

    # Gazebo transport topic names (must match `gz topic -l`)
    gz_left_image = f"/world/{WORLD_NAME}/model/{MODEL_NAME}/link/left_camera_link/sensor/left_cam/image"
    gz_left_info  = f"/world/{WORLD_NAME}/model/{MODEL_NAME}/link/left_camera_link/sensor/left_cam/camera_info"

    gz_right_image = f"/world/{WORLD_NAME}/model/{MODEL_NAME}/link/right_camera_link/sensor/right_cam/image"
    gz_right_info  = f"/world/{WORLD_NAME}/model/{MODEL_NAME}/link/right_camera_link/sensor/right_cam/camera_info"

    return LaunchDescription([
        SetEnvironmentVariable(name="GZ_SIM_RESOURCE_PATH", value=gz_resource_path),

        ExecuteProcess(
            cmd=["gz", "sim", "-r", world],
            output="screen"
        ),

        # Bridge cameras + clock then REMAP to clean ROS topic names
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
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
            output="screen"
        ),


        # Run the dual lane detector (subscribe to ROS topics, not Gazebo topics)
        Node(
            package="pavbot_vision",
            executable="lane_detector_dual",
            name="lane_detector_dual",
            output="screen",
            parameters=[{
                "left_camera_topic": "/left_cam/image_raw",
                "right_camera_topic": "/right_cam/image_raw",
                "path_frame": "base_link",
                "sync_slop_sec": 0.50,   # forgiving in sim
                # Optional tuning:
                # "roi_top_frac": 0.50,
                # "mppx": 0.025,
                # "mppy": 0.025,
                # "nominal_lane_half_width_m": 0.75,
            }]
        ),
    ])
