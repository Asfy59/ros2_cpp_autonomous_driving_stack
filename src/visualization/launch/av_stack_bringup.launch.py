from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue

def generate_launch_description() -> LaunchDescription:
    dataset_path = LaunchConfiguration("dataset_path")
    dataset_number = LaunchConfiguration("dataset_number")
    stack_config_path = LaunchConfiguration("stack_config_path")
    start_time = LaunchConfiguration("start_time")
    end_time = LaunchConfiguration("end_time")
    enable_camera_csv_logging = LaunchConfiguration("enable_camera_csv_logging")
    enable_lidar_csv_logging = LaunchConfiguration("enable_lidar_csv_logging")
    enable_fusion_csv_logging = LaunchConfiguration("enable_fusion_csv_logging")
    launch_rviz = LaunchConfiguration("launch_rviz")
    enable_point_cloud = LaunchConfiguration("enable_point_cloud")
    enable_gray_images = LaunchConfiguration("enable_gray_images")
    enable_colour_images = LaunchConfiguration("enable_colour_images")
    ground_truth_data_frame_prefix = LaunchConfiguration("ground_truth_data_frame_prefix")
    odometry_data_frame_prefix = LaunchConfiguration("odometry_data_frame_prefix")
    odometry_package = LaunchConfiguration("odometry_package")
    odometry_plugin = LaunchConfiguration("odometry_plugin")
    odometry_config_path = LaunchConfiguration("odometry_config_path")
    vehicle_sensor_link = LaunchConfiguration("vehicle_sensor_link")
    stack_parameters = ParameterFile(stack_config_path, allow_substs=True)

    replayer_launch = PathJoinSubstitution(
        [FindPackageShare("ros2_kitti_replay"), "launch", "replayer_bringup.launch.py"]
    )


    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "dataset_path",
                default_value=str("kitti_dataset"),
                description="Path where the KITTI odometry dataset is located",
            ),
            DeclareLaunchArgument(
                "dataset_number",
                default_value="00",
                description="KITTI odometry sequence number to replay",
            ),
            DeclareLaunchArgument(
                "stack_config_path",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("visualization"), "config", "av_stack_nodes.yaml"]
                ),
                description="YAML file with the node parameter set for the current stack run",
            ),
            DeclareLaunchArgument(
                "start_time",
                default_value="0.0",
                description="Replay start time in seconds",
            ),
            DeclareLaunchArgument(
                "end_time",
                default_value="0.0",
                description="Replay end time in seconds",
            ),
            DeclareLaunchArgument(
                "enable_camera_csv_logging",
                default_value="false",
                description="Enable interval CSV logging for the camera processing node",
            ),
            DeclareLaunchArgument(
                "enable_lidar_csv_logging",
                default_value="false",
                description="Enable interval CSV logging for the lidar processing node",
            ),
            DeclareLaunchArgument(
                "enable_fusion_csv_logging",
                default_value="false",
                description="Enable interval CSV logging for the fusion core node",
            ),
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="true",
                description="Launch RViz for the current stack bringup",
            ),
            DeclareLaunchArgument(
                "enable_point_cloud",
                default_value="true",
                description="Publish LiDAR point clouds from the replay source",
            ),
            DeclareLaunchArgument(
                "enable_gray_images",
                default_value="false",
                description="Publish gray stereo images from the replay source",
            ),
            DeclareLaunchArgument(
                "enable_colour_images",
                default_value="true",
                description="Publish color stereo images from the replay source",
            ),
            DeclareLaunchArgument(
                "ground_truth_data_frame_prefix",
                default_value="ground_truth",
                description="Frame prefix used for ground-truth vehicle data",
            ),
            DeclareLaunchArgument(
                "odometry_data_frame_prefix",
                default_value="odometry",
                description="Frame prefix used for replayed sensor data",
            ),
            DeclareLaunchArgument(
                "odometry_package",
                default_value="",
                description="Optional odometry package to compose with the replay source",
            ),
            DeclareLaunchArgument(
                "odometry_plugin",
                default_value="",
                description="Optional odometry component plugin to compose with the replay source",
            ),
            DeclareLaunchArgument(
                "odometry_config_path",
                default_value="",
                description="Optional odometry configuration file path",
            ),
            DeclareLaunchArgument(
                "vehicle_sensor_link",
                default_value="lidar",
                description="Sensor link used by the composed odometry component",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(replayer_launch),
                launch_arguments={
                    "dataset_path": dataset_path,
                    "dataset_number": dataset_number,
                    "start_time": start_time,
                    "end_time": end_time,
                    "launch_rviz": launch_rviz,
                    "enable_point_cloud": enable_point_cloud,
                    "enable_gray_images": enable_gray_images,
                    "enable_colour_images": enable_colour_images,
                    "ground_truth_data_frame_prefix": ground_truth_data_frame_prefix,
                    "odometry_data_frame_prefix": odometry_data_frame_prefix,
                    "odometry_package": odometry_package,
                    "odometry_plugin": odometry_plugin,
                    "odometry_config_path": odometry_config_path,
                    "vehicle_sensor_link": vehicle_sensor_link,
                }.items(),
            ),
            # Node(
            # package='fusion_core',
            # name="fusion",
            # executable='fusion_core',
            # remappings={
            #         ("camera_topic", "p2_img"),
            # }
            Node(
                package='lidar_processing',
                name="lidar_processing",
                executable='lidar_processing',
                remappings=[
                    ("lidar_in", "lidar_pc"),
                    ("lidar_out", "processed_lidar_pc"),
                    ("lidar_detections", "lidar_detections"),
                    ("lidar_detection_markers", "lidar_detection_markers")
                ],
                parameters=[
                    stack_parameters,
                    {"enable_csv_logging": enable_lidar_csv_logging},
                    {"dataset_sequence": ParameterValue(dataset_number, value_type=str)},
                ],
            ),
            Node(
                package='camera_processing',
                name="camera_processing",
                executable='camera_processing',
                remappings=[
                    ("camera_in", "/p2_img"),
                    ("object_detections", "object_detections"),
                    ("camera_info", "p2_camera_info"),
                ],
                parameters=[
                    stack_parameters,
                    {"dataset_path": ParameterValue(dataset_path, value_type=str)},
                    {"enable_csv_logging": enable_camera_csv_logging},
                    {"dataset_sequence": ParameterValue(dataset_number, value_type=str)},
                ],
            ),
            Node(
                package='fusion_core',
                name="fusion_core",
                executable='tracking_based_fusion',
                parameters=[
                    stack_parameters,
                    {"enable_csv_logging": enable_fusion_csv_logging},
                    {"dataset_sequence": ParameterValue(dataset_number, value_type=str)},
                ],
            ),

            
        ]
    )
