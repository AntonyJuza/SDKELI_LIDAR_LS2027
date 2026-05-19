from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    hostname = LaunchConfiguration("hostname")
    frame_id = LaunchConfiguration("frame_id")
    base_frame = LaunchConfiguration("base_frame")

    laser_x = LaunchConfiguration("laser_x")
    laser_y = LaunchConfiguration("laser_y")
    laser_z = LaunchConfiguration("laser_z")
    laser_roll = LaunchConfiguration("laser_roll")
    laser_pitch = LaunchConfiguration("laser_pitch")
    laser_yaw = LaunchConfiguration("laser_yaw")

    def param(name, value_type):
        return ParameterValue(LaunchConfiguration(name), value_type=value_type)

    return LaunchDescription([
        DeclareLaunchArgument("hostname", default_value=""),
        DeclareLaunchArgument("port", default_value="2112"),
        DeclareLaunchArgument("frame_id", default_value="laser"),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument("range_min", default_value="0.05"),
        DeclareLaunchArgument("range_max", default_value="12.0"),
        DeclareLaunchArgument("angle_min", default_value="-2.35619"),
        DeclareLaunchArgument("angle_max", default_value="2.35619"),
        DeclareLaunchArgument("scan_frequency", default_value="23.0"),
        DeclareLaunchArgument("scan_time", default_value="0.0"),
        DeclareLaunchArgument("time_increment", default_value="-1.0"),
        DeclareLaunchArgument("time_offset", default_value="0.0"),
        DeclareLaunchArgument("publish_intensity", default_value="true"),
        DeclareLaunchArgument("inverted", default_value="false"),
        DeclareLaunchArgument("skip", default_value="0"),
        DeclareLaunchArgument("laser_x", default_value="0.0"),
        DeclareLaunchArgument("laser_y", default_value="0.0"),
        DeclareLaunchArgument("laser_z", default_value="0.0"),
        DeclareLaunchArgument("laser_roll", default_value="0.0"),
        DeclareLaunchArgument("laser_pitch", default_value="0.0"),
        DeclareLaunchArgument("laser_yaw", default_value="0.0"),
        Node(
            package="sdkeli_ls_udp",
            executable="sdkeli_ls1207de",
            name="sdkeli_ls1207de",
            output="screen",
            parameters=[{
                "hostname": hostname,
                "port": param("port", int),
                "frame_id": frame_id,
                "range_min": param("range_min", float),
                "range_max": param("range_max", float),
                "angle_min": param("angle_min", float),
                "angle_max": param("angle_max", float),
                "scan_frequency": param("scan_frequency", float),
                "scan_time": param("scan_time", float),
                "time_increment": param("time_increment", float),
                "time_offset": param("time_offset", float),
                "publish_intensity": param("publish_intensity", bool),
                "inverted": param("inverted", bool),
                "skip": param("skip", int),
            }],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_to_laser_tf",
            arguments=[
                laser_x,
                laser_y,
                laser_z,
                laser_roll,
                laser_pitch,
                laser_yaw,
                base_frame,
                frame_id,
            ],
        ),
    ])
