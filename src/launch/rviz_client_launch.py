from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    urdf_path = os.path.join(
        get_package_share_directory("tachimawari"), "urdf/robot.urdf")
    rviz_conf_path = os.path.join(
        get_package_share_directory("tachimawari"), "rviz/rviz.rviz")
    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="launch_urdf",
            arguments=[urdf_path]
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz_app",
            arguments=["-d", rviz_conf_path]
        ),
        Node(
            package="tachimawari",
            executable="rviz_client",
            name="rviz_client"
            # arguments=["-port 3030"]
        )

    ])
