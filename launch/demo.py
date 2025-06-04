# Demo launch file for the RL inference system on the Panda robot.
#
# Noah Pragin
#
# This launch file starts the RL inference system on the Panda robot.

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="panda_manager",
                executable="panda_manager",
                name="panda_manager",
            ),
            Node(
                package="panda_py_interface",
                executable="panda_py_interface",
                name="panda_py_interface",
            ),
            Node(
                package="point_cloud_pruner",
                executable="prune_pointcloud",
                name="point_cloud_pruner",
            ),
            Node(
                package="point_cloud_pruner",
                executable="calibrate_camera",
                name="camera_calibrator",
            ),
            Node(
                package="policy",
                executable="policy",
                name="policy",
            ),
            Node(
                package="policy",
                executable="policy",
                name="policy",
            )
        ]
    )
