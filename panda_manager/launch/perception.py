# Launch file for the perception system on the Panda robot.
#
# Noah Pragin

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="point_cloud_pruner",
                executable="calibrate_camera",
                name="camera_calibrator",
            ),
            Node(
                package="point_cloud_pruner",
                executable="point_cloud_transformer",
                name="point_cloud_transformer",
            ),
            Node(
                package="point_cloud_pruner",
                executable="prune_pointcloud",
                name="pruner",
                parameters=[{
                    "min_x": -0.85,
                    "min_y": -0.4,
                    "max_y": 0.4,
                    "min_z": -0.01,
                }],
            ),
        ]
    )
