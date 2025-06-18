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
                parameters=[{
                    "additional_x_translation": -0.2,
                    "additional_y_translation": -0.2,
                }],
            ),
            Node(
                package="point_cloud_pruner",
                executable="prune_pointcloud",
                name="pruner",
                parameters=[{
                    "min_x": -0.7,
                    "max_x": 0.45,
                    "max_y": 0.2,
                    "min_z": -0.15,
                }],
            ),
        ]
    )
