import launch
import launch_ros.actions
from launch.actions import ExecuteProcess

from ament_index_python.packages import get_package_share_directory
import os

pkg_share = get_package_share_directory('point_cloud_pruner')


def generate_launch_description():
    # define relative paths for various configs.
    rviz_config = os.path.join(pkg_share, 'rviz2', 'default.rviz')

    return launch.LaunchDescription([
        # A  node that prunes the point cloud.
        launch_ros.actions.Node(
            package='point_cloud_pruner',
            executable='prune_pointcloud',
            name='pruner',
        ),
        # rviz with custom config.
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        ),
        ])
