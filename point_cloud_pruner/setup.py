from setuptools import find_packages, setup

import os
from glob import glob

package_name = 'point_cloud_pruner'

setup(
    name=package_name,
    version='0.1.0',

    packages=find_packages(exclude=['test']),

    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/rviz', ['rviz2/default.rviz']),


        (os.path.join('share', package_name, 'configs'), glob('configs/*.yaml')),
        (os.path.join('share', package_name, 'rviz2'), glob('rviz2/*.rviz')),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.xml'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,

    maintainer='Skand',
    maintainer_email='peris@oregonstate.edu',
    description='Point Cloud Pruner: prunes point cloud from realsense camera beyond a certain depth.',
    license='BSD-3-Clause',

    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # A node that prunes the point cloud
            'prune_pointcloud = point_cloud_pruner.point_cloud_pruner:main',
            'calibrate_camera = point_cloud_pruner.camera_calibrator:main',
        ],
    },
)
