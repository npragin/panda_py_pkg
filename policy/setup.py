from setuptools import find_packages, setup

package_name = 'policy'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='npragin',
    maintainer_email='npragin@gmail.com',
    description='A package for 2D/3D vision RL policy inference in ROS2 Humble',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'policy = policy.policy:main',
        ],
    },
)
