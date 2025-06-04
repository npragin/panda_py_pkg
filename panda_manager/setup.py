from setuptools import find_packages, setup

package_name = 'panda_manager'

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
    maintainer='Noah Pragin',
    maintainer_email='npragin@gmail.com',
    description='Manager Node for RL Policy inference on FR3 Panda arm using panda-py',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'panda_manager = panda_manager.manager:main',
        ],
    },
)
