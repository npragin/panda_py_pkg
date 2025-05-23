# panda_py_pkg

 ROS2 Package for interacting with the Franka Research 3 Arm via the panda-py package. 

## Usage

- `ros2 run panda_py_interface panda_py_interface`
- `ros2 param set panda_py_interface hostname <your_pandas_hostname>`
- `ros2 service call reinitialize std_srvs/srv/Trigger`
- `ros2 service call move_to_start std_srvs/srv/Trigger`
- `ros2 service call end_effector_delta_pos panda_py_msgs/srv/EndEffectorDeltaPos {"x: 0, y: 0, z: -.1"}`


### Usage: Point cloud pruner package

- `ros2 launch point_cloud_pruner launch_pruner.py`


 ## Requirements
Humble, libfranka, [panda-py](https://github.com/JeanElsner/panda-py)
