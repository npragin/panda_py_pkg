# panda_py_pkg

 ROS2 Package for interacting with the Franka Research 3 Arm via the panda-py package. 

## Usage

- `ros2 launch panda_manager demo.py`

### Usage: Panda Manager

- `ros2 run panda_manager panda_manager`
- `ros2 service call startup_system`
- `ros2 service call shutdown_system`

- The `panda_manager` node contains a list of node names. The nodes in this list will be configured, activated, and shut down in the order they appear.
- When the `startup_system` service is called, it will configure the nodes, wait until camera calibration is complete by sleeping until a transform with the name `table` is published, and then activate all the nodes
- When the `shutdown_system` service is called, it will shut down all lifecycle nodes.

**Only lifecycle nodes should be put in the `managed_nodes` list!**

### Usage: PandaPy Interface

- `ros2 run panda_py_interface panda_py_interface`

- The `panda_py_interface` node exposes three parameters:
  - `hostname`: Configure to the IP or hostname your panda arm exposes
  - `gripper_speed`: Controls how fast your gripper opens/closes
  - `scaling_constant`: The scaling constant for delta actions to be multiplied by. If you're unsure why you'd need this, set it to 1.
 
- The `panda_py_interface` node exposes five services:
  - `stop`: Stops the robot using a `std_srvs/Trigger`
  - `move_to_start`: Moves the robot to its starting position using a `std_srvs/Trigger`
  - `end_effector_delta_pos`: Takes an `EndEffectorDeltaPos.Request` which contains an `x, y, z` which encode the delta end effector position the robot should move to.
  - `joint_delta_pos`: Takes a `JointPos.Request` which contains an eight-element array. The first seven elements are the delta position for the arm joints, and the last element is the delta width for the gripper.
  - `joint_pos`: Takes a `JointPos.Request` which contains an eight-element array. The first seven elements represent the positions of the arm joints, and the last element is the width for the gripper.

### Usage: Point Cloud Pruner

- `ros2 run point_cloud_pruner prune_pointcloud`

- The `point_cloud_pruner` node filters and transforms point cloud data:
  - Subscribes to `/camera/camera/depth/color/points` for input point cloud
  - Publishes to `/pruned_pointcloud` for filtered output
  - Transforms point cloud to table frame using TF
  - Filters points based on maximum depth threshold (1.5 meters)

### Usage: Camera Calibrator

- `ros2 run point_cloud_pruner camera_calibrator`

- The `camera_calibrator` node performs camera calibration using a checkerboard pattern:
  - Subscribes to `/camera/camera/color/image_raw` for RGB images
  - Subscribes to `/camera/camera/color/camera_info` for camera intrinsics
  - Publishes a static transform from `camera_depth_optical_frame` to `table` frame
  - Uses a 9x8 checkerboard pattern with 5cm square size

  ### Usage: Policy Node

- `ros2 run policy policy`

- The `policy` node handles policy inference and trajectory execution:
  - Subscribes to both `image` and `point_cloud` topics for input data
  - Provides a `trajectory` service that executes predefined joint positions
  - Calls the `joint_pos` service to control the robot, but this can be changed to any service the `panda_py_interface` node provides

 ## Requirements
Humble, libfranka, [panda-py](https://github.com/JeanElsner/panda-py)
