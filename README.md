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
  - `joint_pos_delta`: Takes a `JointPos.Request` which contains an eight-element array. The first seven elements are the delta position for the arm joints, and the last element is the delta width for the gripper.
  - `joint_pos`: Takes a `JointPos.Request` which contains an eight-element array. The first seven elements represent the positions of the arm joints, and the last element is the width for the gripper.

### Usage: 


 ## Requirements
Humble, libfranka, [panda-py](https://github.com/JeanElsner/panda-py)
