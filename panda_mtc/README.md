# Panda MoveIt Task Constructor (MTC) Demo

This repository demonstrates a basic pick-and-place pipeline using MoveIt Task Constructor (MTC) with the Franka Panda robot in ROS 2 Humble.

## Launch Instructions

Start the RViz visualization:
```bash
ros2 launch panda_mtc panda_mtc_rviz.launch.py
```

Run the MTC pick-and-place pipeline:
```bash
ros2 launch panda_mtc pick_place_mtc.launch.py
```

## Dependencies

Install the required MoveIt Task Constructor packages:

```bash
sudo apt install ros-humble-moveit-task-constructor-core
sudo apt install ros-humble-moveit-task-constructor-capabilities
sudo apt install ros-humble-moveit-task-constructor-visualization
```