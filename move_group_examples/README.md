# move_group_examples

MoveIt2 tutorial nodes for the Franka Panda arm. Two self-contained demos walk through the most common `MoveGroupInterface` workflows, each visualised interactively in RViz.

| Node | Source | What it demonstrates |
|---|---|---|
| `move_group_interface_tutorial` | [src/move_group_interface_tutorial.cpp](src/move_group_interface_tutorial.cpp) | Pose goals, joint-space goals, path constraints, Cartesian paths, collision objects, object attach/detach |
| `cartesian_tutorial` | [src/cartesian_tutorial.cpp](src/cartesian_tutorial.cpp) | Cartesian path planning and execution from a known start pose |

---

## Prerequisites

- ROS 2 Jazzy
- `panda_moveit_config` package (provides the Panda URDF, SRDF, and controller configs)
- `moveit_visual_tools`

Build the workspace once before running:

```bash
cd ~/ws/robotics/ros2/panda_ws
colcon build --symlink-install
source install/setup.bash
```

---

## Running the Demos

Every demo requires the MoveGroup server, RViz, and the controllers to be running **in a separate terminal** before launching the tutorial node.

### Terminal 1 — MoveGroup infrastructure + RViz

```bash
source install/setup.bash
ros2 launch move_group_examples move_group.launch.py
```

This starts:
- `move_group` action server
- `robot_state_publisher`
- `ros2_control_node` with a fake hardware interface
- `panda_arm_controller`, `panda_hand_controller`, `joint_state_broadcaster`
- RViz with the pre-configured `move_group.rviz` layout

#### RViz panel setup (first run only)

Open **Panels → Add New Panel** and add `RvizVisualToolsGui`. This panel provides the **Next** button used to step through each demo.

---

### Tutorial 1 — MoveGroup Interface Tutorial

```bash
# Terminal 2
source install/setup.bash
ros2 launch move_group_examples move_group_interface_tutorial.launch.py
```

Step through with **Next** in `RvizVisualToolsGui`. The demo runs in order:

| Step | What happens |
|---|---|
| 1 | Plans to a Cartesian pose goal and visualises the trajectory |
| 2 | Plans a joint-space goal (joint 0 → −1.0 rad) at 5% speed |
| 3 | Plans with an orientation constraint on `panda_link7` |
| 4 | Plans a 4-waypoint Cartesian path |
| 5 | Plans around a box collision object added to the scene |
| 6 | Attaches a cylinder to the gripper and replans accounting for it |
| 7 | Detaches the cylinder and removes all objects |

---

### Tutorial 2 — Cartesian Tutorial

```bash
# Terminal 2
source install/setup.bash
ros2 launch move_group_examples cartesian_tutorial.launch.py
```

| Step | What happens |
|---|---|
| 1 | Robot moves to the `ready` named state |
| 2 | Plans a 4-waypoint Cartesian path: start → down 20 cm → right 20 cm → up-and-left |
| 3 | Visualises the planned path in RViz (green line + labelled axes) |
| 4 | Executes the trajectory on the robot |

---

## Package structure

```
move_group_examples/
├── launch/
│   ├── move_group.launch.py                   # Infrastructure (run first)
│   ├── move_group_interface_tutorial.launch.py
│   └── cartesian_tutorial.launch.py
├── rviz/
│   └── move_group.rviz
├── src/
│   ├── move_group_interface_tutorial.cpp
│   └── cartesian_tutorial.cpp
├── docs/
│   └── move_group_concepts.md
├── CMakeLists.txt
└── package.xml
```
