# MoveIt Motion Planning Concepts

This document explains the fundamental motion planning paradigms demonstrated in the `move_group_interface_tutorial` example. The focus is on **concepts** — what each planning task means, when to use it, and the trade-offs involved — rather than the C++ API itself.

---

## Table of Contents

1. [Pose Goal Planning](#1-pose-goal-planning)
2. [Joint-Space Goal Planning](#2-joint-space-goal-planning)
3. [Planning with Path Constraints](#3-planning-with-path-constraints)
4. [Cartesian Path Planning](#4-cartesian-path-planning)
5. [Collision Objects (World Obstacles)](#5-collision-objects-world-obstacles)
6. [Attached Collision Objects (Carried Objects)](#6-attached-collision-objects-carried-objects)
7. [How These Compose in Practice](#how-these-compose-in-practice)

---

## 1. Pose Goal Planning

**What it is:** You tell the robot *where* you want the end-effector (gripper/tool tip) to end up in 3D space — specifying both position (x, y, z) and orientation (roll, pitch, yaw or quaternion). The planner figures out *how* to get there.

**The key idea:** You don't care what the individual joints do. You just care about the final pose of the tool. The planner internally solves inverse kinematics (IK) to find valid joint configurations that achieve your target pose, then plans a collision-free path to one of them.

**When to use:**
- Pick-and-place tasks ("move the gripper above this object")
- Any task defined in terms of the workspace rather than the robot's body
- When the orientation matters (e.g., a gripper must approach from above)

**Trade-offs:** Multiple joint configurations can produce the same end-effector pose (redundancy), so the planner has freedom to choose. This is usually good, but it means the same goal can produce different-looking motions on different runs.

---

## 2. Joint-Space Goal Planning

**What it is:** Instead of specifying the tool's pose, you specify the exact angle of each joint. The robot moves until every joint reaches its target value.

**The key idea:** This is the most "direct" form of motion specification — you're saying "I want joint 1 at -1.0 radians, joint 2 at 0.5 radians..." and so on. No IK needed because you've already told the planner exactly where every joint should be.

**When to use:**
- Moving to a known "home" or "ready" configuration
- Reproducing a previously recorded posture
- When you need a predictable, repeatable arm shape (e.g., avoiding self-collisions or reaching into tight spaces where the arm's posture matters)
- Calibration routines

**Velocity/acceleration scaling:** The code drops these to 5% of maximum, which is common for safety during testing. Real robots can move fast and break things (or people), so scaling is essentially a global "speed limit" knob.

---

## 3. Planning with Path Constraints

**What it is:** A constraint on the *entire trajectory*, not just the endpoints. The example constrains the orientation of `panda_link7` to stay nearly upright (small tolerance around the identity quaternion) throughout the motion.

**The key idea:** Sometimes the journey matters as much as the destination. You might reach the goal pose just fine, but if the arm tilts wildly mid-motion, that's unacceptable for certain tasks.

**When to use:**
- **Carrying liquids** — the cup must stay upright the whole way, not just at the end
- **Carrying long objects** — keeping a tray level, not flipping a pipe end-over-end
- **Welding or painting** — the tool must maintain a specific angle to the workpiece
- **Avoiding cable tangling** — restricting wrist rotation

**Why it's slow:** Without constraints, the planner can sample any joint configuration freely. With constraints, every sampled configuration must be *checked* (or projected) to satisfy the constraint. Most random samples get rejected, which is why the code bumps planning time up from 5 to 10 seconds.

**Joint space vs. Cartesian space sampling:** When constraints are active, MoveIt can sample either in joint space (random joint values, then check constraint — slow but thorough) or Cartesian space (sample end-effector poses that satisfy the constraint, then IK back to joints — faster but depends on IK quality). This is an internal planner choice, but it can be forced to joint-space sampling via the `enforce_joint_model_state_space` parameter in `ompl_planning.yaml`.

---

## 4. Cartesian Path Planning

**What it is:** Instead of asking "get from A to B somehow," you provide a *list of waypoints* the end-effector must pass through, and the planner interpolates a straight-line path in Cartesian space between them.

**The key difference from pose-goal planning:**

| Aspect | Pose Goal | Cartesian Path |
|---|---|---|
| What you specify | Final pose only | Sequence of waypoints |
| Tool trajectory | Planner's choice (curved, swooped, whatever) | Straight lines between waypoints |
| Use case | Reaching a target | Controlled path through space |

**When to use:**
- **Approach and retreat motions** — moving the gripper straight down onto an object, then straight up, so you don't knock it over
- **Insertion tasks** — pegs into holes, screws into threads
- **Surface following** — wiping, sanding, drawing
- **Any time the tool's path through space is part of the task definition**, not just the endpoints

**The `fraction` return value:** Cartesian planning can fail partway through if a waypoint is unreachable or the straight-line path passes through an obstacle/singularity. The planner returns the fraction of the path it successfully computed (`1.0` = full success, `0.5` = made it halfway). You decide whether partial success is acceptable.

**Why velocity scaling doesn't work here:** Cartesian paths are computed geometrically without the usual time-parameterization pipeline, so you can't just slow them down with a scaling factor. You'd have to retime the trajectory manually. This is a known rough edge in MoveIt.

---

## 5. Collision Objects (World Obstacles)

**What it is:** You register virtual objects (boxes, cylinders, meshes) in the planning scene. The planner then ensures every motion avoids collision with them.

**The key idea:** The planner only knows about objects you tell it about. If there's a real table the robot doesn't know about, it will happily plan through it. The planning scene is the robot's "mental model" of the world.

**When to use:**
- Modeling known static obstacles (tables, walls, fixtures)
- Representing objects detected by sensors (point clouds, vision systems feeding into the scene)
- Safety zones the robot shouldn't enter

**What happens behind the scenes:** During planning, every candidate configuration is checked against all collision objects. If any link of the robot intersects any object, that configuration is rejected. This is why dense obstacle environments slow planning down.

---

## 6. Attached Collision Objects (Carried Objects)

**What it is:** An object that was sitting in the world gets "attached" to a robot link (typically the gripper). From the planner's perspective, the object now moves rigidly with that link.

**The key idea:** This simulates *grasping*. Once the robot picks something up, the carried object becomes part of the robot's collision geometry — it can hit things, fit through gaps, etc.

**When to use:**
- Any pick-and-place pipeline, after the "pick" phase
- Tool changes (the robot picks up a different end-effector)
- Carrying anything where the object's shape affects what motions are feasible

**The `touch_links` concept:** Once attached, the cylinder would normally be "in collision" with the gripper fingers (since they're touching it — that's how grasping works). You explicitly tell MoveIt: "ignore collisions between the cylinder and these specific links." Without this, the planner would think the grasp itself is invalid.

**Detaching:** When you "release" the object, it goes back to being a world object at its current pose. It doesn't disappear — it just stops moving with the gripper.

---

## How These Compose in Practice

A real manipulation task usually chains several of these together:

1. **Joint-space goal** to a home position
2. **Pose goal** to hover above an object
3. **Cartesian path** straight down to grasp it
4. **Attach** the object
5. **Pose goal** (possibly with **path constraints** if it's fragile) to the destination
6. **Cartesian path** straight down to place
7. **Detach** the object
8. **Cartesian path** straight up to retreat

Each tool in this tutorial is a piece of that larger vocabulary.

---

## Quick Reference

| Task | Specify | Best For |
|---|---|---|
| Pose Goal | End-effector pose | Reaching targets in workspace |
| Joint-Space Goal | All joint angles | Home positions, fixed postures |
| Path Constraints | Constraint on trajectory | Carrying liquids, level surfaces |
| Cartesian Path | Waypoint sequence | Straight-line approach/retreat |
| Collision Object | World geometry | Obstacle avoidance |
| Attached Object | Object on gripper | Manipulating held items |
