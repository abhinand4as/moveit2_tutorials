// Demonstrates the core MoveGroupInterface API on the Panda arm.
// Covers: pose goals, joint-space goals, path constraints, Cartesian paths,
// collision objects, and object attach/detach.

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit_visual_tools/moveit_visual_tools.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

  // Spin on a separate thread so MoveGroupInterface can receive robot-state updates
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  static const std::string PLANNING_GROUP = "panda_arm";

  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  const moveit::core::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // ── Visualisation setup ──────────────────────────────────────────────────
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools(
      move_group_node, "panda_link0", "move_group_tutorial", move_group.getRobotModel());
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "MoveGroupInterface_Demo", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  RCLCPP_INFO(LOGGER, "Planning frame  : %s", move_group.getPlanningFrame().c_str());
  RCLCPP_INFO(LOGGER, "End-effector link: %s", move_group.getEndEffectorLink().c_str());
  RCLCPP_INFO(LOGGER, "Available planning groups:");
  std::copy(move_group.getJointModelGroupNames().begin(),
            move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));

  visual_tools.prompt("Press 'next' in RvizVisualToolsGui to start the demo");

  // ── Demo 1: Pose goal ────────────────────────────────────────────────────
  geometry_msgs::msg::Pose target_pose1;
  target_pose1.orientation.w = 1.0;
  target_pose1.position.x = 0.28;
  target_pose1.position.y = -0.2;
  target_pose1.position.z = 0.5;
  move_group.setPoseTarget(target_pose1);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Plan 1 (pose goal): %s", success ? "SUCCESS" : "FAILED");

  visual_tools.publishAxisLabeled(target_pose1, "pose1");
  visual_tools.publishText(text_pose, "Pose_Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in RvizVisualToolsGui to continue");

  // ── Demo 2: Joint-space goal ─────────────────────────────────────────────
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState(10);
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  joint_group_positions[0] = -1.0;  // radians
  bool within_bounds = move_group.setJointValueTarget(joint_group_positions);
  if (!within_bounds)
    RCLCPP_WARN(LOGGER, "Target joint position(s) outside limits — will clamp");

  // Slow to 5% of max velocity/acceleration for a controlled motion
  move_group.setMaxVelocityScalingFactor(0.05);
  move_group.setMaxAccelerationScalingFactor(0.05);

  success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Plan 2 (joint-space goal): %s", success ? "SUCCESS" : "FAILED");

  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Joint_Space_Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in RvizVisualToolsGui to continue");

  // ── Demo 3: Orientation path constraint ──────────────────────────────────
  moveit_msgs::msg::OrientationConstraint ocm;
  ocm.link_name = "panda_link7";
  ocm.header.frame_id = "panda_link0";
  ocm.orientation.w = 1.0;
  ocm.absolute_x_axis_tolerance = 0.1;
  ocm.absolute_y_axis_tolerance = 0.1;
  ocm.absolute_z_axis_tolerance = 0.1;
  ocm.weight = 1.0;

  moveit_msgs::msg::Constraints test_constraints;
  test_constraints.orientation_constraints.push_back(ocm);
  move_group.setPathConstraints(test_constraints);

  // The start state must already satisfy the path constraint.
  // We set it explicitly so the planner doesn't reject the initial state.
  moveit::core::RobotState start_state(*move_group.getCurrentState());
  geometry_msgs::msg::Pose start_pose2;
  start_pose2.orientation.w = 1.0;
  start_pose2.position.x = 0.55;
  start_pose2.position.y = -0.05;
  start_pose2.position.z = 0.8;
  start_state.setFromIK(joint_model_group, start_pose2);
  move_group.setStartState(start_state);

  move_group.setPoseTarget(target_pose1);
  // Constrained planning samples via IK on every state — increase timeout accordingly
  move_group.setPlanningTime(10.0);

  success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Plan 3 (orientation constraint): %s", success ? "SUCCESS" : "FAILED");

  visual_tools.deleteAllMarkers();
  visual_tools.publishAxisLabeled(start_pose2, "start");
  visual_tools.publishAxisLabeled(target_pose1, "goal");
  visual_tools.publishText(text_pose, "Constrained_Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in RvizVisualToolsGui to continue");

  move_group.clearPathConstraints();

  // ── Demo 4: Cartesian path ────────────────────────────────────────────────
  // Starting from start_pose2 set above (the constrained-planning start state).
  // Adding start_pose2 as the first waypoint helps visualise the full path in RViz.
  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(start_pose2);

  geometry_msgs::msg::Pose target_pose3 = start_pose2;

  target_pose3.position.z -= 0.2;
  waypoints.push_back(target_pose3);  // down

  target_pose3.position.y -= 0.2;
  waypoints.push_back(target_pose3);  // right

  target_pose3.position.z += 0.2;
  target_pose3.position.y += 0.2;
  target_pose3.position.x -= 0.2;
  waypoints.push_back(target_pose3);  // up and left

  const double eef_step = 0.01;
  moveit_msgs::msg::RobotTrajectory trajectory;
  double fraction = move_group.computeCartesianPath(waypoints, eef_step, trajectory);
  RCLCPP_INFO(LOGGER, "Plan 4 (Cartesian path): %.2f%% achieved", fraction * 100.0);

  // Note: Cartesian trajectory speed cannot be set via setMaxVelocityScalingFactor;
  // the trajectory must be time-parameterised manually if speed control is needed.
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Cartesian_Path", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
  for (std::size_t i = 0; i < waypoints.size(); ++i)
    visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in RvizVisualToolsGui to continue");

  // ── Demo 5: Planning around a collision object ────────────────────────────
  move_group.setStartState(*move_group.getCurrentState());
  geometry_msgs::msg::Pose another_pose;
  another_pose.orientation.w = 0;
  another_pose.orientation.x = -1.0;
  another_pose.position.x = 0.7;
  another_pose.position.y = 0.0;
  another_pose.position.z = 0.59;
  move_group.setPoseTarget(another_pose);

  success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Plan 5 (no obstacles): %s", success ? "SUCCESS" : "FAILED");

  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Clear_Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishAxisLabeled(another_pose, "goal");
  visual_tools.publishTrajectoryLine(my_plan.trajectory, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in RvizVisualToolsGui to add a collision object");

  // Add a box obstacle into the planning scene
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = move_group.getPlanningFrame();
  collision_object.id = "box1";

  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = 0.1;
  primitive.dimensions[primitive.BOX_Y] = 1.5;
  primitive.dimensions[primitive.BOX_Z] = 0.5;

  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.48;
  box_pose.position.y = 0.0;
  box_pose.position.z = 0.25;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  RCLCPP_INFO(LOGGER, "Adding collision object to the world");
  planning_scene_interface.addCollisionObjects({collision_object});

  visual_tools.publishText(text_pose, "Add_object", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' once the collision object appears in RViz");

  // Replan with the obstacle present — MoveIt will route around it
  success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Plan 6 (avoid obstacle): %s", success ? "SUCCESS" : "FAILED");

  visual_tools.publishText(text_pose, "Obstacle_Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in RvizVisualToolsGui to continue");

  // ── Demo 6: Attach object to gripper ─────────────────────────────────────
  moveit_msgs::msg::CollisionObject object_to_attach;
  object_to_attach.id = "cylinder1";

  shape_msgs::msg::SolidPrimitive cylinder_primitive;
  cylinder_primitive.type = primitive.CYLINDER;
  cylinder_primitive.dimensions.resize(2);
  cylinder_primitive.dimensions[primitive.CYLINDER_HEIGHT] = 0.20;
  cylinder_primitive.dimensions[primitive.CYLINDER_RADIUS] = 0.04;

  // Pose is relative to the end-effector link so the cylinder appears in the gripper
  object_to_attach.header.frame_id = move_group.getEndEffectorLink();
  geometry_msgs::msg::Pose grab_pose;
  grab_pose.orientation.w = 1.0;
  grab_pose.position.z = 0.2;

  object_to_attach.primitives.push_back(cylinder_primitive);
  object_to_attach.primitive_poses.push_back(grab_pose);
  object_to_attach.operation = object_to_attach.ADD;
  planning_scene_interface.applyCollisionObject(object_to_attach);

  // Attach the cylinder; finger links are listed as touch_links so MoveIt allows
  // their contact with the object during planning
  RCLCPP_INFO(LOGGER, "Attaching cylinder to robot");
  std::vector<std::string> touch_links = {"panda_rightfinger", "panda_leftfinger"};
  move_group.attachObject(object_to_attach.id, "panda_hand", touch_links);

  visual_tools.publishText(text_pose, "Object_attached_to_robot", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' once the cylinder is attached in RViz");

  // Replan with the attached object — collision checking includes the cylinder
  move_group.setStartStateToCurrentState();
  success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Plan 7 (with attached object): %s", success ? "SUCCESS" : "FAILED");

  visual_tools.publishTrajectoryLine(my_plan.trajectory, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in RvizVisualToolsGui to detach and clean up");

  // ── Demo 7: Detach and remove objects ─────────────────────────────────────
  RCLCPP_INFO(LOGGER, "Detaching cylinder from robot");
  move_group.detachObject(object_to_attach.id);

  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Object_detached_from_robot", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' once the cylinder is detached in RViz");

  RCLCPP_INFO(LOGGER, "Removing all objects from the world");
  planning_scene_interface.removeCollisionObjects({collision_object.id, object_to_attach.id});

  visual_tools.publishText(text_pose, "Objects_removed", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' once all objects have disappeared from RViz");

  visual_tools.deleteAllMarkers();
  visual_tools.trigger();

  rclcpp::shutdown();
  return 0;
}
