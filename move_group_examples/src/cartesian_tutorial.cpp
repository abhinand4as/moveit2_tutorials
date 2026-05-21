// Demonstrates planning and executing a Cartesian path on the Panda arm using MoveIt2.
// The robot moves to its "ready" pose, then executes a 3-segment Cartesian trajectory
// visualised step-by-step in RViz.

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("cartesian_demo");

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("cartesian_tutorial", node_options);

  // Spin on a separate thread so MoveGroupInterface can receive robot-state updates
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  static const std::string PLANNING_GROUP = "panda_arm";
  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);

  RCLCPP_INFO(LOGGER, "Planning frame  : %s", move_group.getPlanningFrame().c_str());
  RCLCPP_INFO(LOGGER, "End-effector link: %s", move_group.getEndEffectorLink().c_str());

  // ── Visualisation setup ──────────────────────────────────────────────────
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools(
      move_group_node, "panda_link0", "cartesian_tutorial", move_group.getRobotModel());
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "Cartesian_Path_Demo", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  // ── Step 1: Move to a known start configuration ──────────────────────────
  // "ready" is a named state in panda_moveit_config that places the arm in a
  // workspace-friendly pose, giving the Cartesian planner enough clearance.
  visual_tools.prompt("Press 'next' in RvizVisualToolsGui to move to start pose");

  move_group.setNamedTarget("ready");
  move_group.move();

  geometry_msgs::msg::Pose start_pose = move_group.getCurrentPose().pose;

  // ── Step 2: Build waypoints ───────────────────────────────────────────────
  visual_tools.prompt("Press 'next' in RvizVisualToolsGui to plan the Cartesian path");

  // Trajectory: start → down 20 cm → right 20 cm → up-and-left (back near start)
  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(start_pose);

  geometry_msgs::msg::Pose target_pose = start_pose;

  target_pose.position.z -= 0.2;
  waypoints.push_back(target_pose);  // down

  target_pose.position.y -= 0.2;
  waypoints.push_back(target_pose);  // right

  target_pose.position.z += 0.2;
  target_pose.position.y += 0.2;
  target_pose.position.x -= 0.2;
  waypoints.push_back(target_pose);  // up and left

  // ── Step 3: Plan Cartesian path ───────────────────────────────────────────
  // eef_step: maximum distance (m) between consecutive IK samples along the path.
  // fraction: 1.0 means the full path was planned; anything lower means the planner
  //           stopped early (e.g., due to a joint limit or collision).
  const double eef_step = 0.01;
  moveit_msgs::msg::RobotTrajectory trajectory;
  double fraction = move_group.computeCartesianPath(waypoints, eef_step, trajectory);
  RCLCPP_INFO(LOGGER, "Cartesian path planned (%.2f%% achieved)", fraction * 100.0);

  // ── Step 4: Visualise ─────────────────────────────────────────────────────
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Cartesian_Path", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
  for (std::size_t i = 0; i < waypoints.size(); ++i)
    visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::MEDIUM);
  visual_tools.trigger();

  // ── Step 5: Execute ───────────────────────────────────────────────────────
  // Note: Cartesian trajectory speed is controlled by timing the trajectory
  // manually; setMaxVelocityScalingFactor() does not apply here.
  visual_tools.prompt("Press 'next' in RvizVisualToolsGui to execute the trajectory");
  move_group.execute(trajectory);

  rclcpp::shutdown();
  return 0;
}
