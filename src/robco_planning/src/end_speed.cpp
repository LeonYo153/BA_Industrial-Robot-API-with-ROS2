#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> 
#include <thread>
#include <vector>

int main(int argc, char* argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "end_speed", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("end_speed");

  // We spin up a SingleThreadedExecutor so MoveItVisualTools interact with ROS
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "arm");

  // Construct and initialize MoveItVisualTools
  auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
      node, "base", rviz_visual_tools::RVIZ_MARKER_TOPIC,
      move_group_interface.getRobotModel()};
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();

  // Create closures for visualization
  auto const draw_title = [&moveit_visual_tools](auto text) {
    auto const text_pose = [] {
      auto msg = Eigen::Isometry3d::Identity();
      msg.translation().z() = 1.0;
      return msg;
    }();
    moveit_visual_tools.publishText(text_pose, text, rviz_visual_tools::WHITE,
                                    rviz_visual_tools::XLARGE);
  };
  auto const draw_trajectory_tool_path =
      [&moveit_visual_tools,
      jmg = move_group_interface.getRobotModel()->getJointModelGroup(
          "arm")](auto const trajectory) {
        moveit_visual_tools.publishTrajectoryLine(trajectory, jmg);
      };

  // Get the current pose of the end effector
  geometry_msgs::msg::PoseStamped current_pose = move_group_interface.getCurrentPose();
  geometry_msgs::msg::Pose target_pose = current_pose.pose;

  // Set the target pose (modify x and y coordinates for a straight line movement, z remains the same)
  // target_pose.position.x -= 0.2; // Move -20 cm in the x direction
  // target_pose.position.x -= 0.3; // Move -30 cm in the x direction
  // target_pose.position.x -= 0.4; // Move -40 cm in the x direction
  // target_pose.position.x -= 0.5; // Move -50 cm in the x direction
  target_pose.position.x -= 0.6; // Move -60 cm in the x direction
  
  // z position remains the same: target_pose.position.z

  // Create a vector to hold the waypoints
  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(target_pose);

  // Compute the Cartesian path
  moveit_msgs::msg::RobotTrajectory trajectory_msg;
  const double jump_threshold = 0.0;
  const double eef_step = 0.001; // The resolution in meters for computing the Cartesian path

  double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory_msg);

  if (fraction < 0.9)
  {
    RCLCPP_WARN(logger, "Could not compute the Cartesian path successfully. Only achieved %.2f%% of the path", fraction * 100.0);
  }
  else
  {
    // Execute the Cartesian path
    moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
    cartesian_plan.trajectory_ = trajectory_msg;

    // Visualize the planned trajectory
    draw_trajectory_tool_path(cartesian_plan.trajectory_);
    moveit_visual_tools.trigger();

    // Execute the plan
    move_group_interface.execute(cartesian_plan);
  }

  rclcpp::shutdown();
  spinner.join();
  return 0;
}
