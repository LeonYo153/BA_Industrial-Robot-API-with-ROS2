#include <memory>
#include <fstream>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <thread>
#include <vector>
#include <sys/stat.h>
#include <sstream>
#include <cstdlib>

// Function to check if a file exists
bool fileExists(const std::string &filename)
{
  struct stat buffer;
  return (stat(filename.c_str(), &buffer) == 0);
}

// Function to generate a new filename if the original exists
std::string generateFilename(const std::string &directory, const std::string &baseName, const std::string &extension)
{
  std::string filename = directory + "/" + baseName + extension;
  int index = 1;
  while (fileExists(filename))
  {
    std::stringstream ss;
    ss << directory << "/" << baseName << "(" << index << ")" << extension;
    filename = ss.str();
    index++;
  }
  return filename;
}

class JointSpeedLogger : public rclcpp::Node
{
public:
  JointSpeedLogger()
  : Node("joint_speed_logger"), last_velocity_time_(0)
  {
    joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10, std::bind(&JointSpeedLogger::jointStateCallback, this, std::placeholders::_1));
  }

  void startLogging(const std::string &filename)
  {
    output_file_.open(filename);
    if (!output_file_.is_open())
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", filename.c_str());
      return;
    }
    // Write CSV header
    output_file_ << "timestamp,velocity,acceleration\n";
    logging_ = true;
    start_time_ = this->now();
    last_velocity_time_ = rclcpp::Time(0); // Reset the last velocity time
    RCLCPP_INFO(this->get_logger(), "Started logging to %s", filename.c_str());
  }

  void stopLogging()
  {
    logging_ = false;
    if (output_file_.is_open())
    {
      output_file_.close();
    }
    RCLCPP_INFO(this->get_logger(), "Stopped logging");
  }

private:
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (logging_)
    {
      auto now = this->now();
      double timestamp = (now - start_time_).seconds();

      for (size_t i = 0; i < msg->name.size(); ++i)
      {
        if (msg->name[i] == "drive1_joint")  // Update to the actual joint name you want to log
        {
          double current_velocity = msg->velocity[i];
          double current_acceleration = 0.0;

          if (last_velocity_time_.seconds() != 0.0)
          {
            double dt = (now - last_velocity_time_).seconds();
            current_acceleration = (current_velocity - last_velocity_) / dt;
          }

          output_file_ << timestamp << "," << current_velocity << "," << current_acceleration << "\n";
          RCLCPP_INFO(this->get_logger(), "Logged data: timestamp=%.3f, velocity=%.3f, acceleration=%.3f",
                      timestamp, current_velocity, current_acceleration);

          last_velocity_ = current_velocity;
          last_velocity_time_ = now;
        }
      }
    }
  }

  bool logging_ = false;
  std::ofstream output_file_;
  rclcpp::Time start_time_;
  double last_velocity_ = 0.0;
  rclcpp::Time last_velocity_time_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
};

int main(int argc, char* argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("angle_speed");
  auto speed_logger_node = std::make_shared<JointSpeedLogger>();

  // Create a ROS logger
  auto logger = rclcpp::get_logger("angle_speed");

  // We spin up a SingleThreadedExecutor so MoveItVisualTools interact with ROS
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(speed_logger_node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "arm");

  // Get the current joint values
  std::vector<double> joint_group_positions = move_group_interface.getCurrentJointValues();

  // Set the target for drive1_joint (change accordingly)
  // joint_group_positions[0] = M_PI / 36; // Set joint1 to 5 degrees (PI/36 radians)
  joint_group_positions[0] = M_PI / 18; // Set joint1 to 10 degrees (PI/18 radians)
  // joint_group_positions[0] = M_PI / 6; // Set joint1 to 30 degrees (PI/6 radians)
  // joint_group_positions[0] = M_PI / 4; // Set joint1 to 45 degrees (PI/4 radians)
  // joint_group_positions[0] = M_PI / 2; // Set joint1 to 90 degrees (PI/2 radians)

  // Set the maximum velocity and acceleration scaling factors to maximum
  move_group_interface.setMaxVelocityScalingFactor(0.5); // 100% of the maximum velocity
  move_group_interface.setMaxAccelerationScalingFactor(0.5); // 100% of the maximum acceleration

  // Get home directory and generate filename
  std::string home_dir = std::getenv("HOME");
  std::string filename = generateFilename(home_dir, "angle10", ".csv");

  // Set the target joint values
  move_group_interface.setJointValueTarget(joint_group_positions);

  // Create a plan to the target joint values
  auto const [success, plan] = [&move_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if (success)
  {
    // Start logging right before execution
    speed_logger_node->startLogging(filename);

    // Execute the plan
    move_group_interface.execute(plan);

    // Stop logging right after execution
    speed_logger_node->stopLogging();
  }
  else
  {
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  // Stop logging
  speed_logger_node->stopLogging();

  rclcpp::shutdown();
  spinner.join();
  return 0;
}



