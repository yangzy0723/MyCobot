#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

/**
 * @brief The main function that starts our program.
 *
 * This function sets up our ROS 2 environment and prepares it for robot control.
 *
 * @param argc The number of input arguments our program receives.
 * @param argv The list of input arguments our program receives.
 * @return int A number indicating if our program finished successfully (0) or not.
 */
int main(int argc, char *argv[])
{
  // Start up ROS 2
  rclcpp::init(argc, argv);

  // Creates a node named "hello_moveit". The node is set up to automatically
  // handle any settings (parameters) we might want to change later without editing the code.
  auto const node = std::make_shared<rclcpp::Node>(
      "hello_moveit",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Creates a "logger" that we can use to print out information or error messages
  // as our program runs.
  auto const logger = rclcpp::get_logger("hello_moveit");

  // We'll add code here later to actually plan robot movements
  RCLCPP_INFO(logger, "We'll add code here later to actually plan robot movements");

  // Shut down ROS 2 cleanly when we're done
  rclcpp::shutdown();
  return 0;
}