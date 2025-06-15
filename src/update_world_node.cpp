#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <coveragecontrol_sim/update_world.hpp>

int main(int argc, char * argv[]) {

  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor executor;
	// rclcpp::executors::SingleThreadedExecutor executor;
	auto node = std::make_shared<UpdateWorld>(rclcpp::NodeOptions());
  RCLCPP_INFO(node->get_logger(), "UpdateWorld node has been created.");
	executor.add_node(node);
	executor.spin();

  rclcpp::shutdown();
  return 0;
}
