#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <coveragecontrol_sim/centralized_cvt.hpp>

using namespace CoverageControlSim;

int main(int argc, char * argv[]) {

  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor executor;
	// rclcpp::executors::SingleThreadedExecutor executor;
  auto centralized_cvt = std::make_shared<RosCentralizedCVT>();
  executor.add_node(centralized_cvt);
	executor.spin();

  rclcpp::shutdown();
  return 0;
}
