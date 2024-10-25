#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <coveragecontrol_sim/decentralized_cvt.hpp>

using namespace CoverageControlSim;

int main(int argc, char * argv[]) {

  rclcpp::init(argc, argv);

  /* rclcpp::executors::MultiThreadedExecutor executor; */
	rclcpp::executors::SingleThreadedExecutor executor;
  auto decentralized_cvt = std::make_shared<RosDecentralizedCVT>();
  executor.add_node(decentralized_cvt);
	executor.spin();

  rclcpp::shutdown();
  return 0;
}
