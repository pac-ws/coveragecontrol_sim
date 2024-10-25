#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <coveragecontrol_sim/clairvoyant_cvt.hpp>

using namespace CoverageControlSim;

int main(int argc, char * argv[]) {

  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor executor;
	// rclcpp::executors::SingleThreadedExecutor executor;
  auto clairvoyant_cvt = std::make_shared<RosClairvoyantCVT>();
  executor.add_node(clairvoyant_cvt);
	executor.spin();

  rclcpp::shutdown();
  return 0;
}
