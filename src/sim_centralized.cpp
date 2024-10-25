#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <coveragecontrol_sim/centralized.hpp>
// #include <coveragecontrol_sim/clairvoyant_cvt.hpp>
// #include <coveragecontrol_sim/centralized_cvt.hpp>

using namespace CoverageControlSim;

int main(int argc, char * argv[]) {

  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor executor;
	// rclcpp::executors::SingleThreadedExecutor executor;
	auto sim_node = std::make_shared<CoverageControlSimCentralized>();
  /* auto clairvoyant_cvt = std::make_shared<RosClairvoyantCVT>(sim_node->GetParameters(), sim_node->GetWorldMap()); */
  /* auto centralized_cvt = std::make_shared<RosCentralizedCVT>(sim_node->GetParameters()); */
  RCLCPP_INFO(sim_node->get_logger(), "Centralized sim_node has been created.");
	executor.add_node(sim_node);
  /* executor.add_node(clairvoyant_cvt); */
  /* executor.add_node(centralized_cvt); */
	executor.spin();

  rclcpp::shutdown();
  return 0;
}
