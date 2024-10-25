#ifndef COVERAGECONTROL_SIM_CENTRALIZED_CVT_HPP_
#define COVERAGECONTROL_SIM_CENTRALIZED_CVT_HPP_

#include <CoverageControl/parameters.h>
#include <CoverageControl/typedefs.h>
#include <CoverageControl/voronoi.h>

#include <coveragecontrol_sim/utils.hpp>
#include <filesystem>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <string>
#include <vector>

namespace CoverageControlSim {
class RosCentralizedCVT : public rclcpp::Node {
 private:
  std::vector<std::string> namespaces_of_robots_;
  CoverageControl::Parameters parameters_;
  CoverageControl::MapType explored_idf_map_;
  CoverageControl::PointVector robot_poses_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr
      explored_idf_map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr
      robot_poses_sub_;
  std::vector<rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr>
      cmd_vel_pubs_;
  rclcpp::TimerBase::SharedPtr timer_;

  void ComputeActions() {
    CoverageControl::Voronoi voronoi(
        robot_poses_, explored_idf_map_,
        CoverageControl::Point2(parameters_.pWorldMapSize,
                                parameters_.pWorldMapSize),
        parameters_.pResolution);
    auto voronoi_cells = voronoi.GetVoronoiCells();
    CoverageControl::PointVector actions;
    for (int robot_id = 0; robot_id < parameters_.pNumRobots; ++robot_id) {
      actions.push_back({0, 0});
      CoverageControl::Point2 voronoi_centroid = voronoi_cells[robot_id].centroid();
      CoverageControl::Point2 diff = voronoi_centroid - robot_poses_[robot_id];

      double speed = std::min(parameters_.pMaxRobotSpeed,
                              diff.norm() / parameters_.pTimeStep);
      CoverageControl::Point2 direction(diff);
      direction.normalize();
      actions[robot_id] = speed * direction;
      geometry_msgs::msg::TwistStamped cmd_vel_msg;
      cmd_vel_msg.header.stamp = this->get_clock()->now();
      cmd_vel_msg.twist.linear.x = actions[robot_id][0];
      cmd_vel_msg.twist.linear.y = actions[robot_id][1];
      cmd_vel_pubs_[robot_id]->publish(cmd_vel_msg);
    }
  }

 public:
  RosCentralizedCVT() : Node("centralized_cvt") {
    std::string params_file =
        this->declare_parameter<std::string>("params_file", "");
    if (params_file.empty()) {
      RCLCPP_ERROR(this->get_logger(), "params_file is empty.");
      return;
    }
    if (std::filesystem::exists(params_file)) {
      parameters_ = CoverageControl::Parameters(params_file);
    } else {
      RCLCPP_ERROR(this->get_logger(), "params_file does not exist.");
      return;
    }

    namespaces_of_robots_ = this->declare_parameter<std::vector<std::string>>(
        "namespaces_of_robots", std::vector<std::string>());
    if (namespaces_of_robots_.size() !=
        static_cast<size_t>(parameters_.pNumRobots)) {
      RCLCPP_WARN(
          this->get_logger(),
          "Number of robot namespaces does not match number of robots: %ld vs %d",
          namespaces_of_robots_.size(), parameters_.pNumRobots);
      RCLCPP_WARN(this->get_logger(),
                  "Creating robot namespaces with default names");
      namespaces_of_robots_.clear();
      for (int i = 0; i < parameters_.pNumRobots; ++i) {
        namespaces_of_robots_.push_back("robot" + std::to_string(i));
      }
    }

    robot_poses_ = CoverageControl::PointVector(parameters_.pNumRobots,
                                                CoverageControl::Point2(0, 0));

    robot_poses_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "all_robot_poses", 1,
        [this](geometry_msgs::msg::PoseArray::SharedPtr msg) {
          for (int i = 0; i < parameters_.pNumRobots; ++i) {
            robot_poses_[i] = CoverageControl::Point2(msg->poses[i].position.x,
                                                      msg->poses[i].position.y);
          }
        });

    explored_idf_map_sub_ =
        this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "global_explored_idf_map", 1,
            [this](std_msgs::msg::Float32MultiArray::SharedPtr msg) {
              explored_idf_map_ = Float32MultiArrayToEigenMatrixRowMajor(*msg);
            });

    // Check if robot_poses_ is updated
    while (robot_poses_[0] == CoverageControl::Point2(0, 0)) {
      RCLCPP_WARN(this->get_logger(), "robot_poses_ is not updated");
      rclcpp::sleep_for(std::chrono::milliseconds(100));
      rclcpp::spin_some(this->get_node_base_interface());
    }

    for (int i = 0; i < parameters_.pNumRobots; ++i) {
      cmd_vel_pubs_.push_back(
          this->create_publisher<geometry_msgs::msg::TwistStamped>(
              namespaces_of_robots_[i] + "/cmd_vel", 1));
    }

    timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(
                                         parameters_.pTimeStep * 1000)),
                                     [this]() { ComputeActions(); });
  }
};
}  // namespace CoverageControlSim

#endif  // COVERAGECONTROL_SIM_CENTRALIZED_CVT_HPP_
