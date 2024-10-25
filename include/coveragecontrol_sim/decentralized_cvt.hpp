#ifndef COVERAGECONTROL_SIM_DECENTRALIZED_CVT_HPP_
#define COVERAGECONTROL_SIM_DECENTRALIZED_CVT_HPP_

#include <CoverageControl/map_utils.h>
#include <CoverageControl/parameters.h>
#include <CoverageControl/typedefs.h>
#include <CoverageControl/voronoi.h>

#include <coveragecontrol_sim/utils.hpp>
#include <filesystem>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <string>
#include <vector>

namespace CoverageControlSim {
using namespace CoverageControl;
class RosDecentralizedCVT : public rclcpp::Node {
 private:
  Parameters parameters_;
  MapType robot_map_;
  Point2 pos_;
  PointVector neighbors_pos_;

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr
      robot_map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      robot_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr neighbors_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  Point2 ComputeGoals() {
    if (robot_map_.rows() == 0 || robot_map_.cols() == 0) {
      RCLCPP_WARN(this->get_logger(), "Robot map is empty.");
      return pos_;
    }
    MapUtils::MapBounds index, offset;
    MapUtils::ComputeOffsets(parameters_.pResolution, pos_,
                             parameters_.pLocalMapSize,
                             parameters_.pWorldMapSize, index, offset);
    MapType robot_local_map =
        robot_map_.block(index.left + offset.left, index.bottom + offset.bottom,
                         offset.width, offset.height);
    Point2 map_translation(
        (index.left + offset.left) * parameters_.pResolution,
        (index.bottom + offset.bottom) * parameters_.pResolution);

    PointVector robot_positions(neighbors_pos_.size() + 1);
    robot_positions[0] = pos_ - map_translation;
    int count = 1;
    for (Point2 const &pos : neighbors_pos_) {
      robot_positions[count] = pos - map_translation;
      ++count;
    }
    Point2 map_size(offset.width, offset.height);
    Voronoi voronoi(robot_positions, robot_local_map, map_size,
                    parameters_.pResolution, true, 0);
    auto vcell = voronoi.GetVoronoiCell();
    Point2 goal = vcell.centroid() + robot_positions[0] + map_translation;
    return goal;
  }

  void ComputeActions() {
    Point2 goal = ComputeGoals();
    Point2 action{0, 0};
    Point2 diff = goal - pos_;
    double dist = diff.norm();
    double speed = dist / parameters_.pTimeStep;
    speed = std::min(parameters_.pMaxRobotSpeed, speed);
    Point2 direction(diff);
    direction.normalize();
    action = speed * direction;
    geometry_msgs::msg::TwistStamped cmd_vel_msg;
    cmd_vel_msg.header.stamp = this->get_clock()->now();
    cmd_vel_msg.twist.linear.x = action[0];
    cmd_vel_msg.twist.linear.y = action[1];
    cmd_vel_pub_->publish(cmd_vel_msg);
  }

 public:
  RosDecentralizedCVT() : Node("decentralized_cvt") {
    std::string params_file =
        this->declare_parameter<std::string>("params_file", "");
    if (params_file.empty()) {
      RCLCPP_ERROR(this->get_logger(), "params_file is empty.");
      return;
    }
    if (std::filesystem::exists(params_file)) {
      parameters_ = Parameters(params_file);
    } else {
      RCLCPP_ERROR(this->get_logger(), "params_file does not exist.");
      return;
    }
    pos_ = Point2(0, 0);
    RCLCPP_INFO(this->get_logger(), "Decentralized CVT node started.");

    robot_pose_sub_ =
        this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "pose", 1, [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
              pos_ = {msg->pose.position.x, msg->pose.position.y};
            });
    RCLCPP_INFO(this->get_logger(), "Subscribed to pose topic.");

    robot_map_sub_ =
        this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "map", 1, [this](std_msgs::msg::Float32MultiArray::SharedPtr msg) {
              robot_map_ = Float32MultiArrayToEigenMatrixRowMajor(*msg);
            });
    RCLCPP_INFO(this->get_logger(), "Subscribed to map topic.");

    neighbors_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "neighbors_pose", 1,
        [this](geometry_msgs::msg::PoseArray::SharedPtr msg) {
          neighbors_pos_.clear();
          for (auto const &pose : msg->poses) {
            Point2 relative_pos(pose.position.x, pose.position.y);
            if (relative_pos.norm() < parameters_.pCommunicationRange) {
              neighbors_pos_.push_back(relative_pos);
            }
          }
        });
    RCLCPP_INFO(this->get_logger(), "Subscribed to neighbors_pose topic.");

    while (pos_ == Point2(0, 0)) {
      RCLCPP_INFO(this->get_logger(), "Waiting for robot pose.");
      rclcpp::sleep_for(std::chrono::milliseconds(100));
      rclcpp::spin_some(this->get_node_base_interface());
    }

    cmd_vel_pub_ =
        this->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 1);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(
                                         parameters_.pTimeStep * 1000)),
                                     [this]() { ComputeActions(); });
    RCLCPP_INFO(this->get_logger(), "Timer started.");
  }
};
}  // namespace CoverageControlSim

#endif  // COVERAGECONTROL_SIM_DECENTRALIZED_CVT_HPP_
