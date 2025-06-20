#ifndef COVERAGE_CONTROL_SIM_UPDATE_WORLD_HPP_
#define COVERAGE_CONTROL_SIM_UPDATE_WORLD_HPP_

#include <async_pac_gnn_interfaces/action/update_world_file.hpp>
#include <async_pac_gnn_interfaces/srv/update_world_file.hpp>
#include <async_pac_gnn_interfaces/srv/namespaces_robots.hpp>
#include <future>
#include <list>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <string>
#include <vector>

class UpdateWorld : public rclcpp::Node {
  using UpdateWorldFile = async_pac_gnn_interfaces::srv::UpdateWorldFile;
  using UpdateWorldFileAction =
      async_pac_gnn_interfaces::action::UpdateWorldFile;
  using GoalHandleUpdateWorldFile =
      rclcpp_action::ServerGoalHandle<UpdateWorldFileAction>;
  using NamespacesRobots = async_pac_gnn_interfaces::srv::NamespacesRobots;

 public:
  UpdateWorld(const rclcpp::NodeOptions& options);

 private:
  struct Robot {
    std::string ns;
    std::string service_name;
    rclcpp::Client<UpdateWorldFile>::SharedPtr client;
    std::optional<rclcpp::Client<UpdateWorldFile>::FutureAndRequestId> fut;
    Robot(std::string const& in_ns) : ns{in_ns} {
      service_name = "/" + ns + "/update_world_file";
    }
  };

  size_t num_ns_ = 1;

  rclcpp_action::GoalResponse HandleGoal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const UpdateWorldFileAction::Goal> goal);

  rclcpp_action::CancelResponse HandleCancel(
      std::shared_ptr<GoalHandleUpdateWorldFile> goal_handle);

  void HandleAccepted(
      std::shared_ptr<GoalHandleUpdateWorldFile> goal_handle);

  void ExecuteUpdateWorld(
      std::shared_ptr<GoalHandleUpdateWorldFile> goal_handle);

  void CheckRobotStatus(
      std::shared_ptr<GoalHandleUpdateWorldFile> goal_handle);

  void PublishResult(
    std::shared_ptr<GoalHandleUpdateWorldFile> goal_handle);

  std::vector<std::string> GetNamespaces();

  rclcpp_action::Server<UpdateWorldFileAction>::SharedPtr action_server_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  std::list<std::shared_ptr<Robot>> robots_;
  std::chrono::steady_clock::time_point start_time_;
  std::chrono::seconds service_timeout_s_{5};
  rclcpp::Duration future_timeout_s_{15, 0};

  // Persistent feedback data
  size_t num_success_ = 0;
  size_t num_completed_ = 0;
  std::vector<std::string> completed_namespaces_;
  std::vector<std::string> failed_namespaces_;
  std::string persistent_error_message_;
  rmw_qos_profile_t qos_profile_sensor_data_ = rmw_qos_profile_sensor_data;
  rclcpp::QoS qos_ =
      rclcpp::QoS(rclcpp::QoSInitialization(qos_profile_sensor_data_.history,
                                            qos_profile_sensor_data_.depth),
                  qos_profile_sensor_data_);

  rclcpp::Client<NamespacesRobots>::SharedPtr namespaces_client_;

  void ResetSystem() {
    for (auto robot : robots_) {
      if (robot) {
        if (robot->client) {
          robot->client->prune_pending_requests();
          robot->client.reset();
        }
        robot.reset();
      }
    }
    robots_.clear();
    if (status_timer_) {
      status_timer_->cancel();
      status_timer_.reset();
    }
    completed_namespaces_.clear();
    failed_namespaces_.clear();
    persistent_error_message_.clear();
    num_ns_ = 1;
    num_success_ = 0;
    num_completed_ = 0;
  }

  std::string CreateErrorMessage(const std::string& ns, const std::string& message) {
    std::string error_msg = "[Error: " + ns + " ] " + message + "\n";
    RCLCPP_ERROR(this->get_logger(), "%s", error_msg.c_str());
    return error_msg;
  }
};

#endif  // COVERAGE_CONTROL_SIM_UPDATE_WORLD_HPP_
