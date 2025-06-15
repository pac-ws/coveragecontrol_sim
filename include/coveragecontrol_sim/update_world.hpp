#ifndef COVERAGE_CONTROL_SIM_UPDATE_WORLD_HPP_
#define COVERAGE_CONTROL_SIM_UPDATE_WORLD_HPP_

#include <async_pac_gnn_interfaces/action/update_world_file.hpp>
#include <async_pac_gnn_interfaces/srv/update_world_file.hpp>
#include <atomic>
#include <future>
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

 public:
  UpdateWorld(const rclcpp::NodeOptions& options);

 private:
  struct Robot {
    std::string ns;
    bool status;
    std::atomic<bool> call_success;
    std::string service_name;
    rclcpp::Client<UpdateWorldFile>::SharedPtr client;
    std::optional<
        rclcpp::Client<UpdateWorldFile>::SharedFutureWithRequestAndRequestId>
        fut;
    Robot(std::string const& in_ns)
        : ns{in_ns}, status{true}, call_success{false} {
      service_name = "/" + ns + "/update_world_file";
    }
  };

  size_t num_ns_ = 1;

  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const UpdateWorldFileAction::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleUpdateWorldFile> goal_handle);

  void handle_accepted(
      const std::shared_ptr<GoalHandleUpdateWorldFile> goal_handle);

  void execute_update_world(
      const std::shared_ptr<GoalHandleUpdateWorldFile> goal_handle);

  void check_robot_status(
      const std::shared_ptr<GoalHandleUpdateWorldFile> goal_handle);

  rclcpp_action::Server<UpdateWorldFileAction>::SharedPtr action_server_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  std::vector<std::shared_ptr<Robot>> robots_;
  std::shared_ptr<GoalHandleUpdateWorldFile> current_goal_handle_;
  std::chrono::steady_clock::time_point start_time_;
  std::chrono::seconds service_timeout_s_{2};
  rclcpp::Duration future_timeout_s_{10, 0};
  rmw_qos_profile_t qos_profile_sensor_data_ = rmw_qos_profile_sensor_data;
  rclcpp::QoS qos_ =
      rclcpp::QoS(rclcpp::QoSInitialization(qos_profile_sensor_data_.history,
                                            qos_profile_sensor_data_.depth),
                  qos_profile_sensor_data_);
};

#endif  // COVERAGE_CONTROL_SIM_UPDATE_WORLD_HPP_
