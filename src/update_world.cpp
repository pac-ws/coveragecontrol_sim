#include <chrono>
#include <coveragecontrol_sim/update_world.hpp>
#include <thread>

using namespace std::placeholders;

UpdateWorld::UpdateWorld(const rclcpp::NodeOptions& options)
    : Node("update_world", options) {
  action_server_ = rclcpp_action::create_server<UpdateWorldFileAction>(
      this, "update_world", std::bind(&UpdateWorld::handle_goal, this, _1, _2),
      std::bind(&UpdateWorld::handle_cancel, this, _1),
      std::bind(&UpdateWorld::handle_accepted, this, _1));

  RCLCPP_INFO(this->get_logger(), "UpdateWorld action server ready");
}

rclcpp_action::GoalResponse UpdateWorld::handle_goal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const UpdateWorldFileAction::Goal> goal) {
  RCLCPP_INFO(this->get_logger(),
              "Received goal request for file: %s with %zu namespaces",
              goal->file.c_str(), goal->namespaces.size());
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse UpdateWorld::handle_cancel(
    const std::shared_ptr<GoalHandleUpdateWorldFile> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void UpdateWorld::handle_accepted(
    const std::shared_ptr<GoalHandleUpdateWorldFile> goal_handle) {
  std::thread{std::bind(&UpdateWorld::execute_update_world, this, goal_handle)}
      .detach();
}

void UpdateWorld::execute_update_world(
    const std::shared_ptr<GoalHandleUpdateWorldFile> goal_handle) {
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<UpdateWorldFileAction::Feedback>();
  auto result = std::make_shared<UpdateWorldFileAction::Result>();

  current_goal_handle_ = goal_handle;
  start_time_ = std::chrono::steady_clock::now();

  std::vector<std::string> namespaces = goal->namespaces;
  namespaces.insert(namespaces.begin(), "sim");
  num_ns_ = namespaces.size();

  RCLCPP_INFO(this->get_logger(),
              "Starting update_world action for file: %s with %zu namespaces",
              goal->file.c_str(), num_ns_);

  robots_.clear();
  for (const auto& ns : namespaces) {
    robots_.emplace_back(std::make_shared<Robot>(ns));
  }

  for (auto& robot : robots_) {
    robot->client = this->create_client<UpdateWorldFile>(robot->service_name);
  }

  std::string error_message;
  std::string file = goal->file;

  for (auto& robot : robots_) {
    if (!robot->client->wait_for_service(service_timeout_s_)) {
      error_message += "[Error: " + robot->ns + " ] service not available\n";
      robot->status = false;
      feedback->failed_namespaces.push_back(robot->ns);
    } else {
      auto robot_request = std::make_shared<UpdateWorldFile::Request>();
      robot_request->file = file;

      auto cb =
          [robot](rclcpp::Client<UpdateWorldFile>::SharedFutureWithRequest) {
            robot->call_success.store(true);
          };

      robot->fut =
          robot->client->async_send_request(robot_request, std::move(cb));
    }
  }

  feedback->robots_total = num_ns_;
  feedback->robots_completed = 0;
  feedback->message = "Requests sent to all robots, waiting for responses...";
  goal_handle->publish_feedback(feedback);

  status_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      [this, goal_handle]() { check_robot_status(goal_handle); });
}

void UpdateWorld::check_robot_status(
    const std::shared_ptr<GoalHandleUpdateWorldFile> goal_handle) {
  if (!goal_handle->is_active()) {
    if (status_timer_) {
      status_timer_->cancel();
      status_timer_.reset();
    }
    return;
  }

  auto feedback = std::make_shared<UpdateWorldFileAction::Feedback>();
  auto result = std::make_shared<UpdateWorldFileAction::Result>();

  std::string error_message;
  size_t num_success = 0;
  size_t num_completed = 0;

  feedback->completed_namespaces.clear();
  feedback->failed_namespaces.clear();

  auto elapsed = std::chrono::steady_clock::now() - start_time_;
  auto timeout_duration =
      std::chrono::nanoseconds(future_timeout_s_.nanoseconds());
  bool timed_out = elapsed >= timeout_duration;

  for (auto& robot : robots_) {
    if (robot->status == false) {
      feedback->failed_namespaces.push_back(robot->ns);
      num_completed++;
      continue;
    }

    if (robot->call_success.load()) {
      if (robot->fut.has_value()) {
        try {
          auto request_response_pair = robot->fut->future.get();
          auto robot_response = request_response_pair.second;
          if (robot_response->success) {
            feedback->completed_namespaces.push_back(robot->ns);
            num_success++;
          } else {
            feedback->failed_namespaces.push_back(robot->ns);
            error_message +=
                "[Error: " + robot->ns + " ] " + robot_response->message + "\n";
          }
        } catch (const std::exception& e) {
          feedback->failed_namespaces.push_back(robot->ns);
          error_message += "[Error: " + robot->ns +
                           " ] exception: " + std::string(e.what()) + "\n";
        }
      } else {
        feedback->failed_namespaces.push_back(robot->ns);
        error_message += "[Error: " + robot->ns + " ] no future available\n";
      }
      num_completed++;
    } else if (timed_out) {
      feedback->failed_namespaces.push_back(robot->ns);
      error_message += "[Error: " + robot->ns + " ] timeout\n";
      num_completed++;
    }
  }

  feedback->robots_total = num_ns_;
  feedback->robots_completed = num_completed;
  feedback->message = "Completed: " + std::to_string(num_completed) + "/" +
                      std::to_string(num_ns_) +
                      " (Success: " + std::to_string(num_success) + ")";

  goal_handle->publish_feedback(feedback);

  if (num_completed >= num_ns_ || timed_out) {
    if (status_timer_) {
      status_timer_->cancel();
      status_timer_.reset();
    }

    result->success = (num_success == num_ns_);
    result->message = error_message;
    result->message += "UpdateWorld: " + std::to_string(num_success) + " of " +
                       std::to_string(num_ns_) + "\n";
    result->num_success = num_success;
    result->num_total = num_ns_;

    if (result->success) {
      RCLCPP_INFO(this->get_logger(), "Successfully updated all worlds");
      goal_handle->succeed(result);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to update all worlds\n%s",
                   error_message.c_str());
      goal_handle->succeed(result);
    }

    current_goal_handle_.reset();
  }
}
