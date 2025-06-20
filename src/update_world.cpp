#include <chrono>
#include <coveragecontrol_sim/update_world.hpp>
#include <thread>

using namespace std::placeholders;

UpdateWorld::UpdateWorld(const rclcpp::NodeOptions& options)
    : Node("update_world", options) {
  action_server_ = rclcpp_action::create_server<UpdateWorldFileAction>(
      this, "update_world", std::bind(&UpdateWorld::HandleGoal, this, _1, _2),
      std::bind(&UpdateWorld::HandleCancel, this, _1),
      std::bind(&UpdateWorld::HandleAccepted, this, _1));

  RCLCPP_INFO(this->get_logger(), "UpdateWorld action server ready");
}

rclcpp_action::GoalResponse UpdateWorld::HandleGoal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const UpdateWorldFileAction::Goal> goal) {
  RCLCPP_INFO(this->get_logger(),
              "Received goal request for file: %s",
              goal->file.c_str());
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse UpdateWorld::HandleCancel(
    std::shared_ptr<GoalHandleUpdateWorldFile> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void UpdateWorld::HandleAccepted(
    std::shared_ptr<GoalHandleUpdateWorldFile> goal_handle) {
  std::thread{std::bind(&UpdateWorld::ExecuteUpdateWorld, this, goal_handle)}
      .detach();
}

void UpdateWorld::ExecuteUpdateWorld(
    const std::shared_ptr<GoalHandleUpdateWorldFile> goal_handle) {
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<UpdateWorldFileAction::Feedback>();
  auto result = std::make_shared<UpdateWorldFileAction::Result>();

  std::vector<std::string> namespaces = GetNamespaces();
  if (namespaces.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get robot namespaces, aborting action");
    auto result = std::make_shared<UpdateWorldFileAction::Result>();
    result->success = false;
    result->message = "Failed to retrieve robot namespaces";
    result->num_success = 0;
    result->num_total = 0;
    goal_handle->abort(result);
    return;
  }
  
  namespaces.insert(namespaces.begin(), "sim");
  num_ns_ = namespaces.size();

  RCLCPP_INFO(this->get_logger(),
              "Starting update_world action for file: %s with %zu namespaces",
              goal->file.c_str(), num_ns_);

  for (const auto& ns : namespaces) {
    robots_.emplace_back(std::make_shared<Robot>(ns));
  }

  for (auto& robot : robots_) {
    robot->client = this->create_client<UpdateWorldFile>(robot->service_name);
  }

  std::string file = goal->file;

  // Use iterator for safe removal during iteration
  for (auto it = robots_.begin(); it != robots_.end();) {
    auto& robot = *it;
    if (!robot->client->wait_for_service(service_timeout_s_)) {
      persistent_error_message_ += CreateErrorMessage(robot->ns, "service not available");
      failed_namespaces_.push_back(robot->ns);
      num_completed_++;
      
      // Remove robot from active list
      it = robots_.erase(it);
    } else {
      auto robot_request = std::make_shared<UpdateWorldFile::Request>();
      robot_request->file = file;

      robot->fut = robot->client->async_send_request(robot_request);
      ++it;
    }
  }

  feedback->robots_total = num_ns_;
  feedback->robots_completed = num_completed_;
  feedback->failed_namespaces = failed_namespaces_;
  feedback->message = persistent_error_message_;
  feedback->message += "Requests sent. Waiting responses...";
  goal_handle->publish_feedback(feedback);
  start_time_ = std::chrono::steady_clock::now();

  status_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      [this, goal_handle]() { CheckRobotStatus(goal_handle); });
}

void UpdateWorld::PublishResult(
    std::shared_ptr<GoalHandleUpdateWorldFile> goal_handle) {
    if (status_timer_) {
      status_timer_->cancel();
      status_timer_.reset();
    }
    
    auto result = std::make_shared<UpdateWorldFileAction::Result>();
    result->success = (num_success_ == num_ns_);
    result->message = persistent_error_message_;
    result->message += "UpdateWorld: " + std::to_string(num_success_) + " of " +
                       std::to_string(num_ns_) + "\n";
    result->num_success = num_success_;
    result->num_total = num_ns_;

    if (result->success) {
      RCLCPP_INFO(this->get_logger(), 
                  "UpdateWorld action completed successfully! Updated %zu/%zu robots",
                  num_success_, num_ns_);
      goal_handle->succeed(result);
    } else {
      RCLCPP_ERROR(this->get_logger(), 
                   "UpdateWorld action completed with failures! Success: %zu/%zu robots\n%s",
                   num_success_, num_ns_, persistent_error_message_.c_str());
      goal_handle->succeed(result);
    }
    
    goal_handle.reset();
    ResetSystem();
}

std::vector<std::string> UpdateWorld::GetNamespaces() {
  namespaces_client_ = this->create_client<NamespacesRobots>("get_namespaces_robots");
  
  if (!namespaces_client_->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR(this->get_logger(), "get_namespaces_robots service not available");
    return {};
  }

  auto request = std::make_shared<NamespacesRobots::Request>();
  
  try {
    auto future = namespaces_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) != 
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Failed to call get_namespaces_robots service");
      return {};
    }
    
    auto response = future.get();
    RCLCPP_INFO(this->get_logger(), "Retrieved %zu robot namespaces", response->namespaces.size());
    return response->namespaces;
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Exception calling get_namespaces_robots: %s", e.what());
    return {};
  }
}

void UpdateWorld::CheckRobotStatus(
    std::shared_ptr<GoalHandleUpdateWorldFile> goal_handle) {
  if (!goal_handle->is_active()) {
    ResetSystem();
    return;
  }

  if (robots_.empty()) {
    PublishResult(goal_handle);
    return;
  }

  auto feedback = std::make_shared<UpdateWorldFileAction::Feedback>();

  auto elapsed = std::chrono::steady_clock::now() - start_time_;
  auto timeout_duration =
      std::chrono::nanoseconds(future_timeout_s_.nanoseconds());
  bool timed_out = elapsed >= timeout_duration;

  // Use iterator for safe removal during iteration
  for (auto it = robots_.begin(); it != robots_.end();) {
    auto& robot = *it;
    bool remove_robot = false;

    // Check if future is ready (regardless of call_success flag)
    if (robot->fut.has_value() && 
        robot->fut->wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      try {
        auto robot_response = robot->fut->get();
        if (robot_response->success) {
          num_success_++;
        } else {
          failed_namespaces_.push_back(robot->ns);
          persistent_error_message_ += CreateErrorMessage(robot->ns, robot_response->message);
        }
      } catch (const std::exception& e) {
        failed_namespaces_.push_back(robot->ns);
        persistent_error_message_ += CreateErrorMessage(robot->ns, "exception: " + std::string(e.what()));
      }
      num_completed_++;
      remove_robot = true;
    } else if (timed_out) {
      failed_namespaces_.push_back(robot->ns);
      persistent_error_message_ += CreateErrorMessage(robot->ns, "timeout");
      num_completed_++;
      remove_robot = true;
    }

    if (remove_robot) {
      it = robots_.erase(it);
    } else {
      ++it;
    }
  }

  // Update feedback with persistent data
  feedback->robots_total = num_ns_;
  feedback->robots_completed = num_completed_;
  feedback->failed_namespaces = failed_namespaces_;
  feedback->message = "Completed: " + std::to_string(num_completed_) + "/" +
                      std::to_string(num_ns_) +
                      " (Success: " + std::to_string(num_success_) + ")";
  feedback->message += persistent_error_message_;

  goal_handle->publish_feedback(feedback);
  if (num_completed_ >= num_ns_ || timed_out) {
    PublishResult(goal_handle);
    return;
  }
}
