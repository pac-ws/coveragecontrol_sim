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
  // Lambda function for error message construction and logging
  auto create_error_msg = [this](const std::string& ns, const std::string& message) -> std::string {
    std::string error_msg = "[Error: " + ns + " ] " + message + "\n";
    RCLCPP_ERROR(this->get_logger(), "%s", error_msg.c_str());
    return error_msg;
  };

  // Initialize persistent feedback data
  num_success_ = 0;
  num_completed_ = 0;
  completed_namespaces_.clear();
  failed_namespaces_.clear();
  persistent_error_message_.clear();

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

  std::string file = goal->file;

  // Use iterator for safe removal during iteration
  for (auto it = robots_.begin(); it != robots_.end();) {
    auto& robot = *it;
    if (!robot->client->wait_for_service(service_timeout_s_)) {
      // Add to persistent error message and failed namespaces
      persistent_error_message_ += create_error_msg(robot->ns, "service not available");
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
  feedback->completed_namespaces = completed_namespaces_;
  feedback->failed_namespaces = failed_namespaces_;
  feedback->message = persistent_error_message_;
  feedback->message += "Requests sent. Waiting responses...";
  goal_handle->publish_feedback(feedback);
  start_time_ = std::chrono::steady_clock::now();

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

  // Early termination if no robots left to check
  if (robots_.empty()) {
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

    auto elapsed_time = std::chrono::steady_clock::now() - start_time_;
    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed_time).count();
    
    if (result->success) {
      RCLCPP_INFO(this->get_logger(), 
                  "UpdateWorld action completed successfully! Updated %zu/%zu robots in %ld ms (early termination)",
                  num_success_, num_ns_, elapsed_ms);
      goal_handle->succeed(result);
    } else {
      RCLCPP_ERROR(this->get_logger(), 
                   "UpdateWorld action completed with failures! Success: %zu/%zu robots in %ld ms (early termination)\n%s",
                   num_success_, num_ns_, elapsed_ms, persistent_error_message_.c_str());
      goal_handle->succeed(result);
    }
    
    current_goal_handle_.reset();
    return;
  }

  auto feedback = std::make_shared<UpdateWorldFileAction::Feedback>();
  auto result = std::make_shared<UpdateWorldFileAction::Result>();

  // Lambda function for error message construction and logging
  auto create_error_msg = [this](const std::string& ns, const std::string& message) -> std::string {
    std::string error_msg = "[Error: " + ns + " ] " + message + "\n";
    RCLCPP_ERROR(this->get_logger(), "%s", error_msg.c_str());
    return error_msg;
  };

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
          completed_namespaces_.push_back(robot->ns);
          num_success_++;
        } else {
          failed_namespaces_.push_back(robot->ns);
          persistent_error_message_ += create_error_msg(robot->ns, robot_response->message);
        }
      } catch (const std::exception& e) {
        failed_namespaces_.push_back(robot->ns);
        persistent_error_message_ += create_error_msg(robot->ns, "exception: " + std::string(e.what()));
      }
      num_completed_++;
      remove_robot = true;
    } else if (timed_out) {
      failed_namespaces_.push_back(robot->ns);
      persistent_error_message_ += create_error_msg(robot->ns, "timeout");
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
  feedback->completed_namespaces = completed_namespaces_;
  feedback->failed_namespaces = failed_namespaces_;
  feedback->message = "Completed: " + std::to_string(num_completed_) + "/" +
                      std::to_string(num_ns_) +
                      " (Success: " + std::to_string(num_success_) + ")";
  feedback->message += persistent_error_message_;

  goal_handle->publish_feedback(feedback);

  // Check if all robots are completed or timed out
  if (num_completed_ >= num_ns_ || timed_out) {
    if (status_timer_) {
      status_timer_->cancel();
      status_timer_.reset();
    }

    result->success = (num_success_ == num_ns_);
    result->message = persistent_error_message_;
    result->message += "UpdateWorld: " + std::to_string(num_success_) + " of " +
                       std::to_string(num_ns_) + "\n";
    result->num_success = num_success_;
    result->num_total = num_ns_;

    auto elapsed_time = std::chrono::steady_clock::now() - start_time_;
    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed_time).count();
    
    if (result->success) {
      RCLCPP_INFO(this->get_logger(), 
                  "UpdateWorld action completed successfully! Updated %zu/%zu robots in %ld ms",
                  num_success_, num_ns_, elapsed_ms);
      goal_handle->succeed(result);
    } else {
      RCLCPP_ERROR(this->get_logger(), 
                   "UpdateWorld action completed with failures! Success: %zu/%zu robots in %ld ms\n%s",
                   num_success_, num_ns_, elapsed_ms, persistent_error_message_.c_str());
      goal_handle->succeed(result);
    }

    current_goal_handle_.reset();
  }
}
