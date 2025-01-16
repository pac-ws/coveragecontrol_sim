#include <coveragecontrol_sim/centralized.hpp>
#include <memory_resource>

namespace CoverageControlSim {

void CoverageControlSimCentralized::CreateServiceServers() {
  world_map_service_ = this->create_service<
      async_pac_gnn_interfaces::srv::WorldMap>(
      "get_world_map",
      [this](const std::shared_ptr<async_pac_gnn_interfaces::srv::WorldMap::Request> request,
         std::shared_ptr<async_pac_gnn_interfaces::srv::WorldMap::Response>
             response) {
        RCLCPP_INFO(this->get_logger(),
                    "Incoming request\nmap_size: %d", request->map_size);
        if(request->map_size != parameters_.pWorldMapSize) {
          response->success = false;
          response->map = EigenMatrixRowMajorToFloat32MultiArray(
              Eigen::MatrixXf::Zero(parameters_.pWorldMapSize,
                                    parameters_.pWorldMapSize));
          RCLCPP_ERROR(this->get_logger(),
                       "World map size does not match with the system");
        } else {
          response->success = true;
          response->map = EigenMatrixRowMajorToFloat32MultiArray(
              coverage_system_ptr_->GetWorldMap());
          RCLCPP_INFO(this->get_logger(), "World map sent");
        }
      });
}

void CoverageControlSimCentralized::CreateSimCentralizedSetup() {
  if (idf_file_ != "") {
    // Check if the files exist
    if (rcpputils::fs::exists(idf_file_)) {
      RCLCPP_INFO(this->get_logger(), "Reading IDF file %s", idf_file_.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "IDF file %s does not exist",
                   idf_file_.c_str());
      rclcpp::shutdown();
    }
    WorldIDF world_idf(parameters_, idf_file_);
    parameters_.pNumGaussianFeatures = world_idf.GetNumFeatures();
    CoverageControl::PointVector robot_positions(parameters_.pNumRobots);
    coverage_system_ptr_ = std::make_shared<CoverageSystem>(
        parameters_, world_idf, robot_positions);
  } else {
    RCLCPP_ERROR(this->get_logger(), "idf_file is empty.");
  }
  RCLCPP_INFO(this->get_logger(), "Created coverage system");
}

void CoverageControlSimCentralized::CreateCmdSubscribers() {
  cbg_cmd_pos_sub_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto cbg_cmd_pos_opts = rclcpp::SubscriptionOptions();
  cbg_cmd_pos_opts.callback_group = cbg_cmd_pos_sub_;

  cbg_cmd_global_pos_sub_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto cbg_cmd_global_pos_opts = rclcpp::SubscriptionOptions();
  cbg_cmd_pos_opts.callback_group = cbg_cmd_global_pos_sub_;

  cbg_cmd_vel_sub_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto cbg_cmd_vel_opts = rclcpp::SubscriptionOptions();
  cbg_cmd_vel_opts.callback_group = cbg_cmd_vel_sub_;

  for (int robot_id = 0; robot_id < parameters_.pNumRobots; ++robot_id) {
    std::string topic_name =
        namespaces_of_robots_[robot_id] + "/cmd_relative_pose";
    auto sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        topic_name, rclcpp::QoS(buffer_size_),
        [this, robot_id](
            const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) -> void {
          Point2 robot_pos;
          robot_pos[0] = msg->pose.position.x;
          robot_pos[1] = msg->pose.position.y;
          if (coverage_system_ptr_ != nullptr) {
            coverage_system_ptr_->SetLocalRobotPosition(robot_id, robot_pos);
          } else {
            RCLCPP_WARN(this->get_logger(), "Coverage system not initialized");
          }
        },
        cbg_cmd_pos_opts);
    cmd_pos_subs_.push_back(sub);
  }

  for (int robot_id = 0; robot_id < parameters_.pNumRobots; ++robot_id) {
    std::string topic_name =
        namespaces_of_robots_[robot_id] + "/cmd_global_pose";
    auto sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        topic_name, rclcpp::QoS(buffer_size_),
        [this, robot_id](
            const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) -> void {
          // Update robot position
          Point2 robot_pos;
          robot_pos[0] = msg->pose.position.x;
          robot_pos[1] = msg->pose.position.y;
          if (coverage_system_ptr_ != nullptr) {
            coverage_system_ptr_->SetGlobalRobotPosition(robot_id, robot_pos);
          } else {
            RCLCPP_WARN(this->get_logger(), "Coverage system not initialized");
          }
        },
        cbg_cmd_global_pos_opts);
    cmd_global_pos_subs_.push_back(sub);
  }

  for (int robot_id = 0; robot_id < parameters_.pNumRobots; ++robot_id) {
    std::string topic_name = namespaces_of_robots_[robot_id] + "/cmd_vel";
    auto sub = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        topic_name, rclcpp::QoS(buffer_size_),
        [this,
         robot_id](const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
            -> void {
          // Update robot position
          Point2 robot_vel;
          robot_vel[0] = msg->twist.linear.x;
          robot_vel[1] = msg->twist.linear.y;
          bool res = true;
          if (coverage_system_ptr_ != nullptr) {
            res = coverage_system_ptr_->StepAction(robot_id, robot_vel);
          } else {
            RCLCPP_WARN(this->get_logger(), "Coverage system not initialized");
          }
          if (res) {
            RCLCPP_ERROR(this->get_logger(),
                         "Error stepping action for robot %d", robot_id);
          }
        },
        cbg_cmd_vel_opts);
    cmd_vel_subs_.push_back(sub);
  }
}

void CoverageControlSimCentralized::CreateRobotPosPublishers() {
  for (int robot_id = 0; robot_id < parameters_.pNumRobots; ++robot_id) {
    std::string topic_name = namespaces_of_robots_[robot_id] + "/world_pose";
    robot_pos_pubs_.push_back(
        this->create_publisher<geometry_msgs::msg::PoseStamped>(topic_name,
                                                                buffer_size_));
    auto robot_pos_pub = robot_pos_pubs_.back();
    auto robot_pos_pub_timer_callback = [this, robot_id,
                                         robot_pos_pub]() -> void {
      auto robot_pos_msg = XYtoPoseStamped(world_robot_positions_[robot_id][0],
                                           world_robot_positions_[robot_id][1]);
      robot_pos_msg.header.frame_id = "map";
      robot_pos_msg.header.stamp = this->now();
      robot_pos_pub->publish(robot_pos_msg);
    };
    robot_pos_pub_timers_.push_back(
        this->create_wall_timer(30ms, robot_pos_pub_timer_callback));
  }
}

void CoverageControlSimCentralized::CreateRobotSimPosPublishers() {
  for (int robot_id = 0; robot_id < parameters_.pNumRobots; ++robot_id) {
    std::string topic_name = namespaces_of_robots_[robot_id] + "/sim_pose";
    robot_sim_pos_pubs_.push_back(
        this->create_publisher<geometry_msgs::msg::PoseStamped>(topic_name,
                                                                buffer_size_));
    auto robot_sim_pos_pub = robot_sim_pos_pubs_.back();
    auto robot_sim_pos_pub_timer_callback = [this, robot_id,
                                             robot_sim_pos_pub]() -> void {
      sim_robot_positions_[robot_id] =
          coverage_system_ptr_->GetRobotPosition(robot_id);
      auto robot_pos_msg = XYtoPoseStamped(sim_robot_positions_[robot_id][0],
                                           sim_robot_positions_[robot_id][1]);
      robot_pos_msg.header.frame_id = "map";
      robot_pos_msg.header.stamp = this->now();
      robot_sim_pos_pub->publish(robot_pos_msg);
    };
    robot_sim_pos_pub_timers_.push_back(
        this->create_wall_timer(30ms, robot_sim_pos_pub_timer_callback));
  }
}

void CoverageControlSimCentralized::CreateRobotMapPublishers() {
  for (int robot_id = 0; robot_id < parameters_.pNumRobots; ++robot_id) {
    std::string topic_name = namespaces_of_robots_[robot_id] + "/map";
    robot_map_pubs_.push_back(
        this->create_publisher<std_msgs::msg::Float32MultiArray>(topic_name,
                                                                 buffer_size_));
    auto robot_map_pub = robot_map_pubs_.back();
    auto robot_map_pub_timer_callback = [this, robot_id,
                                         robot_map_pub]() -> void {
      MapType robot_map = coverage_system_ptr_->GetRobotMap(robot_id);
      auto robot_map_msg = EigenMatrixRowMajorToFloat32MultiArray(robot_map);
      robot_map_pub->publish(robot_map_msg);
    };
    robot_map_pub_timers_.push_back(
        this->create_wall_timer(100ms, robot_map_pub_timer_callback));
  }
}

void CoverageControlSimCentralized::CreateNeigborsPosPublisher() {
  for (int robot_id = 0; robot_id < parameters_.pNumRobots; ++robot_id) {
    std::string topic_name =
        namespaces_of_robots_[robot_id] + "/neighbors_pose";
    robot_neighbors_pose_pubs_.push_back(
        this->create_publisher<geometry_msgs::msg::PoseArray>(topic_name,
                                                              buffer_size_));
    auto robot_neighbor_pos_pub = robot_neighbors_pose_pubs_.back();
    auto robot_neighbor_pos_pub_timer_callback =
        [this, robot_id, robot_neighbor_pos_pub]() -> void {
      auto neighbors_pos =
          coverage_system_ptr_->GetRelativePositonsNeighbors(robot_id);
      geometry_msgs::msg::PoseArray neighbor_pos_msg;
      neighbor_pos_msg.header.frame_id = "map";
      neighbor_pos_msg.header.stamp = this->now();
      for (size_t i = 0; i < neighbors_pos.size(); i++) {
        neighbor_pos_msg.poses.push_back(
            XYtoPose(neighbors_pos[i][0], neighbors_pos[i][1]));
      }
      robot_neighbor_pos_pub->publish(neighbor_pos_msg);
    };
    robot_neighbors_pose_pub_timers_.push_back(
        this->create_wall_timer(100ms, robot_neighbor_pos_pub_timer_callback));
  }
}

void CoverageControlSimCentralized::CreateNeigborsIDPublisher() {
  for (int robot_id = 0; robot_id < parameters_.pNumRobots; ++robot_id) {
    std::string topic_name = namespaces_of_robots_[robot_id] + "/neighbors_id";
    robot_neighbors_id_pubs_.push_back(
        this->create_publisher<std_msgs::msg::Int32MultiArray>(topic_name,
                                                               buffer_size_));
    auto robot_neighbor_id_pub = robot_neighbors_id_pubs_.back();
    auto robot_neighbor_id_pub_timer_callback =
        [this, robot_id, robot_neighbor_id_pub]() -> void {
      auto neighbors_id = coverage_system_ptr_->GetNeighborIDs(robot_id);
      std_msgs::msg::Int32MultiArray neighbor_id_msg;
      neighbor_id_msg.layout.dim.push_back(
          std_msgs::msg::MultiArrayDimension());
      neighbor_id_msg.layout.dim[0].size = neighbors_id.size();
      neighbor_id_msg.layout.dim[0].stride = neighbors_id.size();
      neighbor_id_msg.layout.dim[0].label = "neighbors";
      neighbor_id_msg.data = neighbors_id;
      robot_neighbor_id_pub->publish(neighbor_id_msg);
    };
    robot_neighbors_id_pub_timers_.push_back(
        this->create_wall_timer(100ms, robot_neighbor_id_pub_timer_callback));
  }
}

void CoverageControlSimCentralized::CreateGlobalMapPublisher() {
  std::string topic_name = "global_map";
  global_map_pub_ =
      this->create_publisher<std_msgs::msg::Float32MultiArray>(topic_name, 3);
  auto global_map_pub = global_map_pub_;
  // Create timer to publish local map
  auto global_map_pub_timer_callback = [this, global_map_pub]() -> void {
    auto global_map = coverage_system_ptr_->GetWorldMap();
    auto global_map_msg = EigenMatrixRowMajorToFloat32MultiArray(global_map);
    global_map_pub->publish(global_map_msg);
  };
  global_map_pub_timer_ =
      this->create_wall_timer(500ms, global_map_pub_timer_callback);
}

void CoverageControlSimCentralized::CreateSystemMapPublisher() {
  std::string topic_name = "system_map";
  system_map_pub_ =
      this->create_publisher<std_msgs::msg::Float32MultiArray>(topic_name, 3);
  auto system_map_pub = system_map_pub_;
  // Create timer to publish local map
  auto system_map_pub_timer_callback = [this, system_map_pub]() -> void {
    auto system_map = coverage_system_ptr_->GetSystemMap();
    auto system_map_msg = EigenMatrixRowMajorToFloat32MultiArray(system_map);
    /* auto system_map_msg =
     * EigenMatrixRowMajorToFloat32MultiArray(system_map(Eigen::seq(0,
     * Eigen::last, 2), Eigen::seq(0, Eigen::last, 2))); */
    system_map_pub->publish(system_map_msg);
  };
  system_map_pub_timer_ =
      this->create_wall_timer(500ms, system_map_pub_timer_callback);
}

void CoverageControlSimCentralized::CreateExploredIDFMapPublisher() {
  std::string topic_name = "global_explored_idf_map";
  global_explored_idf_map_pub_ =
      this->create_publisher<std_msgs::msg::Float32MultiArray>(topic_name, 3);
  auto global_explored_idf_map_pub = global_explored_idf_map_pub_;
  // Create timer to publish local map
  auto global_explored_idf_map_pub_timer_callback =
      [this, global_explored_idf_map_pub]() -> void {
    auto global_explored_idf_map =
        coverage_system_ptr_->GetSystemExploredIDFMap();
    auto global_explored_idf_map_msg =
        EigenMatrixRowMajorToFloat32MultiArray(global_explored_idf_map);
    global_explored_idf_map_pub->publish(global_explored_idf_map_msg);
  };
  global_explored_idf_map_pub_timer_ = this->create_wall_timer(
      100ms, global_explored_idf_map_pub_timer_callback);
}

void CoverageControlSimCentralized::CreateAllRobotsPosesPublisher() {
  robot_poses_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
      "all_robot_sim_poses", buffer_size_);
  auto robot_poses_pub = robot_poses_pub_;
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  // Create timer to publish local map
  auto robot_poses_pub_timer_callback = [this, robot_poses_pub]() -> void {
    geometry_msgs::msg::PoseArray robot_poses_msg;
    // Header for pose array
    robot_poses_msg.header.frame_id = "map";
    robot_poses_msg.header.stamp = this->now();

    /* sim_robot_positions_ = coverage_system_ptr_->GetRobotPositions(); */
    for (int robot_id = 0; robot_id < parameters_.pNumRobots; ++robot_id) {
      sim_robot_positions_[robot_id] = world_robot_positions_[robot_id] * env_scale_factor_;
      robot_poses_msg.poses.push_back(
          XYtoPose(sim_robot_positions_[robot_id][0],
                   sim_robot_positions_[robot_id][1]));
      coverage_system_ptr_->SetGlobalRobotPosition(robot_id, sim_robot_positions_[robot_id]);
    }
    robot_poses_pub->publish(robot_poses_msg);
    for (int robot_id = 0; robot_id < parameters_.pNumRobots; ++robot_id) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp = this->now();
      transform.header.frame_id = "map";
      transform.child_frame_id = namespaces_of_robots_[robot_id];
      transform.transform.translation.x = sim_robot_positions_[robot_id][0];
      transform.transform.translation.y = sim_robot_positions_[robot_id][1];
      transform.transform.translation.z = 1.0;
      transform.transform.rotation.x = 0.0;
      transform.transform.rotation.y = 0.0;
      transform.transform.rotation.z = 0.0;
      transform.transform.rotation.w = 1.0;
      tf_broadcaster_->sendTransform(transform);
    }
  };
  robot_poses_pub_timer_ =
      this->create_wall_timer(30ms, robot_poses_pub_timer_callback);
}

void CoverageControlSimCentralized::CreateRobotLocalMapPublishers() {
  for (int robot_id = 0; robot_id < parameters_.pNumRobots; ++robot_id) {
    std::string topic_name = namespaces_of_robots_[robot_id] + "/local_map";
    robot_local_map_pubs_.push_back(
        this->create_publisher<std_msgs::msg::Float32MultiArray>(topic_name,
                                                                 buffer_size_));
    auto robot_local_map_pub = robot_local_map_pubs_.back();
    // Create timer to publish local map
    auto robot_local_map_pub_timer_callback = [this, robot_id,
                                               robot_local_map_pub]() -> void {
      MapType local_map = coverage_system_ptr_->GetRobotLocalMap(robot_id);
      auto local_map_msg = EigenMatrixRowMajorToFloat32MultiArray(local_map);
      robot_local_map_pub->publish(local_map_msg);
    };
    robot_local_map_pub_timers_.push_back(
        this->create_wall_timer(100ms, robot_local_map_pub_timer_callback));
  }
}

void CoverageControlSimCentralized::CreateObstacleMapsPublisher() {
  for (int robot_id = 0; robot_id < parameters_.pNumRobots; ++robot_id) {
    std::string topic_name = namespaces_of_robots_[robot_id] + "/obstacle_map";
    robot_obstacle_map_pubs_.push_back(
        this->create_publisher<std_msgs::msg::Float32MultiArray>(topic_name,
                                                                 buffer_size_));
    auto robot_obstacle_map_pub = robot_obstacle_map_pubs_.back();
    auto robot_obstacle_map_pub_timer_callback =
        [this, robot_id, robot_obstacle_map_pub]() -> void {
      MapType obstacle_map =
          coverage_system_ptr_->GetRobotObstacleMap(robot_id);
      auto obstacle_map_msg =
          EigenMatrixRowMajorToFloat32MultiArray(obstacle_map);
      robot_obstacle_map_pub->publish(obstacle_map_msg);
    };
    robot_obstacle_map_pub_timers_.push_back(
        this->create_wall_timer(100ms, robot_obstacle_map_pub_timer_callback));
  }
}

void CoverageControlSimCentralized::CreateSensorViewPublisher() {
  for (int robot_id = 0; robot_id < parameters_.pNumRobots; ++robot_id) {
    std::string topic_name = namespaces_of_robots_[robot_id] + "/sensor_view";
    robot_sensor_view_pubs_.push_back(
        this->create_publisher<std_msgs::msg::Float32MultiArray>(topic_name,
                                                                 buffer_size_));
    auto robot_sensor_view_pub = robot_sensor_view_pubs_.back();
    auto robot_sensor_view_pub_timer_callback =
        [this, robot_id, robot_sensor_view_pub]() -> void {
      MapType sensor_view = coverage_system_ptr_->GetRobotSensorView(robot_id);
      auto sensor_view_msg =
          EigenMatrixRowMajorToFloat32MultiArray(sensor_view);
      robot_sensor_view_pub->publish(sensor_view_msg);
    };
    robot_sensor_view_pub_timers_.push_back(
        this->create_wall_timer(100ms, robot_sensor_view_pub_timer_callback));
  }
}

void CoverageControlSimCentralized::CreateCmdVelPublisher() {
  for (int robot_id = 0; robot_id < parameters_.pNumRobots; ++robot_id) {
    std::string topic_name = namespaces_of_robots_[robot_id] + "/cmd_vel";
    cmd_vel_pubs_.push_back(
        this->create_publisher<geometry_msgs::msg::TwistStamped>(topic_name,
                                                                 buffer_size_));
  }
  sim_robot_positions_ = coverage_system_ptr_->GetRobotPositions();
  auto cmd_vel_pub_timer_callback = [this]() -> void {
    auto voronoi = Voronoi(
        sim_robot_positions_, coverage_system_ptr_->GetSystemExploredIDFMap(),
        Point2(parameters_.pWorldMapSize, parameters_.pWorldMapSize),
        parameters_.pResolution);
    auto voronoi_cells = voronoi.GetVoronoiCells();
    PointVector actions;
    for (int robot_id = 0; robot_id < parameters_.pNumRobots; ++robot_id) {
      actions.push_back({0, 0});
      Point2 diff =
          voronoi_cells[robot_id].centroid() - sim_robot_positions_[robot_id];
      double speed = std::min(parameters_.pMaxRobotSpeed,
                              diff.norm() / parameters_.pTimeStep);
      Point2 direction(diff);
      direction.normalize();
      actions[robot_id] = speed * direction;
      geometry_msgs::msg::TwistStamped cmd_vel_msg;
      cmd_vel_msg.header.stamp = this->get_clock()->now();
      cmd_vel_msg.twist.linear.x = actions[robot_id][0];
      cmd_vel_msg.twist.linear.y = actions[robot_id][1];
      Point2 robot_vel = {cmd_vel_msg.twist.linear.x,
                          cmd_vel_msg.twist.linear.y};
      cmd_vel_pubs_[robot_id]->publish(cmd_vel_msg);
      auto res = coverage_system_ptr_->StepAction(robot_id, robot_vel);
      if (res) {
        RCLCPP_WARN(this->get_logger(), "Error stepping action for robot %d",
                    robot_id);
      }
    }
  };
  cmd_vel_pub_timer_ =
      this->create_wall_timer(100ms, cmd_vel_pub_timer_callback);
}

}  // namespace CoverageControlSim

/* void CreateCmdVelPublisher1() { */
/*   RCLCPP_INFO(this->get_logger(), "Creating cmd vel publishers"); */
/*   for (int robot_id = 0; robot_id < parameters_.pNumRobots; ++robot_id) { */
/*     std::string topic_name = */
/*         robot_namespace_prefix_ + std::to_string(robot_id) + "/sim_cmd_vel";
 */
/*     cmd_vel_pubs_.push_back( */
/*         this->create_publisher<geometry_msgs::msg::TwistStamped>( */
/*             topic_name, buffer_size_)); */
/*     auto cmd_vel_pub = cmd_vel_pubs_[robot_id]; */
/*     auto cmd_vel_pub_timer_callback = [this, cmd_vel_pub, */
/*                                        robot_id]() -> void { */
/*       RCLCPP_INFO(this->get_logger(), "Robot %d actions %f %f", robot_id, */
/*                   actions[robot_id][0], actions[robot_id][1]); */
/*       geometry_msgs::msg::TwistStamped cmd_vel_msg; */
/*       cmd_vel_msg.header.stamp = this->get_clock()->now(); */
/*       cmd_vel_msg.twist.linear.x = actions[robot_id][0] / (scale_factor_); */
/*       cmd_vel_msg.twist.linear.y = actions[robot_id][1] / (scale_factor_); */
/*       cmd_vel_pub->publish(cmd_vel_msg); */
/*       /1* RCLCPP_INFO(this->get_logger(), "Robot %d cmd vel %f %f", robot_id,
 */
/*        *1/ */
/*       /1*             cmd_vel_msg.twist.linear.x,
 * cmd_vel_msg.twist.linear.y); */
/*        *1/ */
/*     }; */
/*     cmd_vel_pub_timers_.push_back( */
/*         this->create_wall_timer(100ms, cmd_vel_pub_timer_callback)); */
/*   } */
/* } */
