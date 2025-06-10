#include <coveragecontrol_sim/centralized.hpp>

namespace CoverageControlSim {

void CoverageControlSimCentralized::InitializeParameters() {
  params_file_ = this->declare_parameter<std::string>("params_file");
  parameters_ = Parameters(params_file_);
  idf_file_ = this->declare_parameter<std::string>("idf_file");
  env_scale_factor_ = this->declare_parameter<double>("env_scale_factor", 1);
  vel_scale_factor_ = this->declare_parameter<double>("vel_scale_factor", 1);
  pose_timeout_ = this->declare_parameter<double>("pose_timeout", 30.0);

  namespaces_of_robots_ = this->declare_parameter<std::vector<std::string>>(
      "namespaces_of_robots", std::vector<std::string>());
  parameters_.pNumRobots = namespaces_of_robots_.size();
  buffer_size_ = this->declare_parameter<int>("buffer_size", 10);
}

void CoverageControlSimCentralized::CreateStaticTFBroadcaster() {
  static_tf_broadcaster_ =
      std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
  tf_map_idf_static_msg_.header.stamp = this->now();
  tf_map_idf_static_msg_.header.frame_id = "map";
  tf_map_idf_static_msg_.child_frame_id = "idf";
  tf_map_idf_static_msg_.transform = IdentityTransform();
  tf_map_idf_static_msg_.transform.translation.z =
      1.0;  // Set z to 1.0 for visibility

  static_tf_broadcaster_->sendTransform(tf_map_idf_static_msg_);
}

void CoverageControlSimCentralized::CreateServiceServers() {
  system_info_service_ = this->create_service<SystemInfo>(
      "get_system_info",
      [this](const std::shared_ptr<SystemInfo::Request> request,
             std::shared_ptr<SystemInfo::Response> response) {
        RCLCPP_INFO(this->get_logger(), "Incoming request, map_size: %d",
                    request->map_size);
        if (request->map_size != parameters_.pWorldMapSize) {
          response->success = false;
          {
            std::shared_lock lock(idf_file_mutex_);
            response->idf_file = idf_file_;
          }
          RCLCPP_ERROR(this->get_logger(),
                       "World map size does not match with the system");
        } else {
          response->success = true;
          response->velocity_scale_factor = vel_scale_factor_;
          response->namespaces = namespaces_of_robots_;
          {
            std::shared_lock lock(idf_file_mutex_);
            response->idf_file = idf_file_;
          }
          RCLCPP_INFO(this->get_logger(), "System Info sent");
        }
      });

  world_file_service_ = this->create_service<WorldFile>(
      "get_world_file",
      [this](const std::shared_ptr<WorldFile::Request> request,
             std::shared_ptr<WorldFile::Response> response) {
        RCLCPP_INFO(this->get_logger(), "Incoming request for world file: %s",
                    request->name.c_str());
        response->success = true;
        {
          std::shared_lock lock(idf_file_mutex_);
          response->file = idf_file_;
        }
        RCLCPP_INFO(this->get_logger(), "World file sent");
      });

  update_world_file_service_ = this->create_service<UpdateWorldFile>(
      "update_world_file",
      [this](const std::shared_ptr<UpdateWorldFile::Request> request,
             std::shared_ptr<UpdateWorldFile::Response> response) {
        RCLCPP_INFO(this->get_logger(),
                    "Incoming request to update world file: %s",
                    request->file.c_str());
        auto in_idf_file = request->file;
        if (not rcpputils::fs::is_regular_file(in_idf_file)) {
          RCLCPP_ERROR(this->get_logger(), "IDF file %s does not exist",
                       idf_file_.c_str());
          response->success = false;
          response->message = "IDF file does not exist: " + in_idf_file;
          return;
        }
        response->success = true;
        {
          std::unique_lock lock(idf_file_mutex_);
          idf_file_ = in_idf_file;
          this->set_parameter(rclcpp::Parameter("idf_file", idf_file_));
        }
        CreateCoverageControlSystem();
        RCLCPP_INFO(this->get_logger(), "World file updated.");
      });
  RCLCPP_INFO(this->get_logger(), "Created service servers");
}

void CoverageControlSimCentralized::CreateStatusPacSubscriber() {
  status_pac_sub_ = this->create_subscription<Int32>(
      "pac_gcs/status_pac", qos_, [this](Int32::SharedPtr msg) {
        std::unique_lock lock(status_pac_mutex_);
        status_pac_ = msg->data;
      });
  RCLCPP_INFO(this->get_logger(), "Created status pac subscriber");
}

void CoverageControlSimCentralized::WaitForRobotPoses() {
  std::vector<PoseStamped> robot_poses;
  robot_poses.resize(parameters_.pNumRobots);
  std::vector<std::future<bool>> futures;
  futures.reserve(parameters_.pNumRobots);

  RCLCPP_INFO(this->get_logger(),
              "Waiting for robot poses for timeout: %.2f seconds",
              pose_timeout_);
  for (int i = 0; i < parameters_.pNumRobots; ++i) {
    auto fut =
        std::async(std::launch::async, [this, i, &robot_poses]() -> bool {
          return rclcpp::wait_for_message<PoseStamped>(
              robots_[i]->start_pose, robots_[i]->subs.pose,
              this->get_node_options().context(),
              std::chrono::duration<double>(pose_timeout_));
        });
    futures.push_back(std::move(fut));
  }

  int successful_poses = 0;
  for (int i = 0; i < parameters_.pNumRobots; ++i) {
    if (not futures[i].get()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to receive pose for robot %d with ns: %s", i,
                   robots_[i]->ns.c_str());
      robots_[i]->tf_msg.header.stamp = this->now();
      robots_[i]->pubs.tf_broadcaster->sendTransform(robots_[i]->tf_msg);
    } else {
      RCLCPP_INFO(this->get_logger(), "Received pose for robot %d with ns: %s",
                  i, robots_[i]->ns.c_str());
      successful_poses++;
    }
  }

  RCLCPP_INFO(this->get_logger(), "Received %d out of %d robot poses",
              successful_poses, parameters_.pNumRobots);
}

void CoverageControlSimCentralized::CreateTFBroadcasters() {
  for (auto &robot : robots_) {
    robot->pubs.tf_broadcaster =
        std::make_shared<tf2_ros::TransformBroadcaster>(this);
    auto pub_cb = [robot, scale = env_scale_factor_]() -> void {
      Point2 sim_pos = robot->GetWorldPose() * scale;
      robot->SetSimPose(sim_pos);
      robot->tf_msg.header.stamp = rclcpp::Clock().now();
      robot->tf_msg.transform.translation.x = sim_pos[0];
      robot->tf_msg.transform.translation.y = sim_pos[1];
      robot->pubs.tf_broadcaster->sendTransform(robot->tf_msg);
    };
    robot->timers.tf_broadcaster_timer =
        this->create_wall_timer(short_interval_, pub_cb, robot->cbg_reentrant_);
  }
}

void CoverageControlSimCentralized::CreateRobotPoseSubscribers() {
  for (auto robot : robots_) {
    auto cbg_opt = rclcpp::SubscriptionOptions();
    cbg_opt.callback_group = robot->cbg_reentrant_;
    std::string topic_name = "/" + robot->ns + "/pose";
    robot->subs.pose = this->create_subscription<PoseStamped>(
        topic_name, qos_,
        [robot](PoseStamped::SharedPtr msg) {
          robot->SetWorldPose(msg->pose.position.x, msg->pose.position.y);
        },
        cbg_opt);
  }
}

void CoverageControlSimCentralized::UpdateSimRobotPositions() {
  for (auto &robot : robots_) {
    robot->GetSimPose(sim_robot_positions_[robot->id]);
  }
}

void CoverageControlSimCentralized::CreateCoverageControlSystem() {
  std::shared_lock idf_lock(idf_file_mutex_);
  if (idf_file_ != "") {
    // Check if the files exist
    if (rcpputils::fs::is_regular_file(idf_file_)) {
      RCLCPP_INFO(this->get_logger(), "Reading IDF file %s", idf_file_.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "IDF file %s does not exist",
                   idf_file_.c_str());
      rclcpp::shutdown();
    }
    WorldIDF world_idf(parameters_, idf_file_);
    parameters_.pNumGaussianFeatures = world_idf.GetNumFeatures();
    UpdateSimRobotPositions();
    std::unique_lock cc_lock(cc_mutex_);
    coverage_system_ptr_.reset();
    coverage_system_ptr_ = std::make_shared<CoverageSystem>(
        parameters_, world_idf, sim_robot_positions_);
  } else {
    RCLCPP_ERROR(this->get_logger(), "idf_file is empty.");
    rclcpp::shutdown();
  }
  RCLCPP_INFO(this->get_logger(), "Created coverage system using IDF file %s",
              idf_file_.c_str());
}

void CoverageControlSimCentralized::CreateWorldMapServiceServer() {
  world_map_service_ = this->create_service<WorldMap>(
      "get_world_map", [this](const std::shared_ptr<WorldMap::Request> request,
                              std::shared_ptr<WorldMap::Response> response) {
        RCLCPP_INFO(this->get_logger(), "Incoming request\nmap_size: %d",
                    request->map_size);
        int world_size = parameters_.pWorldMapSize;
        response->success = false;
        if (request->map_size != world_size) {
          response->map = zero_world_map_;
          RCLCPP_ERROR(this->get_logger(),
                       "World map size does not match with the system");
        } else if (coverage_system_ptr_ == nullptr) {
          response->map = zero_world_map_;
        } else {
          std::shared_lock lock(cc_mutex_);
          response->map = EigenMatrixRowMajorToFloat32MultiArray(
              coverage_system_ptr_->GetWorldMap());
          response->success = true;
        }
      });
}

void CoverageControlSimCentralized::CreateRobotSimPosPublishers() {
  for (auto robot : robots_) {
    std::string topic_name = robot->ns + "/sim_pose";
    robot->pubs.sim_pose =
        this->create_publisher<PoseStamped>(topic_name, qos_);
    auto pos_cb = [robot]() -> void {
      Point2 robot_pose = robot->GetSimPose();
      auto robot_pos_msg = XYtoPoseStamped(robot_pose[0], robot_pose[1]);
      robot->pubs.sim_pose->publish(robot_pos_msg);
    };
    robot->timers.sim_pose_timer =
        this->create_wall_timer(short_interval_, pos_cb);
  }
}

void CoverageControlSimCentralized::CreateNeighborsPosPublisher() {
  for (auto robot : robots_) {
    std::string topic_name = robot->ns + "/neighbors_pose";
    robot->pubs.neighbors_pose =
        this->create_publisher<PoseArray>(topic_name, qos_);
    auto pub_cb = [this, robot]() -> void {
      if (coverage_system_ptr_ == nullptr) {
        RCLCPP_WARN(this->get_logger(), "Coverage system not initialized");
        return;
      }
      PointVector neighbors_pos;
      {
        std::shared_lock lock(cc_mutex_, std::try_to_lock);
        if (not lock.owns_lock()) {
          return;
        }
        neighbors_pos =
            coverage_system_ptr_->GetRelativePositonsNeighbors(robot->id);
      }
      PoseArray neighbor_pos_msg;
      neighbor_pos_msg.header.frame_id = "map";
      neighbor_pos_msg.header.stamp = this->now();
      for (Point2 pos : neighbors_pos) {
        neighbor_pos_msg.poses.push_back(XYtoPose(pos[0], pos[1]));
      }
      robot->pubs.neighbors_pose->publish(neighbor_pos_msg);
    };
    robot->timers.neighbors_pose_timer =
        this->create_wall_timer(short_interval_, pub_cb, robot->cbg_reentrant_);
  }
}

void CoverageControlSimCentralized::CreateNeighborsIDPublisher() {
  for (auto robot : robots_) {
    std::string topic_name = robot->ns + "/neighbors_id";
    robot->pubs.neighbors_id =
        this->create_publisher<Int32MultiArray>(topic_name, qos_);
    auto pub_cb = [this, robot]() -> void {
      if (coverage_system_ptr_ == nullptr) {
        RCLCPP_WARN(this->get_logger(), "Coverage system not initialized");
        return;
      }
      std::vector<int> neighbors_id;
      {
        std::shared_lock lock(cc_mutex_, std::try_to_lock);
        if (not lock.owns_lock()) {
          return;
        }
        neighbors_id = coverage_system_ptr_->GetNeighborIDs(robot->id);
      }
      Int32MultiArray neighbor_id_msg;
      neighbor_id_msg.layout.dim.push_back(
          std_msgs::msg::MultiArrayDimension());
      neighbor_id_msg.layout.dim[0].size = neighbors_id.size();
      neighbor_id_msg.layout.dim[0].stride = neighbors_id.size();
      neighbor_id_msg.layout.dim[0].label = "neighbors";
      neighbor_id_msg.data = neighbors_id;
      robot->pubs.neighbors_id->publish(neighbor_id_msg);
    };
    robot->timers.neighbors_id_timer =
        this->create_wall_timer(short_interval_, pub_cb, robot->cbg_reentrant_);
  }
}

void CoverageControlSimCentralized::CreateGlobalMapPublisher() {
  std::string topic_name = "global_map";
  global_map_pub_ = this->create_publisher<PointCloud2>(topic_name, qos_);

  auto pub_cb = [this]() -> void {
    if (coverage_system_ptr_ == nullptr) {
      return;
    }
    PointCloud2 global_map_msg;
    {
      std::shared_lock cc_mutex_lock(cc_mutex_, std::try_to_lock);
      if (not cc_mutex_lock.owns_lock()) {
        return;
      }
      MapType global_map = coverage_system_ptr_->GetWorldMap();
      global_map_msg = EigenMatrixRowMajorToPointCloud2(global_map, 2);
    }
    global_map_pub_->publish(global_map_msg);
  };
  global_map_pub_timer_ =
      this->create_wall_timer(vlong_interval, pub_cb, cbg_reentrant_);
}

void CoverageControlSimCentralized::CreateSystemMapPublisher() {
  std::string topic_name = "system_map";
  system_map_pub_ = this->create_publisher<PointCloud2>(topic_name, qos_);
  auto pub_cb = [this]() -> void {
    if (coverage_system_ptr_ == nullptr) {
      return;
    }
    PointCloud2 system_map_msg;
    MapType system_map;
    {
      std::shared_lock cc_mutex_lock(cc_mutex_, std::try_to_lock);
      if (not cc_mutex_lock.owns_lock()) {
        return;
      }
      system_map = coverage_system_ptr_->GetSystemMap();
    }
    system_map_msg = EigenMatrixRowMajorToPointCloud2(system_map, 2);
    system_map_pub_->publish(system_map_msg);
  };
  system_map_pub_timer_ =
      this->create_wall_timer(long_interval_, pub_cb, cbg_reentrant_);
}

void CoverageControlSimCentralized::CreateExploredIDFMapPublisher() {
  std::string topic_name = "global_explored_idf_map";
  global_explored_idf_map_pub_ =
      this->create_publisher<PointCloud2>(topic_name, qos_);
  auto pub_cb = [this]() -> void {
    if (coverage_system_ptr_ == nullptr) {
      RCLCPP_WARN(this->get_logger(), "Coverage system not initialized");
      return;
    }
    MapType global_explored_idf_map;
    PointCloud2 global_explored_idf_map_msg;
    {
      std::shared_lock cc_mutex_lock(cc_mutex_, std::try_to_lock);
      if (not cc_mutex_lock.owns_lock()) {
        return;
      }
      global_explored_idf_map = coverage_system_ptr_->GetSystemExploredIDFMap();
    }
    global_explored_idf_map_msg =
        EigenMatrixRowMajorToPointCloud2(global_explored_idf_map, 2);
    global_explored_idf_map_pub_->publish(global_explored_idf_map_msg);
  };
  global_explored_idf_map_pub_timer_ =
      this->create_wall_timer(system_interval_, pub_cb, cbg_reentrant_);
}

void CoverageControlSimCentralized::CreateCoverageCostPublisher() {
  std::string topic_name = "coverage_cost";
  coverage_cost_pub_ = this->create_publisher<Float32>(topic_name, qos_);
  auto pub_cb = [this]() -> void {
    if (coverage_system_ptr_ == nullptr) {
      return;
    }
    double coverage_cost = 0.0;
    {
      std::shared_lock lock(cc_mutex_, std::try_to_lock);
      if (not lock.owns_lock()) {
        return;
      }
      coverage_cost = coverage_system_ptr_->GetObjectiveValue();
    }
    Float32 coverage_cost_msg;
    coverage_cost_msg.data = static_cast<float>(coverage_cost);
    coverage_cost_pub_->publish(coverage_cost_msg);
  };
  coverage_cost_pub_timer_ =
      this->create_wall_timer(system_interval_, pub_cb, cbg_reentrant_);
}

void CoverageControlSimCentralized::CreateAllRobotsPosesPublisher() {
  robot_poses_pub_ =
      this->create_publisher<PoseArray>("all_robot_sim_poses", qos_);
  auto pub_cb = [this]() -> void {
    robot_poses_msg_.header.stamp = this->now();

    UpdateSimRobotPositions();

    for (int i = 0; i < parameters_.pNumRobots; ++i) {
      robot_poses_msg_.poses[i].position.x = sim_robot_positions_[i][0];
      robot_poses_msg_.poses[i].position.y = sim_robot_positions_[i][1];
    }
    if (coverage_system_ptr_ == nullptr) {
      return;
    }
    {
      std::shared_lock lock(status_pac_mutex_);
      if (status_pac_ == 0) {
        std::unique_lock cc_lock(cc_mutex_);
        if (coverage_system_ptr_ != nullptr) {
          coverage_system_ptr_->SetGlobalRobotPositions(sim_robot_positions_);
        }
      }
    }
    robot_poses_pub_->publish(robot_poses_msg_);
  };
  robot_poses_pub_timer_ = this->create_wall_timer(short_interval_, pub_cb);
}

void CoverageControlSimCentralized::CreateRobotMapPublishers() {
  for (auto robot : robots_) {
    robot->pubs.robot_map =
        this->create_publisher<PointCloud2>(robot->ns + "/map", qos_);
    auto pub_cb = [this, robot]() -> void {
      if (coverage_system_ptr_ == nullptr) {
        RCLCPP_WARN(this->get_logger(), "Coverage system not initialized");
        return;
      }
      MapType robot_map;
      PointCloud2 msg;
      {
        std::shared_lock lock(cc_mutex_, std::try_to_lock);
        if (not lock.owns_lock()) {
          return;
        }
        robot_map = coverage_system_ptr_->GetRobotMap(robot->id);
      }
      msg = EigenMatrixRowMajorToPointCloud2(robot_map, 2);
      robot->pubs.robot_map->publish(msg);
    };
    robot->timers.robot_map_timer = this->create_wall_timer(
        system_interval_, pub_cb, robot->cbg_reentrant_);
  }
}

void CoverageControlSimCentralized::CreateRobotLocalMapPublishers() {
  for (auto robot : robots_) {
    robot->pubs.local_map =
        this->create_publisher<PointCloud2>(robot->ns + "/local_map", qos_);
    auto pub_cb = [this, robot]() -> void {
      if (coverage_system_ptr_ == nullptr) {
        RCLCPP_WARN(this->get_logger(), "Coverage system not initialized");
        return;
      }
      MapType local_map;
      PointCloud2 msg;
      Point2 pos = robot->GetWorldPose();
      {
        std::shared_lock lock(cc_mutex_, std::try_to_lock);
        if (not lock.owns_lock()) {
          return;
        }
        local_map = coverage_system_ptr_->GetRobotLocalMap(robot->id);
      }
      msg = EigenMatrixRowMajorToPointCloud2(local_map, 2, pos[0], pos[1]);
      robot->pubs.local_map->publish(msg);
    };
    robot->timers.local_map_timer = this->create_wall_timer(
        system_interval_, pub_cb, robot->cbg_reentrant_);
  }
}

void CoverageControlSimCentralized::CreateObstacleMapsPublisher() {
  for (auto robot : robots_) {
    robot->pubs.obstacle_map =
        this->create_publisher<PointCloud2>(robot->ns + "/obstacle_map", qos_);
    auto pub_cb = [this, robot]() -> void {
      if (coverage_system_ptr_ == nullptr) {
        RCLCPP_WARN(this->get_logger(), "Coverage system not initialized");
        return;
      }
      MapType obstacle_map;
      PointCloud2 msg;
      Point2 pos = robot->GetWorldPose();
      {
        std::shared_lock lock(cc_mutex_, std::try_to_lock);
        if (not lock.owns_lock()) {
          return;
        }
        obstacle_map = coverage_system_ptr_->GetRobotObstacleMap(robot->id);
      }
      msg = EigenMatrixRowMajorToPointCloud2(obstacle_map, 2, pos[0], pos[1]);
      robot->pubs.obstacle_map->publish(msg);
    };
    robot->timers.obstacle_map_timer = this->create_wall_timer(
        system_interval_, pub_cb, robot->cbg_reentrant_);
  }
}

void CoverageControlSimCentralized::CreateSensorViewPublisher() {
  for (auto robot : robots_) {
    robot->pubs.sensor_view =
        this->create_publisher<PointCloud2>(robot->ns + "/sensor_view", qos_);
    auto pub_cb = [this, robot]() -> void {
      if (coverage_system_ptr_ == nullptr) {
        RCLCPP_WARN(this->get_logger(), "Coverage system not initialized");
        return;
      }
      MapType sensor_view;
      Point2 pos = robot->GetWorldPose();
      PointCloud2 msg;
      {
        std::shared_lock lock(cc_mutex_, std::try_to_lock);
        if (not lock.owns_lock()) {
          return;
        }
        sensor_view = coverage_system_ptr_->GetRobotSensorView(robot->id);
      }
      msg = EigenMatrixRowMajorToPointCloud2(sensor_view, 2, pos[0], pos[1]);
      robot->pubs.sensor_view->publish(msg);
    };
    robot->timers.sensor_view_timer = this->create_wall_timer(
        system_interval_, pub_cb, robot->cbg_reentrant_);
  }
}

}  // namespace CoverageControlSim
