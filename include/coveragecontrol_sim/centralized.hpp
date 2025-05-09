#ifndef COVERAGE_CONTROL_SIM_CENTRALIZED_HPP_
#define COVERAGE_CONTROL_SIM_CENTRALIZED_HPP_

#include <CoverageControl/coverage_system.h>
#include <CoverageControl/parameters.h>
#include <CoverageControl/world_idf.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <ament_index_cpp/get_package_prefix.hpp>
#include <async_pac_gnn_interfaces/srv/world_map.hpp>
#include <async_pac_gnn_interfaces/srv/system_info.hpp>
#include <chrono>
#include <coveragecontrol_sim/utils.hpp>
#include <functional>
#include <future>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <vector>

using namespace std::chrono_literals;

namespace CoverageControlSim {

using namespace CoverageControl;

class CoverageControlSimCentralized : public rclcpp::Node {
 private:
  std::shared_ptr<CoverageSystem> coverage_system_ptr_;
  std::string package_prefix_;
  std::vector<std::string> namespaces_of_robots_;
  Parameters parameters_;
  double time_step_ = 0.1;
  int buffer_size_ = 10;
  double env_scale_factor_;
  double vel_scale_factor_;
  int robot_id_ = 0;
  double pose_timeout_ = 30.0;

  PointVector world_robot_positions_;  // Actual robot positions in the world
  PointVector sim_robot_positions_;  // Scaled robot positions in the simulation

  std::string params_file_, idf_file_;

  rmw_qos_profile_t qos_profile_sensor_data_ = rmw_qos_profile_sensor_data;
  rclcpp::QoS qos_ =
      rclcpp::QoS(rclcpp::QoSInitialization(qos_profile_sensor_data_.history,
                                            qos_profile_sensor_data_.depth),
                  qos_profile_sensor_data_);
  std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr>
      world_pos_subs_;
  rclcpp::CallbackGroup::SharedPtr cbg_world_pos_sub_;

  std::vector<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr>
      robot_pos_pubs_;
  std::vector<rclcpp::TimerBase::SharedPtr> robot_pos_pub_timers_;

  std::vector<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr>
      robot_sim_pos_pubs_;  // Scaled simulation robot positions
  std::vector<rclcpp::TimerBase::SharedPtr> robot_sim_pos_pub_timers_;

  // Create a publisher for poses of all robots as pose array message
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr robot_poses_pub_;
  rclcpp::TimerBase::SharedPtr robot_poses_pub_timer_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

  std::vector<rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr>
      robot_local_map_pubs_;
  std::vector<rclcpp::TimerBase::SharedPtr> robot_local_map_pub_timers_;

  std::vector<rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr>
      robot_map_pubs_;
  std::vector<rclcpp::TimerBase::SharedPtr> robot_map_pub_timers_;

  // Obstacle maps publisher
  std::vector<rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr>
      robot_obstacle_map_pubs_;
  std::vector<rclcpp::TimerBase::SharedPtr> robot_obstacle_map_pub_timers_;

  // Publish sensor view for each robot
  std::vector<rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr>
      robot_sensor_view_pubs_;
  std::vector<rclcpp::TimerBase::SharedPtr> robot_sensor_view_pub_timers_;

  // Publishers for relative neighbor positions for each robot
  std::vector<rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr>
      robot_neighbors_pose_pubs_;
  std::vector<rclcpp::TimerBase::SharedPtr> robot_neighbors_pose_pub_timers_;

  // Publishers for relative neighbor ids for each robot
  std::vector<rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr>
      robot_neighbors_id_pubs_;
  std::vector<rclcpp::TimerBase::SharedPtr> robot_neighbors_id_pub_timers_;

  // Publisher for global map
  // rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr
  //     global_map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_map_pub_;
  rclcpp::TimerBase::SharedPtr global_map_pub_timer_;

  // Publisher for system map
  // rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr
  //     system_map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr system_map_pub_;
  rclcpp::TimerBase::SharedPtr system_map_pub_timer_;

  // Publisher for global explored idf map
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr
      global_explored_idf_map_pub_;
  rclcpp::TimerBase::SharedPtr global_explored_idf_map_pub_timer_;

  // Publisher for coverage cost
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr
      coverage_cost_pub_;
  rclcpp::TimerBase::SharedPtr coverage_cost_pub_timer_;

  // Subscribe to robot positions for each robot
  std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr>
      cmd_pos_subs_;
  rclcpp::CallbackGroup::SharedPtr cbg_cmd_pos_sub_;

  // Subscribe to robot positions for each robot
  std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr>
      cmd_global_pos_subs_;
  rclcpp::CallbackGroup::SharedPtr cbg_cmd_global_pos_sub_;

  // Subscribe to velocity commands for each robot
  std::vector<rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr>
      cmd_vel_subs_;
  rclcpp::CallbackGroup::SharedPtr cbg_cmd_vel_sub_;

  // Subscribe to robot positions for each robot
  std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr>
      robot_pos_subs_;
  rclcpp::CallbackGroup::SharedPtr cbg_robot_pos_subs_;

  rclcpp::CallbackGroup::SharedPtr cbg_cmd_vel_pub_;
  std::vector<rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr>
      cmd_vel_pubs_;
  // timers
  std::vector<rclcpp::TimerBase::SharedPtr> cmd_vel_pub_timers_;
  rclcpp::TimerBase::SharedPtr cmd_vel_pub_timer_;
  geometry_msgs::msg::TransformStamped tf_map_msg_;

 public:
  CoverageControlSimCentralized()
      : Node("sim_centralized"),
        coverage_system_ptr_(nullptr),
        package_prefix_(
            ament_index_cpp::get_package_prefix("coveragecontrol_sim")) {
    params_file_ = this->declare_parameter<std::string>("params_file");
    parameters_ = Parameters(params_file_);
    idf_file_ = this->declare_parameter<std::string>("idf_file");
    env_scale_factor_ = this->declare_parameter<double>("env_scale_factor", 1);
    vel_scale_factor_ = this->declare_parameter<double>("vel_scale_factor", 1);
    pose_timeout_ = this->declare_parameter<double>("pose_timeout", 30.0);

    namespaces_of_robots_ = this->declare_parameter<std::vector<std::string>>(
        "namespaces_of_robots", std::vector<std::string>());
    if (namespaces_of_robots_.size() !=
        static_cast<size_t>(parameters_.pNumRobots)) {
      RCLCPP_WARN(this->get_logger(),
                  "Number of robot namespaces does not match number of "
                  "robots in coveragecontrol.toml. This is not a problem. FYI: "
                  "%ld vs %d",
                  namespaces_of_robots_.size(), parameters_.pNumRobots);
      RCLCPP_WARN(this->get_logger(),
                  "Forcing number of robots to number of items in "
                  "namespaces_of_robots: %ld",
                  namespaces_of_robots_.size());
      parameters_.pNumRobots = namespaces_of_robots_.size();
    }

    buffer_size_ = this->declare_parameter<int>("buffer_size", 10);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    static_tf_broadcaster_ =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    tf_map_msg_.header.frame_id = "map";
    tf_map_msg_.child_frame_id = "";
    tf_map_msg_.transform.translation.x = 0.0;
    tf_map_msg_.transform.translation.y = 0.0;
    tf_map_msg_.transform.translation.z = 1.0;
    tf_map_msg_.transform.rotation.x = 0.0;
    tf_map_msg_.transform.rotation.y = 0.0;
    tf_map_msg_.transform.rotation.z = 0.0;
    tf_map_msg_.transform.rotation.w = 1.0;

    geometry_msgs::msg::TransformStamped transform_idf = tf_map_msg_;
    transform_idf.header.stamp = this->now();
    transform_idf.child_frame_id = "idf";
    static_tf_broadcaster_->sendTransform(transform_idf);

    world_robot_positions_.resize(parameters_.pNumRobots, Point2(0, 0));
    sim_robot_positions_.resize(parameters_.pNumRobots, Point2(0, 0));

    cbg_world_pos_sub_ =
        this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    auto cbg_world_pos_sub_opt = rclcpp::SubscriptionOptions();
    cbg_world_pos_sub_opt.callback_group = cbg_world_pos_sub_;

    for (int i = 0; i < parameters_.pNumRobots; ++i) {
      std::string ns = namespaces_of_robots_[i];
      std::string topic_name = "/" + ns + "/pose";
      auto subs = this->create_subscription<geometry_msgs::msg::PoseStamped>(
          topic_name, qos_,
          [this, robot_id=i, ns](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            world_robot_positions_[robot_id] =
                Point2(msg->pose.position.x, msg->pose.position.y);
            double sim_x = msg->pose.position.x * env_scale_factor_;
            double sim_y = msg->pose.position.y * env_scale_factor_;
            sim_robot_positions_[robot_id] = Point2(sim_x, sim_y);
            geometry_msgs::msg::TransformStamped tf_msg = tf_map_msg_;
            tf_msg.header.stamp = msg->header.stamp;
            tf_msg.child_frame_id = ns;
            tf_msg.transform.translation.x = sim_x;
            tf_msg.transform.translation.y = sim_y;
            tf_broadcaster_->sendTransform(tf_msg);
          });
      world_pos_subs_.push_back(subs);
    }

    std::vector<geometry_msgs::msg::PoseStamped> robot_poses;
    robot_poses.resize(parameters_.pNumRobots);
    std::vector<std::future<bool>> futures;
    futures.reserve(parameters_.pNumRobots);

    RCLCPP_INFO(this->get_logger(),
                "Waiting for robot poses for timeout: %.2f seconds",
                pose_timeout_);
    for (int i = 0; i < parameters_.pNumRobots; ++i) {
      auto fut =
          std::async(std::launch::async, [this, i, &robot_poses]() -> bool {
            return rclcpp::wait_for_message<geometry_msgs::msg::PoseStamped>(
                robot_poses[i], world_pos_subs_[i],
                this->get_node_options().context(),
                std::chrono::duration<double>(pose_timeout_));
          });
      futures.push_back(std::move(fut));
    }

    for (int i = 0; i < parameters_.pNumRobots; ++i) {
      futures[i].wait();
    }
    for (int i = 0; i < parameters_.pNumRobots; ++i) {
      if (not futures[i].get()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to receive pose for robot %d with ns: %s", i + 1,
                     namespaces_of_robots_[i].c_str());
        geometry_msgs::msg::TransformStamped tf_msg = tf_map_msg_;
        tf_msg.header.stamp = this->now();
        tf_msg.child_frame_id = namespaces_of_robots_[i];
        tf_broadcaster_->sendTransform(tf_msg);
      } else {
        RCLCPP_INFO(this->get_logger(),
                    "Received pose for robot %d with ns: %s", i,
                    namespaces_of_robots_[i].c_str());
      }
    }
    RCLCPP_INFO(this->get_logger(), "Received all robot poses");

    // std::vector<int> received_pos(parameters_.pNumRobots, 0);
    // cbg_world_pos_sub_ = this->create_callback_group(
    //     rclcpp::CallbackGroupType::Reentrant);
    // auto cbg_world_pos_sub_opt = rclcpp::SubscriptionOptions();
    // cbg_world_pos_sub_opt.callback_group = cbg_world_pos_sub_;
    // for (int i = 0; i < parameters_.pNumRobots; ++i) {
    //   world_pos_subs_.push_back(
    //       this->create_subscription<geometry_msgs::msg::PoseStamped>(
    //           "/" + namespaces_of_robots_[i] + "/pose",
    //           qos_,
    //           [this, i,
    //            &received_pos](geometry_msgs::msg::PoseStamped::SharedPtr msg)
    //            {
    //             world_robot_positions_[i] =
    //                 Point2(msg->pose.position.x, msg->pose.position.y);
    //             if (coverage_system_ptr_ == nullptr) {
    //               received_pos[i] = 1;
    //             }
    //           },
    //           cbg_world_pos_sub_opt));
    // }

    // int num_received_pos = 0;
    // // Get current ROS time
    // auto start_time = this->now();
    // auto elapsed_time = this->now() - start_time;
    // while (
    //     (num_received_pos < static_cast<int>(parameters_.pNumRobots)) &&
    //     elapsed_time.seconds() < pose_timeout_ &&
    //     rclcpp::ok()) {
    //   RCLCPP_INFO(this->get_logger(),
    //               "Waiting for robot positions, received %d of %d. Elapsed
    //               time: %.2f of %.2f seconds", num_received_pos,
    //               parameters_.pNumRobots, elapsed_time.seconds(),
    //               pose_timeout_);
    //   num_received_pos = std::accumulate(received_pos.begin(),
    //                                      received_pos.end(), 0);
    //   rclcpp::sleep_for(500ms);
    //   rclcpp::spin_some(this->get_node_base_interface());
    //   elapsed_time = this->now() - start_time;
    // }

    if (rcpputils::fs::exists(idf_file_)) {
      RCLCPP_INFO(this->get_logger(), "Reading IDF file %s", idf_file_.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "IDF file %s does not exist",
                   idf_file_.c_str());
      rclcpp::shutdown();
    }
    WorldIDF world_idf(parameters_, idf_file_);
    parameters_.pNumGaussianFeatures = world_idf.GetNumFeatures();
    coverage_system_ptr_ = std::make_shared<CoverageSystem>(
        parameters_, world_idf, sim_robot_positions_);

    CreateServiceServers();

    // CreateRobotPosPublishers();

    // CreateRobotSimPosPublishers();

    // CreateRobotMapPublishers();
    /* CreateRobotLocalMapPublishers(); */
    /* CreateObstacleMapsPublisher(); */
    CreateSystemMapPublisher();
    CreateGlobalMapPublisher();
    /* CreateExploredIDFMapPublisher(); */
    // CreateSensorViewPublisher();
    RCLCPP_INFO(this->get_logger(), "Created map publishers");

    // CreateNeigborsPosPublisher();
    // CreateNeigborsIDPublisher();
    RCLCPP_INFO(this->get_logger(), "Created neighbors publishers");

    CreateAllRobotsPosesPublisher();

    CreateCoverageCostPublisher();
    RCLCPP_INFO(this->get_logger(), "Created cost publisher");

    /* CreateCmdVelPublisher(); */
    RCLCPP_INFO(this->get_logger(), "Constructor finished");
  }

  MapType GetWorldMap() { return coverage_system_ptr_->GetWorldMap(); }
  Parameters const &GetParameters() { return parameters_; }

 private:
  rclcpp::Service<async_pac_gnn_interfaces::srv::WorldMap>::SharedPtr
      world_map_service_;

  rclcpp::Service<async_pac_gnn_interfaces::srv::SystemInfo>::SharedPtr
      system_info_service_;

  void CreateServiceServers();

  void CreateSimCentralizedSetup();

  void CreateCmdSubscribers();

  void CreateAllRobotsPosesPublisher();

  void CreateRobotPosPublishers();
  void CreateRobotSimPosPublishers();

  void CreateSystemMapPublisher();
  void CreateGlobalMapPublisher();
  void CreateExploredIDFMapPublisher();
  void CreateCoverageCostPublisher();

  void CreateRobotMapPublishers();
  void CreateRobotLocalMapPublishers();
  void CreateObstacleMapsPublisher();
  void CreateSensorViewPublisher();

  void CreateNeigborsPosPublisher();
  void CreateNeigborsIDPublisher();

  void CreateCmdVelPublisher();  // Deprecated
};
}  // namespace CoverageControlSim
#endif  // COVERAGE_CONTROL_SIM_CENTRALIZED_HPP_
