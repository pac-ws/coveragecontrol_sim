#ifndef COVERAGE_CONTROL_SIM_CENTRALIZED_HPP_
#define COVERAGE_CONTROL_SIM_CENTRALIZED_HPP_

#include <CoverageControl/coverage_system.h>
#include <CoverageControl/parameters.h>
#include <CoverageControl/world_idf.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <ament_index_cpp/get_package_prefix.hpp>
#include <async_pac_gnn_interfaces/srv/system_info.hpp>
#include <async_pac_gnn_interfaces/srv/update_world_file.hpp>
#include <async_pac_gnn_interfaces/srv/world_file.hpp>
#include <async_pac_gnn_interfaces/srv/world_map.hpp>
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
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <vector>

using namespace std::chrono_literals;

namespace CoverageControlSim {

using namespace CoverageControl;
using UpdateWorldFile = async_pac_gnn_interfaces::srv::UpdateWorldFile;
using WorldMap = async_pac_gnn_interfaces::srv::WorldMap;
using SystemInfo = async_pac_gnn_interfaces::srv::SystemInfo;
using WorldFile = async_pac_gnn_interfaces::srv::WorldFile;

using PoseStamped = geometry_msgs::msg::PoseStamped;
using PoseArray = geometry_msgs::msg::PoseArray;
using TwistStamped = geometry_msgs::msg::TwistStamped;

using Int32 = std_msgs::msg::Int32;
using Float32 = std_msgs::msg::Float32;
using Float32MultiArray = std_msgs::msg::Float32MultiArray;
using Int32MultiArray = std_msgs::msg::Int32MultiArray;

using PointCloud2 = sensor_msgs::msg::PointCloud2;

struct RobotSubs {
  rclcpp::Subscription<PoseStamped>::SharedPtr pose;
};

struct RobotPubs {
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
  rclcpp::Publisher<PoseStamped>::SharedPtr sim_pose;

  rclcpp::Publisher<PointCloud2>::SharedPtr robot_map;
  rclcpp::Publisher<PointCloud2>::SharedPtr local_map;
  rclcpp::Publisher<PointCloud2>::SharedPtr obstacle_map;
  rclcpp::Publisher<PointCloud2>::SharedPtr sensor_view;

  rclcpp::Publisher<PoseArray>::SharedPtr neighbors_pose;
  rclcpp::Publisher<Int32MultiArray>::SharedPtr neighbors_id;
};

struct RobotTimers {
  rclcpp::TimerBase::SharedPtr sim_pose_timer;
  rclcpp::TimerBase::SharedPtr tf_broadcaster_timer;

  rclcpp::TimerBase::SharedPtr robot_map_timer;
  rclcpp::TimerBase::SharedPtr local_map_timer;
  rclcpp::TimerBase::SharedPtr obstacle_map_timer;
  rclcpp::TimerBase::SharedPtr sensor_view_timer;

  rclcpp::TimerBase::SharedPtr neighbors_pose_timer;
  rclcpp::TimerBase::SharedPtr neighbors_id_timer;
};

struct Robot {
  int id;
  std::string ns;
  Point2 sim_pose;
  Point2 world_pose;
  PoseStamped start_pose;
  geometry_msgs::msg::TransformStamped tf_msg;
  rclcpp::Time last_pose_time;
  std::shared_mutex sim_pose_mutex;
  std::shared_mutex world_pose_mutex;
  RobotSubs subs;
  RobotPubs pubs;
  RobotTimers timers;
  rclcpp::CallbackGroup::SharedPtr cbg_reentrant_;
  Robot(int id, std::string ns)
      : id(id), ns(ns), sim_pose(Point2::Zero()), world_pose(Point2::Zero()) {
    tf_msg.header.frame_id = "map";
    tf_msg.child_frame_id = ns;
    tf_msg.transform = IdentityTransform();
    tf_msg.transform.translation.z = 1.0;  // Set z to 1.0 for visibility
  }

  void GetWorldPose(Point2 &pose) {
    std::shared_lock lock(world_pose_mutex);
    pose = world_pose;
  }
  Point2 GetWorldPose() {
    std::shared_lock lock(world_pose_mutex);
    return world_pose;
  }
  void GetSimPose(Point2 &pose) {
    std::shared_lock lock(sim_pose_mutex);
    pose = sim_pose;
  }
  Point2 GetSimPose() {
    std::shared_lock lock(sim_pose_mutex);
    return sim_pose;
  }
  void SetSimPose(Point2 const &pose) {
    std::unique_lock lock(sim_pose_mutex);
    sim_pose = pose;
  }
  void SetWorldPose(double const &x, double const &y) {
    std::unique_lock lock(world_pose_mutex);
    world_pose[0] = x;
    world_pose[1] = y;
  }
};

class CoverageControlSimCentralized : public rclcpp::Node {
 private:
  std::shared_ptr<CoverageSystem> coverage_system_ptr_;
  std::shared_mutex cc_mutex_;
  std::shared_mutex status_pac_mutex_;
  std::shared_mutex idf_file_mutex_;
  std::string package_prefix_;
  std::vector<std::string> namespaces_of_robots_;
  PointVector sim_robot_positions_;
  Parameters parameters_;
  int status_pac_ = 0;
  double time_step_ = 0.1;
  int buffer_size_ = 10;
  double env_scale_factor_;
  double vel_scale_factor_;
  double pose_timeout_ = 30.0;

  PoseArray robot_poses_msg_;
  geometry_msgs::msg::TransformStamped tf_map_idf_static_msg_;

  std::chrono::milliseconds short_interval_{30};
  std::chrono::milliseconds long_interval_{100};
  std::chrono::milliseconds system_interval_{200};
  std::chrono::milliseconds vlong_interval{2000};

  std::vector<std::shared_ptr<Robot>> robots_;

  std::string params_file_, idf_file_;

  Float32MultiArray zero_world_map_;  // Zero world map for service responses

 public:
  CoverageControlSimCentralized()
      : Node("sim_centralized"),
        coverage_system_ptr_(nullptr),
        package_prefix_(
            ament_index_cpp::get_package_prefix("coveragecontrol_sim")) {
    InitializeParameters();

    robot_poses_msg_.header.frame_id = "map";
    robot_poses_msg_.header.stamp = this->now();
    robot_poses_msg_.poses.reserve(parameters_.pNumRobots);
    for (int i = 0; i < parameters_.pNumRobots; ++i) {
      robot_poses_msg_.poses.push_back(XYtoPose(0.0, 0.0));
    }
    sim_robot_positions_ = PointVector(parameters_.pNumRobots, Point2::Zero());
    zero_world_map_ = ZeroSquareMap(parameters_.pWorldMapSize);
    cbg_reentrant_ =
        this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    cbg_service_ =
        this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    for (int i = 0; i < parameters_.pNumRobots; ++i) {
      robots_.emplace_back(
          std::make_shared<Robot>(i, namespaces_of_robots_[i]));
      robots_.back()->cbg_reentrant_ =
          this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    }

    system_interval_ = std::chrono::milliseconds(
        static_cast<int>(parameters_.pTimeStep * 1000));

    CreateStaticTFBroadcaster();
    CreateServiceServers();

    CreateStatusPacSubscriber();

    CreateRobotPoseSubscribers();
    WaitForRobotPoses();
    CreateTFBroadcasters();
    CreateAllRobotsPosesPublisher();

    CreateCoverageControlSystem();
    /* CreateWorldMapServiceServer(); */

    /* CreateRobotSimPosPublishers(); */

    /* CreateRobotMapPublishers(); */
    /* CreateRobotLocalMapPublishers(); */
    /* CreateObstacleMapsPublisher(); */

    CreateSystemMapPublisher();
    CreateGlobalMapPublisher();
    /* CreateExploredIDFMapPublisher(); */
    /* CreateSensorViewPublisher(); */

    /* CreateNeighborsPosPublisher(); */
    /* CreateNeighborsIDPublisher(); */

    /* CreateCoverageCostPublisher(); */
  }

  Parameters const &GetParameters() { return parameters_; }

 private:

  rmw_qos_profile_t qos_profile_sensor_data_ = rmw_qos_profile_sensor_data;
  rclcpp::QoS qos_ =
      rclcpp::QoS(rclcpp::QoSInitialization(qos_profile_sensor_data_.history,
                                            qos_profile_sensor_data_.depth),
                  qos_profile_sensor_data_);

  rclcpp::CallbackGroup::SharedPtr cbg_reentrant_;
  rclcpp::CallbackGroup::SharedPtr cbg_service_;

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

  rclcpp::Subscription<Int32>::SharedPtr status_pac_sub_;

  // Publisher for robot poses
  rclcpp::Publisher<PoseArray>::SharedPtr robot_poses_pub_;
  rclcpp::TimerBase::SharedPtr robot_poses_pub_timer_;

  // Publisher for global map
  rclcpp::Publisher<PointCloud2>::SharedPtr global_map_pub_;
  rclcpp::TimerBase::SharedPtr global_map_pub_timer_;

  // Publisher for system map
  rclcpp::Publisher<PointCloud2>::SharedPtr system_map_pub_;
  rclcpp::TimerBase::SharedPtr system_map_pub_timer_;

  // Publisher for global explored idf map
  rclcpp::Publisher<PointCloud2>::SharedPtr global_explored_idf_map_pub_;
  rclcpp::TimerBase::SharedPtr global_explored_idf_map_pub_timer_;

  // Publisher for coverage cost
  rclcpp::Publisher<Float32>::SharedPtr coverage_cost_pub_;
  rclcpp::TimerBase::SharedPtr coverage_cost_pub_timer_;

  rclcpp::Service<WorldMap>::SharedPtr world_map_service_;

  rclcpp::Service<SystemInfo>::SharedPtr system_info_service_;

  rclcpp::Service<WorldFile>::SharedPtr world_file_service_;

  rclcpp::Service<UpdateWorldFile>::SharedPtr update_world_file_service_;

  void InitializeParameters();
  void CreateStaticTFBroadcaster();
  void CreateTFBroadcasters();
  void UpdateWorldFileCallback(
      const std::shared_ptr<UpdateWorldFile::Request> request,
      std::shared_ptr<UpdateWorldFile::Response> response);
  void CreateWorldMapServiceServer();
  void CreateServiceServers();
  void CreateStatusPacSubscriber();
  void CreateRobotPoseSubscribers();
  void WaitForRobotPoses();

  void CreateCoverageControlSystem();
  void UpdateSimRobotPositions();

  void CreateAllRobotsPosesPublisher();

  void CreateRobotSimPosPublishers();

  void CreateSystemMapPublisher();
  void CreateGlobalMapPublisher();
  void CreateExploredIDFMapPublisher();
  void CreateCoverageCostPublisher();

  void CreateRobotMapPublishers();
  void CreateRobotLocalMapPublishers();
  void CreateObstacleMapsPublisher();
  void CreateSensorViewPublisher();

  void CreateNeighborsPosPublisher();
  void CreateNeighborsIDPublisher();
};
}  // namespace CoverageControlSim
#endif  // COVERAGE_CONTROL_SIM_CENTRALIZED_HPP_
