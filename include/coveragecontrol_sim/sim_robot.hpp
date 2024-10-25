#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>

#include <CoverageControl/constants.h>
#include <CoverageControl/parameters.h>
#include <CoverageControl/typedefs.h>
#include <CoverageControl/bivariate_normal_distribution.h>
#include <CoverageControl/world_idf.h>
#include <CoverageControl/coverage_system.h>

#include <CoverageControl/algorithms/lloyd_global_online.h>
#include <CoverageControl/algorithms/simul_explore_exploit.h>
#include <CoverageControl/algorithms/lloyd_local_voronoi.h>
#include <CoverageControl/algorithms/oracle_global_offline.h>

using namespace std::chrono_literals;
using namespace CoverageControl;

/* typedef LloydLocalVoronoi CoverageAlgorithm; */
typedef LloydGlobalOnline CoverageAlgorithm;

class CoverageControlSim : public rclcpp::Node {

	private:

		std::shared_ptr<CoverageSystem> coverage_system_ptr_;
		std::string package_prefix_;
		Parameters parameters_;
		float time_step_ = 0.1;
		int buffer_size_ = 10;
		std::string mode_;
		float scale_factor_;
		int robot_id_ = 0;
		
		PointVector robot_positions_;
		std::string robot_namespace_prefix_;
		std::string node_namespace_prefix_ = "cc_sim";
		std::string env_file_, pos_file_, idf_file_;

		std::unordered_set<int> ready_for_pac_;
		std::unordered_set<int> received_robot_pos_;
	
		rclcpp::CallbackGroup::SharedPtr cbg_robot_positions_pub_;
		std::vector<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> robot_pos_pubs_;
		// timers
		std::vector<rclcpp::TimerBase::SharedPtr> robot_pos_pub_timers_;

		// Create a publisher for poses of all robots as pose array message
		rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr robot_poses_pub_;
		rclcpp::TimerBase::SharedPtr robot_poses_pub_timer_;

		std::vector<rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr> robot_local_map_pubs_;
		std::vector<rclcpp::TimerBase::SharedPtr> robot_local_map_pub_timers_;

		// Obstacle maps publisher
		std::vector<rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr> robot_obstacle_map_pubs_;
		std::vector<rclcpp::TimerBase::SharedPtr> robot_obstacle_map_pub_timers_;

		// Publishers for relative neighbor positions for each robot
		std::vector<rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr> robot_neighbor_pos_pubs_;
		std::vector<rclcpp::TimerBase::SharedPtr> robot_neighbor_pos_pub_timers_;

		// Publishers for relative neighbor ids for each robot
		std::vector<rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr> robot_neighbor_id_pubs_;
		std::vector<rclcpp::TimerBase::SharedPtr> robot_neighbor_id_pub_timers_;

		// Publish sensor view for each robot
		std::vector<rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr> robot_sensor_view_pubs_;
		std::vector<rclcpp::TimerBase::SharedPtr> robot_sensor_view_pub_timers_;

		// Publisher for global system map
		rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr global_map_pub_;
		rclcpp::TimerBase::SharedPtr global_map_pub_timer_;

		// Subscribe to velocity commands for each robot
		std::vector<rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr> cmd_vel_subs_;
		rclcpp::CallbackGroup::SharedPtr cbg_cmd_vel_sub_;

		// Subscribe to robot positions for each robot
		std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> cmd_pos_subs_;
		rclcpp::CallbackGroup::SharedPtr cbg_cmd_pos_sub_;

		// Subscribe to robot positions for each robot
		std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> cmd_global_pos_subs_;
		rclcpp::CallbackGroup::SharedPtr cbg_cmd_global_pos_sub_;
		std::shared_ptr<CoverageAlgorithm> controller_;
		//
		// Subscribe to robot positions for each robot
		std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> robot_pos_subs_;
		rclcpp::CallbackGroup::SharedPtr cbg_robot_pos_subs_;

		rclcpp::CallbackGroup::SharedPtr cbg_cmd_vel_pub_;
		std::vector<rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr> cmd_vel_pubs_;
		// timers
		std::vector<rclcpp::TimerBase::SharedPtr> cmd_vel_pub_timers_;

		//Subscribe to 'ready_for_pac_' topic for each robot
		/* rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ready_for_pac_sub_; */
		std::vector<rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr> ready_for_pac_subs_;
		rclcpp::CallbackGroup::SharedPtr cbg_ready_for_pac_subs_;

	public:

		CoverageControlSim() : Node("coveragecontrol_sim"), coverage_system_ptr_(nullptr) {

			package_prefix_ = ament_index_cpp::get_package_prefix("coveragecontrol_sim");
			env_file_ = this->declare_parameter<std::string>("env_file", package_prefix_ + "/config/env_params.yaml");
			parameters_ = Parameters(env_file_);
			pos_file_ = this->declare_parameter<std::string>("pos_file", package_prefix_ + "/config/sample.pos");
			idf_file_ = this->declare_parameter<std::string>("idf_file", package_prefix_ + "/config/sample.idf");
			mode_ = this->declare_parameter<std::string>("mode", "sim");
			scale_factor_ = this->declare_parameter<float>("scale_factor", 100);

			robot_namespace_prefix_ = this->declare_parameter<std::string>("robot_namespace_prefix", "robot_");
			/* RCLCPP_INFO(this->get_logger(), "Robot namespace prefix: %s", robot_namespace_prefix_.c_str()); */

			/* time_step_ = this->declare_parameter<float>("time_step", 0.1); */
			/* std::chrono::milliseconds timeout = std::chrono::milliseconds(int(time_step_ * 1000)); */
			buffer_size_ = this->declare_parameter<int>("buffer_size", 10);

			robot_positions_.resize(parameters_.pNumRobots, Point2(0, 0));
			/* auto cbg_robot_pos_pub_opt = rclcpp::PublisherOptions(); */
			/* cbg_robot_pos_pub_opt.callback_group = cbg_robot_positions_pub_; */

			// Make callback group for cmd_pos_subs_ that are mutually exclusive

			if(mode_ == "robot") {
				CreateRobotPosSubscribers();
				CreateRobotSetup();
				robot_id_ = this->declare_parameter<int>("robot_id", 0);
			}

			CreateRobotPosPublishers();
			CreateRobotLocalMapPublishers();
			CreateObstacleMapsPublisher();
			CreateSystemMapPublisher();
			CreateNeigborPosPublisher();
			CreateNeigborIDPublisher();
			CreateSensorViewPublisher();
			CreateRobotPosesPublisher();
			CreateCmdVelPublisher();

		}

		void CreateReadyForPACSubscribers() {
			cbg_ready_for_pac_subs_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
			auto cbg_ready_for_pac_opts = rclcpp::SubscriptionOptions();
			cbg_ready_for_pac_opts.callback_group = cbg_ready_for_pac_subs_;
			for (int robot_id = 0; robot_id < parameters_.pNumRobots; ++robot_id) {
				std::string topic_name = robot_namespace_prefix_  + "/ready_for_pac";
				ready_for_pac_subs_.push_back(this->create_subscription<std_msgs::msg::Bool>(topic_name, rclcpp::QoS(buffer_size_), [this, robot_id](const std_msgs::msg::Bool::ConstSharedPtr msg) -> void {
							if(msg->data) {
							ready_for_pac_.insert(robot_id);
							}
							}, cbg_ready_for_pac_opts));
			}
		}

		void CreateCmdVelPublisher() {
			for (int robot_id = 0; robot_id < parameters_.pNumRobots; robot_id++) {
				std::string topic_name = robot_namespace_prefix_ + "/sim/cmd_vel";
				cmd_vel_pubs_.push_back(this->create_publisher<geometry_msgs::msg::TwistStamped>(topic_name, buffer_size_));
				auto cmd_vel_pub = cmd_vel_pubs_[robot_id];
				// Create timer to publish robot position
				auto cmd_vel_pub_timer_callback = [this, cmd_vel_pub, robot_id]() -> void {
					controller_->Step();
					auto actions = controller_->GetActions();
					std::cout << "actions: " << actions[robot_id][0] << " " << actions[robot_id][1] << std::endl;
					/* std::cout << "cmd_vel for " << robot_id << std::endl; */
					/* Point2 dist = actions[robot_id] * parameters_.pTimeStep / scale_factor_; */
					/* Point2 current_pos = coverage_system_ptr_->GetRobotPosition(robot_id); */
					/* Point2 goal_pos = current_pos + dist; */
					geometry_msgs::msg::TwistStamped cmd_vel_msg;
					cmd_vel_msg.header.stamp = this->get_clock()->now();
					/* cmd_vel_msg.linear.x = goal_pos[0]; */
					/* cmd_vel_msg.linear.y = goal_pos[1]; */
					cmd_vel_msg.twist.linear.x = actions[robot_id][0]/(scale_factor_);
					cmd_vel_msg.twist.linear.y = actions[robot_id][1]/(scale_factor_);
					std::cout << "Linear velocities (" << robot_namespace_prefix_ << "/"<< parameters_.pNumRobots << "): " << cmd_vel_msg.twist.linear.x << " " << cmd_vel_msg.twist.linear.y<< std::endl;
					cmd_vel_pub->publish(cmd_vel_msg);
					if(cmd_vel_msg.twist.linear.x < 0.05) { cmd_vel_msg.twist.linear.x = 0; }
					if(cmd_vel_msg.twist.linear.y < 0.05) { cmd_vel_msg.twist.linear.y = 0; }
					std::cout << "Linear velocities (" << robot_namespace_prefix_ << "/"<< parameters_.pNumRobots << "): " << cmd_vel_msg.twist.linear.x << " " << cmd_vel_msg.twist.linear.y<< std::endl;
					cmd_vel_pub->publish(cmd_vel_msg);
				};
				cmd_vel_pub_timers_.push_back(this->create_wall_timer(100ms, cmd_vel_pub_timer_callback));
			}
		}

		void CreateRobotPosSubscribers() {
			cbg_robot_pos_subs_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
			auto cbg_cmd_pos_opts = rclcpp::SubscriptionOptions();
			cbg_cmd_pos_opts.callback_group = cbg_robot_pos_subs_;

			for (int robot_id = 0; robot_id < parameters_.pNumRobots; ++robot_id) {
				std::string topic_name;
				/* topic_name = "/robot_" + std::to_string(robot_id) + "/sim/pose"; */
				/* topic_name = robot_namespace_prefix_ + "/pose"; */
				topic_name = robot_namespace_prefix_ + "/mavros/local_position/pose";
				auto sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(topic_name, rclcpp::QoS(buffer_size_), [this, robot_id](const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) -> void {
						Point2 scaled_robot_pos(msg->pose.position.x, msg->pose.position.y);
						scaled_robot_pos *= scale_factor_;
						scaled_robot_pos += Point2(robot_id + 1, robot_id+1);
						if(coverage_system_ptr_ == nullptr) {
						/* init_pos_sub_count_++; */
						robot_positions_[robot_id] = scaled_robot_pos;
						received_robot_pos_.insert(robot_id);
						}
						else{
						std::cout << "scaled_robot_pos: " << scaled_robot_pos[0] << " " << scaled_robot_pos[1] << std::endl;
						coverage_system_ptr_->SetGlobalRobotPosition(robot_id, scaled_robot_pos);
						}
						}, cbg_cmd_pos_opts);
				robot_pos_subs_.push_back(sub);
			}
		}

		void CreateRobotSetup() {
			if(std::filesystem::exists(idf_file_)) {
				RCLCPP_INFO(this->get_logger(), "Reading IDF file %s", idf_file_.c_str());
			} else {
				RCLCPP_ERROR(this->get_logger(), "IDF file %s does not exist", idf_file_.c_str());
				rclcpp::shutdown();
			}
			RCLCPP_INFO(this->get_logger(), "IDF file %s exists", idf_file_.c_str());
			WorldIDF world_idf(parameters_, idf_file_);
			RCLCPP_INFO(this->get_logger(), "Created world");

			while(received_robot_pos_.size() < (size_t)parameters_.pNumRobots and coverage_system_ptr_ == nullptr and rclcpp::ok()) {
				RCLCPP_INFO(this->get_logger(), "Waiting for robot positions");
				rclcpp::spin_some(this->get_node_base_interface());
				std::this_thread::sleep_for(std::chrono::milliseconds(100));
			}
			RCLCPP_INFO(this->get_logger(), "CoverageSystem initializing");
			coverage_system_ptr_ = std::make_shared<CoverageSystem>(parameters_, world_idf, robot_positions_);
			RCLCPP_INFO(this->get_logger(), "CoverageSystem created");
			controller_ = std::make_shared<CoverageAlgorithm>(parameters_, parameters_.pNumRobots, *coverage_system_ptr_);
			std::cout << "controller created" << std::endl;
		}

		void CreateRobotPosPublishers() {
			for (int robot_id = 0; robot_id < parameters_.pNumRobots; robot_id++) {
				/* RCLCPP_INFO(this->get_logger(), "Creating robot position publisher for robot %d", robot_id); */
				std::string topic_name = robot_namespace_prefix_ + "/" + node_namespace_prefix_ + "/pose";
				/* RCLCPP_INFO(this->get_logger(), "Creating robot position publisher for robot %d with topic name %s", robot_id, topic_name.c_str()); */
				robot_pos_pubs_.push_back(this->create_publisher<geometry_msgs::msg::PoseStamped>(topic_name, buffer_size_));
				auto robot_pos_pub = robot_pos_pubs_.back();
				// Create timer to publish robot position
				auto robot_pos_pub_timer_callback = [this, robot_id, robot_pos_pub]() -> void {
					auto robot_pos = coverage_system_ptr_->GetRobotPosition(robot_id);
					geometry_msgs::msg::PoseStamped robot_pos_msg;
					robot_pos_msg.header.frame_id = "map";
					robot_pos_msg.header.stamp = this->now();
					robot_pos_msg.pose.position.x = robot_pos[0];
					robot_pos_msg.pose.position.y = robot_pos[1];
					robot_pos_pub->publish(robot_pos_msg);
				};
				robot_pos_pub_timers_.push_back(this->create_wall_timer(30ms, robot_pos_pub_timer_callback));
			}
			RCLCPP_INFO(this->get_logger(), "Created robot position publishers");
		}

		void CreateRobotLocalMapPublishers() {
			for (int robot_id = 0; robot_id < parameters_.pNumRobots; robot_id++) {
				std::string topic_name = robot_namespace_prefix_ + "/" + node_namespace_prefix_ + "/local_map";
				robot_local_map_pubs_.push_back(this->create_publisher<std_msgs::msg::Float32MultiArray>(topic_name, buffer_size_));
				auto robot_local_map_pub = robot_local_map_pubs_.back();
				// Create timer to publish local map
				auto robot_local_map_pub_timer_callback = [this, robot_id, robot_local_map_pub]() -> void {
					auto local_map = coverage_system_ptr_->GetRobotLocalMap(robot_id);
					auto local_map_msg = EigenMatrixRowMajorToFloat32MultiArray(local_map);
					robot_local_map_pub->publish(local_map_msg);
				};
				robot_local_map_pub_timers_.push_back(this->create_wall_timer(30ms, robot_local_map_pub_timer_callback));
			}

		}

		void CreateObstacleMapsPublisher() {
			for (int robot_id = 0; robot_id < parameters_.pNumRobots; robot_id++) {
				std::string topic_name = robot_namespace_prefix_ + "/" + node_namespace_prefix_ + "/obstacle_map";
				robot_obstacle_map_pubs_.push_back(this->create_publisher<std_msgs::msg::Float32MultiArray>(topic_name, buffer_size_));
				auto robot_obstacle_map_pub = robot_obstacle_map_pubs_.back();
				// Create timer to publish local map
				auto robot_obstacle_map_pub_timer_callback = [this, robot_id, robot_obstacle_map_pub]() -> void {
					auto obstacle_map = coverage_system_ptr_->GetRobotObstacleMap(robot_id);
					auto obstacle_map_msg = EigenMatrixRowMajorToFloat32MultiArray(obstacle_map);
					robot_obstacle_map_pub->publish(obstacle_map_msg);
				};
				robot_obstacle_map_pub_timers_.push_back(this->create_wall_timer(30ms, robot_obstacle_map_pub_timer_callback));
			}
		}

		void CreateNeigborPosPublisher() {
			for (int robot_id = 0; robot_id < parameters_.pNumRobots; ++robot_id) {
				std::string topic_name = robot_namespace_prefix_ + "/" + node_namespace_prefix_ + "/neighbors_pos";
				robot_neighbor_pos_pubs_.push_back(this->create_publisher<geometry_msgs::msg::PoseArray>(topic_name, buffer_size_));
				auto robot_neighbor_pos_pub = robot_neighbor_pos_pubs_.back();
				auto robot_neighbor_pos_pub_timer_callback = [this, robot_id, robot_neighbor_pos_pub]() -> void {
					auto neighbors_pos = coverage_system_ptr_->GetRelativePositonsNeighbors(robot_id);
					geometry_msgs::msg::PoseArray neighbor_pos_msg;
					for (size_t i = 0; i < neighbors_pos.size(); i++) {
						geometry_msgs::msg::Pose neighbor_pos_i_msg;
						neighbor_pos_i_msg.position.x = neighbors_pos[i][0];
						neighbor_pos_i_msg.position.y = neighbors_pos[i][1];
						neighbor_pos_msg.poses.push_back(neighbor_pos_i_msg);
					}
					robot_neighbor_pos_pub->publish(neighbor_pos_msg);
				};
				robot_neighbor_pos_pub_timers_.push_back(this->create_wall_timer(30ms, robot_neighbor_pos_pub_timer_callback));
			}
		}

		void CreateNeigborIDPublisher() {
			for (int robot_id = 0; robot_id < parameters_.pNumRobots; ++robot_id) {
				std::string topic_name = robot_namespace_prefix_ + "/" + node_namespace_prefix_ + "/neighbors_id";
				robot_neighbor_id_pubs_.push_back(this->create_publisher<std_msgs::msg::Int32MultiArray>(topic_name, buffer_size_));
				auto robot_neighbor_id_pub = robot_neighbor_id_pubs_.back();
				auto robot_neighbor_id_pub_timer_callback = [this, robot_id, robot_neighbor_id_pub]() -> void {
					auto neighbors_id = coverage_system_ptr_->GetNeighborIDs(robot_id);
					std_msgs::msg::Int32MultiArray neighbor_id_msg;
					neighbor_id_msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
					neighbor_id_msg.layout.dim[0].size = neighbors_id.size();
					neighbor_id_msg.layout.dim[0].stride = neighbors_id.size();
					neighbor_id_msg.layout.dim[0].label = "neighbors";
					neighbor_id_msg.data = neighbors_id;
					/* for (size_t i = 0; i < neighbors_id.size(); i++) { */
					/* 	neighbor_id_msg.data.push_back(neighbors_id[i]); */
					/* } */
					robot_neighbor_id_pub->publish(neighbor_id_msg);
				};
				robot_neighbor_id_pub_timers_.push_back(this->create_wall_timer(30ms, robot_neighbor_id_pub_timer_callback));
			}
		}

		void CreateSensorViewPublisher() {
			for (int robot_id = 0; robot_id < parameters_.pNumRobots; robot_id++) {
				std::string topic_name = robot_namespace_prefix_ + "/" + node_namespace_prefix_ + "/sensor_view";
				robot_sensor_view_pubs_.push_back(this->create_publisher<std_msgs::msg::Float32MultiArray>(topic_name, buffer_size_));
				auto robot_sensor_view_pub = robot_sensor_view_pubs_.back();
				// Create timer to publish local map
				auto robot_sensor_view_pub_timer_callback = [this, robot_id, robot_sensor_view_pub]() -> void {
					MapType sensor_view = coverage_system_ptr_->GetRobotSensorView(robot_id);
					/* if(robot_id == 0) */
					/* 	std::cout << "sensor_view: " << sensor_view.sum() << std::endl; */
					//  Print thread id
					auto sensor_view_msg = EigenMatrixRowMajorToFloat32MultiArray(sensor_view);
					robot_sensor_view_pub->publish(sensor_view_msg);
				};
				robot_sensor_view_pub_timers_.push_back(this->create_wall_timer(30ms, robot_sensor_view_pub_timer_callback));
			}
		}

		void CreateSystemMapPublisher() {
			/* std::string topic_name = robot_namespace_prefix_ + std::to_string(robot_id_) + "/" + node_namespace_prefix_ + "/global_map"; */
			std::string topic_name = "/global_map";
			global_map_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(topic_name, buffer_size_);
			auto global_map_pub = global_map_pub_;
			// Create timer to publish local map
			auto global_map_pub_timer_callback = [this, global_map_pub]() -> void {
				auto global_map = coverage_system_ptr_->GetSystemMap();
				auto global_map_msg = EigenMatrixRowMajorToFloat32MultiArray(global_map);
				global_map_pub->publish(global_map_msg);
			};
			global_map_pub_timer_ = this->create_wall_timer(30ms, global_map_pub_timer_callback);
		}

		void CreateRobotPosesPublisher() {
			robot_poses_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("robot_poses", buffer_size_);
			auto robot_poses_pub = robot_poses_pub_;
			// Create timer to publish local map
			auto robot_poses_pub_timer_callback = [this, robot_poses_pub]() -> void {
				geometry_msgs::msg::PoseArray robot_poses_msg;
				// Header for pose array
				robot_poses_msg.header.frame_id = "map";
				robot_poses_msg.header.stamp = this->now();

				for (int robot_id = 0; robot_id < parameters_.pNumRobots; robot_id++) {
					auto robot_pos = coverage_system_ptr_->GetRobotPosition(robot_id);
					geometry_msgs::msg::Pose robot_pose_i_msg;
					robot_pose_i_msg.position.x = robot_pos[0];
					robot_pose_i_msg.position.y = robot_pos[1];
					robot_poses_msg.poses.push_back(robot_pose_i_msg);
				}
				robot_poses_pub->publish(robot_poses_msg);
			};
			robot_poses_pub_timer_ = this->create_wall_timer(30ms, robot_poses_pub_timer_callback);
		}

		// Function for eigen matrix to std_msgs::msg::Float32MultiArray conversion
		std_msgs::msg::Float32MultiArray EigenMatrixToFloat32MultiArray(Eigen::MatrixXf matrix) {
			std_msgs::msg::Float32MultiArray msg;
			// Convert eigen matrix to std_msgs::msg::Float32MultiArray
			msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
			msg.layout.dim[0].size = matrix.rows();
			msg.layout.dim[0].stride = matrix.rows();
			msg.layout.dim[0].label = "rows";
			msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
			msg.layout.dim[1].size = matrix.cols();
			msg.layout.dim[1].stride = matrix.cols();
			msg.layout.dim[1].label = "cols";
			msg.data.clear();
			for (int i = 0; i < matrix.rows(); i++) {
				for (int j = 0; j < matrix.cols(); j++) {
					msg.data.push_back(matrix(i, j));
				}
			}
			return msg;
		}

		// Function for eigen matrix (row major) to std_msgs::msg::Float32MultiArray conversion
		std_msgs::msg::Float32MultiArray EigenMatrixRowMajorToFloat32MultiArray(Eigen::MatrixXf matrix) {
			std_msgs::msg::Float32MultiArray msg;
			// Convert eigen matrix to std_msgs::msg::Float32MultiArray
			msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
			msg.layout.dim[0].size = matrix.rows();
			msg.layout.dim[0].stride = matrix.cols() * matrix.rows();
			msg.layout.dim[0].label = "rows";
			msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
			msg.layout.dim[1].size = matrix.cols();
			msg.layout.dim[1].stride = matrix.cols();
			msg.layout.dim[1].label = "cols";
			msg.data.clear();
			msg.data.reserve(matrix.rows() * matrix.cols());
			for (int i = 0; i < matrix.rows(); i++) {
				for (int j = 0; j < matrix.cols(); j++) {
					msg.data.push_back(matrix(i, j));
				}
			}
			return msg;
		}
};
