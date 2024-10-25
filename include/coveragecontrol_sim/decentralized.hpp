#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>

#include <CoverageControl/constants.h>
#include <CoverageControl/parameters.h>
#include <CoverageControl/typedefs.h>
#include <CoverageControl/bivariate_normal_distribution.h>
#include <CoverageControl/world_idf.h>
#include <CoverageControl/coverage_system.h>

using namespace std::chrono_literals;
using namespace CoverageControl;

class CoverageControlSimDecentralized : public rclcpp::Node {

	private:

		std::shared_ptr<CoverageSystem> coverage_system_ptr_;
		std::string package_prefix_;
		Parameters parameters_;
		int robot_id_ = 0;
		int buffer_size_ = 10;
		float scale_factor_ = 1.0;
		std::mutex mutex_;

		// Robot position
		geometry_msgs::msg::Pose phoenix_robot_pos_;
		geometry_msgs::msg::Pose robot_pos_;

		// Publishers for local map
		rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr robot_local_map_pub_;
		rclcpp::TimerBase::SharedPtr robot_local_map_pub_timer_;

		// Publishers for obstacle map
		rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr robot_obstacle_map_pub_;
		rclcpp::TimerBase::SharedPtr robot_obstacle_map_pub_timer_;

		// Publish sensor view
		rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr robot_sensor_view_pub_;
		rclcpp::TimerBase::SharedPtr robot_sensor_view_pub_timer_;

		// Publisher for global system map
		rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr global_map_pub_;
		rclcpp::TimerBase::SharedPtr global_map_pub_timer_;

		rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_pub_;

		// Subscribe to robot positions
		rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr robot_pos_sub_;
		rclcpp::CallbackGroup::SharedPtr cbg_robot_pos_sub_;

	public:

		CoverageControlSimDecentralized(int robot_id, const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions()) : Node("coveragecontrol_sim_decentralized" + std::to_string(robot_id), node_options), coverage_system_ptr_(nullptr), robot_id_(robot_id) {

			package_prefix_ = ament_index_cpp::get_package_prefix("coveragecontrol_sim_decentralized");
			std::string env_file = this->declare_parameter<std::string>("env_file", package_prefix_ + "/config/env_params.yaml");
			parameters_ = Parameters(env_file);
			std::string idf_file = this->declare_parameter<std::string>("idf_file", package_prefix_ + "/config/sample.idf");

			float time_step = this->declare_parameter<float>("time_step", 0.1);
			std::chrono::milliseconds timeout = std::chrono::milliseconds(int(time_step * 1000));
			buffer_size_ = this->declare_parameter<int>("buffer_size", 10);
			scale_factor_ = this->declare_parameter<float>("scale_factor", 1.0);
			RCLCPP_INFO(this->get_logger(), "Scale factor: %f", scale_factor_);

			if(std::filesystem::exists(idf_file)) {
				RCLCPP_INFO(this->get_logger(), "Reading IDF file %s", idf_file.c_str());
			} else {
				RCLCPP_ERROR(this->get_logger(), "IDF file %s does not exist", idf_file.c_str());
				rclcpp::shutdown();
			}
			WorldIDF world_idf(parameters_, idf_file);

			// Make callback group for cmd_pos_subs_ that are mutually exclusive
			cbg_robot_pos_sub_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
			auto cbg_opts  = rclcpp::SubscriptionOptions();
			cbg_opts.callback_group = cbg_robot_pos_sub_;

			pose_pub_ = this->create_publisher<geometry_msgs::msg::Pose>("pose", buffer_size_);
			// Create subscriber for robot position
			std::string topic_name = "phoenix_pose";
			robot_pos_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(topic_name, rclcpp::QoS(buffer_size_), [this, &world_idf](const geometry_msgs::msg::PoseStamped::SharedPtr msg) -> void {
					auto msg_pose = msg->pose;
				Point2 scaled_robot_pos(msg_pose.position.x, msg_pose.position.y);
				scaled_robot_pos *= scale_factor_;
				robot_pos_.position.x = scaled_robot_pos.x();
				robot_pos_.position.y = scaled_robot_pos.y();
				pose_pub_->publish(robot_pos_);
				if (coverage_system_ptr_ == nullptr) {
					std::lock_guard<std::mutex> lock(mutex_);
					PointVector robot_pos;
					robot_pos.push_back(scaled_robot_pos);
					coverage_system_ptr_ = std::make_shared<CoverageSystem>(parameters_, world_idf, robot_pos);
					}
					phoenix_robot_pos_ = msg->pose;
					coverage_system_ptr_->SetGlobalRobotPosition(robot_id_, scaled_robot_pos);
			}, cbg_opts);


			while (coverage_system_ptr_ == nullptr) {
				rclcpp::spin_some(this->get_node_base_interface());
				std::this_thread::sleep_for(std::chrono::milliseconds(1000));
				RCLCPP_INFO(this->get_logger(), "Waiting for robot position");
			}

			robot_local_map_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("local_map", buffer_size_);
			robot_local_map_pub_timer_ = this->create_wall_timer(timeout, [this]() -> void {
				auto local_map = coverage_system_ptr_->GetRobotLocalMap(robot_id_);
				auto local_map_msg = EigenMatrixRowMajorToFloat32MultiArray(local_map);
				robot_local_map_pub_->publish(local_map_msg);
			});

			robot_obstacle_map_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("obstacle_map", buffer_size_);
			robot_obstacle_map_pub_timer_ = this->create_wall_timer(timeout, [this]() -> void {
				auto obstacle_map = coverage_system_ptr_->GetRobotObstacleMap(robot_id_);
				auto obstacle_map_msg = EigenMatrixRowMajorToFloat32MultiArray(obstacle_map);
				robot_obstacle_map_pub_->publish(obstacle_map_msg);
			});

			robot_sensor_view_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("sensor_view", buffer_size_);
			robot_sensor_view_pub_timer_ = this->create_wall_timer(timeout, [this]() -> void {
				MapType sensor_view = coverage_system_ptr_->GetRobotSensorView(robot_id_);
				auto sensor_view_msg = EigenMatrixRowMajorToFloat32MultiArray(sensor_view);
				robot_sensor_view_pub_->publish(sensor_view_msg);
			});

			global_map_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("global_map", buffer_size_);
			global_map_pub_timer_ = this->create_wall_timer(timeout, [this]() -> void {
				auto global_map = coverage_system_ptr_->GetSystemMap();
				auto global_map_msg = EigenMatrixRowMajorToFloat32MultiArray(global_map);
				global_map_pub_->publish(global_map_msg);
			});

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

