#ifndef COVERAGECONTROL_SIM_UTILS_HPP_
#define COVERAGECONTROL_SIM_UTILS_HPP_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <cmath>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <vector>

namespace CoverageControlSim {

using RowMajorMatrixXf =
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

inline auto XYtoPose(double const &x, double const &y) {
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = 0;
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.orientation.w = 1;
  return pose;
}

inline auto XYtoPoseStamped(double const &x, double const &y) {
  /* inline XYtoPoseStamped (double const &x, double const &y) { */
  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.header.stamp = rclcpp::Time();
  pose_stamped.header.frame_id = "map";
  pose_stamped.pose = XYtoPose(x, y);
  return pose_stamped;
}
//
// Function for eigen matrix to std_msgs::msg::Float32MultiArray conversion
inline auto EigenMatrixToFloat32MultiArray(Eigen::MatrixXf matrix) {
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

// Function for eigen matrix (row major) to std_msgs::msg::Float32MultiArray
// conversion
inline auto EigenMatrixRowMajorToFloat32MultiArrayInefficient(Eigen::MatrixXf matrix) {
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

inline auto EigenMatrixRowMajorToFloat32MultiArray(
    const RowMajorMatrixXf &in_matrix, int scale = 1) {
  RowMajorMatrixXf matrix = in_matrix(Eigen::seq(0, Eigen::last, scale), Eigen::seq(0, Eigen::last, scale));
  std_msgs::msg::Float32MultiArray msg;

  // Initialize dimensions
  std_msgs::msg::MultiArrayDimension dim_row;
  dim_row.label = "rows";
  dim_row.size = static_cast<uint32_t>(matrix.rows());
  dim_row.stride = static_cast<uint32_t>(matrix.rows() * matrix.cols());
  msg.layout.dim.push_back(dim_row);

  std_msgs::msg::MultiArrayDimension dim_col;
  dim_col.label = "cols";
  dim_col.size = static_cast<uint32_t>(matrix.cols());
  dim_col.stride = static_cast<uint32_t>(matrix.cols());
  msg.layout.dim.push_back(dim_col);

  // Assign data in one go
  msg.data = std::vector<float>(matrix.data(), matrix.data() + matrix.size());

  return msg;
}

inline RowMajorMatrixXf Float32MultiArrayToEigenMatrixRowMajor(
    const std_msgs::msg::Float32MultiArray &msg) {
  // Ensure the matrix is initialized as row-major
  RowMajorMatrixXf matrix(msg.layout.dim[0].size, msg.layout.dim[1].size);

  // Use Eigen::Map to directly map data
  Eigen::Map<RowMajorMatrixXf>(matrix.data(), matrix.rows(), matrix.cols()) =
      Eigen::Map<const RowMajorMatrixXf>(msg.data.data(), matrix.rows(),
                                         matrix.cols());

  return matrix;
}
}  // namespace CoverageControlSim

#endif  // COVERAGECONTROL_SIM_UTILS_HPP_
