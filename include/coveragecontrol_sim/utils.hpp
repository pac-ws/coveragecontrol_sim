#ifndef COVERAGECONTROL_SIM_UTILS_HPP_
#define COVERAGECONTROL_SIM_UTILS_HPP_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <cmath>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <vector>

namespace CoverageControlSim {

using RowMajorMatrixXf =
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

inline auto IdentityTransform() {
  geometry_msgs::msg::Transform transform;
  transform.translation.x = 0.0;
  transform.translation.y = 0.0;
  transform.translation.z = 0.0;
  transform.rotation.x = 0.0;
  transform.rotation.y = 0.0;
  transform.rotation.z = 0.0;
  transform.rotation.w = 1.0;
  return transform;
}

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

inline auto ZeroPose() {
  geometry_msgs::msg::Pose pose;
  pose.position.x = 0;
  pose.position.y = 0;
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

inline auto ZeroSquareMap(int size) {
  return EigenMatrixToFloat32MultiArray(Eigen::MatrixXf::Zero(size, size));
}

// Function for eigen matrix (row major) to std_msgs::msg::Float32MultiArray
// conversion
inline auto EigenMatrixRowMajorToFloat32MultiArrayInefficient(
    Eigen::MatrixXf matrix) {
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
  RowMajorMatrixXf matrix = in_matrix(Eigen::seq(0, Eigen::last, scale),
                                      Eigen::seq(0, Eigen::last, scale));
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

// EigenMatrixRowMajor to PointCloud2 conversion
inline auto EigenMatrixRowMajorToPointCloud2(const RowMajorMatrixXf &in_matrix,
                                             const float scale = 1.0,
                                             const float offset_x = 0.0,
                                             const float offset_y = 0.0) {
  RowMajorMatrixXf matrix = in_matrix(Eigen::seq(0, Eigen::last, scale),
                                      Eigen::seq(0, Eigen::last, scale));
  // Create PointCloud2 message
  sensor_msgs::msg::PointCloud2 msg;
  msg.header.frame_id = "map";
  msg.header.stamp = rclcpp::Clock().now();

  sensor_msgs::msg::PointField x_field;
  x_field.name = "x";
  x_field.offset = 0;
  x_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  x_field.count = 1;

  sensor_msgs::msg::PointField y_field = x_field;
  y_field.name = "y";
  y_field.offset = 4;

  sensor_msgs::msg::PointField z_field = x_field;
  z_field.name = "z";
  z_field.offset = 8;

  sensor_msgs::msg::PointField intensity_field = x_field;
  intensity_field.name = "intensity";
  intensity_field.offset = 12;

  msg.fields = {x_field, y_field, z_field, intensity_field};
  msg.point_step = 16;
  msg.is_bigendian = false;
  msg.is_dense = true;

  float z = 0.0f;
  std::vector<float> point_data;
  for (int i = 0; i < matrix.rows(); i++) {
    for (int j = 0; j < matrix.cols(); j++) {
      float val = matrix(i, j);
      if (val == -1) {
        continue;
      }
      point_data.push_back(i * scale + offset_x);
      point_data.push_back(j * scale + offset_y);
      point_data.push_back(z);
      point_data.push_back(val);
    }
  }
  msg.width = point_data.size() / 4;
  msg.height = 1;
  msg.row_step = msg.point_step * msg.width;

  msg.data.resize(point_data.size() * sizeof(float));
  memcpy(msg.data.data(), point_data.data(), msg.data.size());

  return msg;
}

}  // namespace CoverageControlSim

#endif  // COVERAGECONTROL_SIM_UTILS_HPP_
