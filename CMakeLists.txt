cmake_minimum_required(VERSION 3.8)
project(coveragecontrol_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if (NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE Release)
endif()

if (NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(CoverageControl REQUIRED)
find_package(tf2_ros REQUIRED)

set(target_dependencies
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  ament_index_cpp
  tf2_ros
  CoverageControl)

add_executable(sim_centralized src/sim_centralized.cpp src/centralized.cpp)
target_include_directories(sim_centralized PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(sim_centralized ${target_dependencies})

target_link_libraries(sim_centralized CoverageControl::CoverageControl)

target_compile_features(sim_centralized PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS sim_centralized
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
add_executable(decentralized_cvt src/decentralized_cvt.cpp)
target_include_directories(decentralized_cvt PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(decentralized_cvt ${target_dependencies})
target_link_libraries(decentralized_cvt CoverageControl::CoverageControl)
target_compile_features(decentralized_cvt PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
install(TARGETS decentralized_cvt
  DESTINATION lib/${PROJECT_NAME})

add_executable(centralized_cvt src/centralized_cvt.cpp)
target_include_directories(centralized_cvt PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(centralized_cvt ${target_dependencies})
target_link_libraries(centralized_cvt CoverageControl::CoverageControl)
target_compile_features(centralized_cvt PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
install(TARGETS centralized_cvt
  DESTINATION lib/${PROJECT_NAME})

add_executable(clairvoyant_cvt src/clairvoyant_cvt.cpp)
target_include_directories(clairvoyant_cvt PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(clairvoyant_cvt ${target_dependencies})
target_link_libraries(clairvoyant_cvt CoverageControl::CoverageControl)
target_compile_features(clairvoyant_cvt PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
install(TARGETS clairvoyant_cvt
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
