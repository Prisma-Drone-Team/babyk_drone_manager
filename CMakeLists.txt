cmake_minimum_required(VERSION 3.8)
project(babyk_drone_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(px4_msgs REQUIRED)

# RTABMap and SLAM dependencies
find_package(rtabmap_ros REQUIRED)
find_package(rtabmap_util REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(Eigen3 REQUIRED)

# Add move manager node executable
add_executable(move_manager_node 
  src/move_manager_node.cpp
)

ament_target_dependencies(move_manager_node 
  rclcpp 
  nav_msgs 
  geometry_msgs 
  std_msgs 
  sensor_msgs
  tf2 
  tf2_ros 
  tf2_geometry_msgs
)

add_executable(vio_aligner_node src/vio_aligner.cpp)
ament_target_dependencies(vio_aligner_node
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  px4_msgs
)

add_executable(autonomous_test_node src/autonomous_test_node.cpp)

ament_target_dependencies(autonomous_test_node 
  rclcpp 
  std_msgs 
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  octomap
  octomap_msgs
  grid_map_core
)
target_include_directories(autonomous_test_node PUBLIC ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(autonomous_test_node Eigen3::Eigen)

add_executable(sewer_autonomous_test_node src/sewer_autonomous_test_node.cpp)

ament_target_dependencies(sewer_autonomous_test_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  octomap
  octomap_msgs
  grid_map_core
)
target_include_directories(sewer_autonomous_test_node PUBLIC ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(sewer_autonomous_test_node Eigen3::Eigen)

add_executable(open_box src/open_box.cpp)
ament_target_dependencies(open_box
  rclcpp
  px4_msgs
)

add_executable(exploration_node src/exploration_node.cpp)

ament_target_dependencies(exploration_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  octomap_msgs
)
target_link_libraries(exploration_node ${OCTOMAP_LIBRARIES})

target_include_directories(move_manager_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(autonomous_test_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(sewer_autonomous_test_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(exploration_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OCTOMAP_INCLUDE_DIRS}
)

target_link_libraries(move_manager_node 
  ${geometry_msgs_LIBRARIES}
)

# Install targets
install(TARGETS move_manager_node autonomous_test_node sewer_autonomous_test_node exploration_node vio_aligner_node open_box
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/vio_aligner.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and config files
install(DIRECTORY 
  launch
  config
  rviz
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

# Install TMUX files
install(FILES 
  simulation.yml
  flight.yml
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
