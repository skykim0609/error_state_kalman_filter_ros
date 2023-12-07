cmake_minimum_required(VERSION 3.5.1)
project(error_state_kalman_filter_ros)
set(CMAKE_BUILD_TYPE Debug) # Release, RelWithDebInfo
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -pthread -pg -O2")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR})
find_package(catkin REQUIRED cmake_modules)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

message("Eigen : ${Eigen3_VERSION}")
message("yaml libs : ${YAML_CPP_LIBRARIES}")
message("Cmake module path : ${CMAKE_MODULE_PATH}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  apriltag_ros
  tf
)

catkin_package(
  DEPENDS EIGEN3
  CATKIN_DEPENDS roscpp
  INCLUDE_DIRS include
)

set(SRC ${SRC}
    src/signal_handler_linux.cpp
    src/timer.cpp
    src/error_state_kalman_filter.cpp
    src/geometry_library.cpp
)

set(includeDirs ${includeDirs} ${CMAKE_CURRENT_BINARY_DIR})
set(includeDirs ${includeDirs} include)

include_directories( ${includeDirs} )
include_directories( ${catkin_INCLUDE_DIRS} )
include_directories( ${EIGEN3_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIRS})

message("Eigen_Include_dirs : ${EIGEN3_INCLUDE_DIRS}")
message("Yaml include directory : ${YAML_CPP_INCLUDE_DIRS}")

# executables
add_executable(optitrack_state_estimator_node src/node_state_estimator_optitrack.cpp src/optitrack_state_estimator_ros.cpp ${SRC})
target_link_libraries (optitrack_state_estimator_node ${catkin_LIBRARIES} Eigen3::Eigen)

add_executable(apriltag_state_estimator_node src/node_state_estimator_apriltag.cpp src/apriltag_state_estimator_ros.cpp ${SRC})
target_link_libraries (apriltag_state_estimator_node ${catkin_LIBRARIES} Eigen3::Eigen ${YAML_CPP_LIBRARIES})

add_executable(test_expm src/test_expm.cpp ${SRC})
target_link_libraries (test_expm ${catkin_LIBRARIES} Eigen3::Eigen)

add_executable(test_quat src/test_quaternion.cpp ${SRC})
target_link_libraries (test_quat ${catkin_LIBRARIES} Eigen3::Eigen)

add_executable(traj_comparator src/traj_comparator.cpp src/traj_comparator_node.cpp ${SRC})
target_link_libraries(traj_comparator ${catkin_LIBRARIES} Eigen3::Eigen)
