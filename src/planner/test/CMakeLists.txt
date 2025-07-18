cmake_minimum_required(VERSION 3.0.2)
project(test)


SET(CMAKE_BUILD_TYPE Debug)


## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  plan_env
  path_searching
  traj_optimization
  cv_bridge
  quadrotor_msgs
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(OsqpEigen REQUIRED)

catkin_package(
  # INCLUDE_DIRS include
  # LIBRARIES test
  # CATKIN_DEPENDS roscpp rospy std_msgs plan_env
  # DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/test.cpp
# )

## Declare a C++ executable
# path_searching
add_executable(test_astar_node src/test_astar.cpp)
# add_executable(test_rrt_node src/test_rrt.cpp)
add_executable(test_rrt_star_node src/test_rrt_star.cpp)
# add_executable(test_kino_astar_node src/test_kino_astar.cpp)

add_executable(test_astar_searching src/test_astar_searching.cpp)

# add_executable(test_rrt_searching src/test_rrt_searching.cpp)
add_executable(test_rrt_star_searching src/test_rrt_star_searching.cpp)
# add_executable(test_kino_astar_searching src/test_kino_astar_searching.cpp)

# trajectory_optimization
add_executable(test_qpsolve src/test_qpsolve.cpp)
add_executable(test_minimum_jerk src/test_minimum_jerk.cpp)
add_executable(test_minimum_jerk_astar src/test_minimum_jerk_astar.cpp)

## Specify libraries to link a library or executable target against

target_link_libraries(test_astar_node
  ${catkin_LIBRARIES}
)

# target_link_libraries(test_rrt_node
# ${catkin_LIBRARIES}
# )

target_link_libraries(test_rrt_star_node
${catkin_LIBRARIES}
)

# target_link_libraries(test_kino_astar_node
# ${catkin_LIBRARIES}
# )

target_link_libraries(test_astar_searching
  ${catkin_LIBRARIES}
)

# target_link_libraries(test_rrt_searching
# ${catkin_LIBRARIES}
# )

target_link_libraries(test_rrt_star_searching
${catkin_LIBRARIES}
)

# target_link_libraries(test_kino_astar_searching
# ${catkin_LIBRARIES}
# )

target_link_libraries(test_qpsolve
${catkin_LIBRARIES}
OsqpEigen::OsqpEigen
)

target_link_libraries(test_minimum_jerk
${catkin_LIBRARIES}
traj_optimization
OsqpEigen::OsqpEigen
)

target_link_libraries(test_minimum_jerk_astar
${catkin_LIBRARIES}
traj_optimization
OsqpEigen::OsqpEigen
)