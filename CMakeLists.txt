cmake_minimum_required(VERSION 3.0.2)
project(eval_helper)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  snapstack_msgs
)

find_package(PCL 1.7 REQUIRED)
find_package(Eigen3 REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs
)

include_directories(
  SYSTEM
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_executable(noise_maker src/noise_maker.cpp)
target_link_libraries(noise_maker ${catkin_LIBRARIES})

add_executable(traj_noise_maker src/traj_noise_maker.cpp)
target_link_libraries(traj_noise_maker ${catkin_LIBRARIES})

add_executable(multi_eval src/multi_eval_node.cpp)
target_link_libraries(multi_eval ${catkin_LIBRARIES})

add_executable(trigger src/trigger.cpp)
target_link_libraries(trigger ${catkin_LIBRARIES})

# add_executable(noisy_ground_truth src/noisy_ground_truth.cpp )
# target_link_libraries(noisy_ground_truth ${catkin_LIBRARIES})
