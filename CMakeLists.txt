cmake_minimum_required(VERSION 2.8.3)
project(ssf_imu)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  qualisys
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES ssf_imu
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)


## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
  "/usr/include/eigen3"
)


## Declare a C++ library
# add_library(ssf_imu
#   src/${PROJECT_NAME}/ssf_imu.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
 #add_dependencies(ssf_imu ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(ssf_imu_node src/ssf_imu_node.cpp)
add_dependencies(ssf_imu_node ssf_imu_generate_messages_cpp)
target_link_libraries(ssf_imu_node   ${catkin_LIBRARIES} )

add_executable(ssf_imu_position_node src/ssf_imu_position_node.cpp)
add_dependencies(ssf_imu_position_node ssf_imu_position_generate_messages_cpp)
target_link_libraries(ssf_imu_position_node   ${catkin_LIBRARIES} )

add_executable(ssf_imu_mocap_node src/ssf_imu_mocap_node.cpp)
add_dependencies(ssf_imu_mocap_node ssf_imu_mocap_generate_messages_cpp)
target_link_libraries(ssf_imu_mocap_node   ${catkin_LIBRARIES} )
