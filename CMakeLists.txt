cmake_minimum_required(VERSION 2.8.3)
project(assembly_dummy_controller)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  assembly_msgs
  actionlib
  actionlib_msgs
  franka_gripper
  assembly_dxl_gripper
  franka_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
  roscpp
  assembly_msgs
  actionlib
  actionlib_msgs
  franka_gripper
  #DEPENDS 
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  coppelia_src/remoteApi
  coppelia_src/include
  include
)

add_definitions (-DNON_MATLAB_PARSING)
add_definitions (-DMAX_EXT_API_CONNECTIONS=255)
#add_definitions (-DDO_NOT_USE_SHARED_MEMORY)

set (COPPELIA_SRC
  coppelia_src/remoteApi/extApi.c
  coppelia_src/remoteApi/extApiPlatform.c
  coppelia_src/common/shared_memory.c
)

set (SRC
	# src/controller.cpp
  src/vrep_bridge.cpp
  src/dummy_joint_trajectory_action_server.cpp
  src/main.cpp
)


add_library(coppelia_remote_api ${COPPELIA_SRC})
add_executable(assembly_dummy_controller ${SRC})
add_executable(assembly_dummy_controller_for_all src/dummy_for_all.cpp)

target_link_libraries(coppelia_remote_api rt)
target_link_libraries(assembly_dummy_controller
  ${catkin_LIBRARIES}
  coppelia_remote_api
  rt
)
target_link_libraries(assembly_dummy_controller_for_all
  ${catkin_LIBRARIES}
  rt
)
