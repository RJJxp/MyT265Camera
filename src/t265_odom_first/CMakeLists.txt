cmake_minimum_required(VERSION 2.8.3)
project(t265_odom_first)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(realsense2)
if(NOT realsense2_FOUND)
  message(FATAL ERROR "\n\n Intel Realsense SDK2.0 is missing \n\n")
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
)

find_package(Eigen3 REQUIRED)

catkin_package(
   CATKIN_DEPENDS roscpp tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${realsense_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(t265_odom_node 
  src/t265_odom_node.cpp
  src/t265_odom.cpp
)

target_link_libraries(t265_odom_node
  ${realsense2_LIBRARY}
  ${catkin_LIBRARIES}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS t265_odom_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)