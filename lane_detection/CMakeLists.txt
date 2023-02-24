cmake_minimum_required(VERSION 3.0.2)
project(lane_detection)

add_compile_options(-std=c++14)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  sensor_msgs
  std_msgs
        xycar_msgs
        rospy

)
find_package(OpenCV REQUIRED)

catkin_package(
INCLUDE_DIRS include
CATKIN_DEPENDS cv_bridge roscpp sensor_msgs std_msgs xycar_msgs rospy
)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)
add_executable(${PROJECT_NAME} src/lane_detection_node.cpp include/lane_detection/lane_detection.h src/lane_detection.cpp)

target_link_libraries(${PROJECT_NAME}
 ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
