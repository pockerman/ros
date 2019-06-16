cmake_minimum_required(VERSION 2.8.3)
project(ocv_lane_tracker)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ocv_lane_tracker
  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node_lane_tracker src/node_1/node_1.cpp)
add_executable(${PROJECT_NAME}_node_camera src/node_2/node_2.cpp)

LINK_DIRECTORIES(OpenCV_LIBRARIES_DIR /usr/local/lib)
SET(OpenCV_LIBRARIES opencv_highgui )
target_link_libraries(${PROJECT_NAME}_node  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_node_camera  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
