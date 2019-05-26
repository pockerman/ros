cmake_minimum_required(VERSION 2.8.3)
project(ros_services)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
	message_generation
)

add_service_files(
   FILES
   word_count.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_generation
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node_1  src/node_1/node_1.cpp)
add_executable(${PROJECT_NAME}_node_2  src/node_2/node_2.cpp)
target_link_libraries(${PROJECT_NAME}_node_1 ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_node_2 ${catkin_LIBRARIES})

