cmake_minimum_required(VERSION 2.8.3)
project(fwmav_of_sim)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation geometry_msgs sensor_msgs)
find_package( OpenCV REQUIRED )

add_message_files(FILES MsgOAOF.msg)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
  LIBRARIES fwmav_of_sim
  CATKIN_DEPENDS roscpp std_msgs cv_bridge image_transport geometry_msgs sensor_msgs
  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(fwmav_of_sim src/fwmav_of_sim.cpp)
target_link_libraries(fwmav_of_sim ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(fwmav_of_sim fwmav_of_sim_generate_messages_cpp)
