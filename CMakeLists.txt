cmake_minimum_required(VERSION 2.8)
project(ros_flydra)

find_package(catkin REQUIRED message_generation sensor_msgs geometry_msgs)

add_message_files()
add_service_files()

catkin_python_setup()

generate_messages(DEPENDENCIES sensor_msgs geometry_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime sensor_msgs geometry_msgs
)

catkin_install_python(PROGRAMS nodes/camhistograms nodes/camnode nodes/flydra2ros nodes/flydra2rviz
  nodes/flydra2stats nodes/joysuperpacket nodes/liveplot nodes/main_brain nodes/showpose nodes/speakpose
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
