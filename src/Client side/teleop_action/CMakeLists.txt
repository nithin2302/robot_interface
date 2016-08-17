cmake_minimum_required(VERSION 2.8.3)
project(teleop_action)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs actionlib actionlib_msgs)
find_package(Boost REQUIRED COMPONENTS system)

add_action_files(
  DIRECTORY action
  FILES TeleOp.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(TeleOp_Client src/TeleOp_Client.cpp)
target_link_libraries(TeleOp_Client ${catkin_LIBRARIES})

add_executable(TeleOpKeyRead src/TeleOpKeyRead.cpp)
target_link_libraries(TeleOpKeyRead ${catkin_LIBRARIES})

