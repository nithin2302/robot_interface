cmake_minimum_required(VERSION 2.8.3)
project(init_prog)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  roslaunch
  message_generation
)


find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} )

find_package(Aria QUIET COMPONENTS ArNetworking)
if(Aria_FOUND)
	if(EXISTS "${Aria_INCLUDE_DIRS}/Aria.h")
		add_definitions(-DADEPT_PKG)
	endif()
  include_directories(${Aria_INCLUDE_DIRS})
  link_directories(${Aria_LIBRARY_DIRS})
else()
	# The installation package provided by Adept doesn't follow Debian policies
	if(EXISTS "/usr/local/Aria/include/Aria.h")
	  add_definitions(-DADEPT_PKG)
	  include_directories( /usr/local/Aria/include /usr/local/Aria/ArNetworking/include)
	  link_directories(/usr/local/Aria/lib)
	endif()
endif()

include_directories(include ${catkin_INCLUDE_DIRS})

add_service_files(FILES 
  PBGo.srv)


generate_messages(
  DEPENDENCIES
  std_msgs)

catkin_package(
  LIBRARIES robot_interface ptu46
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

## Declare a cpp executable
add_executable(start_roscore src/PBGo_client.cpp)
add_executable(interface src/interface.cpp)
add_executable(get_map src/get_map.cpp)

target_link_libraries(PBGo_client 
   ${catkin_LIBRARIES}
)



target_link_libraries(interface
  ${catkin_LIBRARIES}
  )


target_link_libraries(get_map
  ${catkin_LIBRARIES}
  )

add_executable(PBGo_client src/PBGo_client.cpp)
add_dependencies( PBGo_client init_prog_generate_messages_cpp )
target_link_libraries(PBGo_client ${catkin_LIBRARIES} ${Boost_LIBRARIES} Aria pthread dl rt ArNetworking)
set_target_properties(PBGo_client PROPERTIES COMPILE_FLAGS "-fPIC")



#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_turtle_test.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
roslaunch_add_file_check(launch)
