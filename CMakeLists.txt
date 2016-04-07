cmake_minimum_required(VERSION 2.8.3)
project(beebot_driver)

find_package(catkin REQUIRED COMPONENTS
	std_msgs
	geometry_msgs
	roscpp
	message_generation
)

generate_messages(DEPENDENCIES
	std_msgs
	geometry_msgs
)

catkin_package(CATKIN_DEPENDS
	std_msgs
	geometry_msgs
	roscpp
	message_runtime
)

include_directories(
   	${catkin_INCLUDE_DIRS}
	/usr/local/lib
)

add_executable(beebot_driver_node src/beebot_driver.cpp)
target_link_libraries(beebot_driver_node 
	${catkin_LIBRARIES}
	libserial.so
	)


