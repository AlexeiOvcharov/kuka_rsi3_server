cmake_minimum_required (VERSION 2.8.3)
project(kuka_rsi3_server)

add_compile_options(-std=c++11)

find_package(catkin COMPONENTS roscpp roslib REQUIRED)
find_package(Boost COMPONENTS system thread filesystem REQUIRED)
find_package(yaml-cpp REQUIRED)

catkin_package()

link_directories(/usr/local/lib) # For pugixml
include_directories(
	include/
	${catkin_INCLUDE_DIRS}
)

add_executable(server src/boost_UDPSocket.cpp)
target_link_libraries(server ${Boost_LIBRARIES} ${YAML_CPP_LIBRARIES} pugixml ${catkin_LIBRARIES})

add_executable(client src/CommandClient.cpp)
target_link_libraries(client ${Boost_LIBRARIES} ${YAML_CPP_LIBRARIES} pugixml ${catkin_LIBRARIES})

add_executable(trajtest src/TrajectoryTest.cpp)
target_link_libraries(trajtest ${Boost_LIBRARIES} ${catkin_LIBRARIES})