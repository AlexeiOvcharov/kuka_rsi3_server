cmake_minimum_required (VERSION 2.6)
project(RSI_Server)

add_definitions(-std=c++11)

find_package(Boost COMPONENTS system thread filesystem REQUIRED)
find_package(yaml-cpp REQUIRED)

link_directories(/usr/local/lib) # For pugixml
include_directories(include/)

add_executable(server src/boost_UDPSocket.cpp)
target_link_libraries(server ${Boost_LIBRARIES} ${YAML_CPP_LIBRARIES} pugixml)

add_executable(client src/CommandClient.cpp)
target_link_libraries(client ${Boost_LIBRARIES} ${YAML_CPP_LIBRARIES} pugixml)

add_executable(trajtest src/TrajectoryTest.cpp)
target_link_libraries(trajtest ${Boost_LIBRARIES}) 
