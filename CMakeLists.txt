cmake_minimum_required(VERSION 2.8.3)
project(map_scan)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    std_msgs
    nav_msgs
    sensor_msgs
    geometry_msgs
    roscpp
    rospy
    tf
)

add_service_files(
    DIRECTORY srv
    FILES
    GetMapScan.srv
)

generate_messages(DEPENDENCIES std_msgs nav_msgs sensor_msgs geometry_msgs)

catkin_package(
    CATKIN_DEPENDS roscpp rospy message_generation std_msgs nav_msgs sensor_msgs geometry_msgs
    INCLUDE_DIRS include
    LIBRARIES map_lib map_scan_lib
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    include
)

add_library(map_scan_lib src/map_scan.cpp)
add_library(map_lib src/map.c src/map_range.c)
target_link_libraries(map_lib map_scan_lib)

# add_dependencies(${PROJECT_NAME} ${catkin_LIBRARIES} map_scan_gencpp)
# target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(map_scan_node src/map_scan_node.cpp)
target_link_libraries(map_scan_node map_lib map_scan_lib ${catkin_LIBRARIES})