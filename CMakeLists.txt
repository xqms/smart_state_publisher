
cmake_minimum_required(VERSION 3.10)
project(smart_state_publisher)

find_package(catkin REQUIRED COMPONENTS
    tf2_ros
    moveit_ros_planning
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

set(CMAKE_CXX_STANDARD 17)

add_executable(smart_state_publisher
    src/smart_state_publisher.cpp
)
target_link_libraries(smart_state_publisher
    ${catkin_LIBRARIES}
)

install(TARGETS smart_state_publisher
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
