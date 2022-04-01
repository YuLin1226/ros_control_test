cmake_minimum_required(VERSION 3.0.2)
project(rrbot_hw)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    controller_manager
    hardware_interface
    roscpp
)

include_directories(
    include/
    ${catkin_INCLUDE_DIRS}
)


catkin_package(
    #  INCLUDE_DIRS include
    #  LIBRARIES rrbot_hw
    #  CATKIN_DEPENDS controller_manager hardware_interface roscpp
    #  DEPENDS system_lib
)
  
install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
)
    

add_library(rrbot_hardware_interface src/rrbot_hardware_interface.cpp)
# add_dependencies(rrbot_hardware_interface)
target_link_libraries(rrbot_hardware_interface
	${catkin_LIBRARIES}
)

add_executable(rrbot_hardware_interface_node src/rrbot_hardware_interface_node.cpp)

add_dependencies(rrbot_hardware_interface_node rrbot_hardware_interface)

target_link_libraries(rrbot_hardware_interface_node
    rrbot_hardware_interface
    ${catkin_LIBRARIES}
)


