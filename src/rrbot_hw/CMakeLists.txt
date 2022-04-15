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
find_package(Boost REQUIRED)
# find_package(Boost REQUIRED COMPONENTS thread)

include_directories(
    include/
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

catkin_package(
    INCLUDE_DIRS 
        include/
    LIBRARIES 
        ${PROJECT_NAME}
    CATKIN_DEPENDS 
        controller_manager
        hardware_interface
        roscpp
    DEPENDS 
        Boost
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
)
    

add_library(${PROJECT_NAME}
src/modbus.cpp
src/motor_driver.cpp
src/rrbot_hardware_interface.cpp
)
target_link_libraries(${PROJECT_NAME}
	${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
)


add_executable(rrbot_hardware_interface_node 
    src/rrbot_hardware_interface_node.cpp 
    # src/rrbot_hardware_interface.cpp 
    # src/motor_driver.cpp 
    # src/modbus.cpp
)

add_dependencies(rrbot_hardware_interface_node 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(rrbot_hardware_interface_node
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    # rrbot_hardware_interface
)



install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)