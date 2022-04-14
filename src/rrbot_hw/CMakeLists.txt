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

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

catkin_package(
    INCLUDE_DIRS 
        include
    LIBRARIES 
        ${PROJECT_NAME}
    CATKIN_DEPENDS 
        controller_manager
        hardware_interface
        roscpp
    DEPENDS 
        Boost
)



add_library(${PROJECT_NAME}
    src/motor_driver.cpp
    src/modbus.cpp
    src/rrbot_hardware_interface
    src/rrbot_hardware_interface_node.cpp
)
# target_link_libraries(${PROJECT_NAME}
# 	${catkin_LIBRARIES}
#     ${Boost_LIBRARIES}
# )


# add_library(modbus src/modbus.cpp)
# target_link_libraries(modbus
# 	${catkin_LIBRARIES}
#     ${Boost_LIBRARIES}
# )

# add_library(motor_driver src/motor_driver.cpp)
# add_dependencies(motor_driver modbus)
# target_link_libraries(motor_driver
# 	${catkin_LIBRARIES}
# )

# add_library(rrbot_hardware_interface src/rrbot_hardware_interface.cpp)
# add_dependencies(rrbot_hardware_interface motor_driver)
# target_link_libraries(rrbot_hardware_interface
# 	${catkin_LIBRARIES}
# )

add_executable(rrbot_hardware_interface_node 
    src/rrbot_hardware_interface_node.cpp 
    src/rrbot_hardware_interface.cpp 
    src/motor_driver.cpp 
    src/modbus.cpp
)

add_dependencies(rrbot_hardware_interface_node 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(rrbot_hardware_interface_node
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
)

# install(DIRECTORY include/${PROJECT_NAME}/
#     DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#     FILES_MATCHING PATTERN "*.h"
#     PATTERN ".svn" EXCLUDE
# )
    