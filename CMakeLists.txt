cmake_minimum_required(VERSION 2.8.3)
project(flir_gige)

add_definitions(-std=c++11 -Wall -Wno-unknown-pragmas)

# Set library paths for pleora ebus sdk on 64-bit linux
# For more information, see cmake/Findebus.cmake
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    # Make CMake aware of the cmake folder for local FindXXX scripts.
    set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
    find_package(ebus REQUIRED)
else()
    message(FATAL_ERROR "This package only supports 64-bit linux")
endif()

find_package(catkin REQUIRED COMPONENTS
    roscpp nodelet camera_base cv_bridge
    )
find_package(OpenCV)

# Dynamic reconfigure option, as to be placed right before catkin_package
generate_dynamic_reconfigure_options(
    cfg/FlirGigeDyn.cfg
    cfg/ThermalProcDyn.cfg
    )

catkin_package(
    #  INCLUDE_DIRS include
    #  LIBRARIES flir
    #  CATKIN_DEPENDS roscpp sensor_msgs
    #  DEPENDS system_lib
    )

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EBUS_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    )

# library
add_library(${PROJECT_NAME}
    src/flir_gige/flir_gige.cpp
    src/flir_gige/flir_gige_ros.cpp
    src/flir_gige/flir_gige_node.cpp
    src/flir_gige/flir_gige_nodelet.cpp
    src/thermal_proc/thermal_proc_node.cpp
    src/thermal_proc/thermal_proc_nodelet.cpp
    )
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${EBUS_LIBRARIES}
    )

# node
add_executable(${PROJECT_NAME}_node
    src/flir_gige/flir_gige_main.cpp
    )
target_link_libraries(${PROJECT_NAME}_node
    ${PROJECT_NAME}
    )
add_executable(thermal_proc_node
    src/thermal_proc/thermal_proc_main.cpp
    )
target_link_libraries(thermal_proc_node
    ${PROJECT_NAME}
    )

## Add cmake target dependencies of the executable/library
add_dependencies(${PROJECT_NAME}
    ${catkin_EXPORTED_TARGETS}
    ${PROJECT_NAME}_gencfg
    ${PROJECT_NAME}_gencpp
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    )
