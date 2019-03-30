cmake_minimum_required(VERSION 2.8.3)
project(pupil_gazebo)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
    roscpp
    gazebo_ros
    image_transport
    camera_info_manager
    sensor_msgs
    cv_bridge
    rostest
)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(gazebo REQUIRED)
find_package(Threads REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

catkin_package(
    DEPENDS
        roscpp
        gazebo_ros
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
    ${GAZEBO_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS})

set(R200_GAZEBO_PLUGIN "r200_gazebo_plugin")
add_library(${R200_GAZEBO_PLUGIN} src/rs_plugin/RealSensePlugin.cpp src/rs_plugin/r200_ros_plugin.cpp)
target_link_libraries(${R200_GAZEBO_PLUGIN} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(${R200_GAZEBO_PLUGIN} ${catkin_EXPORTED_TARGETS})

install(
    TARGETS ${R200_GAZEBO_PLUGIN}
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
