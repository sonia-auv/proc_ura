# \file     CMakeLists.txt
# \author   Francis Masse <francis.masse05@gmail.com>
# \date     05/04/2017
# \copyright    2017 S.O.N.I.A. AUV, ETS. All rights reserved.
# Use of this source code is governed by the GNU GENERAL PUBLIC license that can be
# found in the LICENSE file.

#===============================================================================
# G L O B A L   P R O J E C T   C O N F I G U R A T I O N

cmake_minimum_required(VERSION 2.8.3)
project(proc_ura)

if ( NOT CMAKE_BUILD_TYPE )
    set( CMAKE_BUILD_TYPE Debug CACHE STRING
            "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
            FORCE )
endif ( NOT CMAKE_BUILD_TYPE )

set( CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -std=c++11 -Wall" )
set( CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -std=c++11 -O3" )

set(proc_ura_SRC_DIR "src")

#===============================================================================
# C A T K I N   C O N F I G U R A T I O N

find_package(catkin REQUIRED COMPONENTS
        cmake_modules
        interactive_markers
        moveit_core
        moveit_ros_perception
        moveit_ros_planning_interface
        pluginlib
        actionlib
        dynamixel_msgs
        roscpp
        rospy
        tf
        urdf
        xacro
        sensor_msgs
        geometry_msgs
        gazebo_msgs
        gazebo_plugins
        gazebo_ros
        gazebo_ros_control
        std_msgs)

find_package(Boost REQUIRED COMPONENTS system)

# Declare this project as a catkin package and export the necessary build flags
catkin_package(
        INCLUDE_DIRS ${proc_ura_SRC_DIR}
        LIBRARIES
        CATKIN_DEPENDS
        actionlib
        dynamixel_msgs
        roscpp
        rospy
        std_msgs)

#===============================================================================
# I N C L U D E   F I L E S

file(GLOB_RECURSE proc_ura_FILES
        "${proc_ura_SRC_DIR}/*.cc"
        "${proc_ura_SRC_DIR}/*.h")

list(REMOVE_ITEM proc_ura_FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/${proc_ura_SRC_DIR}/${PROJECT_NAME}/main.cc)

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${proc_ura_SRC_DIR}
)

#==========================================================================
# C R E A T E   E X E C U T A B L E

add_executable(${PROJECT_NAME}_node ${proc_ura_SRC_DIR}/${PROJECT_NAME}/main.cc ${proc_ura_FILES})
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_node
        ${catkin_LIBRARIES})