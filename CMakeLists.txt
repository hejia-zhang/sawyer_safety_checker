cmake_minimum_required(VERSION 2.8.3)
project(sawyer_safety_checker)
set(CMAKE_CXX_STANDARD 11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
             roscpp
             rospy
             moveit_core
             moveit_msgs
             shape_msgs
             message_generation
             sensor_msgs
             message_runtime
             moveit_ros_planning_interface
)

add_service_files(
        FILES
        CollisionChecker.srv
        CollisionPredictor.srv
)

generate_messages(
        DEPENDENCIES
        sensor_msgs
        std_msgs
)

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
)

macro(sawyer_safety_checker T)
    add_executable(${T} ${T}/${T}.cpp)
    target_link_libraries(${T} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
    add_dependencies(${T} ${catkin_EXPORTED_TARGETS})
endmacro()

foreach (dir
        sawyer_safety_checker_cli
        sawyer_safety_checker_srv)
    sawyer_safety_checker(${dir})
endforeach ()




