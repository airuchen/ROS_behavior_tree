cmake_minimum_required(VERSION 2.8.3)
project(bt_test_2)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  behaviortree_cpp_v3
  roscpp
  std_msgs
  geometry_msgs
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need (for catkin compiling)
## DEPENDS: system dependencies of this project that dependent projects also need (for system on the computer)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES bt_test
    CATKIN_DEPENDS geometry_msgs behaviortree_cpp_v3 roscpp std_msgs
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(${PROJECT_NAME}_node
   src/bt_turtle.cpp
   src/bt_sequence.cpp
   )

add_executable(tutorial_2_node  src/bt_tutorial_2.cpp)
add_executable(tutorial_3_node  src/bt_tutorial_3.cpp)
add_executable(tutorial_4_node  src/bt_tutorial_4.cpp)
add_executable(tutorial_5_node  src/bt_tutorial_5.cpp)
add_executable(tutorial_6_node  src/bt_tutorial_6.cpp)
add_executable(tutorial_7_node  src/bt_tutorial_7.cpp)
add_executable(elevator_node  src/bt_elevator.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
## set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME bt_turtle_go PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_dependencies(tutorial_2_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(tutorial_3_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(tutorial_4_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(tutorial_5_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(tutorial_6_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(tutorial_7_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(elevator_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
target_link_libraries(tutorial_2_node ${catkin_LIBRARIES})
target_link_libraries(tutorial_3_node ${catkin_LIBRARIES} )
target_link_libraries(tutorial_4_node ${catkin_LIBRARIES} )
target_link_libraries(tutorial_5_node ${catkin_LIBRARIES} )
target_link_libraries(tutorial_6_node ${catkin_LIBRARIES} )
target_link_libraries(tutorial_7_node ${catkin_LIBRARIES} )
target_link_libraries(elevator_node ${catkin_LIBRARIES} )

