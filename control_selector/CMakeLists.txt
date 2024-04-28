cmake_minimum_required(VERSION 2.8.3)
project(control_selector)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS nav_msgs roscpp std_msgs
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(control_selector
             src/control_selector_node.cpp
             src/control.h
             src/dummy_control.h
             src/dummy_control.cpp
             src/voyagercontrol.h
             src/voyagercontrol.cpp
             src/wallfollower.h
             src/wallfollower.cpp
                                )
add_executable(selecting_menu
		src/selecting_menu.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(control_selector
   ${catkin_LIBRARIES}
 )

target_link_libraries(selecting_menu
   ${catkin_LIBRARIES}
)

