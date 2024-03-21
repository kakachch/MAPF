cmake_minimum_required(VERSION 3.0.2)
project(mapf_environment)


find_package(
    catkin REQUIRED COMPONENTS  visualization_msgs)


catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS  visualization_msgs)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(mapf_environment src/grid_environment.cpp src/base_environment.cpp src/orientation.cpp)
add_dependencies(mapf_environment ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mapf_environment ${catkin_LIBRARIES})

install(TARGETS mapf_environment
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING
  PATTERN "*.h")

add_definitions("-Wall -g") 