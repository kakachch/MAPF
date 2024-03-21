cmake_minimum_required(VERSION 3.0.2)
project(mapf_planner)

find_package(
  catkin REQUIRED
  COMPONENTS nav_msgs
             mapf_msgs
             costmap_2d
             pluginlib
             base_local_planner
             nav_core
             mapf_environment)

find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS  nav_msgs
                  mapf_msgs
                  costmap_2d
                  pluginlib
                  base_local_planner
                  nav_core
                  mapf_environment)
                  
file(GLOB SOURCES "src/*.cpp")

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_library(${PROJECT_NAME} ${SOURCES})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${catkin_LIBRARIES})

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(FILES mapf_planner_plugin.xml blp_plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING
  PATTERN "*.h")

add_definitions("-Wall -g") 