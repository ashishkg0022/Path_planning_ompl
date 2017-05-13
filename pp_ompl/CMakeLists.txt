cmake_minimum_required(VERSION 2.8.3)
project(pp_ompl)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  graph_msgs
  roscpp
  roslib
  visualization_msgs
  cv_bridge
  image_transport
)

find_package(OMPL REQUIRED)
find_package(Boost)
include_directories(${Boost_INCLUDE_DIRS} ${OMPL_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS
    graph_msgs
    roscpp
    roslib
    visualization_msgs
  INCLUDE_DIRS
    include
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
  include
)




# Executable for Demos and Testing
add_executable(${PROJECT_NAME}_rrt_demo
  src/rrt_demo.cpp
)
set_target_properties(${PROJECT_NAME}_rrt_demo
  PROPERTIES OUTPUT_NAME rrt_demo PREFIX ""
)
target_link_libraries(${PROJECT_NAME}_rrt_demo
  ${OMPL_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}

)

## Install

# Install libraries

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# Install shared resources
install(DIRECTORY launch    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY resources DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# Install executables
install(TARGETS
    ${PROJECT_NAME}_rrt_demo
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
