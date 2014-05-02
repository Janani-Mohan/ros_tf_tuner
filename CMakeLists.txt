cmake_minimum_required(VERSION 2.8.3)
project(ros_tf_tuner)

find_package(catkin REQUIRED
    COMPONENTS
      roscpp
      geometry_msgs
      dynamic_reconfigure
      tf
     )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
generate_dynamic_reconfigure_options(
    cfg/TransformTuner.cfg
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}
#  DEPENDS Eigen
  CATKIN_DEPENDS
    dynamic_reconfigure
    roscpp
    geometry_msgs
    dynamic_reconfigure
    tf
)
include_directories(${catkin_INCLUDE_DIRS})
add_executable(${PROJECT_NAME} src/transform_tuner_node.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
