cmake_minimum_required(VERSION 2.8.3)
project(drone_cv)

find_package(catkin REQUIRED COMPONENTS
  ardrone_autonomy
  roscpp
  rospy
  image_transport 
  cv_bridge
  sensor_msgs
  cv_bridge
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES drone
  CATKIN_DEPENDS 
    ardrone_autonomy 
    roscpp 
    rospy 
    image_transport 
    cv_bridge
    sensor_msgs
    cv_bridge
    #tum_simulator
    #tum_ardrone
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node 
	src/main.cpp
)

add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

