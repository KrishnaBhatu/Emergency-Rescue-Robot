cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_walker)
add_compile_options(-std=c++11)
find_package( OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  move_base_msgs
  nav_msgs
  tf
  cv_bridge 
  image_transport
  std_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(walker src/main.cpp src/bot.cpp src/camera.cpp src/sensor.cpp)
target_link_libraries(walker
  ${catkin_LIBRARIES}
)
target_link_libraries( walker ${OpenCV_LIBS} )
