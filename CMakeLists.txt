cmake_minimum_required(VERSION 2.8.3)
project(rescue_robot)
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

if (CATKIN_ENABLE_TESTING)
  set(CMAKE_BUILD_TYPE Debug)
  find_package(rostest REQUIRED)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 --coverage")
  add_rostest_gtest(
   test_rescue_bot
   test/daredevil_test.launch 
   test/main.cpp 
   test/camera_test.cpp 
   test/bot_test.cpp 
   test/sensor_test.cpp 
   src/bot.cpp 
   src/camera.cpp 
   src/sensor.cpp)

  target_link_libraries(
   test_rescue_bot 
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS})
  
  add_dependencies(
    walker
    test_rescue_bot
    ${catkin_EXPORTED_TARGETS}
  )
endif()

