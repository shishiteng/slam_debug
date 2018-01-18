cmake_minimum_required(VERSION 2.8.3)
project(slam_debug)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

#add_definitions(-D NOISE3)
#add_definitions(-D FIXBIAS)
      
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    sensor_msgs
    tf
    cv_bridge
    )

find_package(OpenCV REQUIRED)

find_package(Pangolin REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

add_executable(show_bias
    src/show_bias.cpp
    )

target_link_libraries(show_bias ${catkin_LIBRARIES} ${OpenCV_LIBS}  ${Pangolin_LIBRARIES}) 

