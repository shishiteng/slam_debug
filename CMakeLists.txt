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
    rosbag
    tf
    cv_bridge
    dynamic_reconfigure
    )

find_package(OpenCV REQUIRED)
find_package(Pangolin REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/Tutorials.cfg
  #...
)

catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)


add_executable(show_bias
    src/show_bias.cpp
    )
target_link_libraries(show_bias ${catkin_LIBRARIES} ${OpenCV_LIBS}  ${Pangolin_LIBRARIES}) 


# record msckf results,then play the bag file
add_executable(show_msckf_bag
    src/show_msckf_bag.cpp
    )
target_link_libraries(show_msckf_bag ${catkin_LIBRARIES} ${OpenCV_LIBS}  ${Pangolin_LIBRARIES}) 

# test ros dynamic_reconfig
add_executable(dynamic_reconfig
    src/dynamic_reconfig.cpp
    )
target_link_libraries(dynamic_reconfig ${catkin_LIBRARIES} ${OpenCV_LIBS}  ${Pangolin_LIBRARIES})

# show cost of move_base/costmap
add_executable(show_costmap
    src/show_costmap.cpp
    )
target_link_libraries(show_costmap ${catkin_LIBRARIES} ${OpenCV_LIBS}) 
