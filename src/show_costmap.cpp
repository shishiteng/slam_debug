#include <stdlib.h>
#include <stdio.h>

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "map_msgs/OccupancyGridUpdate.h"
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

Mat local_image_, global_image_, static_image_,
  local_image_update_,global_image_update_;

void localCostmapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
  int w = msg->info.width;
  int h = msg->info.height;

  Mat image(h, w, CV_8UC1);
  for(int i=0;i<w;i++) {
    for(int j=0;j<h;j++) {
      image.data[i*h+j] = msg->data[i*h+j];
    } 
  }
  local_image_ = image.clone();
}

void localCostmapUpdateCallback(const map_msgs::OccupancyGridUpdate::ConstPtr &msg)
{
  int w = msg->width;
  int h = msg->height;

  Mat image(h, w, CV_8UC1);
  for(int i=0;i<w;i++) {
    for(int j=0;j<h;j++) {
      image.data[i*h+j] = msg->data[i*h+j];
    } 
  }
  local_image_update_ = image.clone();
}

void globalCostmapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
  int w = msg->info.width;
  int h = msg->info.height;
    
  Mat image(h, w, CV_8UC1);
  for(int i=0;i<w;i++) {
    for(int j=0;j<h;j++) {
      image.data[i*h+j] = msg->data[i*h+j];
    } 
  }
  global_image_ = image.clone();
}

void globalCostmapUpdateCallback(const map_msgs::OccupancyGridUpdate::ConstPtr &msg)
{
  int w = msg->width;
  int h = msg->height;

  Mat image(h, w, CV_8UC1);
  for(int i=0;i<w;i++) {
    for(int j=0;j<h;j++) {
      image.data[i*h+j] = msg->data[i*h+j];
    } 
  }
  global_image_update_ = image.clone();
}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
  int w = msg->info.width;
  int h = msg->info.height;
    
  Mat image(h, w, CV_8UC1);
  for(int i=0;i<w;i++) {
    for(int j=0;j<h;j++) {
      image.data[i*h+j] = msg->data[i*h+j];
    } 
  }
  static_image_ = image.clone();
  printf(".");
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "show_costmap");
  ros::NodeHandle nh;

  local_image_ = Mat(100,100,CV_8UC1);
  global_image_ = Mat(100,100,CV_8UC1);
  local_image_update_ = Mat(100,100,CV_8UC1);
  global_image_update_ = Mat(100,100,CV_8UC1);
  static_image_ = Mat(100,100,CV_8UC1);
    
  ros::Subscriber sub0 = nh.subscribe("/move_base/local_costmap/costmap", 1, localCostmapCallback);
  ros::Subscriber sub1 = nh.subscribe("/move_base/global_costmap/costmap", 1, globalCostmapCallback);
  ros::Subscriber sub2 = nh.subscribe("/map", 1, mapCallback);
  ros::Subscriber sub00 = nh.subscribe("/move_base/local_costmap/costmap_updates", 1, localCostmapUpdateCallback);
  ros::Subscriber sub10 = nh.subscribe("/move_base/global_costmap/costmap_updates", 1, globalCostmapUpdateCallback);

  namedWindow("local_costmap", 0);
  namedWindow("global_costmap", 0);
  namedWindow("local_costmap_update", 0);
  namedWindow("global_costmap_update", 0);
  namedWindow("map", 0);
  
  while(ros::ok()) {
    imshow("local_costmap", local_image_);
    imshow("global_costmap", global_image_);
    imshow("local_costmap_update", local_image_update_);
    imshow("global_costmap_update", global_image_update_);
    imshow("map", static_image_);
    waitKey(10);
    
    ros::spinOnce();
  }
  
  return 0;
  
}
