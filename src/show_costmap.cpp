#include <stdlib.h>
#include <stdio.h>

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

Mat local_image_, global_image_;

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


int main(int argc, char **argv)
{
  ros::init(argc, argv, "show_costmap");
  ros::NodeHandle nh;

  local_image_ = Mat(100,100,CV_8UC1);
  global_image_ = Mat(100,100,CV_8UC1);
    
  ros::Subscriber sub0 = nh.subscribe("/move_base/local_costmap/costmap", 1, localCostmapCallback);
  ros::Subscriber sub1 = nh.subscribe("/move_base/global_costmap/costmap", 1, globalCostmapCallback);

  namedWindow("local_costmap", 0);
  namedWindow("global_costmap", 0);
  
  while(ros::ok()) {
    imshow("local_costmap", local_image_);
    imshow("global_costmap", global_image_);
    waitKey(10);
    
    ros::spinOnce();
  }
  
  return 0;
  
}
