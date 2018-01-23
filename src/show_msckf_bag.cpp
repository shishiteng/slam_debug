#include <stdlib.h>
#include <stdio.h>

#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/Image.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <msckf/CameraMeasurement.h>

#define PREV ('p') //prev
#define NEXT ('n') //next
#define STEP ('s') //step by step
#define CONTINUE ('c') //continue
#define SKIP (' ') //space
#define RESTART ('r') //restart
#define EXIT (27) //esc

using namespace std;
using namespace cv;

ros::Publisher path_pub;
ros::Publisher odom_pub;
ros::Publisher window_pub;

int index_ = 0;
int length = 0;
std::vector<rosbag::View::iterator> vIter;

map<double,sensor_msgs::CompressedImage::ConstPtr> image_msgs;
map<double,msckf::CameraMeasurementConstPtr> tracking_features_msgs;
map<double,msckf::CameraMeasurementConstPtr> lost_features_msgs;
map<double,nav_msgs::Odometry::ConstPtr> odom_msgs;
map<double,nav_msgs::Path::ConstPtr> path_msgs;
map<double,nav_msgs::Path::ConstPtr> window_msgs;
map<double,nav_msgs::Path::ConstPtr> marg_frame_msgs;

void printHelp()
{
  printf("-------command----\n");
  printf("  prev:     p\n");
  printf("  next:     n\n");
  printf("  step:     s\n");
  printf("  skip:     space\n");
  printf("  continue: c\n");
  printf("  restart:  r\n");
  printf("  exit:     esc\n");
  printf("------------------\n");
}


void getAllMessages(rosbag::View *pView)
{
  //rosbag::View view(bag, rosbag::TopicQuery(topics));
  for(rosbag::View::iterator iter=pView->begin();iter!=pView->end();iter++) {
    rosbag::MessageInstance m = (*iter);
    
    //current debug image
    //sensor_msgs::Image::ConstPtr image_msg = m.instantiate<sensor_msgs::Image>();
    sensor_msgs::CompressedImage::ConstPtr image_msg = m.instantiate<sensor_msgs::CompressedImage>();
    if(image_msg != NULL) {
      double timestamp = image_msg->header.stamp.toSec();
      //cv_bridge::CvImage cvi;
      //cvi.header = image_msg->header;
      //cvi.encoding = "bgr8";
      //cvi.image = cv::imdecode(Mat(image_msg->data),1).clone();
      //sensor_msgs::Image::Ptr raw_image_msg = cvi.toImageMsg();
      image_msgs.insert(make_pair(timestamp,image_msg));
    }

    //features
    msckf::CameraMeasurementConstPtr features_msg = m.instantiate<msckf::CameraMeasurement>();
    if(features_msg != NULL) {
      double timestamp = features_msg->header.stamp.toSec();
      if(features_msg->header.frame_id == "features") {
	//tracking features
	tracking_features_msgs.insert(make_pair(timestamp,features_msg));
	//tracking_features_msgs
	//tracking_features_msgs.push_back(features_msg);
      }else if(features_msg->header.frame_id == "lost_features") {
	//lost features
	lost_features_msgs.insert(make_pair(timestamp,features_msg));
	//lost_features_msgs.push_back(features_msg);
      }
    }

    //current odometry
    nav_msgs::Odometry::ConstPtr odom_msg = m.instantiate<nav_msgs::Odometry>();
    if(odom_msg != NULL) {
      double timestamp = odom_msg->header.stamp.toSec();
      odom_msgs.insert(make_pair(timestamp,odom_msg));
      //odom_msgs.push_back(odom_msg);
    }

    //current path
    nav_msgs::Path::ConstPtr path_msg = m.instantiate<nav_msgs::Path>();
    if(path_msg != NULL) {
      double timestamp = path_msg->header.stamp.toSec();
      path_msgs.insert(make_pair(timestamp,path_msg));
      //path_msgs.push_back(path_msg);
    }
  }
}


void processMessage(int nIndex)
{
  //get timestamp
  map<double,sensor_msgs::CompressedImage::ConstPtr>::iterator iter = image_msgs.begin();
  advance(iter,nIndex);
  double timestamp = (*iter).first;
  
  //current debug image
  sensor_msgs::CompressedImage::ConstPtr image_msg = image_msgs[timestamp];
  if(image_msg != NULL) {
    char str[128];
    //Mat show_img = cv_bridge::toCvShare(image_msg, "bgr8")->image;
    Mat show_img = cv::imdecode(Mat(image_msg->data), 1);
    sprintf(str,"%d of %d",index_,length);
    putText(show_img, str, Point2f(10,50), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0));
    sprintf(str,"time: %lf",image_msg->header.stamp.toSec());
    putText(show_img, str, Point2f(10,65), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0));
    imshow("stereo_debug", show_img);
    waitKey(1);
  }

  //current lost features
  vector<unsigned long long int> lost_features_id;
  msckf::CameraMeasurementConstPtr lost_features_msg = lost_features_msgs[timestamp];
  if(lost_features_msg != NULL) {
    if(lost_features_msg->header.frame_id == "lost_features") {
      for (const auto& feature : lost_features_msg->features) {
	//draw the lost features in last frame
	//printf("%lf",(double)lost_features_msg->header.stamp.toSec());
	//cout<<" current lost feature:"<<feature.id<<endl;
	//cout<<feature.du0<<","<<feature.dv0<<endl;
	lost_features_id.push_back(feature.id);
      }
    }
  }

  //current tracking features
  msckf::CameraMeasurementConstPtr tracking_features_msg = tracking_features_msgs[timestamp];
  if(tracking_features_msg != NULL) {
    if(tracking_features_msg->header.frame_id == "features") {
      for (const auto& feature : tracking_features_msg->features) {
	//draw the lost features in last frame
	//cout<<"lost id:"<<feature.id<<"    ";
	//cout<<feature.du0<<","<<feature.dv0<<endl;
	//tracking_features_id.push_back(feature.id);
      }
    }
  }


  //prev
  if(nIndex > 0 && nIndex < length) {
    map<double,sensor_msgs::CompressedImage::ConstPtr>::iterator iter1 = image_msgs.begin();
    advance(iter1,nIndex-1);
    double prev_timestamp = (*iter1).first;
  
    //prev tracking features
    map<unsigned long long int,Point2f> prev_tracking_pts;
    msckf::CameraMeasurementConstPtr prev_tracking_features_msg = tracking_features_msgs[prev_timestamp];
    if(prev_tracking_features_msg != NULL) {
      if(prev_tracking_features_msg->header.frame_id == "features") {
	for (const auto& feature : prev_tracking_features_msg->features) {
	  //printf("%lf",(double)prev_tracking_features_msg->header.stamp.toSec());
	  //cout<<" prev tracking feature:"<<feature.id<<" "<<Point2f(feature.du0,feature.dv0)<<endl;
	  prev_tracking_pts.insert(make_pair(feature.id,Point2f(feature.du0,feature.dv0)));
	}
      }
    }

    //prev debug message
    sensor_msgs::CompressedImage::ConstPtr prev_image_msg = image_msgs[prev_timestamp];
    if(prev_image_msg != NULL) {
      char str[16];
      Mat show_img = cv::imdecode(Mat(prev_image_msg->data),1);
      //Mat show_img = cv_bridge::toCvShare(prev_image_msg, "bgr8")->image.clone();
      sprintf(str,"%d of %d",index_-1,length);
      putText(show_img, str, Point2f(10,50), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0));

      for (const auto& feature_id : lost_features_id) {
	map<unsigned long long int, Point2f>::iterator it;
	it = prev_tracking_pts.find(feature_id);
	if(it != prev_tracking_pts.end()) {
	  Point2f pt = it->second;
	  circle(show_img, pt, 5, Scalar(0,0,255),3);
	}
      }
      imshow("stereo_debug_prev", show_img);
      waitKey(1);
    }
  }

  
  //current odometry
  nav_msgs::Odometry::ConstPtr odom_msg = odom_msgs[timestamp];
  if(odom_msg != NULL) {
    //cout<<"odom:"<<odom_msg->header.stamp<<endl;
    odom_pub.publish(*odom_msg);
  }

  //current path
  nav_msgs::Path::ConstPtr path_msg = path_msgs[timestamp];
  if(path_msg != NULL) {
    //cout<<"path:"<<path_msg->header.stamp<<endl;
    path_pub.publish(*path_msg);
  }

  //current sliding window
  nav_msgs::Path::ConstPtr window_msg = window_msgs[timestamp];
  if(window_msg != NULL) {
    //cout<<"path:"<<path_msg->header.stamp<<endl;
    window_pub.publish(*window_msg);
  }
}


int main(int argc, char **argv)
{
  //topics:
  //track features
  //new features
  //updated features
  //debug stereo image
  //imu bias
  //odometry
  //path

  ros::init(argc, argv, "show_msckf_bag");
  ros::NodeHandle nh;

  printHelp();
  
  // register publisher
  path_pub = nh.advertise<nav_msgs::Path>("path", 1);
  odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 1);

#ifdef LIVE
  // open bag
  rosbag::Bag bag;
  bag.open("/home/sst/catkin_ws2/test_live.bag", rosbag::bagmode::Read);

  std::vector<std::string> topics;
  topics.push_back(std::string("/msckf/debug_stereo_image"));
  topics.push_back(std::string("/msckf/path"));
  topics.push_back(std::string("/msckf/odom"));
  topics.push_back(std::string("/msckf/imu_bias"));
  topics.push_back(std::string("/msckf/features"));
  topics.push_back(std::string("/msckf/lost_features"));
#else
    // open bag
  rosbag::Bag bag;
  bag.open("/home/sst/catkin_ws2/test_dataset.bag", rosbag::bagmode::Read);

  std::vector<std::string> topics;
  topics.push_back(std::string("/image_processor/debug_stereo_image/compressed"));
  topics.push_back(std::string("/image_processor/features"));
  topics.push_back(std::string("/MsckfVio/path"));
  topics.push_back(std::string("/MsckfVio/odom"));
  topics.push_back(std::string("/MsckfVio/sliding_window"));
  topics.push_back(std::string("/MsckfVio/marginalized_frame"));
  topics.push_back(std::string("/MsckfVio/imu_bias"));
  topics.push_back(std::string("/MsckfVio/lost_features"));
#endif
  

  rosbag::View view(bag, rosbag::TopicQuery(topics));
  getAllMessages(&view);

  cout<<" image_msgs:"<< image_msgs.size()<<endl;
  cout<<" tracking_features_msgs:"<< tracking_features_msgs.size()<<endl;
  cout<<" lost_features_msgs:"<< lost_features_msgs.size()<<endl;
  cout<<" odom_msgs:"<< odom_msgs.size()<<endl;
  cout<<" path_msgs:"<< path_msgs.size()<<endl;
  cout<<" sliding_window_msgs:"<< window_msgs.size()<<endl;
  cout<<" marg_frame_msgs:"<< marg_frame_msgs.size()<<endl;

  namedWindow("stereo_debug");
  namedWindow("stereo_debug_prev");
  
  int steps = 0;
  int waitNum = 0;
  length = image_msgs.size();
  //Mat console_img(100,100,CV_8UC1);
  
  while(ros::ok()) {
    //imshow("console",console_img);
    int key = waitKey(waitNum);
    //cout<<"key:"<<key<<endl;
    switch(key) {
      case PREV: {
	//iter = std::find(iter,--);
	//--iter;
	index_--;
	break;
      }
      case NEXT: {
	index_++;
	break;
      }
      case SKIP: {
	cout<<"enter steps:";
	scanf("%d",&steps);
	index_ += steps;
	//iter = std::find(iter,steps);
	break;
      }
      case STEP: {
	waitNum = 0;
	break;
      }
      case CONTINUE: {
	waitNum = 20;
	break;
      }
      case RESTART: {
	index_ = 0;
	break;
      }
      case EXIT: {
	bag.close();
	return -1;
      }

      default: {
	index_++;
	break;
      }
    }

    
    //cout<<"["<<index_<<" of "<<length<<"]"<<endl;
    if( !(index_ >= 0 && index_ <length ) ) {
      cerr<<"step invalid:"<< index_ <<endl;
      continue;
    }

    //void processMessage(int nIndex)
    processMessage(index_);
  }
 
  bag.close();

  return 0;
}
