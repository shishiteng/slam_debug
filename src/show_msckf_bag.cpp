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
#define JUMP ('j') //jump
#define RESTART ('r') //restart
#define EXIT (27) //esc

using namespace std;
using namespace cv;

ros::Publisher path_pub;
ros::Publisher odom_pub;
ros::Publisher sliding_window_pub;
ros::Publisher marg_frames_pub;

int index_ = 0;
int length = 0;
std::vector<rosbag::View::iterator> vIter;

string TOPIC_DEBUG_IMAGE = "/image_processor/debug_stereo_image/compressed";
string TOPIC_TRACKING_FEATURES= "/image_processor/features";
string TOPIC_USED_FEATURES = "/MsckfVio/used_features";
string TOPIC_LOST_FEATURES = "/MsckfVio/lost_features";
string TOPIC_ODOMETRY = "/MsckfVio/odom";
string TOPIC_PATH = "/MsckfVio/path";
string TOPIC_SLIDING_WINDOW = "/MsckfVio/sliding_window";
string TOPIC_MARGINALIZED_FRAME = "/MsckfVio/marginalized_frame";

map<double,sensor_msgs::CompressedImage::ConstPtr> image_msgs;
map<double,msckf::CameraMeasurementConstPtr> current_features_msgs;
map<double,msckf::CameraMeasurementConstPtr> used_features_msgs;
map<double,msckf::CameraMeasurementConstPtr> lost_features_msgs;
map<double,nav_msgs::Odometry::ConstPtr> odom_msgs;
map<double,nav_msgs::Path::ConstPtr> path_msgs;
map<double,nav_msgs::Path::ConstPtr> sliding_window_msgs;
map<double,nav_msgs::Path::ConstPtr> marg_frame_msgs;

void printHelp()
{
  printf("-------command----\n");
  printf("  prev:     p\n");
  printf("  next:     n\n");
  printf("  step:     s\n");
  printf("  jump:     j\n");
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
    string topic_name = m.getTopic();
    double timestamp;
    
    //current debug image
    //sensor_msgs::Image::ConstPtr image_msg = m.instantiate<sensor_msgs::Image>();
    sensor_msgs::CompressedImage::ConstPtr image_msg = m.instantiate<sensor_msgs::CompressedImage>();
    if(image_msg != NULL) {
      timestamp = image_msg->header.stamp.toSec();
      if( topic_name == TOPIC_DEBUG_IMAGE )
	image_msgs.insert(make_pair(timestamp,image_msg));
    }

    //features
    msckf::CameraMeasurementConstPtr features_msg = m.instantiate<msckf::CameraMeasurement>();
    if(features_msg != NULL) {
      timestamp = features_msg->header.stamp.toSec();
      if( topic_name == TOPIC_TRACKING_FEATURES ) {
	current_features_msgs.insert(make_pair(timestamp,features_msg));
      }else if( topic_name == TOPIC_LOST_FEATURES ) {
	lost_features_msgs.insert(make_pair(timestamp,features_msg));
      }else if( topic_name == TOPIC_USED_FEATURES) {
	used_features_msgs.insert(make_pair(timestamp,features_msg));
      }
    }

    //current odometry
    nav_msgs::Odometry::ConstPtr odom_msg = m.instantiate<nav_msgs::Odometry>();
    if(odom_msg != NULL) {
      timestamp = odom_msg->header.stamp.toSec();
      if( topic_name == TOPIC_ODOMETRY ) {
	odom_msgs.insert(make_pair(timestamp,odom_msg));
      }
    }

    //current path
    nav_msgs::Path::ConstPtr path_msg = m.instantiate<nav_msgs::Path>();
    if(path_msg != NULL) {
      timestamp = path_msg->header.stamp.toSec();
      if( topic_name == TOPIC_PATH )
	path_msgs.insert(make_pair(timestamp,path_msg));
      else if( topic_name == TOPIC_SLIDING_WINDOW )
	sliding_window_msgs.insert(make_pair(timestamp,path_msg));
      else if( topic_name == TOPIC_MARGINALIZED_FRAME )
	marg_frame_msgs.insert(make_pair(timestamp,path_msg));
    }
  }
}


void processMessage(int nIndex)
{
  //get timestamp,通过时间戳访问各种消息
  map<double,sensor_msgs::CompressedImage::ConstPtr>::iterator iter = image_msgs.begin();
  advance(iter,nIndex);
  double timestamp = (*iter).first;

  sensor_msgs::CompressedImage::ConstPtr image_msg = image_msgs[timestamp];
  msckf::CameraMeasurementConstPtr lost_features_msg = lost_features_msgs[timestamp];
  msckf::CameraMeasurementConstPtr current_features_msg = current_features_msgs[timestamp];
  msckf::CameraMeasurementConstPtr used_features_msg = used_features_msgs[timestamp];
  nav_msgs::Path::ConstPtr sliding_window_msg = sliding_window_msgs[timestamp];
  nav_msgs::Odometry::ConstPtr odom_msg = odom_msgs[timestamp];
  nav_msgs::Path::ConstPtr path_msg = path_msgs[timestamp];
  nav_msgs::Path::ConstPtr marg_msg = marg_frame_msgs[timestamp];
 
  /*
   *  第一部分：当前帧的处理
   *      1.画出当前帧 
   *      2.画出当前帧中用来update的点
   */
  //current debug image
  if(image_msg != NULL) {
    // 1. 画出debug_image
    char str[128];
    //Mat show_img = cv_bridge::toCvShare(image_msg, "bgr8")->image;
    Mat show_img = cv::imdecode(Mat(image_msg->data), 1);
    sprintf(str,"%d of %d",index_,length);
    putText(show_img, str, Point2f(10,50), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0));
    sprintf(str,"time: %lf",image_msg->header.stamp.toSec());
    putText(show_img, str, Point2f(10,65), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0));
    int n_used=0, n_lost=0;
    if(used_features_msg)
      n_used = used_features_msg->features.size();
    if(lost_features_msg)
      n_lost = lost_features_msg->features.size();
    sprintf(str,"features used/lost: %d/%d",n_used,n_lost);
    putText(show_img, str, Point2f(10,80), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0));
    
    // 2.1 在当前的debug_image中画出used features
    //current tracking features,得到一个map<feature_id，point坐标>
    map<unsigned long long int,Point2f> current_pts;
    if(current_features_msg != NULL) {
      for (const auto& feature : current_features_msg->features) {
	current_pts.insert(make_pair(feature.id,Point2f(feature.du0,feature.dv0)));
      }
    }

    // 2.2 找到当前帧用来update的点
    //current used features
    vector<unsigned long long int> used_features_id;
    if(used_features_msg != NULL) {
      for (const auto& feature : used_features_msg->features) {
	used_features_id.push_back(feature.id);
      }
    }

    // 2.3 找到used features的图像坐标，画出来
    //current used features
    for (const auto& feature_id : used_features_id) {
      map<unsigned long long int, Point2f>::iterator it;
      it = current_pts.find(feature_id);
      if(it != current_pts.end()) {
	Point2f pt = it->second;
	circle(show_img, pt, 5, Scalar(255,0,0),3);
      }
    }

    imshow("stereo_debug", show_img);
    waitKey(1);

    // 2.3 把当前帧和sliding window中帧的共视关系show出来
    if(sliding_window_msg != NULL) {
      map<double,int>covisible_features;
      for (const auto& pose : sliding_window_msg->poses) {
	int n_covisible = 0;
	double t = pose.header.stamp.toSec();
	msckf::CameraMeasurementConstPtr window_features_msg = current_features_msgs[t];
	if(window_features_msg != NULL) {
	  for (const auto& feature_ : window_features_msg->features) {
	    for (const auto& feature : current_features_msg->features) {
	      if(feature.id == feature_.id) n_covisible++;
	    }
	  }
	  covisible_features.insert(make_pair(t,n_covisible));
	}
      }
      printf("\n---------sliding window共视关系------\n");
      printf("index  timestamp           covisible\n");
      //for(int i=0;i<covisible_features.size();i++) {
      int i=0;
      map<double,int>::iterator it;
      for(it=covisible_features.begin();it!=covisible_features.end();it++,i++) {
	char str[256];
	sprintf(str, "%3d    %lf     %d",i , it->first, it->second);
	printf("%s\n",str);
      }
      printf(" cur   %lf     %d\n",timestamp, current_features_msg->features.size());
    }
      
  }



  /*
   *  第二部分：前一帧的处理
   *      1.画出在当前帧丢失、并用来update的点
   */
  
  // 前一帧的debug_image
  if(nIndex > 0 && nIndex < length) {
    map<double,sensor_msgs::CompressedImage::ConstPtr>::iterator iter1 = image_msgs.begin();
    advance(iter1,nIndex-1);
    double prev_timestamp = (*iter1).first;

    //prev debug message
    sensor_msgs::CompressedImage::ConstPtr prev_image_msg = image_msgs[prev_timestamp];
    if(prev_image_msg != NULL) {
      // 1.前一帧的图像
      char str[16];
      Mat show_img = cv::imdecode(Mat(prev_image_msg->data),1);
      //Mat show_img = cv_bridge::toCvShare(prev_image_msg, "bgr8")->image.clone();
      sprintf(str,"%d of %d",index_-1,length);
      putText(show_img, str, Point2f(10,50), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0));

      // 2.找到前一帧图像中所有的点，做一个map<feature_id,Point>
      //prev tracking features
      map<unsigned long long int,Point2f> prev_pts;
      msckf::CameraMeasurementConstPtr prev_features_msg = current_features_msgs[prev_timestamp];
      if(prev_features_msg != NULL) {
	for (const auto& feature : prev_features_msg->features) {
	  prev_pts.insert(make_pair(feature.id,Point2f(feature.du0,feature.dv0)));
	}
      }

      // 3.找到从前一帧到当前帧跟踪丢失的点
      //current lost features
      vector<unsigned long long int> lost_features_id;
      if(lost_features_msg != NULL) {
	for (const auto& feature : lost_features_msg->features) {
	  lost_features_id.push_back(feature.id);
	}
      }


      // 4.把当前帧丢失的点在前一帧中画出来
      for (const auto& feature_id : lost_features_id) {
	map<unsigned long long int, Point2f>::iterator it;
	it = prev_pts.find(feature_id);
	if(it != prev_pts.end()) {
	  Point2f pt = it->second;
	  circle(show_img, pt, 5, Scalar(0,0,255),3);
	}
      }
      imshow("stereo_debug_prev", show_img);
      waitKey(1);
    }
  }


  /*
   * 其它message
   *
   */
  if(odom_msg != NULL) {
    odom_pub.publish(*odom_msg);
  }

  if(path_msg != NULL) {
    path_pub.publish(*path_msg);
  }

  if(sliding_window_msg != NULL) {
    sliding_window_pub.publish(*sliding_window_msg);
  }

  // marginalized frame
  if(marg_msg != NULL) {
    marg_frames_pub.publish(*marg_msg);
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

  if(NULL == argv[1]) {
    cout<<"parameter invalid,you should used command:"<<endl;
    cout<<"  rosrun slam_debug show_msckf_bag [bag file path]"<<endl;
    return -1;
  }
    
  printHelp();
  
  // register publisher
  path_pub = nh.advertise<nav_msgs::Path>("path", 1);
  odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 1);
  sliding_window_pub = nh.advertise<nav_msgs::Path>("sliding_window", 1);
  marg_frames_pub = nh.advertise<nav_msgs::Path>("maginalized_frame", 1);

  // open bag
  rosbag::Bag bag;
  cout<<"opening bag file...";
  bag.open(argv[1], rosbag::bagmode::Read);
  cout<<"done"<<endl;

  // query topics
  std::vector<std::string> topics;
  topics.push_back(TOPIC_DEBUG_IMAGE);
  topics.push_back(TOPIC_USED_FEATURES);
  topics.push_back(TOPIC_LOST_FEATURES);
  topics.push_back(TOPIC_TRACKING_FEATURES);
  topics.push_back(TOPIC_PATH);
  topics.push_back(TOPIC_ODOMETRY);
  topics.push_back(TOPIC_SLIDING_WINDOW);
  topics.push_back(TOPIC_MARGINALIZED_FRAME);
  
  rosbag::View view(bag, rosbag::TopicQuery(topics));
  cout<<"quering topics...";
  getAllMessages(&view);
  cout<<"done"<<endl;

  cout<<" | debug_image_msgs:"<< image_msgs.size()<<endl;
  cout<<" | current_features_msgs:"<< current_features_msgs.size()<<endl;
  cout<<" | lost_features_msgs:"<< lost_features_msgs.size()<<endl;
  cout<<" | used_features_msgs:"<< used_features_msgs.size()<<endl;
  cout<<" | odom_msgs:"<< odom_msgs.size()<<endl;
  cout<<" | path_msgs:"<< path_msgs.size()<<endl;
  cout<<" | sliding_window_msgs:"<< sliding_window_msgs.size()<<endl;
  cout<<" | marg_frame_msgs:"<< marg_frame_msgs.size()<<endl;

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
      case JUMP: {
	cout<<"jump to:";
	scanf("%d",&steps);
	index_ = steps;
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
