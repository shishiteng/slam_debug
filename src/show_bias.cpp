#include <stdlib.h>
#include <stdio.h>

#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"

#include <pangolin/pangolin.h>

pangolin::DataLog *pLog;
pangolin::Plotter *pPlotter;

void imuBiasCallback(const sensor_msgs::PointCloud::ConstPtr &msg)
{
  // Default hooks for exiting (Esc) and fullscreen (tab).
  if( !pangolin::ShouldQuit()) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    float ba[3],bg[3];
    for(int i=0;i<3;i++) {
      ba[i] = msg->channels[0].values[i];
      bg[i] = msg->channels[1].values[i];
    }
      
    pPlotter->Activate();
    pLog->Log(ba[0],ba[1],ba[2],bg[0],bg[1],bg[2]);

    // Render graph, Swap frames and Process Events
    pangolin::FinishFrame();
  }

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "show_bias");
  ros::NodeHandle n;
      
  // Create OpenGL window in single line
  pangolin::CreateWindowAndBind("bias",800,600);
  pangolin::DataLog log;  

  // Optionally add named labels
  std::vector<std::string> labels;
  labels.push_back(std::string("bias_accl_x"));
  labels.push_back(std::string("bias_accl_y"));
  labels.push_back(std::string("bias_accl_z"));
  labels.push_back(std::string("bias_gyro_x"));
  labels.push_back(std::string("bias_gyro_y"));
  labels.push_back(std::string("bias_gyro_z"));
  log.SetLabels(labels);

  // OpenGL 'view' of data. We might have many views of the same data.
  pangolin::Plotter plotter(&log,
			    0,100,
			    -0.3,0.3,
			    10,0.0001); //x:s y:0.001
  plotter.SetBounds(0.0, 1.0, 0.0, 1.0);
  plotter.Track("$i");
  pangolin::DisplayBase().AddDisplay(plotter);

  pLog = &log;
  pPlotter = &plotter;
  
  {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    pLog->Log(0,0,0,0,0,0);
    pangolin::FinishFrame();
  }


  
  ros::Subscriber sub = n.subscribe("/msckf/imu_bias", 1000, imuBiasCallback);

  ros::spin();

  return 0;
}
