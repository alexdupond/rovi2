#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
//#include <cv_bridge/cv_bridge.h>

// void pcl_callback()
// {

// }

int main(int argc, char **argv)
{

  ros::init(argc, argv, "cam2tcp_node");

  ros::NodeHandle nh;
  ros::Publisher cam2tcp_pub = nh.advertise<std_msgs::String>("cam2tcp", 1);
  // ros::Subscriber pointcloud_sub = nh.subscribe("cam_topic", 1, pcl_callback);
  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}