#include <caros/serial_device_si_proxy.h>
#include <ros/ros.h>

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "simple_caros_universalrobot_demo_using_move_ptp");
  ros::NodeHandle nh;

  caros::SerialDeviceSIProxy sd_sip(nh, "caros_universalrobot");

  // Obtain current joint configuration of robot
  ros::Time current_timestamp = ros::Time::now();
  ros::Time obtained_timestamp = sd_sip.getTimeStamp();
  while (current_timestamp > obtained_timestamp)
  {
    ros::Duration(0.1).sleep();  // In seconds
    ros::spinOnce();
    obtained_timestamp = sd_sip.getTimeStamp();
  }

  const rw::math::Q current_q = sd_sip.getQ();

  // Define a goal configuration which is current configurations plus a q_change
  const double q_change = 0.2;
  rw::math::Q goal_q = current_q + rw::math::Q(current_q.size(), q_change);

  ROS_INFO_STREAM("Moving to '" << goal_q << "'.");
  bool ret = false;
  ret = sd_sip.movePtp(goal_q);
  if (!ret)
  {
    ROS_ERROR_STREAM("The serial device didn't acknowledge the movePtp command.");
  }

  return 0;
}
