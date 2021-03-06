#include "qtros.h"
#include <math.h>
#include <caros_common_msgs/Q.h>
#include <caros/common_robwork.h>

#define SUB_CAROS "/caros_universalrobot/caros_serial_device_service_interface/robot_state"

#define SUB_MULTI_UR5E "/robot_state/multirobot/ur5e"


QtROS::QtROS()
{

  ROS_INFO("Connected to roscore");

  // Subscribe to caros robot state
  _sub = _nh.subscribe(SUB_CAROS, 1, &QtROS::stateCallback, this);
  _subMulti = _nh.subscribe(SUB_MULTI_UR5E, 1, &QtROS::stateMultiCallback, this);


  _robot = new caros::SerialDeviceSIProxy(_nh, "caros_universalrobot");

  quitfromgui = false;
}

void QtROS::quitNow()
{
  quitfromgui = true;
}

void QtROS::stateMultiCallback(const caros_control_msgs::RobotState & msg){
    // Extract configuration from RobotState message
    caros_common_msgs::Q conf = msg.q;

    // Convert from ROS msg to Robwork Q
    rw::math::Q conf_rw = caros::toRw(conf);

    // Emit the configuration
    emit newMultiState(conf_rw); 
}


void QtROS::stateCallback(const caros_control_msgs::RobotState & msg)
{
    // Extract configuration from RobotState message
    caros_common_msgs::Q conf = msg.q;

    // Convert from ROS msg to Robwork Q
    rw::math::Q conf_rw = caros::toRw(conf);

    // Emit the configuration
    emit newState(conf_rw);

}

void QtROS::moveHome()
{
    ROS_INFO("Called move home");
    rw::math::Q home = rw::math::Q(6, 0, -M_PI/2.0, 0, -M_PI/2.0, 0, 0);
    _robot->movePtp(home);
}



void QtROS::run()
{
  while(ros::ok() && !quitfromgui)
  {
    ros::spinOnce(); 
    
    // Adjust the sleep to, according to how often you will check ROS for new messages
    ros::Duration(0.1).sleep();
  }
  if (!quitfromgui)
  {
    emit rosQuits();
    ROS_INFO("ROS-Node Terminated\n"); 
  }
}
