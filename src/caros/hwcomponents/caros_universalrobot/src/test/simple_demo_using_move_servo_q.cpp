#include <caros/serial_device_si_proxy.h>
#include <ros/ros.h>
#include <thread>  // NOLINT(build/c++11)
#include <chrono>  // NOLINT(build/c++11)

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "simple_caros_universalrobot_demo_using_move_servo_q");
  ros::NodeHandle nh;

  caros::SerialDeviceSIProxy sd_sip(nh, "caros_universalrobot");

  // Test servoJ
  double time = 0.3;
  double lookahead_time = 0.1;
  double gain = 300;
  rw::math::Q joint_q1(6, -1.54, -1.83, -2.28, -0.59, 1.60, 0.023);
  rw::math::Q joint_q2(6, -0.69, -2.37, -1.79, -0.37, 1.93, 0.87);

  sd_sip.moveServoQ(joint_q1, time, lookahead_time, gain);
  for (unsigned int i=0; i<30; i++)
  {
    sd_sip.moveServoUpdate(joint_q1);
    std::this_thread::sleep_for(std::chrono::milliseconds(280));
    sd_sip.moveServoUpdate(joint_q2);
    std::this_thread::sleep_for(std::chrono::milliseconds(280));
  }
  sd_sip.moveServoStop();

  return 0;
}
