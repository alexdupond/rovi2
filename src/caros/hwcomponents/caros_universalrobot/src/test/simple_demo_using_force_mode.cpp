#include <caros/serial_device_si_proxy.h>
#include <ros/ros.h>
#include <thread>  // NOLINT(build/c++11)
#include <chrono>  // NOLINT(build/c++11)

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "simple_caros_universalrobot_demo_using_force_mode");
  ros::NodeHandle nh;

  caros::SerialDeviceSIProxy sd_sip(nh, "caros_universalrobot");

  // Test force mode
  rw::math::Vector3D<> pos(0, 0, 0);
  rw::math::Transform3D<> task_frame(pos);
  rw::math::Q selection_vector(6, 0, 0, 1, 0, 0, 0);
  rw::math::Wrench6D<> wrench_down(0, 0, -20, 0, 0, 0);
  rw::math::Wrench6D<> wrench_up(0, 0, 20, 0, 0, 0);
  int force_type = 2;
  rw::math::Q limits(6, 2, 2, 1.5, 1, 1, 1);

  sd_sip.moveForceModeStart(task_frame, selection_vector, wrench_down, force_type, limits);
  std::cout << std::endl << "Going Down!" << std::endl;
  std::this_thread::sleep_for(std::chrono::seconds(1));
  std::cout << std::endl << "Going Up!" << std::endl << std::endl;
  sd_sip.moveForceModeUpdate(wrench_up);
  std::this_thread::sleep_for(std::chrono::seconds(1));
  sd_sip.moveForceModeStop();

  return 0;
}
