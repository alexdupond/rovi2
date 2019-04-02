#ifndef CAROS_TEST_SERIAL_DEVICE_SERVICE_INTERFACE_DUMMY_H
#define CAROS_TEST_SERIAL_DEVICE_SERVICE_INTERFACE_DUMMY_H

#include <caros/serial_device_service_interface.h>

#include <string>
#include <stdexcept>

class SerialDeviceServiceInterfaceDummy : public caros::SerialDeviceServiceInterface
{
 public:
  SerialDeviceServiceInterfaceDummy(ros::NodeHandle nodehandle, const bool return_value, const bool cause_error);
  virtual ~SerialDeviceServiceInterfaceDummy();

  const std::string& getMostRecentFunctionCalled() const;

  bool moveLin(const TransformAndSpeedAndAccelerationContainer_t& targets);
  bool movePtp(const QAndSpeedAndAccelerationContainer_t& targets);
  bool movePtpT(const TransformAndSpeedAndAccelerationContainer_t& targets);
  bool moveVelQ(const rw::math::Q& q_vel);
  bool moveVelT(const rw::math::VelocityScrew6D<>& t_vel);
  bool moveServoQ(const rw::math::Q& target, const float time, const float lookahead_time, const float gain);
  bool moveServoUpdate(const rw::math::Q& target);
  bool moveServoStop();
  bool moveStop();

 private:
  bool return_value_;
  bool cause_error_;
  std::string causing_error_msg_;
  std::string most_recent_function_called_;
};

#endif  // CAROS_TEST_SERIAL_DEVICE_SERVICE_INTERFACE_DUMMY_H
