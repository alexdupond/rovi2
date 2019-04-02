#include <caros/serial_device_si_proxy.h>

#include <caros/serial_device_service_interface.h>
#include <caros/common.h>
#include <caros/common_robwork.h>
#include <caros_common_msgs/EmptySrv.h>
#include <caros_control_msgs/SerialDeviceMoveLin.h>
#include <caros_control_msgs/SerialDeviceMovePtp.h>
#include <caros_control_msgs/SerialDeviceMovePtpT.h>
#include <caros_control_msgs/SerialDeviceMoveVelQ.h>
#include <caros_control_msgs/SerialDeviceMoveVelT.h>
#include <caros_control_msgs/SerialDeviceMoveServoQ.h>
#include <caros_control_msgs/SerialDeviceMoveServoUpdate.h>
#include <caros_control_msgs/SerialDeviceMoveForceModeStart.h>
#include <caros_control_msgs/SerialDeviceMoveForceModeUpdate.h>

#define SPEED_MIN 0.0f
#define SPEED_MAX 1.0f
#define ACCEL_MIN 0.0f
#define ACCEL_MAX 2.0f
#define BLEND_MIN 0.0f
#define BLEND_MAX 2.0f
#define SERVO_LOOKAHEAD_TIME_MIN 0.03f
#define SERVO_LOOKAHEAD_TIME_MAX 0.2f
#define SERVO_GAIN_MIN 100
#define SERVO_GAIN_MAX 2000

using namespace caros;

SerialDeviceSIProxy::SerialDeviceSIProxy(ros::NodeHandle nodehandle, const std::string& devname,
                                         const bool use_persistent_connections)
    : nodehandle_(nodehandle),
      use_persistent_connections_(use_persistent_connections),
      ros_namespace_("/" + devname + "/" + SERIAL_DEVICE_SERVICE_INTERFACE_SUB_NAMESPACE),
      srv_move_lin_(nodehandle_, "move_lin", ros_namespace_, use_persistent_connections_),
      srv_move_ptp_(nodehandle_, "move_ptp", ros_namespace_, use_persistent_connections_),
      srv_move_ptp_t_(nodehandle_, "move_ptp_t", ros_namespace_, use_persistent_connections_),
      srv_move_servo_q_(nodehandle_, "move_servo_q", ros_namespace_, use_persistent_connections_),
      srv_move_servo_update_(nodehandle_, "move_servo_update", ros_namespace_, use_persistent_connections_),
      srv_move_servo_stop_(nodehandle_, "move_servo_stop", ros_namespace_, use_persistent_connections_),
      srv_move_force_mode_start_(nodehandle_, "move_force_mode_start", ros_namespace_, use_persistent_connections_),
      srv_move_force_mode_update_(nodehandle_, "move_force_mode_update", ros_namespace_, use_persistent_connections_),
      srv_move_force_mode_stop_(nodehandle_, "move_force_mode_stop", ros_namespace_, use_persistent_connections_),
      srv_move_vel_q_(nodehandle_, "move_vel_q", ros_namespace_, use_persistent_connections_),
      srv_move_vel_t_(nodehandle_, "move_vel_t", ros_namespace_, use_persistent_connections_),
      srv_stop_(nodehandle_, "move_stop", ros_namespace_, use_persistent_connections_)
{
  // states
  sub_robot_state_ =
      nodehandle_.subscribe(ros_namespace_ + "/robot_state", 1, &SerialDeviceSIProxy::handleRobotState, this);
}

SerialDeviceSIProxy::~SerialDeviceSIProxy()
{
}

bool SerialDeviceSIProxy::moveLin(const rw::math::Transform3D<>& target, const float speed, const float acceleration)
{
  verifyValueIsWithin(speed, SPEED_MIN, SPEED_MAX);
  verifyValueIsWithin(acceleration, ACCEL_MIN, ACCEL_MAX);
  caros_control_msgs::SerialDeviceMoveLin srv;
  srv.request.targets.push_back(caros::toRos(target));
  srv.request.speeds.push_back(caros::toRos(speed));
  srv.request.accelerations.push_back(caros::toRos(acceleration));

  srv_move_lin_.call<caros_control_msgs::SerialDeviceMoveLin>(srv);

  return srv.response.success;
}

bool SerialDeviceSIProxy::movePtp(const rw::math::Q& target, const float speed, const float acceleration)
{
  verifyValueIsWithin(speed, SPEED_MIN, SPEED_MAX);
  verifyValueIsWithin(acceleration, ACCEL_MIN, ACCEL_MAX);
  caros_control_msgs::SerialDeviceMovePtp srv;
  srv.request.targets.push_back(caros::toRos(target));
  srv.request.speeds.push_back(caros::toRos(speed));
  srv.request.accelerations.push_back(caros::toRos(acceleration));

  srv_move_ptp_.call<caros_control_msgs::SerialDeviceMovePtp>(srv);

  return srv.response.success;
}

bool SerialDeviceSIProxy::movePtpT(const rw::math::Transform3D<>& target, const float speed, const float acceleration)
{
  verifyValueIsWithin(speed, SPEED_MIN, SPEED_MAX);
  verifyValueIsWithin(acceleration, ACCEL_MIN, ACCEL_MAX);
  caros_control_msgs::SerialDeviceMovePtpT srv;
  srv.request.targets.push_back(caros::toRos(target));
  srv.request.speeds.push_back(caros::toRos(speed));
  srv.request.accelerations.push_back(caros::toRos(acceleration));

  srv_move_ptp_t_.call<caros_control_msgs::SerialDeviceMovePtpT>(srv);

  return srv.response.success;
}

bool SerialDeviceSIProxy::moveServoQ(const rw::math::Q& target, const float time, const float lookahead_time,
                                     const float gain)
{
  verifyValueIsWithin(lookahead_time, SERVO_LOOKAHEAD_TIME_MIN, SERVO_LOOKAHEAD_TIME_MAX);
  verifyValueIsWithin(gain, SERVO_GAIN_MIN, SERVO_GAIN_MAX);
  caros_control_msgs::SerialDeviceMoveServoQ srv;
  srv.request.target = caros::toRos(target);
  srv.request.time = caros::toRos(time);
  srv.request.lookahead_time = caros::toRos(lookahead_time);
  srv.request.gain = caros::toRos(gain);

  srv_move_servo_q_.call<caros_control_msgs::SerialDeviceMoveServoQ>(srv);

  return srv.response.success;
}

bool SerialDeviceSIProxy::moveServoUpdate(const rw::math::Q& target)
{
  caros_control_msgs::SerialDeviceMoveServoUpdate srv;
  srv.request.target = caros::toRos(target);

  srv_move_servo_update_.call<caros_control_msgs::SerialDeviceMoveServoUpdate>(srv);

  return srv.response.success;
}

bool SerialDeviceSIProxy::moveServoStop()
{
  caros_common_msgs::EmptySrv srv;
  srv_move_servo_stop_.call<caros_common_msgs::EmptySrv>(srv);

  return srv.response.success;
}

bool SerialDeviceSIProxy::moveForceModeStart(const rw::math::Transform3D<>& ref_t_offset, const rw::math::Q& selection,
                                             const rw::math::Wrench6D<>& wrench_target, int type,
                                             const rw::math::Q& limits)
{
  caros_control_msgs::SerialDeviceMoveForceModeStart srv;
  srv.request.base2forceFrame = caros::toRos(ref_t_offset);
  std::vector<uint32_t> selection_vec;
  for (size_t i = 0; i < selection.size(); i++)
  {
    selection_vec.push_back(static_cast<uint32_t>(selection[i]));
  }

  srv.request.selection = selection_vec;
  srv.request.wrench = caros::toRos(wrench_target);
  srv.request.type = type;

  std::vector<float> limits_vec;
  for (size_t i = 0; i < limits.size(); i++)
  {
    limits_vec.push_back(static_cast<float>(limits[i]));
  }
  srv.request.limits = limits_vec;

  srv_move_force_mode_start_.call<caros_control_msgs::SerialDeviceMoveForceModeStart>(srv);

  return srv.response.success;
}

bool SerialDeviceSIProxy::moveForceModeUpdate(const rw::math::Wrench6D<>& wrench_target)
{
  caros_control_msgs::SerialDeviceMoveForceModeUpdate srv;
  srv.request.wrench = caros::toRos(wrench_target);
  srv_move_force_mode_update_.call<caros_control_msgs::SerialDeviceMoveForceModeUpdate>(srv);

  return srv.response.success;
}

bool SerialDeviceSIProxy::moveForceModeStop()
{
  caros_common_msgs::EmptySrv srv;
  srv_move_force_mode_stop_.call<caros_common_msgs::EmptySrv>(srv);

  return srv.response.success;
}

bool SerialDeviceSIProxy::moveVelQ(const rw::math::Q& target)
{
  caros_control_msgs::SerialDeviceMoveVelQ srv;
  srv.request.vel = caros::toRos(target);

  srv_move_vel_q_.call<caros_control_msgs::SerialDeviceMoveVelQ>(srv);

  return srv.response.success;
}

bool SerialDeviceSIProxy::moveVelT(const rw::math::VelocityScrew6D<>& target)
{
  caros_control_msgs::SerialDeviceMoveVelT srv;
  srv.request.vel = caros::toRos(target);

  srv_move_vel_t_.call<caros_control_msgs::SerialDeviceMoveVelT>(srv);

  return srv.response.success;
}

bool SerialDeviceSIProxy::stop()
{
  caros_common_msgs::EmptySrv srv;

  srv_stop_.call<caros_common_msgs::EmptySrv>(srv);

  return srv.response.success;
}

/* Hardcoded since the connections are not added to a collection that can easily be iterated */
void SerialDeviceSIProxy::closePersistentConnections()
{
  srv_move_lin_.shutdown();
  srv_move_ptp_.shutdown();
  srv_move_ptp_t_.shutdown();
  srv_move_servo_q_.shutdown();
  srv_move_servo_update_.shutdown();
  srv_move_servo_stop_.shutdown();
  srv_move_vel_q_.shutdown();
  srv_move_vel_t_.shutdown();
  srv_stop_.shutdown();
}

void SerialDeviceSIProxy::handleRobotState(const caros_control_msgs::RobotState& state)
{
  robot_state_ = state;
}

rw::math::Q SerialDeviceSIProxy::getQ()
{
  return caros::toRw(robot_state_.q);
}

rw::math::Q SerialDeviceSIProxy::getQd()
{
  return caros::toRw(robot_state_.dq);
}

rw::math::Wrench6D<> SerialDeviceSIProxy::getTCPForce()
{
  return caros::toRw(robot_state_.tcp_force);
}

bool SerialDeviceSIProxy::isEmergencyStopped()
{
  return robot_state_.e_stopped;
}

bool SerialDeviceSIProxy::isSecurityStopped()
{
  return robot_state_.s_stopped;
}

ros::Time SerialDeviceSIProxy::getTimeStamp()
{
  return robot_state_.header.stamp;
}
