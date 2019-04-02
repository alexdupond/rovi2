#include <caros/serial_device_service_interface.h>
#include <caros/common.h>
#include <caros/common_robwork.h>

#include <caros_control_msgs/RobotState.h>

#include <rw/math.hpp>

#include <vector>
#include <stdexcept>
#include <tuple>

namespace
{
template <typename targets_t, typename speeds_t, typename container_t>
bool fillContainerWithTargetsAndSpeeds(const targets_t& targets, const speeds_t& speeds, container_t& res)
{
  if (targets.size() != speeds.size())
  {
    ROS_WARN_STREAM("There are " << targets.size() << " targets and " << speeds.size()
                                 << " speeds, but there should be the same amount of each!");
    return false;
  }

  /* Just an extra "safety" control to verify that our double indexing loop will behave as expected */
  ROS_ASSERT(targets.size() == speeds.size());
  res.clear();
  res.reserve(targets.size());

  /* TODO:
   * Perform the for-loop within a try-catch block to catch out-of-range access within the .at() call
   */
  for (typename targets_t::size_type index = 0; index < targets.size(); ++index)
  {
    res.push_back(std::make_tuple(caros::toRw(targets.at(index)), speeds.at(index)));
  }

  return true;
}

template <typename targets_t, typename speeds_t, typename blends_t, typename container_t>
bool fillContainerWithTargetsAndSpeedsAndBlends(const targets_t& targets, const speeds_t& speeds,
                                                const blends_t& blends, container_t& res)
{
  if (targets.size() != speeds.size())
  {
    ROS_WARN_STREAM("There are " << targets.size() << " targets and " << speeds.size()
                                 << " speeds, but there should be the same amount of each!");
    return false;
  }

  if (targets.size() != blends.size())
  {
    ROS_WARN_STREAM("There are " << targets.size() << " targets and " << blends.size()
                                 << " blends, but there should be the same amount of each!");
    return false;
  }

  /* Just an extra "safety" control to verify that our double indexing loop will behave as expected */
  ROS_ASSERT(targets.size() == speeds.size());
  ROS_ASSERT(targets.size() == blends.size());
  res.clear();
  res.reserve(targets.size());

  try
  {
    for (typename targets_t::size_type index = 0; index < targets.size(); ++index)
    {
      res.push_back(std::make_tuple(caros::toRw(targets.at(index)), speeds.at(index), blends.at(index)));
    }
  }
  catch (const std::out_of_range& oor)
  {
    ROS_ERROR_STREAM("Container Out of Range error: " << oor.what());
    return false;
  }

  return true;
}

template <typename targets_t, typename speeds_t, typename accelerations_t, typename container_t>
bool fillContainerWithTargetsAndSpeedsAndAccelerations(const targets_t& targets, const speeds_t& speeds,
                                                       const accelerations_t& accelerations, container_t& res)
{
  if (targets.size() != speeds.size())
  {
    ROS_WARN_STREAM("There are " << targets.size() << " targets and " << speeds.size()
                                 << " speeds, but there should be the same amount of each!");
    return false;
  }

  if (targets.size() != accelerations.size())
  {
    ROS_WARN_STREAM("There are " << targets.size() << " targets and " << accelerations.size()
                                 << " accelerations, but there should be the same amount of each!");
    return false;
  }

  /* Just an extra "safety" control to verify that our double indexing loop will behave as expected */
  ROS_ASSERT(targets.size() == speeds.size());
  ROS_ASSERT(targets.size() == accelerations.size());
  res.clear();
  res.reserve(targets.size());

  try
  {
    for (typename targets_t::size_type index = 0; index < targets.size(); ++index)
    {
      res.push_back(std::make_tuple(caros::toRw(targets.at(index)), speeds.at(index), accelerations.at(index)));
    }
  }
  catch (const std::out_of_range& oor)
  {
    ROS_ERROR_STREAM("Container Out of Range error: " << oor.what());
    return false;
  }

  return true;
}

}  // end namespace

using namespace caros;

SerialDeviceServiceInterface::SerialDeviceServiceInterface(ros::NodeHandle nodehandle)
    : nodehandle_(nodehandle, SERIAL_DEVICE_SERVICE_INTERFACE_SUB_NAMESPACE)
{
  /* Do nothing */
}

SerialDeviceServiceInterface::SerialDeviceServiceInterface()
{
  /* Do nothing */
  ROS_FATAL_STREAM(
      "The empty constructor of the GripperServiceInterface should never be called! This is undefined behaviour.");
}

SerialDeviceServiceInterface::~SerialDeviceServiceInterface()
{
  /* Nothing special needs to be done - relying on ROS's RAII design */
}

bool SerialDeviceServiceInterface::configureInterface()
{
  return initService();
}

bool SerialDeviceServiceInterface::initService()
{
  if (srv_move_lin_ || srv_move_ptp_ || srv_move_ptp_t_ || srv_move_vel_q_ || srv_move_vel_t_ || srv_move_servo_q_ ||
      srv_move_servo_update_ || srv_move_servo_stop_ || srv_move_force_mode_start_ || srv_move_force_mode_update_ ||
      srv_move_force_mode_stop_ || srv_move_stop_ || device_state_publisher_)
  {
    ROS_WARN_STREAM(
        "Reinitialising one or more SerialDeviceServiceInterface services or publishers. If this is not fully intended "
        "then this should be considered a bug!");
  }

  device_state_publisher_ =
      nodehandle_.advertise<caros_control_msgs::RobotState>("robot_state", SERIAL_DEVICE_STATE_PUBLISHER_QUEUE_SIZE);
  ROS_ERROR_STREAM_COND(!device_state_publisher_, "The RobotState publisher is empty!");

  srv_move_lin_ = nodehandle_.advertiseService("move_lin", &SerialDeviceServiceInterface::moveLinHandle, this);
  ROS_ERROR_STREAM_COND(!srv_move_lin_, "The move_lin service is empty!");

  srv_move_ptp_ = nodehandle_.advertiseService("move_ptp", &SerialDeviceServiceInterface::movePtpHandle, this);
  ROS_ERROR_STREAM_COND(!srv_move_ptp_, "The move_ptp service is empty!");

  srv_move_ptp_t_ = nodehandle_.advertiseService("move_ptp_t", &SerialDeviceServiceInterface::movePtpTHandle, this);
  ROS_ERROR_STREAM_COND(!srv_move_ptp_t_, "The move_ptp_t service is empty!");

  srv_move_vel_q_ = nodehandle_.advertiseService("move_vel_q", &SerialDeviceServiceInterface::moveVelQHandle, this);
  ROS_ERROR_STREAM_COND(!srv_move_vel_q_, "The move_vel_q service is empty!");

  srv_move_vel_t_ = nodehandle_.advertiseService("move_vel_t", &SerialDeviceServiceInterface::moveVelTHandle, this);
  ROS_ERROR_STREAM_COND(!srv_move_vel_t_, "The move_vel_t service is empty!");

  srv_move_servo_q_ =
      nodehandle_.advertiseService("move_servo_q", &SerialDeviceServiceInterface::moveServoQHandle, this);
  ROS_ERROR_STREAM_COND(!srv_move_servo_q_, "The move_servo_q service is empty!");

  srv_move_servo_update_ =
      nodehandle_.advertiseService("move_servo_update", &SerialDeviceServiceInterface::moveServoUpdateHandle, this);
  ROS_ERROR_STREAM_COND(!srv_move_servo_update_, "The move_servo_update service is empty!");

  srv_move_servo_stop_ =
      nodehandle_.advertiseService("move_servo_stop", &SerialDeviceServiceInterface::moveServoStopHandle, this);
  ROS_ERROR_STREAM_COND(!srv_move_servo_stop_, "The move_servo_stop service is empty!");

  srv_move_force_mode_start_ = nodehandle_.advertiseService(
      "move_force_mode_start", &SerialDeviceServiceInterface::moveForceModeStartHandle, this);
  ROS_ERROR_STREAM_COND(!srv_move_force_mode_start_, "The move_force_mode_start service is empty!");

  srv_move_force_mode_update_ = nodehandle_.advertiseService(
      "move_force_mode_update", &SerialDeviceServiceInterface::moveForceModeUpdateHandle, this);
  ROS_ERROR_STREAM_COND(!srv_move_force_mode_update_, "The move_force_mode_update service is empty!");

  srv_move_force_mode_stop_ = nodehandle_.advertiseService(
      "move_force_mode_stop", &SerialDeviceServiceInterface::moveForceModeStopHandle, this);
  ROS_ERROR_STREAM_COND(!srv_move_force_mode_stop_, "The move_force_mode_stop service is empty!");

  srv_move_stop_ = nodehandle_.advertiseService("move_stop", &SerialDeviceServiceInterface::moveStopHandle, this);
  ROS_ERROR_STREAM_COND(!srv_move_stop_, "The move_stop service is empty!");

  if (srv_move_lin_ && srv_move_ptp_ && srv_move_ptp_t_ && srv_move_vel_q_ && srv_move_vel_t_ && srv_move_servo_q_ &&
      srv_move_servo_update_ && srv_move_servo_stop_ && srv_move_stop_ && srv_move_force_mode_start_ &&
      srv_move_force_mode_update_ && srv_move_force_mode_stop_ && device_state_publisher_)
  {
    /* Everything seems to be properly initialised */
    ROS_DEBUG_STREAM(
        "All SerialDeviceServiceInterface publishers and services appear to have been properly initialised");
  }
  else
  {
    ROS_ERROR_STREAM(
        "The SerialDeviceServiceInterface could not be properly initialised - one or more ROS services or publishers "
        "failed to be properly initialised.");
    return false;
  }

  return true;
}

void SerialDeviceServiceInterface::publishState(const caros_control_msgs::RobotState& state)
{
  device_state_publisher_.publish(state);
}

/************************************************************************
 * ROS service handle functions
 ************************************************************************/
bool SerialDeviceServiceInterface::moveLinHandle(caros_control_msgs::SerialDeviceMoveLin::Request& request,
                                                 caros_control_msgs::SerialDeviceMoveLin::Response& response)
{
  TransformAndSpeedAndBlendContainer_t res;
  if (fillContainerWithTargetsAndSpeedsAndAccelerations(request.targets, request.speeds, request.accelerations, res))
  {
    response.success = moveLin(res);
  }
  else
  {
    response.success = false;
  }

  return true;
}

bool SerialDeviceServiceInterface::movePtpHandle(caros_control_msgs::SerialDeviceMovePtp::Request& request,
                                                 caros_control_msgs::SerialDeviceMovePtp::Response& response)
{
  QAndSpeedAndBlendContainer_t res;
  if (fillContainerWithTargetsAndSpeedsAndAccelerations(request.targets, request.speeds, request.accelerations, res))
  {
    response.success = movePtp(res);
  }
  else
  {
    response.success = false;
  }

  return true;
}

bool SerialDeviceServiceInterface::movePtpTHandle(caros_control_msgs::SerialDeviceMovePtpT::Request& request,
                                                  caros_control_msgs::SerialDeviceMovePtpT::Response& response)
{
  TransformAndSpeedAndBlendContainer_t res;
  if (fillContainerWithTargetsAndSpeedsAndAccelerations(request.targets, request.speeds, request.accelerations, res))
  {
    response.success = movePtpT(res);
  }
  else
  {
    response.success = false;
  }

  return true;
}

bool SerialDeviceServiceInterface::moveVelQHandle(caros_control_msgs::SerialDeviceMoveVelQ::Request& request,
                                                  caros_control_msgs::SerialDeviceMoveVelQ::Response& response)
{
  rw::math::Q vel = caros::toRw(request.vel);
  response.success = moveVelQ(vel);

  return true;
}

bool SerialDeviceServiceInterface::moveVelTHandle(caros_control_msgs::SerialDeviceMoveVelT::Request& request,
                                                  caros_control_msgs::SerialDeviceMoveVelT::Response& response)
{
  rw::math::VelocityScrew6D<> vel = caros::toRw(request.vel);
  response.success = moveVelT(vel);

  return true;
}

bool SerialDeviceServiceInterface::moveServoQHandle(caros_control_msgs::SerialDeviceMoveServoQ::Request& request,
                                                    caros_control_msgs::SerialDeviceMoveServoQ::Response& response)
{
  response.success = moveServoQ(caros::toRw(request.target), request.time, request.lookahead_time, request.gain);

  return true;
}

bool SerialDeviceServiceInterface::moveServoUpdateHandle(
    caros_control_msgs::SerialDeviceMoveServoUpdate::Request& request,
    caros_control_msgs::SerialDeviceMoveServoUpdate::Response& response)
{
  response.success = moveServoUpdate(caros::toRw(request.target));

  return true;
}

bool SerialDeviceServiceInterface::moveServoStopHandle(caros_common_msgs::EmptySrv::Request& request,
                                                       caros_common_msgs::EmptySrv::Response& response)
{
  response.success = moveServoStop();

  return true;
}

bool SerialDeviceServiceInterface::moveForceModeStartHandle(
    caros_control_msgs::SerialDeviceMoveForceModeStart::Request& request,
    caros_control_msgs::SerialDeviceMoveForceModeStart::Response& response)
{
  rw::math::Transform3D<> ref_t_offset = caros::toRw(request.base2forceFrame);
  rw::math::Wrench6D<> wrench_target;
  wrench_target(0) = request.wrench.force.x;
  wrench_target(1) = request.wrench.force.y;
  wrench_target(2) = request.wrench.force.z;

  wrench_target(3) = request.wrench.torque.x;
  wrench_target(4) = request.wrench.torque.y;
  wrench_target(5) = request.wrench.torque.z;

  std::size_t index;
  rw::math::Q selection(request.selection.size());
  index = 0;
  for (const auto item : request.selection)
  {
    selection(index++) = static_cast<double>(item);
  }

  rw::math::Q limits(request.limits.size());
  index = 0;
  for (const auto item : request.limits)
  {
    limits(index++) = static_cast<double>(item);
  }
  response.success = moveForceModeStart(ref_t_offset, selection, wrench_target, request.type, limits);

  return true;
}

bool SerialDeviceServiceInterface::moveForceModeUpdateHandle(
    caros_control_msgs::SerialDeviceMoveForceModeUpdate::Request& request,
    caros_control_msgs::SerialDeviceMoveForceModeUpdate::Response& response)
{
  response.success = moveForceModeUpdate(caros::toRw(request.wrench));

  return true;
}

bool SerialDeviceServiceInterface::moveForceModeStopHandle(caros_common_msgs::EmptySrv::Request& request,
                                                           caros_common_msgs::EmptySrv::Response& response)
{
  response.success = moveForceModeStop();

  return true;
}

bool SerialDeviceServiceInterface::moveStopHandle(caros_common_msgs::EmptySrv::Request& request,
                                                  caros_common_msgs::EmptySrv::Response& response)
{
  response.success = moveStop();

  return true;
}
