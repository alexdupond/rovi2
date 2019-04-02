#include <caros/universal_robots.h>

#include <caros/common.h>
#include <caros/common_robwork.h>

#include <rw/math/MetricFactory.hpp>
#include <ros/assert.h>

#include <string>
#include <vector>

namespace caros
{
UniversalRobots::UniversalRobots(const ros::NodeHandle& nodehandle)
    : CarosNodeServiceInterface(nodehandle, -1),
      SerialDeviceServiceInterface(nodehandle),
      URServiceInterface(nodehandle),
      nodehandle_(nodehandle)
{
  /* Currently nothing specific should happen */
}

UniversalRobots::~UniversalRobots()
{
  if (ur_rtde_ != nullptr)
    ur_rtde_->stopRobot();
}

bool UniversalRobots::activateHook()
{
  /************************************************************************
   * Parameters
   ************************************************************************/
  if (!nodehandle_.getParam("device_ip", device_ip_))
  {
    CAROS_FATALERROR("The parameter '" << nodehandle_.getNamespace()
                                       << "/device_ip' was not present on the parameter server! "
                                          "This parameter has to be specified for this "
                                          "node to work properly.",
                     URNODE_MISSING_PARAMETER);
    return false;
  }

  /************************************************************************
   * Interfaces
   ************************************************************************/
  try
  {
    ur_rtde_ = std::make_shared<rwhw::URRTDE>(device_ip_);
  }
  catch (rw::common::Exception& exp)
  {
    CAROS_FATALERROR("Could not initialize ur_rtde interface:" << exp.what(), URNODE_COULD_NOT_INIT_UR_RTDE);
    return false;
  }

  if (!URServiceInterface::configureInterface())
  {
    CAROS_FATALERROR("The URService could not be configured correctly.", URNODE_URSERVICE_CONFIGURE_FAIL);
    return false;
  }

  if (!SerialDeviceServiceInterface::configureInterface())
  {
    CAROS_FATALERROR("The SerialDeviceService could not be configured correctly.",
                     URNODE_SERIALDEVICESERVICE_CONFIGURE_FAIL);
    return false;
  }

  return true;
}

bool UniversalRobots::recoverHook(const std::string& error_msg, const int64_t error_code)
{
  bool resolved = false;

  switch (error_code)
  {
    case URNODE_EXECUTION_TIMEOUT:
      if (ur_rtde_ != nullptr)
      {
        int32_t safety_mode = ur_rtde_->getSafetyMode();
        if (safety_mode == SAFETY_MODE_NORMAL)
        {
          if (ur_rtde_->reuploadScript())
          {
            resolved = true;
            ROS_INFO("Succesfully recovered from URNODE_EXECUTION_TIMEOUT, resuming normal operation");
          }
          else
          {
            resolved = false;
          }
        }
      }

      break;

    case URNODE_UNSUPPORTED_Q_LENGTH:
      /* Simply acknowledge that a wrong Q was provided */
      resolved = true;
      break;
    case URNODE_INTERNAL_ERROR:
      CAROS_FATALERROR("Can not resolve an internal error... ending up in this case/situation is a bug!",
                       URNODE_INTERNAL_ERROR);
      break;
    default:
      CAROS_FATALERROR("The provided error code '"
                           << error_code << "' has no recovery functionality! - this should be considered a bug!",
                       URNODE_INTERNAL_ERROR);
      break;
  }

  return resolved;
}

void UniversalRobots::runLoopHook()
{
  caros_control_msgs::RobotState robot_state;
  qcurrent_ = ur_rtde_->getActualQ();
  robot_state.q = caros::toRos(qcurrent_);
  robot_state.dq = caros::toRos(ur_rtde_->getActualQd());
  robot_state.tcp_force = caros::toRos(ur_rtde_->getActualTCPForce());
  robot_state.header.frame_id = nodehandle_.getNamespace();
  robot_state.header.stamp = ros::Time::now();
  bool emergency_stopped = false;
  bool security_stopped = false;
  int32_t safety_mode = ur_rtde_->getSafetyMode();

  if (safety_mode == SAFETY_MODE_ROBOT_EMERGENCY_STOP || safety_mode == SAFETY_MODE_SYSTEM_EMERGENCY_STOP ||
      safety_mode == SAFETY_MODE_SAFEGUARD_STOP)
  {
    emergency_stopped = true;
  }

  if (safety_mode == SAFETY_MODE_PROTECTIVE_STOP)
  {
    security_stopped = true;
  }

  robot_state.e_stopped = caros::toRos(emergency_stopped);
  robot_state.s_stopped = caros::toRos(security_stopped);

  SerialDeviceServiceInterface::publishState(robot_state);
}

void UniversalRobots::errorLoopHook()
{
  /* TODO:
   * Consider what needs to be done when this node is in error - should any of the urrt_ or ur_ objects/connections be
   * stopped or just let them continue?
   */
}

void UniversalRobots::fatalErrorLoopHook()
{
  /* TODO:
   * Consider what needs to be done when this node is in error - should any of the urrt_ or ur_ objects/connections be
   * stopped or just let them continue?
   */
}

/************************************************************************
 * URServiceInterface functions
 ************************************************************************/
bool UniversalRobots::urServoStop()
{
  ROS_DEBUG_STREAM("Stopping servos");
  if (!isInWorkingCondition())
  {
    return false;
  }

  if (ur_rtde_->servoStop())
  {
    return true;
  }
  else
  {
    CAROS_ERROR("Execution timeout received! The robot might be in protective or emergency stop.",
                URNODE_EXECUTION_TIMEOUT);
    return false;
  }
}

bool UniversalRobots::urServoUpdate(const rw::math::Q& target)
{
  ROS_DEBUG_STREAM("Updating servo target with: " << target);
  if (!isInWorkingCondition() || !supportedQSize(target))
  {
    return false;
  }

  if (ur_rtde_->servoUpdate(target))
  {
    return true;
  }
  else
  {
    CAROS_ERROR("Execution timeout received! The robot might be in protective or emergency stop.",
                URNODE_EXECUTION_TIMEOUT);
    return false;
  }
}

bool UniversalRobots::urServoQ(const rw::math::Q& target, const float time, const float lookahead_time,
                               const float gain)
{
  ROS_DEBUG_STREAM("ServoQ: " << target);

  if (!isInWorkingCondition() || !supportedQSize(target))
  {
    return false;
  }

  if (ur_rtde_->servoJ(target, 0.0, 0.0, time, lookahead_time, gain))
  {
    return true;
  }
  else
  {
    CAROS_ERROR("Execution timeout received! The robot might be in protective or emergency stop.",
                URNODE_EXECUTION_TIMEOUT);
    return false;
  }
}

bool UniversalRobots::urForceModeStart(const rw::math::Transform3D<>& refToffset, const rw::math::Q& selection,
                                       const rw::math::Wrench6D<>& wrench_target, int type, const rw::math::Q& limits)
{
  ROS_DEBUG_STREAM("ForceModeStart arguments begin:");
  ROS_DEBUG_STREAM("refToffset: " << refToffset);
  ROS_DEBUG_STREAM("selection: " << selection);
  ROS_DEBUG_STREAM("wrench_target: " << wrench_target);
  ROS_DEBUG_STREAM("type: " << type);
  ROS_DEBUG_STREAM("limits: " << limits);
  ROS_DEBUG_STREAM("ForceModeStart arguments end");

  if (!isInWorkingCondition())
  {
    return false;
  }

  if (selection.size() != 6)
  {
    ROS_WARN_STREAM("The number of elements in selection is '" << selection.size() << "' but should be '" << 6 << "'");
    return false;
  }
  if (limits.size() != 6)
  {
    ROS_WARN_STREAM("The number of elements in limits is '" << limits.size() << "' but should be '" << 6 << "'");
    return false;
  }

  if (ur_rtde_->forceModeStart(refToffset, selection, wrench_target, type, limits))
  {
    return true;
  }
  else
  {
    CAROS_ERROR("Execution timeout received! The robot might be in protective or emergency stop.",
                URNODE_EXECUTION_TIMEOUT);
    return false;
  }
}

bool UniversalRobots::urForceModeUpdate(const rw::math::Wrench6D<>& wrench_target)
{
  ROS_DEBUG_STREAM("New wrench target for forceModeUpdate: " << wrench_target);

  if (!isInWorkingCondition())
  {
    return false;
  }

  if (ur_rtde_->forceModeUpdate(wrench_target))
  {
    return true;
  }
  else
  {
    CAROS_ERROR("Execution timeout received! The robot might be in protective or emergency stop.",
                URNODE_EXECUTION_TIMEOUT);
    return false;
  }
}

bool UniversalRobots::urForceModeStop()
{
  if (!isInWorkingCondition())
  {
    return false;
  }

  if (ur_rtde_->forceModeStop())
  {
    return true;
  }
  else
  {
    CAROS_ERROR("Execution timeout received! The robot might be in protective or emergency stop.",
                URNODE_EXECUTION_TIMEOUT);
    return false;
  }
}

bool UniversalRobots::urSetPayload(const double& mass, const rw::math::Vector3D<>& com)
{
  if (!isInWorkingCondition())
  {
    return false;
  }

  // ur_.setPayload(mass, com);

  ROS_ERROR_STREAM("urSetPayload is not supported currently...");
  return false;
}

bool UniversalRobots::urSetIO(const int& pin, const bool& value)
{
  if (!isInWorkingCondition())
  {
    return false;
  }
  ur_rtde_->setStandardDigitalOut(pin, value);

  return true;
}

/************************************************************************
 * SerialDeviceServiceInterface
 ************************************************************************/
bool UniversalRobots::moveLin(const TransformAndSpeedAndAccelerationContainer_t& targets)
{
  ROS_DEBUG_STREAM("moveLin with " << targets.size() << " target(s).");

  if (!isInWorkingCondition())
  {
    return false;
  }

  for (const auto& target : targets)
  {
    if (ur_rtde_->moveL(std::get<0>(target), std::get<1>(target), 0.5))
    {
      return true;
    }
    else
    {
      CAROS_ERROR("Execution timeout received! The robot might be in protective or emergency stop.",
                  URNODE_EXECUTION_TIMEOUT);
      return false;
    }
  }

  return true;
}

bool UniversalRobots::movePtp(const QAndSpeedAndAccelerationContainer_t& targets)
{
  ROS_DEBUG_STREAM("movePtp with " << targets.size() << " target(s).");

  if (!isInWorkingCondition())
  {
    return false;
  }

  for (const auto& target : targets)
  {
    const auto& q = std::get<0>(target);
    if (!supportedQSize(q))
    {
      return false;
    }

    if (ur_rtde_->moveJ(q, std::get<1>(target), std::get<2>(target)))
    {
      return true;
    }
    else
    {
      CAROS_ERROR("Execution timeout received! The robot might be in protective or emergency stop.",
                  URNODE_EXECUTION_TIMEOUT);
      return false;
    }
  }

  return true;
}

bool UniversalRobots::movePtpT(const TransformAndSpeedAndAccelerationContainer_t& targets)
{
  ROS_DEBUG_STREAM("movePtpT with " << targets.size() << " target(s).");

  if (!isInWorkingCondition())
  {
    return false;
  }

  for (const auto& target : targets)
  {
    rw::math::Transform3D<> transform = std::get<0>(target);
    if (ur_rtde_->moveJ_IK(transform, std::get<1>(target), std::get<2>(target)))
    {
      return true;
    }
    else
    {
      CAROS_ERROR("Execution timeout received! The robot might be in protective or emergency stop.",
                  URNODE_EXECUTION_TIMEOUT);
      return false;
    }
  }

  return true;
}

bool UniversalRobots::moveVelQ(const rw::math::Q& q_vel)
{
  if (!isInWorkingCondition())
  {
    return false;
  }

  if (ur_rtde_->speedJ(q_vel, 0.5, 0.5))
  {
    return true;
  }
  else
  {
    CAROS_ERROR("Execution timeout received! The robot might be in protective or emergency stop.",
                URNODE_EXECUTION_TIMEOUT);
    return false;
  }
}

bool UniversalRobots::moveVelT(const rw::math::VelocityScrew6D<>& t_vel)
{
  if (!isInWorkingCondition())
  {
    return false;
  }
  rw::math::Q xd(6, t_vel[0], t_vel[1], t_vel[2], t_vel[3], t_vel[3], t_vel[4], t_vel[5]);
  if (ur_rtde_->speedL(xd, 0.5, 0.5))
  {
    return true;
  }
  else
  {
    CAROS_ERROR("Execution timeout received! The robot might be in protective or emergency stop.",
                URNODE_EXECUTION_TIMEOUT);
    return false;
  }
}

bool UniversalRobots::moveServoQ(const rw::math::Q& target, const float time, const float lookahead_time,
                                 const float gain)
{
  if (!isInWorkingCondition() || !supportedQSize(target))
  {
    return false;
  }

  bool res = false;
  res = urServoQ(target, time, lookahead_time, gain);
  return res;
}

bool UniversalRobots::moveServoUpdate(const rw::math::Q& target)
{
  if (!isInWorkingCondition() || !supportedQSize(target))
  {
    return false;
  }

  bool res = false;
  res = urServoUpdate(target);
  return res;
}

bool UniversalRobots::moveServoStop()
{
  if (!isInWorkingCondition())
  {
    return false;
  }

  bool res = false;
  res = urServoStop();
  return res;
}

bool UniversalRobots::moveForceModeStart(const rw::math::Transform3D<> &ref_t_offset,
                                         const rw::math::Q &selection,
                                         const rw::math::Wrench6D<> &wrench_target,
                                         int type,
                                         const rw::math::Q &limits)
{
  if (!isInWorkingCondition())
  {
    return false;
  }

  bool res = false;
  res = urForceModeStart(ref_t_offset, selection, wrench_target, type, limits);
  return res;
}

bool UniversalRobots::moveForceModeUpdate(const rw::math::Wrench6D<> &wrench_target)
{
  if (!isInWorkingCondition())
  {
    return false;
  }

  bool res = false;
  res = urForceModeUpdate(wrench_target);
  return res;
}

bool UniversalRobots::moveForceModeStop()
{
  if (!isInWorkingCondition())
  {
    return false;
  }

  bool res = false;
  res = urForceModeStop();
  return res;
}

bool UniversalRobots::moveStop()
{
  if (!isInWorkingCondition())
  {
    return false;
  }

  ur_rtde_->stopRobot();

  return true;
}

/************************************************************************
 * Utility functions
 ************************************************************************/
bool UniversalRobots::isInWorkingCondition()
{
  if (!isInRunning())
  {
    ROS_WARN_STREAM("Not in running state!");
    return false;
  }

  return true;
}

bool UniversalRobots::supportedQSize(const rw::math::Q& q)
{
  if (q.size() != SUPPORTED_Q_LENGTH_FOR_UR)
  {
    CAROS_ERROR("The length of Q is " << q.size() << " but should be " << SUPPORTED_Q_LENGTH_FOR_UR,
                URNODE_UNSUPPORTED_Q_LENGTH);
    return false;
  }

  return true;
}

}  // namespace caros
