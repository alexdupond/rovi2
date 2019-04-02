#ifndef CAROS_UNIVERSAL_ROBOTS_H
#define CAROS_UNIVERSAL_ROBOTS_H

#include <caros/ur_service_interface.h>
#include <caros/caros_node_service_interface.h>
#include <caros/serial_device_service_interface.h>

#include <rw/math_fwd.hpp>
#include <rw/common/Ptr.hpp>
#include <rw/kinematics/State.hpp>
#include <rwhw/universalrobots/URCallBackInterface.hpp>
#include <rwhw/universalrobots/UniversalRobotsRTLogging.hpp>
#include <rwhw/universalrobots_rtde/URRTDE.hpp>

#include <queue>
#include <string>

#define SUPPORTED_Q_LENGTH_FOR_UR 6

namespace rw
{
namespace models
{
class WorkCell;
class Device;
}
}  // end namespaces

namespace rw
{
namespace invkin
{
class JacobianIKSolver;
}
}  // end namespaces

namespace caros
{
class UniversalRobots : public caros::CarosNodeServiceInterface,
                        public caros::SerialDeviceServiceInterface,
                        public URServiceInterface
{
 public:
  explicit UniversalRobots(const ros::NodeHandle& nodehandle);

  virtual ~UniversalRobots();

  enum URNODE_ERRORCODE
  {
    URNODE_MISSING_PARAMETER = 1,
    URNODE_URSERVICE_CONFIGURE_FAIL,
    URNODE_SERIALDEVICESERVICE_CONFIGURE_FAIL,
    URNODE_COULD_NOT_INIT_UR_RTDE,
    URNODE_UNSUPPORTED_Q_LENGTH,
    URNODE_EXECUTION_TIMEOUT,
    URNODE_INTERNAL_ERROR
  };

  enum SAFETY_MODES
  {
    SAFETY_MODE_NORMAL = 1,
    SAFETY_MODE_REDUCED = 2,
    SAFETY_MODE_PROTECTIVE_STOP = 3,
    SAFETY_MODE_SAFEGUARD_STOP = 5,
    SAFETY_MODE_SYSTEM_EMERGENCY_STOP = 6,
    SAFETY_MODE_ROBOT_EMERGENCY_STOP = 7,
    SAFETY_MODE_VIOLATION = 8,
    SAFETY_MODE_FAULT = 9
  };

  /************************************************************************
   * URServiceInterface functions
   ************************************************************************/
  //! @copydoc URServiceInterface::servoT
  bool urServoUpdate(const rw::math::Q& target);
  //! @copydoc URServiceInterface::servoQ
  bool urServoQ(const rw::math::Q& target, const float time, const float lookahead_time, const float gain);
  //! @copydoc URServiceInterface::servoStop
  bool urServoStop();
  //! @copydoc URServiceInterface::forceModeStart
  bool urForceModeStart(const rw::math::Transform3D<>& ref_t_offset, const rw::math::Q& selection,
                        const rw::math::Wrench6D<>& wrench_target, int type, const rw::math::Q& limits);
  //! @copydoc URServiceInterface::forceModeUpdate
  bool urForceModeUpdate(const rw::math::Wrench6D<>& wrench_target);
  //! @copydoc URServiceInterface::forceModeStop
  bool urForceModeStop();
  //! @copydoc URServiceInterface::setPayload
  bool urSetPayload(const double& mass, const rw::math::Vector3D<>& com);
  //! @copydoc URServiceInterface::setIO
  bool urSetIO(const int& pin, const bool& value);

  /************************************************************************
   * SerialDeviceServiceInterface functions
   ************************************************************************/
  //! @copydoc caros::SerialDeviceServiceInterface::moveLin
  bool moveLin(const TransformAndSpeedAndAccelerationContainer_t& targets);
  //! @copydoc caros::SerialDeviceServiceInterface::movePtp
  bool movePtp(const QAndSpeedAndAccelerationContainer_t& targets);
  //! @copydoc caros::SerialDeviceServiceInterface::movePtpT
  bool movePtpT(const TransformAndSpeedAndAccelerationContainer_t& targets);
  //! @copydoc caros::SerialDeviceServiceInterface::moveVelQ
  bool moveVelQ(const rw::math::Q& q_vel);
  //! @copydoc caros::SerialDeviceServiceInterface::moveVelT
  bool moveVelT(const rw::math::VelocityScrew6D<>& t_vel);
  //! @copydoc caros::SerialDeviceServiceInterface::moveServoQ
  bool moveServoQ(const rw::math::Q& target, const float time, const float lookahead_time, const float gain);
  //! @copydoc caros::SerialDeviceServiceInterface::moveServoUpdate
  bool moveServoUpdate(const rw::math::Q& target);
  //! @copydoc caros::SerialDeviceServiceInterface::moveServoStop
  bool moveServoStop();
  //! @copydoc caros::SerialDeviceServiceInterface::moveForceModeStart
  bool moveForceModeStart(const rw::math::Transform3D<>& ref_t_offset, const rw::math::Q& selection,
                          const rw::math::Wrench6D<>& wrench_target, int type, const rw::math::Q& limits);
  //! @copydoc caros::SerialDeviceServiceInterface::moveForceModeUpdate
  bool moveForceModeUpdate(const rw::math::Wrench6D<>& wrench_target);
  //! @copydoc caros::SerialDeviceServiceInterface::moveForceModeStop
  bool moveForceModeStop();
  //! @copydoc caros::SerialDeviceServiceInterface::moveStop
  bool moveStop();

 protected:
  /************************************************************************
   * Hooks implemented from CarosNodeServiceInterface base class
   ************************************************************************/
  bool activateHook();
  bool recoverHook(const std::string& error_msg, const int64_t error_code);

  void runLoopHook();
  void errorLoopHook();
  void fatalErrorLoopHook();

 private:
  /* convenience functions */
  bool isInWorkingCondition();
  bool supportedQSize(const rw::math::Q& q);

 private:
  ros::NodeHandle nodehandle_;
  rw::math::Q qcurrent_; /* Updated in runLoopHook() */
  std::shared_ptr<rwhw::URRTDE> ur_rtde_;
  std::string device_ip_;
};
}  // namespace caros
#endif  // CAROS_UNIVERSAL_ROBOTS_H
