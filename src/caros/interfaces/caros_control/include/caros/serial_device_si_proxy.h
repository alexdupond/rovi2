#ifndef CAROS_SERIAL_DEVICE_SI_PROXY_H
#define CAROS_SERIAL_DEVICE_SI_PROXY_H

#include <caros/caros_service_client.h>
#include <caros_control_msgs/RobotState.h>

#include <rw/math.hpp>

#include <ros/ros.h>

/* TODO:
 * Could make the 'caros_control_msgs::robot_state pRobotState_' available to the user (as a copy), so if this SIP is
 * being updated automatically in a thread, then it's not certain that a call to getTimeStamp and isMoving will be
 * reading from the same robot state
 * Maybe even pack it into it's own structure with a c++/rw interface/types returned instead of the ROS types.
 * The getQ, getQd, isMoving and getTimeStamp functions could be moved to become member functions of that struct/class.
 */

namespace caros
{
/**
 * @brief This class implements a C++ proxy to control and read data from a SerialDeviceServiceInterface.
 */
class SerialDeviceSIProxy
{
 public:
  /**
   * @brief Constructor
   * @param[in] nodehandle
   * @param[in] devname The name of the CAROS serialdevice node
   * @param[in] use_persistent_connections Define usage of persistent connections
   */
  SerialDeviceSIProxy(ros::NodeHandle nodehandle, const std::string& devname,
                      const bool use_persistent_connections = true);

  //! destructor
  virtual ~SerialDeviceSIProxy();

  /**
   * @brief move robot in a linear Cartesian path
   * @param[in] target The target to move to
   * @param[in] speed The tool speed (a value between 0 and 1 is expected) [m/s]
   * @param[in] acceleration Tool acceleration [m/s^2]
   *
   * @returns a boolean indicating if the serial device accepted the command
   * @throws UnavailableService when the command is currently unavailable. This indicates that the connection to the
   *serial device is not fully working, or the serial device has not announced this service yet.
   * @throws BadServiceCall when an error happened while communicating with the serial device.
   */
  bool moveLin(const rw::math::Transform3D<>& target, const float speed = 0.4f, const float acceleration = 1.2f);

  /**
   * @brief move robot from point to point
   * @param[in] target The target to move to
   * @param[in] speed The joint speed (a value between 0 and 1 is expected) [rad/s]
   * @param[in] acceleration Joint acceleration of leading axis [rad/s^2]
   *
   * @returns a boolean indicating if the serial device accepted the command
   * @throws UnavailableService when the command is currently unavailable. This indicates that the connection to the
   *serial device is not fully working, or the serial device has not announced this service yet.
   * @throws BadServiceCall when an error happened while communicating with the serial device.
   */
  bool movePtp(const rw::math::Q& target, const float speed = 0.4f, const float acceleration = 1.4f);

  /**
   * @brief move robot from point to point but using a pose as target (uses controller invkin)
   * @param[in] target The target to move to
   * @param[in] speed The joint speed (a value between 0 and 1 is expected) [rad/s]
   * @param[in] acceleration Joint acceleration of leading axis [rad/s^2]
   *
   * @returns a boolean indicating if the serial device accepted the command
   * @throws UnavailableService when the command is currently unavailable. This indicates that the connection to the
   *serial device is not fully working, or the serial device has not announced this service yet.
   * @throws BadServiceCall when an error happened while communicating with the serial device.
   */
  bool movePtpT(const rw::math::Transform3D<>& target, const float speed = 0.4f, const float acceleration = 1.4f);

  /**
   * @brief move robot by some implementation specific distance based on the provided joint velocities
   * @param[in] target The joint velocities
   *
   * @returns a boolean indicating if the serial device accepted the command
   * @throws UnavailableService when the command is currently unavailable. This indicates that the connection to the
   *serial device is not fully working, or the serial device has not announced this service yet.
   * @throws BadServiceCall when an error happened while communicating with the serial device.
   */
  bool moveVelQ(const rw::math::Q& target);

  /**
   * @brief move robot by some implementation specific distance based on the provided velocity screw
   * @param[in] target The velocity screw.
   *
   * @returns a boolean indicating if the serial device accepted the command
   * @throws UnavailableService when the command is currently unavailable. This indicates that the connection to the
   *serial device is not fully working, or the serial device has not announced this service yet.
   * @throws BadServiceCall when an error happened while communicating with the serial device.
   */
  bool moveVelT(const rw::math::VelocityScrew6D<>& target);

  /**
   * @brief move robot in a servoing fashion using joint configurations
   * @param[in] target The joint configurations to move it
   * @param[in] speed The movement speed (a value between 0 and 1 is expected)
   *
   * @returns a boolean indicating if the serial device accepted the command
   * @throws UnavailableService when the command is currently unavailable. This indicates that the connection to the
   *serial device is not fully working, or the serial device has not announced this service yet.
   * @throws BadServiceCall when an error happened while communicating with the serial device.
   */
  bool moveServoQ(const rw::math::Q& target, const float time, const float lookahead_time, const float gain);

  /**
   * @brief Update servo joint position
   * @param[in] target joint positions [rad]
   */
  bool moveServoUpdate(const rw::math::Q& target);

  /**
   * @brief Stop the servos
   */
  bool moveServoStop();

  /**
   * @brief Set robot to be controlled in force mode.
   *
   * @general 6D vector = [x, y, z, rx, ry, rz], C = compliant, NC = Non-Compliant
   * @param[in] ref_t_offset Pose vector that defines the force frame relative to the base frame.
   * @param[in] selection 6D vector, setting C in the axis direction (=1) or NC (=0)
   * @param[in] wrench_target The forces/torques the robot will apply to its environment. The robot adjusts its position
   * along/about compliant axis in order to achieve the specified force/torque. Values have no effect for non-compliant
   * axes
   * @param[in] type An integer [1;3] specifying how the robot interprets the force frame. 1: The force frame is
   * transformed in a way such that its y-axis is aligned with a vector pointing from the robot tcp towards the origin
   *of
   * the force frame. 2: The force frame is not transformed. 3: The force frame is transformed in a way such that its
   * x-axis is the projection of the robot tcp velocity vector onto the x-y plane of the force frame.
   * @param[in] limits 6D vector, C: max tcp speed, NV: max deviation from tcp pose
   *
   * @returns a boolean indicating if the serial device accepted the command
   * @throws UnavailableService when the command is currently unavailable. This indicates that the connection to the
   *serial device is not fully working, or the serial device has not announced this service yet.
   * @throws BadServiceCall when an error happened while communicating with the serial device.
   */
  bool moveForceModeStart(const rw::math::Transform3D<>& ref_t_offset, const rw::math::Q& selection,
                          const rw::math::Wrench6D<>& wrench_target, int type, const rw::math::Q& limits);

  /**
   * @brief Update the wrench the robot will apply to its environment.
   * @param[in] wrench_target The forces/torques the robot will apply to its environment. The robot adjusts its position
   * along/about compliant axis in order to achieve the specified force/torque. Values have no effect for non-compliant
   * axes
   *
   * @returns a boolean indicating if the serial device accepted the command
   * @throws UnavailableService when the command is currently unavailable. This indicates that the connection to the
   *serial device is not fully working, or the serial device has not announced this service yet.
   * @throws BadServiceCall when an error happened while communicating with the serial device.
   */
  bool moveForceModeUpdate(const rw::math::Wrench6D<>& wrench_target);

  /**
   * @brief Resets the robot mode from force mode to normal operation.
   *
   * @returns a boolean indicating if the serial device accepted the command
   * @throws UnavailableService when the command is currently unavailable. This indicates that the connection to the
   *serial device is not fully working, or the serial device has not announced this service yet.
   * @throws BadServiceCall when an error happened while communicating with the serial device.
   */
  bool moveForceModeStop();

  /**
   * @brief hard stop the robot
   *
   * @returns a boolean indicating if the serial device accepted the command
   * @throws UnavailableService when the command is currently unavailable. This indicates that the connection to the
   *serial device is not fully working, or the serial device has not announced this service yet.
   * @throws BadServiceCall when an error happened while communicating with the serial device.
   */
  bool stop();

  /**
   * @brief Close established (persistent) connections.
   *
   * @note Is mainly intended for debug purposes, to verify that the reconnect functionality is working as intended.
   */
  void closePersistentConnections();

  /**
   * @brief Get the last reported joint configuration
   *
   * @returns the joint configuration (the type of values are very implementation specific, but as a guideline it's
   *highly recommended to keep angles in radians, and distances in meters)
   *
   * @note Make sure to look at getTimeStamp for ensuring the sample is "current enough"
   */
  rw::math::Q getQ();

  /**
   * @brief Get the last reported velocities
   *
   * @returns the velocities (the type of values are implementation specific, but as a guideline it's recommended to
   *represent velocitites as radians per sec)
   *
   * @note Make sure to look at getTimeStamp for ensuring the sample is "current enough"
   */
  rw::math::Q getQd();

  /**
   * @brief Get the last reported TCP force
   *
   * @returns Returns the wrench (Force/Torque vector) at the TCP
   *
   * @note The external wrench is computed based on the error between the joint torques required to stay on the
   * trajectory and the expected joint torques. The maximum force exerted along each axis is 300 Newtons.
   */
  rw::math::Wrench6D<> getTCPForce();

  /**
   * @brief Get information on whether the device's emergency stop is taken
   *
   * @returns a boolean indicating whether the device's emergency stop is taken or not
   *
   */
  bool isEmergencyStopped();

  /**
   * @brief Get information on whether the device's security stop is in effect.
   * @returns a boolean indicating whether the device's security stop is in effect.
   *
   */
  bool isSecurityStopped();

  /**
   * @brief get the timestamp of the received data - use this to verify that the data is "current enough" and supposedly
   *valid - in the case that no data has yet been reported from the device
   *
   * @returns the timestamp of the last reported robot state
   */
  ros::Time getTimeStamp();

 protected:
  void handleRobotState(const caros_control_msgs::RobotState& state);

  ros::NodeHandle nodehandle_;
  bool use_persistent_connections_;
  std::string ros_namespace_;

  // services
  caros::CarosServiceClient srv_move_lin_;
  caros::CarosServiceClient srv_move_ptp_;
  caros::CarosServiceClient srv_move_ptp_t_;
  caros::CarosServiceClient srv_move_servo_q_;
  caros::CarosServiceClient srv_move_servo_update_;
  caros::CarosServiceClient srv_move_servo_stop_;
  caros::CarosServiceClient srv_move_force_mode_start_;
  caros::CarosServiceClient srv_move_force_mode_update_;
  caros::CarosServiceClient srv_move_force_mode_stop_;
  caros::CarosServiceClient srv_move_vel_q_;
  caros::CarosServiceClient srv_move_vel_t_;
  caros::CarosServiceClient srv_stop_;

  // states
  ros::Subscriber sub_robot_state_;
  caros_control_msgs::RobotState robot_state_;
};
}  // namespace caros

#endif  // CAROS_SERIAL_DEVICE_SI_PROXY_H
