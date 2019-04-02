#RobotState represents the state of one a single robot in the setup.

#Header containing information about time and frameid
Header header

#Joint configuration.
# Angles should be represented as radians and distances in meters.
caros_common_msgs/Q q

#Velocities should be represented as radians per sec
caros_common_msgs/Q dq

#TCP Force
geometry_msgs/Wrench tcp_force

#Is Emergency Stopped
bool e_stopped

#Is Security Stopped
bool s_stopped
