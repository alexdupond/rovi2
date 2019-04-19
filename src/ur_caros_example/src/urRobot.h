#ifndef _URROBOT_H_
#define _URROBOT_H_

#include "caros/serial_device_si_proxy.h"
#include "ros/package.h"

#include <iostream>
#include "rw/rw.hpp"
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rwlibs/pathoptimization/pathlength/PathLengthOptimizer.hpp>

// SBL includes
#include <rwlibs/pathplanners/sbl/SBLPlanner.hpp>
#include <rwlibs/pathplanners/sbl/SBLOptions.hpp>
#include <rwlibs/pathplanners/sbl/SBLSetup.hpp>
#include <rwlibs/pathplanners/sbl/SBLInternal.hpp>

#include <rw/pathplanning.hpp>

#include <caros_common_msgs/Q.h>
#include <caros/common_robwork.h>

#include "prioritizedPlanner.h"


using namespace std;


#define MAXTIME 60


class URRobot {
public:
	URRobot(ros::NodeHandle* nodehandler); 
	rw::math::Q getQ(); 
	bool setDoubleQ(rw::math::Q q);
	bool setQ(rw::math::Q q);
	bool calculateSBLPath(rw::math::Q from, rw::math::Q to);
	bool calculatePrioritizedPath(vector<rw::math::Q> robot1, vector<rw::math::Q> robo2); 
	rw::trajectory::QPath getPath();
    bool checkCollisions(rw::models::Device::Ptr device, const rw::kinematics::State &state, const rw::proximity::CollisionDetector &detector, const rw::math::Q &q);
    ~URRobot();


private:
	ros::NodeHandle nh;
	rw::models::WorkCell::Ptr wc;
	rw::models::Device::Ptr UR5E1;
	rw::models::Device::Ptr UR5E2; 
	rw::kinematics::State state;
	caros::SerialDeviceSIProxy* robot;
	rw::trajectory::QPath path; 
	vector<double> timesteps; 

	ros::Publisher pub_multi_ur5e; 
};

#endif