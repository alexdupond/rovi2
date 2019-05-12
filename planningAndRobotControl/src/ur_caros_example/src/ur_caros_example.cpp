#include <iostream>
#include "rw/rw.hpp"
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include "caros/serial_device_si_proxy.h"
#include "ros/package.h"
#include "ros/ros.h"
#include "prioritizedPlanner.h"


#include <rwlibs/pathoptimization/pathlength/PathLengthOptimizer.hpp>

#include <thread>  // NOLINT(build/c++11)
#include <chrono>  // NOLINT(build/c++11)
#include <math.h>
#include <fstream>
#include <sstream>

// SBL includes
#include <rwlibs/pathplanners/sbl/SBLPlanner.hpp>
#include <rwlibs/pathplanners/sbl/SBLOptions.hpp>
#include <rwlibs/pathplanners/sbl/SBLSetup.hpp>
#include <rwlibs/pathplanners/sbl/SBLInternal.hpp>

#include <rw/pathplanning.hpp>

#include <caros_common_msgs/Q.h>
#include <caros/common_robwork.h>
#include <wsg_50_common/Move.h>

using namespace std;
using namespace rw::common;
using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::models;
using namespace rw::pathplanning;
using namespace rw::proximity;
using namespace rw::trajectory;
using namespace rwlibs::pathplanners;
using namespace rwlibs::proximitystrategies;

#define MAXTIME 15

rw::trajectory::QPath joinPaths(rw::trajectory::QPath& path1, rw::trajectory::QPath& path2){
	int size = 0; 
	rw::trajectory::QPath result; 

	if (path1.size() > path2.size()) {
		size = path1.size(); 
	}else{
		size = path2.size(); 
	}

	for(size_t i = 0; i < size; i++)
	{
		vector<double> q;
		if (i > path1.size() - 1) {
			int j = path1.size()-1; 
			vector<double> q1{path1[j][0], path1[j][1], path1[j][2], path1[j][3], path1[j][4], path1[j][5], path2[i][0], path2[i][1], path2[i][2], path2[i][3], path2[i][4], path2[i][5]}; 
			q = q1; 
		}else if(i > path2.size() - 1){
			int j = path2.size()-1;
			vector<double> q2{path1[i][0], path1[i][1], path1[i][2], path1[i][3], path1[i][4], path1[i][5], path2[j][0], path2[j][1], path2[j][2], path2[j][3], path2[j][4], path2[j][5]}; 
			q = q2; 
		}else{
			vector<double> q3{path1[i][0], path1[i][1], path1[i][2], path1[i][3], path1[i][4], path1[i][5], path2[i][0], path2[i][1], path2[i][2], path2[i][3], path2[i][4], path2[i][5]}; 
			q = q3;
		}
		result.push_back(q); 
				
	}

	return result;	
}

bool checkCollisions(Device::Ptr device, const State &state, const CollisionDetector &detector, const Q &q) {
	State testState;
	CollisionDetector::QueryResult data;
	bool colFrom;

	testState = state;
	device->setQ(q,testState);
	colFrom = detector.inCollision(testState,&data);
	if (colFrom) {
		cerr << "Configuration in collision: " << q << endl;
		cerr << "Colliding frames: " << endl;
		FramePairSet fps = data.collidingFrames;
		for (FramePairSet::iterator it = fps.begin(); it != fps.end(); it++) {
			cerr << (*it).first->getName() << " " << (*it).second->getName() << endl;
		}
		return false;
	}
	return true;
}
void savePathToLua(rw::trajectory::QPath &path)
{
	string luaFilename = "/home/jeppe/catkin_ws_rovi/src/ur_caros_example/filepathTwoRobots.lua";
	ofstream myfile;
	myfile.open(luaFilename, ios::app | ios::out | ios::binary);
	for (rw::trajectory::QPath::iterator it = path.begin(); it < path.end(); it++)
	{
		ostringstream ss;
		ss << *it;
		string str = ss.str();
		str.erase(0,5);
		str.insert(0,"setQ(");
		myfile << str << ')' << endl;
	}
	myfile.close();
}

class URRobot {
	using Q = rw::math::Q;

private:
	ros::NodeHandle nh;
	WorkCell::Ptr wc;
	Device::Ptr UR5E1;
	Device::Ptr UR5E2; 
	State state;
	caros::SerialDeviceSIProxy* robot1;
	caros::SerialDeviceSIProxy* robot2;

	ros::ServiceClient gripper1grasp;
	ros::ServiceClient gripper1release;
	ros::ServiceClient gripper2grasp;
	ros::ServiceClient gripper2release;

	QPath path;
	QPath longPath;
	vector<vector<int>> graspReleaseIndex; 

	ros::Publisher pub_multi_ur5e; 

public:
	URRobot()
	{
		auto packagePath = ros::package::getPath("ur_caros_example");
		//wc = WorkCellLoader::Factory::load(packagePath + "/WorkCell/Scene.wc.xml");
		wc = WorkCellLoader::Factory::load(packagePath + "/RoviScene/Scene.xml");
		UR5E1 = wc->findDevice("UR5e_1");
		UR5E2 = wc->findDevice("UR5e_2");	
		state = wc->getDefaultState();
		robot1 = new caros::SerialDeviceSIProxy(nh, "caros_universalrobot1");
		robot2 = new caros::SerialDeviceSIProxy(nh, "caros_universalrobot2");

		// Publisher for robwork plugin
		pub_multi_ur5e = nh.advertise<caros_control_msgs::RobotState>("/robot_state/multirobot/ur5e", 1);

		// Services for grippers

		gripper1grasp = nh.serviceClient<wsg_50_common::Move>("/wsg_50_driver1/grasp");
		gripper1release = nh.serviceClient<wsg_50_common::Move>("/wsg_50_driver1/release");
		gripper2grasp = nh.serviceClient<wsg_50_common::Move>("/wsg_50_driver2/grasp");
		gripper2release = nh.serviceClient<wsg_50_common::Move>("/wsg_50_driver2/release");

		// Wait for first state message, to make sure robot is ready
		ros::topic::waitForMessage<caros_control_msgs::RobotState>("/caros_universalrobot1/caros_serial_device_service_interface/robot_state", nh);
		ros::topic::waitForMessage<caros_control_msgs::RobotState>("/caros_universalrobot2/caros_serial_device_service_interface/robot_state", nh);
	    ros::spinOnce();
	}
	void graspGripper1()
	{
		wsg_50_common::Move move;
		move.request.speed = 100;
		move.request.width = 50;
		gripper1grasp.call(move);
	}
	void releaseGripper1()
	{
		wsg_50_common::Move move;
		move.request.speed = 100;
		move.request.width = 100;
		gripper1release.call(move);
	}
	void graspGripper2()
	{
		wsg_50_common::Move move;
		move.request.speed = 100;
		move.request.width = 50;
		gripper2grasp.call(move);
	}
	void releaseGripper2()
	{
		wsg_50_common::Move move;
		move.request.speed = 100;
		move.request.width = 100;
		gripper2release.call(move);
	}

	Q getQ1()
	{
		// spinOnce processes one batch of messages, calling all the callbacks
	    ros::spinOnce();
	    Q q = robot1->getQ();
		UR5E1->setQ(q, state);
	    return q;
	}
	Q getQ2()
	{
		// spinOnce processes one batch of messages, calling all the callbacks
	    ros::spinOnce();
	    Q q = robot2->getQ();
		UR5E2->setQ(q, state);
	    return q;
	}
	Q getDoubleQ()
	{
		Q q1 = getQ1();
		Q q2 = getQ2();
		vector<double> q{q1[0],q1[1],q1[2],q1[3],q1[4],q1[5],q2[0],q2[1],q2[2],q2[3],q2[4],q2[5]};
		return Q(q);
	}

	bool setDoubleQ(Q q){
		// Creating  to messages to be send

		caros_common_msgs::Q q_ros = caros::toRos(q);
		
		caros_control_msgs::RobotState msg_multi_q;

		msg_multi_q.q = q_ros; 

		pub_multi_ur5e.publish(msg_multi_q); 

		return true; 
	}

	bool setQ(Q q)
	{
		// Tell robot to move to joint config q
		rw::math::Q q1(6, q[0], q[1], q[2], q[3], q[4], q[5]); 
		rw::math::Q q2(6, q[6], q[7], q[8], q[9], q[10], q[11]); 
		double speed = 0.3;
		if (robot1->movePtp(q1,speed) && robot2->movePtp(q2, speed) && setDoubleQ(q)) { 
			return true;
		} else
			return false;
	}

		bool setQServo(Q q)
	{
		// Tell robot to move to joint config q
		rw::math::Q q1(6, q[0], q[1], q[2], q[3], q[4], q[5]); 
		rw::math::Q q2(6, q[6], q[7], q[8], q[9], q[10], q[11]); 
		robot1->moveServoQ(q1, 1 , 0.1 , 1200 );
		robot2->moveServoQ(q2, 1 , 0.1 , 1200 );
		for(size_t i = 0; i < 10; i++)
		{	
		robot1->moveServoUpdate(q1);
		robot2->moveServoUpdate(q2);
    	std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
		robot1->moveServoStop();
		robot2->moveServoStop();

		// if (robot1->moveServoQ(q1, 5 , 0.1 , 100 ) && robot2->movePtp(q2, speed) && setDoubleQ(q)) { 
		// 	return true;
		// } else
		// 	return false;
	}
	QPath interpolateQPath(QPath & pathQ, double maxStepSize)
	{
		int graspIndex = 0;
		rw::trajectory::QPath interpolatedPath;
		for(size_t i = 0; i < pathQ.size()-1; i++)
		{	
			double longestMove = 0;
			for(size_t j = 0; j < pathQ[0].size(); j++)
			{
				double dist = abs(pathQ[i][j] - pathQ[i+1][j]);
				if(dist > longestMove)
				{
					longestMove = dist;
				}
			}
			int steps = ceil(longestMove/maxStepSize);
			cout << "hej " << endl;
			if(graspReleaseIndex.size() && graspReleaseIndex.size() > graspIndex && graspReleaseIndex[graspIndex][0] == i)
			{
				graspReleaseIndex[graspIndex][0] = interpolatedPath.size();
				graspIndex++;
			}
			rw::trajectory::LinearInterpolator<rw::math::Q> interpolator(pathQ[i],pathQ[i+1],steps);
			for(int i = 0; i < steps; i++)
			{
				//Husk at interpoleringen ikke medtager det sidste step, da den bliver tilføjet som
				// det første næste gang. Så det aller sidste step bliver ikke tilføjet. 
				interpolatedPath.push_back(interpolator.x(i));
			}
		}
		return interpolatedPath;
	}
	void followTrajectory()
	{
		QPath trajectory = interpolateQPath(longPath, 0.012);//0.012
		cout <<"Path size: " << longPath.size() << endl;
		cout <<"Interpolated Path size: " << trajectory.size() << endl;
		rw::math::Q q1(6, trajectory[0][0], trajectory[0][1], trajectory[0][2],trajectory[0][3],trajectory[0][4],trajectory[0][5]); 
		rw::math::Q q2(6, trajectory[0][6], trajectory[0][7], trajectory[0][8],trajectory[0][9],trajectory[0][10],trajectory[0][11]); 
		robot1->moveServoQ(q1, 0.2 , 0.1 , 600 );
		robot2->moveServoQ(q2, 0.2 , 0.1 , 600 );
		int graspIndex = 0;
		for(size_t j = 1; j < trajectory.size(); j++)
		{
			if(graspReleaseIndex.size() && graspReleaseIndex.size() > graspIndex && graspReleaseIndex[graspIndex][0] == j)
			{
				std::this_thread::sleep_for(std::chrono::milliseconds(500));
				if(graspReleaseIndex[graspIndex][1] == 1)
					graspGripper1();
				else
					releaseGripper1();
				if(graspReleaseIndex[graspIndex][2] == 1)
					graspGripper2();
				else
					releaseGripper2();
				graspIndex++;
			}
			q1 = Q(6, trajectory[j][0], trajectory[j][1], trajectory[j][2],trajectory[j][3],trajectory[j][4],trajectory[j][5]); 
			q2 = Q(6, trajectory[j][6], trajectory[j][7], trajectory[j][8],trajectory[j][9],trajectory[j][10],trajectory[j][11]);
			robot1->moveServoUpdate(q1);
			robot2->moveServoUpdate(q2);
    		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
		for(size_t i = 0; i < 2; i++)
		{
			int j = trajectory.size()-1;
			q1 = Q(6, trajectory[j][0], trajectory[j][1], trajectory[j][2],trajectory[j][3],trajectory[j][4],trajectory[j][5]); 
			q2 = Q(6, trajectory[j][6], trajectory[j][7], trajectory[j][8],trajectory[j][9],trajectory[j][10],trajectory[j][11]);
			robot1->moveServoUpdate(q1);
			robot2->moveServoUpdate(q2);
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
		robot1->moveServoStop();
		robot2->moveServoStop();
	}
	void home()
	{
		vector<double> QHome{3.28, -2.2, 2.4, -1.8, -1.5, 0, -0.13, -0.78, -2.2, -1.7, 1.5, 0};
		rw::math::Q home(QHome);
		calculateSBLPath(getDoubleQ(),home);
		longPath = path;
		followTrajectory();
	}
	rw::math::Q verticalIK(rw::models::Device::Ptr device, double deltaH, rw::kinematics::State & state)
	{
		rw::math::Transform3D<> T_world_end = device->baseTend(state);
		rw::math::Transform3D<> T_grasp = T_world_end;
		T_grasp.P()[2] = T_grasp.P()[2] + deltaH;
		
		rw::invkin::JacobianIKSolver IKsolverA(device,state);
		//IKsolver.setMaxError(1e-10); // Default: 1e-06
		rw::trajectory::QPath IK_path = IKsolverA.solve(T_grasp,state);
		return IK_path[0];
	}
	
	Q IK(rw::models::Device::Ptr device, Q startQ, Transform3D<double> TWorldEnd)
	{
		device->setQ(startQ,state);	
		
		rw::math::Transform3D<double> TBaseEnd = device->worldTbase(state);
		rw::math::Transform3D<double>::invMult(TBaseEnd,TWorldEnd);
		
		rw::invkin::JacobianIKSolver IKsolverA(device,state);
		//IKsolver.setMaxError(1e-10); // Default: 1e-06
		rw::trajectory::QPath IK_path = IKsolverA.solve(TBaseEnd,state);
		if(IK_path.size() > 1)
			cout << "Flere loesninger til IK" << endl;
		cout << "hej "<< IK_path.size() << endl;
		return IK_path[0];
	}
	void appendVerticalGrasp(Q fromQ, bool robot1, bool robot2, bool robot1graspOrRelease, bool robot2graspOrRelease)
	{
		vector<double> startPos1 = {fromQ[0],fromQ[1],fromQ[2],fromQ[3],fromQ[4],fromQ[5]};
		vector<double> startPos2 = {fromQ[6],fromQ[7],fromQ[8],fromQ[9],fromQ[10],fromQ[11]};
		UR5E1->setQ(Q(startPos1),state);
		UR5E2->setQ(Q(startPos2),state);

		rw::trajectory::QPath reverseGraspPath;
		for(size_t i = 1; i <= 5; i++)
		{	
			vector<double> Q_grasp_A;
			vector<double> Q_grasp_B;
			if(robot1 == true)
				Q_grasp_A = verticalIK(UR5E1,-0.01*i,state).toStdVector();
			else
				Q_grasp_A = startPos1;
			if(robot2 == true)
				Q_grasp_B = verticalIK(UR5E2,-0.01*i,state).toStdVector();
			else
				Q_grasp_B = startPos2;
			
			Q_grasp_A.insert(Q_grasp_A.end(),Q_grasp_B.begin(),Q_grasp_B.end());
			reverseGraspPath.insert(reverseGraspPath.begin(),rw::math::Q(Q_grasp_A));
			path.push_back(rw::math::Q(Q_grasp_A));
		}
		graspReleaseIndex.push_back({int(path.size()+longPath.size()), robot1graspOrRelease, robot2graspOrRelease});
		for(size_t i = 0; i < reverseGraspPath.size(); i++)
		{
			path.push_back(reverseGraspPath[i]);
		}
	}
	void test(void)
	{
		cout << "virker" << endl;
		setQ(getDoubleQ());
	}
	bool calculatePrioritizedPath(vector<rw::math::Q>& rob1, vector<rw::math::Q>& rob2, rw::trajectory::QPath& result1, rw::trajectory::QPath& result2, double extend, double aggressiveness){
	PrioritizedPlanner planner(wc, UR5E1, UR5E2, extend, aggressiveness); 

		cout << "Robot 1 size = " << rob1.size() << ", and Robot 2 size = " << rob2.size() << endl; 
		if(rob1.size() && rob2.size()){

			if(!planner.calculateRRTPath(rob1, result1))
				return false;

			cout << "Finished planning for robot 1 - Total size = " << result1.size() << endl; 
			test();

			if(planner.calculateDynamicRRTPath(rob2, result1, result2, robot1, robot2)){
				cout << "Finished planning for robot 2 " << endl; 
				return true; 
			}else{
				return false; 
			}
		}
		return false; 
	}
	void plan2()
	{
		longPath.clear();
		vector<double> QHome{3.28, -2.2, 2.4, -1.8, -1.5, 0, -0.13, -0.78, -2.2, -1.7, 1.5, 0};
		vector<double> QvecFrom{2.21, -0.89, 1.13, -1.80, -1.61, -0.14,-0.68, -2.27, -1.08, -1.35, 1.57, 0.02};
		vector<double> QvecTo{3.64, -0.71, 0.83, -1.68, -1.59, 1.26, 0.91, -2.38, -0.87, -1.48, 1.58, -1.48};

		vector<Transform3D<double>> pickTransforms;
		vector<Transform3D<double>> placeTransforms;

		Rotation3D<double> R1(0.998036, 0.0626227, -0.00146539, 0.0626319, -0.997262, 0.0393161, 0.00100071, -0.0393307, -0.999226);
		Vector3D<double> V1(-0.448804, 0.845466, 0.350158);
		Transform3D<double> placePose(V1,R1);
		Rotation3D<double> R2(0.880099, -0.474547, -0.0151736, -0.474558, -0.880219, 0.00309925, -0.0148268, 0.0044731, -0.99988);
		Vector3D<double> V2(-0.982333, 0.437329, 0.352617);
		Transform3D<double> pickPose(V2,R2);
		pickTransforms.push_back(pickPose);
		placeTransforms.push_back(placePose);
		R1 = Rotation3D<double>(0.998036, 0.0626227, -0.00146539, 0.0626319, -0.997262, 0.0393161, 0.00100071, -0.0393307, -0.999226);
		V1 = Vector3D<double>(-0.448804, 0.845466, 0.350158-0.05);
		placePose = Transform3D<double>(V1,R1);
		R2 = Rotation3D<double>(0.880099, -0.474547, -0.0151736, -0.474558, -0.880219, 0.00309925, -0.0148268, 0.0044731, -0.99988);
		V2 =  Vector3D<double>(-0.982333, 0.437329, 0.352617-0.05);
		pickPose = Transform3D<double>(V2,R2);
		pickTransforms.push_back(pickPose);
		placeTransforms.push_back(placePose);
		R1 = Rotation3D<double>(0.998036, 0.0626227, -0.00146539, 0.0626319, -0.997262, 0.0393161, 0.00100071, -0.0393307, -0.999226);
		V1 = Vector3D<double>(-0.448804, 0.845466, 0.350158);
		placePose = Transform3D<double>(V1,R1);
		R2 = Rotation3D<double>(0.880099, -0.474547, -0.0151736, -0.474558, -0.880219, 0.00309925, -0.0148268, 0.0044731, -0.99988);
		V2 =  Vector3D<double>(-0.982333, 0.437329, 0.352617);
		pickPose = Transform3D<double>(V2,R2);
		pickTransforms.push_back(pickPose);
		placeTransforms.push_back(placePose);

		R1 = Rotation3D<double>(0.998036, 0.0626227, -0.00146539, 0.0626319, -0.997262, 0.0393161, 0.00100071, -0.0393307, -0.999226);
		V1 = Vector3D<double>(-0.448804+0.05, 0.845466, 0.350158);
		placePose = Transform3D<double>(V1,R1);
		R2 = Rotation3D<double>(0.880099, -0.474547, -0.0151736, -0.474558, -0.880219, 0.00309925, -0.0148268, 0.0044731, -0.99988);
		V2 =  Vector3D<double>(-0.982333+0.05, 0.437329, 0.352617);
		pickPose = Transform3D<double>(V2,R2);
		pickTransforms.push_back(pickPose);
		placeTransforms.push_back(placePose);
		R1 = Rotation3D<double>(0.998036, 0.0626227, -0.00146539, 0.0626319, -0.997262, 0.0393161, 0.00100071, -0.0393307, -0.999226);
		V1 = Vector3D<double>(-0.448804+0.05, 0.845466, 0.350158-0.05);
		placePose = Transform3D<double>(V1,R1);
		R2 = Rotation3D<double>(0.880099, -0.474547, -0.0151736, -0.474558, -0.880219, 0.00309925, -0.0148268, 0.0044731, -0.99988);
		V2 =  Vector3D<double>(-0.982333+0.05, 0.437329, 0.352617-0.05);
		pickPose = Transform3D<double>(V2,R2);
		pickTransforms.push_back(pickPose);
		placeTransforms.push_back(placePose);
		R1 = Rotation3D<double>(0.998036, 0.0626227, -0.00146539, 0.0626319, -0.997262, 0.0393161, 0.00100071, -0.0393307, -0.999226);
		V1 = Vector3D<double>(-0.448804+0.05, 0.845466, 0.350158);
		placePose = Transform3D<double>(V1,R1);
		R2 = Rotation3D<double>(0.880099, -0.474547, -0.0151736, -0.474558, -0.880219, 0.00309925, -0.0148268, 0.0044731, -0.99988);
		V2 =  Vector3D<double>(-0.982333+0.05, 0.437329, 0.352617);
		pickPose = Transform3D<double>(V2,R2);
		pickTransforms.push_back(pickPose);
		placeTransforms.push_back(placePose);

		R1 = Rotation3D<double>(0.998036, 0.0626227, -0.00146539, 0.0626319, -0.997262, 0.0393161, 0.00100071, -0.0393307, -0.999226);
		V1 = Vector3D<double>(-0.448804-0.05, 0.845466, 0.350158);
		placePose = Transform3D<double>(V1,R1);
		R2 = Rotation3D<double>(0.880099, -0.474547, -0.0151736, -0.474558, -0.880219, 0.00309925, -0.0148268, 0.0044731, -0.99988);
		V2 =  Vector3D<double>(-0.982333-0.05, 0.437329, 0.352617);
		pickPose = Transform3D<double>(V2,R2);
		pickTransforms.push_back(pickPose);
		placeTransforms.push_back(placePose);
		R1 = Rotation3D<double>(0.998036, 0.0626227, -0.00146539, 0.0626319, -0.997262, 0.0393161, 0.00100071, -0.0393307, -0.999226);
		V1 = Vector3D<double>(-0.448804-0.05, 0.845466, 0.350158-0.05);
		placePose = Transform3D<double>(V1,R1);
		R2 = Rotation3D<double>(0.880099, -0.474547, -0.0151736, -0.474558, -0.880219, 0.00309925, -0.0148268, 0.0044731, -0.99988);
		V2 =  Vector3D<double>(-0.982333-0.05, 0.437329, 0.352617-0.05);
		pickPose = Transform3D<double>(V2,R2);
		pickTransforms.push_back(pickPose);
		placeTransforms.push_back(placePose);
		R1 = Rotation3D<double>(0.998036, 0.0626227, -0.00146539, 0.0626319, -0.997262, 0.0393161, 0.00100071, -0.0393307, -0.999226);
		V1 = Vector3D<double>(-0.448804-0.05, 0.845466, 0.350158);
		placePose = Transform3D<double>(V1,R1);
		R2 = Rotation3D<double>(0.880099, -0.474547, -0.0151736, -0.474558, -0.880219, 0.00309925, -0.0148268, 0.0044731, -0.99988);
		V2 =  Vector3D<double>(-0.982333-0.05, 0.437329, 0.352617);
		pickPose = Transform3D<double>(V2,R2);
		pickTransforms.push_back(pickPose);
		placeTransforms.push_back(placePose);

		R1 = Rotation3D<double>(0.998036, 0.0626227, -0.00146539, 0.0626319, -0.997262, 0.0393161, 0.00100071, -0.0393307, -0.999226);
		V1 = Vector3D<double>(-0.448804, 0.845466+0.05, 0.350158);
		placePose = Transform3D<double>(V1,R1);
		R2 = Rotation3D<double>(0.880099, -0.474547, -0.0151736, -0.474558, -0.880219, 0.00309925, -0.0148268, 0.0044731, -0.99988);
		V2 =  Vector3D<double>(-0.982333, 0.437329+0.05, 0.352617);
		pickPose = Transform3D<double>(V2,R2);
		pickTransforms.push_back(pickPose);
		placeTransforms.push_back(placePose);
		R1 = Rotation3D<double>(0.998036, 0.0626227, -0.00146539, 0.0626319, -0.997262, 0.0393161, 0.00100071, -0.0393307, -0.999226);
		V1 = Vector3D<double>(-0.448804, 0.845466+0.05, 0.350158-0.05);
		placePose = Transform3D<double>(V1,R1);
		R2 = Rotation3D<double>(0.880099, -0.474547, -0.0151736, -0.474558, -0.880219, 0.00309925, -0.0148268, 0.0044731, -0.99988);
		V2 =  Vector3D<double>(-0.982333, 0.437329+0.05, 0.352617-0.05);
		pickPose = Transform3D<double>(V2,R2);
		pickTransforms.push_back(pickPose);
		placeTransforms.push_back(placePose);
		R1 = Rotation3D<double>(0.998036, 0.0626227, -0.00146539, 0.0626319, -0.997262, 0.0393161, 0.00100071, -0.0393307, -0.999226);
		V1 = Vector3D<double>(-0.448804, 0.845466+0.05, 0.350158);
		placePose = Transform3D<double>(V1,R1);
		R2 = Rotation3D<double>(0.880099, -0.474547, -0.0151736, -0.474558, -0.880219, 0.00309925, -0.0148268, 0.0044731, -0.99988);
		V2 =  Vector3D<double>(-0.982333, 0.437329+0.05, 0.352617);
		pickPose = Transform3D<double>(V2,R2);
		pickTransforms.push_back(pickPose);
		placeTransforms.push_back(placePose);

		R1 = Rotation3D<double>(0.998036, 0.0626227, -0.00146539, 0.0626319, -0.997262, 0.0393161, 0.00100071, -0.0393307, -0.999226);
		V1 = Vector3D<double>(-0.448804, 0.845466-0.05, 0.350158);
		placePose = Transform3D<double>(V1,R1);
		R2 = Rotation3D<double>(0.880099, -0.474547, -0.0151736, -0.474558, -0.880219, 0.00309925, -0.0148268, 0.0044731, -0.99988);
		V2 =  Vector3D<double>(-0.982333, 0.437329-0.05, 0.352617);
		pickPose = Transform3D<double>(V2,R2);
		pickTransforms.push_back(pickPose);
		placeTransforms.push_back(placePose);
		R1 = Rotation3D<double>(0.998036, 0.0626227, -0.00146539, 0.0626319, -0.997262, 0.0393161, 0.00100071, -0.0393307, -0.999226);
		V1 = Vector3D<double>(-0.448804, 0.845466-0.05, 0.350158-0.05);
		placePose = Transform3D<double>(V1,R1);
		R2 = Rotation3D<double>(0.880099, -0.474547, -0.0151736, -0.474558, -0.880219, 0.00309925, -0.0148268, 0.0044731, -0.99988);
		V2 =  Vector3D<double>(-0.982333, 0.437329-0.05, 0.352617-0.05);
		pickPose = Transform3D<double>(V2,R2);
		pickTransforms.push_back(pickPose);
		placeTransforms.push_back(placePose);
		R1 = Rotation3D<double>(0.998036, 0.0626227, -0.00146539, 0.0626319, -0.997262, 0.0393161, 0.00100071, -0.0393307, -0.999226);
		V1 = Vector3D<double>(-0.448804, 0.845466-0.05, 0.350158);
		placePose = Transform3D<double>(V1,R1);
		R2 = Rotation3D<double>(0.880099, -0.474547, -0.0151736, -0.474558, -0.880219, 0.00309925, -0.0148268, 0.0044731, -0.99988);
		V2 =  Vector3D<double>(-0.982333, 0.437329-0.05, 0.352617);
		pickPose = Transform3D<double>(V2,R2);
		pickTransforms.push_back(pickPose);
		placeTransforms.push_back(placePose);

		vector<Q> rob1;
		vector<Q> rob2;
		Timer t;
		t.resetAndResume();

		rob1.push_back(Q(QHome).getSubPart(0,6));
		rob2.push_back(Q(QHome).getSubPart(6,6));

		rob1.push_back(IK(UR5E1,Q(QHome).getSubPart(0,6),pickTransforms[0]));
		rob1.push_back(IK(UR5E1,Q(QHome).getSubPart(0,6),pickTransforms[1]));
		rob1.push_back(IK(UR5E1,Q(QHome).getSubPart(0,6),pickTransforms[2]));
		rob2.push_back(IK(UR5E2,Q(QHome).getSubPart(6,6),pickTransforms[3]));
		rob2.push_back(IK(UR5E2,Q(QHome).getSubPart(6,6),pickTransforms[4]));
		rob2.push_back(IK(UR5E2,Q(QHome).getSubPart(6,6),pickTransforms[5]));
		rob1.push_back(IK(UR5E1,Q(QHome).getSubPart(0,6),placeTransforms[0]));
		rob1.push_back(IK(UR5E1,Q(QHome).getSubPart(0,6),placeTransforms[1]));
		rob1.push_back(IK(UR5E1,Q(QHome).getSubPart(0,6),placeTransforms[2]));
		rob2.push_back(IK(UR5E2,Q(QHome).getSubPart(6,6),placeTransforms[3]));
		rob2.push_back(IK(UR5E2,Q(QHome).getSubPart(6,6),placeTransforms[4]));
		rob2.push_back(IK(UR5E2,Q(QHome).getSubPart(6,6),placeTransforms[5]));
		rob1.push_back(IK(UR5E1,Q(QHome).getSubPart(0,6),pickTransforms[6]));
		rob1.push_back(IK(UR5E1,Q(QHome).getSubPart(0,6),pickTransforms[7]));
		rob1.push_back(IK(UR5E1,Q(QHome).getSubPart(0,6),pickTransforms[8]));
		rob2.push_back(IK(UR5E2,Q(QHome).getSubPart(6,6),pickTransforms[9]));
		rob2.push_back(IK(UR5E2,Q(QHome).getSubPart(6,6),pickTransforms[10]));
		rob2.push_back(IK(UR5E2,Q(QHome).getSubPart(6,6),pickTransforms[11]));
		rob1.push_back(IK(UR5E1,Q(QHome).getSubPart(0,6),placeTransforms[6]));
		rob1.push_back(IK(UR5E1,Q(QHome).getSubPart(0,6),placeTransforms[7]));
		rob1.push_back(IK(UR5E1,Q(QHome).getSubPart(0,6),placeTransforms[8]));
		rob2.push_back(IK(UR5E2,Q(QHome).getSubPart(6,6),placeTransforms[9]));
		rob2.push_back(IK(UR5E2,Q(QHome).getSubPart(6,6),placeTransforms[10]));
		rob2.push_back(IK(UR5E2,Q(QHome).getSubPart(6,6),placeTransforms[11]));
		rob1.push_back(IK(UR5E1,Q(QHome).getSubPart(0,6),pickTransforms[12]));
		rob1.push_back(IK(UR5E1,Q(QHome).getSubPart(0,6),pickTransforms[13]));
		rob1.push_back(IK(UR5E1,Q(QHome).getSubPart(0,6),pickTransforms[14]));
		rob1.push_back(IK(UR5E1,Q(QHome).getSubPart(0,6),placeTransforms[12]));
		rob1.push_back(IK(UR5E1,Q(QHome).getSubPart(0,6),placeTransforms[13]));
		rob1.push_back(IK(UR5E1,Q(QHome).getSubPart(0,6),placeTransforms[14]));

		rob1.push_back(Q(QHome).getSubPart(0,6));
		rob2.push_back(Q(QHome).getSubPart(6,6));

		rw::trajectory::QPath ret1;
		rw::trajectory::QPath ret2;

		calculatePrioritizedPath(rob1, rob2, ret1, ret2, 0.25, 75);
		longPath = joinPaths(ret1, ret2);
		t.pause();
		cout<< "steps, motion, plan time: " << longPath.size() <<", "<< totalMotion(longPath)<<", " << t.getTime() << endl;

		//saveLongPath();
	}
	void plan()
	{
		longPath.clear();
		vector<double> QHome{3.28, -2.2, 2.4, -1.8, -1.5, 0, -0.13, -0.78, -2.2, -1.7, 1.5, 0};
		vector<double> QvecFrom{2.21, -0.89, 1.13, -1.80, -1.61, -0.14,-0.68, -2.27, -1.08, -1.35, 1.57, 0.02};
		vector<double> QvecTo{3.64, -0.71, 0.83, -1.68, -1.59, 1.26, 0.91, -2.38, -0.87, -1.48, 1.58, -1.48};

		vector<Transform3D<double>> pickTransforms;
		vector<Transform3D<double>> placeTransforms;

		Rotation3D<double> R1(0.998036, 0.0626227, -0.00146539, 0.0626319, -0.997262, 0.0393161, 0.00100071, -0.0393307, -0.999226);
		Vector3D<double> V1(-0.448804, 0.845466, 0.350158);
		Transform3D<double> placePose(V1,R1);
		Rotation3D<double> R2(0.039, 0.998, 0.047, -0.997, 0.036, 0.065, 0.063, -0.049, 0.997);
		Rotation3D<double> correction(0,0,-1,1,0,0,0,-1,0);
		Vector3D<double> V2(-1.091, 0.269, 0.346);
		Transform3D<double> pickPose(V2,correction*R2);
		pickTransforms.push_back(pickPose);
		placeTransforms.push_back(placePose);

		// Rotation3D<double> R1(0.998036, 0.0626227, -0.00146539, 0.0626319, -0.997262, 0.0393161, 0.00100071, -0.0393307, -0.999226);
		// Vector3D<double> V1(-0.448804, 0.845466, 0.350158);
		// Transform3D<double> placePose(V1,R1);
		// Rotation3D<double> R2(0.880099, -0.474547, -0.0151736, -0.474558, -0.880219, 0.00309925, -0.0148268, 0.0044731, -0.99988);
		// Vector3D<double> V2(-0.982333, 0.437329, 0.352617);
		// Transform3D<double> pickPose(V2,R2);
		// pickTransforms.push_back(pickPose);
		// placeTransforms.push_back(placePose);

		R1 = Rotation3D<double>(0.998036, 0.0626227, -0.00146539, 0.0626319, -0.997262, 0.0393161, 0.00100071, -0.0393307, -0.999226);
		V1 = Vector3D<double>(-0.448804+0.05, 0.845466, 0.350158);
		placePose = Transform3D<double>(V1,R1);
		R2 = Rotation3D<double>(0.880099, -0.474547, -0.0151736, -0.474558, -0.880219, 0.00309925, -0.0148268, 0.0044731, -0.99988);
		V2 =  Vector3D<double>(-0.982333+0.05, 0.437329, 0.352617);
		pickPose = Transform3D<double>(V2,R2);
		pickTransforms.push_back(pickPose);
		placeTransforms.push_back(placePose);

		R1 = Rotation3D<double>(0.998036, 0.0626227, -0.00146539, 0.0626319, -0.997262, 0.0393161, 0.00100071, -0.0393307, -0.999226);
		V1 = Vector3D<double>(-0.448804-0.05, 0.845466, 0.350158);
		placePose = Transform3D<double>(V1,R1);
		R2 = Rotation3D<double>(0.880099, -0.474547, -0.0151736, -0.474558, -0.880219, 0.00309925, -0.0148268, 0.0044731, -0.99988);
		V2 =  Vector3D<double>(-0.982333-0.05, 0.437329, 0.352617);
		pickPose = Transform3D<double>(V2,R2);
		pickTransforms.push_back(pickPose);
		placeTransforms.push_back(placePose);

		R1 = Rotation3D<double>(0.998036, 0.0626227, -0.00146539, 0.0626319, -0.997262, 0.0393161, 0.00100071, -0.0393307, -0.999226);
		V1 = Vector3D<double>(-0.448804, 0.845466+0.05, 0.350158);
		placePose = Transform3D<double>(V1,R1);
		R2 = Rotation3D<double>(0.880099, -0.474547, -0.0151736, -0.474558, -0.880219, 0.00309925, -0.0148268, 0.0044731, -0.99988);
		V2 =  Vector3D<double>(-0.982333, 0.437329+0.05, 0.352617);
		pickPose = Transform3D<double>(V2,R2);
		pickTransforms.push_back(pickPose);
		placeTransforms.push_back(placePose);

		R1 = Rotation3D<double>(0.998036, 0.0626227, -0.00146539, 0.0626319, -0.997262, 0.0393161, 0.00100071, -0.0393307, -0.999226);
		V1 = Vector3D<double>(-0.448804, 0.845466-0.05, 0.350158);
		placePose = Transform3D<double>(V1,R1);
		R2 = Rotation3D<double>(0.880099, -0.474547, -0.0151736, -0.474558, -0.880219, 0.00309925, -0.0148268, 0.0044731, -0.99988);
		V2 =  Vector3D<double>(-0.982333, 0.437329-0.05, 0.352617);
		pickPose = Transform3D<double>(V2,R2);
		pickTransforms.push_back(pickPose);
		placeTransforms.push_back(placePose);
		Timer t;
		t.resetAndResume();
		// //Only robot 1:
		// Q q1 = IK(UR5E1,Q(6,3.28, -2.2, 2.4, -1.8, -1.5, 0),pickTransforms[0]);
		// Q q2 = Q(6,-0.13, -0.78, -2.2, -1.7, 1.5, 0);
		// vector<double> q = {q1[0],q1[1],q1[2],q1[3],q1[4],q1[5],q2[0],q2[1],q2[2],q2[3],q2[4],q2[5]};
		// calculateSBLPath(Q(QHome),Q(q));
		// appendVerticalGrasp(Q(q),1,0,0,0);
		// longPath.insert(longPath.end(),path.begin(),path.end());
		
		// for(size_t i = 1; i < pickTransforms.size(); i++)
		// {
		// 		q1 = IK(UR5E1,Q(6,longPath[longPath.size()-1][0], longPath[longPath.size()-1][1], longPath[longPath.size()-1][2], longPath[longPath.size()-1][3], longPath[longPath.size()-1][4], longPath[longPath.size()-1][5]),placeTransforms[i-1]);
		//  		q2 = Q(6,-0.13, -0.78, -2.2, -1.7, 1.5, 0);
		//  		q = {q1[0],q1[1],q1[2],q1[3],q1[4],q1[5],q2[0],q2[1],q2[2],q2[3],q2[4],q2[5]};
		//  		calculateSBLPath(longPath[longPath.size()-1],Q(q));
		//  		appendVerticalGrasp(Q(q),1,0,0,0);
		//  		longPath.insert(longPath.end(),path.begin(),path.end());
		// 		q1 = IK(UR5E1,Q(6,longPath[longPath.size()-1][0], longPath[longPath.size()-1][1], longPath[longPath.size()-1][2], longPath[longPath.size()-1][3], longPath[longPath.size()-1][4], longPath[longPath.size()-1][5]),pickTransforms[i]);
		//  		q2 = Q(6,-0.13, -0.78, -2.2, -1.7, 1.5, 0);
		//  		q = {q1[0],q1[1],q1[2],q1[3],q1[4],q1[5],q2[0],q2[1],q2[2],q2[3],q2[4],q2[5]};
		//  		calculateSBLPath(longPath[longPath.size()-1],Q(q));
		//  		appendVerticalGrasp(Q(q),1,0,0,0);
		//  		longPath.insert(longPath.end(),path.begin(),path.end());
		// }
		
		// q1 = IK(UR5E1,Q(6,longPath[longPath.size()-1][0], longPath[longPath.size()-1][1], longPath[longPath.size()-1][2], longPath[longPath.size()-1][3], longPath[longPath.size()-1][4], longPath[longPath.size()-1][5]),placeTransforms[placeTransforms.size()-1]);
		// q2 = Q(6,-0.13, -0.78, -2.2, -1.7, 1.5, 0);
		// q = {q1[0],q1[1],q1[2],q1[3],q1[4],q1[5],q2[0],q2[1],q2[2],q2[3],q2[4],q2[5]};
		// calculateSBLPath(longPath[longPath.size()-1],Q(q));
		// appendVerticalGrasp(Q(q),1,0,0,0);
		// longPath.insert(longPath.end(),path.begin(),path.end());
		// calculateSBLPath(longPath[longPath.size()-1],QHome);
		// longPath.insert(longPath.end(),path.begin(),path.end());
		// graspReleaseIndex.clear();

		
		
		// Two Robots:
		//home();
		
		Q q1 = IK(UR5E1,Q(6,3.28, -2.2, 2.4, -1.8, -1.5, 0),pickTransforms[0]);
		Q q2 = Q(6,-0.13, -0.78, -2.2, -1.7, 1.5, 0);
		vector<double> q = {q1[0],q1[1],q1[2],q1[3],q1[4],q1[5],q2[0],q2[1],q2[2],q2[3],q2[4],q2[5]};
		calculateSBLPath(Q(QHome),Q(q));
		appendVerticalGrasp(Q(q),1,0,1,0);
		longPath.insert(longPath.end(),path.begin(),path.end());
		for(size_t i = 1; i < pickTransforms.size(); i++)
		{
			if(i%2)
			{
				q1 = IK(UR5E1,Q(6,longPath[longPath.size()-1][0], longPath[longPath.size()-1][1], longPath[longPath.size()-1][2], longPath[longPath.size()-1][3], longPath[longPath.size()-1][4], longPath[longPath.size()-1][5]),placeTransforms[i-1]);
				q2 = IK(UR5E2,Q(6,longPath[longPath.size()-1][6], longPath[longPath.size()-1][7], longPath[longPath.size()-1][8], longPath[longPath.size()-1][9], longPath[longPath.size()-1][10], longPath[longPath.size()-1][11]),pickTransforms[i]);
				q = {q1[0],q1[1],q1[2],q1[3],q1[4],q1[5],q2[0],q2[1],q2[2],q2[3],q2[4],q2[5]};
				calculateSBLPath(longPath[longPath.size()-1],Q(q));
				appendVerticalGrasp(Q(q),1,1,0,1);
				longPath.insert(longPath.end(),path.begin(),path.end());}
			else
			{
				q1 = IK(UR5E1,Q(6,longPath[longPath.size()-1][0], longPath[longPath.size()-1][1], longPath[longPath.size()-1][2], longPath[longPath.size()-1][3], longPath[longPath.size()-1][4], longPath[longPath.size()-1][5]),pickTransforms[i]);
				q2 = IK(UR5E2,Q(6,longPath[longPath.size()-1][6], longPath[longPath.size()-1][7], longPath[longPath.size()-1][8], longPath[longPath.size()-1][9], longPath[longPath.size()-1][10], longPath[longPath.size()-1][11]),placeTransforms[i-1]);
				q = {q1[0],q1[1],q1[2],q1[3],q1[4],q1[5],q2[0],q2[1],q2[2],q2[3],q2[4],q2[5]};
				calculateSBLPath(longPath[longPath.size()-1],Q(q));
				appendVerticalGrasp(Q(q),1,1,1,0);
				longPath.insert(longPath.end(),path.begin(),path.end());
			}

		}
		if(pickTransforms.size()%2)
		{
			q1 = IK(UR5E1,Q(6,longPath[longPath.size()-1][0], longPath[longPath.size()-1][1], longPath[longPath.size()-1][2], longPath[longPath.size()-1][3], longPath[longPath.size()-1][4], longPath[longPath.size()-1][5]),placeTransforms[placeTransforms.size()-1]);
			q2 = Q(6,-0.13, -0.78, -2.2, -1.7, 1.5, 0);
			q = {q1[0],q1[1],q1[2],q1[3],q1[4],q1[5],q2[0],q2[1],q2[2],q2[3],q2[4],q2[5]};
			calculateSBLPath(longPath[longPath.size()-1],Q(q));
			appendVerticalGrasp(Q(q),1,0,0,0);
			longPath.insert(longPath.end(),path.begin(),path.end());
		}
		else
		{
			q1 = Q(6,3.28, -2.2, 2.4, -1.8, -1.5, 0);
			q2 = IK(UR5E2,Q(6,longPath[longPath.size()-1][6], longPath[longPath.size()-1][7], longPath[longPath.size()-1][8], longPath[longPath.size()-1][9], longPath[longPath.size()-1][10], longPath[longPath.size()-1][11]),placeTransforms[placeTransforms.size()-1]);
			q = {q1[0],q1[1],q1[2],q1[3],q1[4],q1[5],q2[0],q2[1],q2[2],q2[3],q2[4],q2[5]};
			calculateSBLPath(longPath[longPath.size()-1],Q(q));
			appendVerticalGrasp(Q(q),0,1,0,0);
			longPath.insert(longPath.end(),path.begin(),path.end());
		}
		calculateSBLPath(longPath[longPath.size()-1],Q(QHome));
		longPath.insert(longPath.end(),path.begin(),path.end());	
		//graspReleaseIndex.clear();
		t.pause();
		cout<< "steps, motion, plan time: " << longPath.size() <<", "<< totalMotion(longPath)<<", " << t.getTime() << endl;
		//calcPathlength();
		//saveLongPath();

		
	}
	void calcPathlength()
	{
		// // Two robots:
		// double pathLengthNorm2 = 0.0;
		// for(size_t i = 0; i < longPath.size()-1; i++)
		// {
		// 	pathLengthNorm2 += (longPath[i]-longPath[i+1]).norm2();
		// }
		// cout << "Pathlength norm2: " << pathLengthNorm2 << endl; 		

		// Only robot1:
		double pathLengthNorm2 = 0.0;
		for(size_t i = 0; i < longPath.size()-1; i++)
		{
			pathLengthNorm2 += (Q(6,longPath[i][0],longPath[i][1],longPath[i][2],longPath[i][3],longPath[i][4],longPath[i][5])-Q(6,longPath[i+1][0],longPath[i+1][1],longPath[i+1][2],longPath[i+1][3],longPath[i+1][4],longPath[i+1][5])).norm2();
		}
		cout << "Pathlength norm2: " << pathLengthNorm2 << endl; 		
	}

	bool calculateSBLPath(Q from, Q to){
		const State state = wc->getDefaultState();
		test();
		// Setting up tree device 
		rw::kinematics::Frame* startFrame = wc->findFrame("WORLD");

		rw::kinematics::Frame* endFrame1 = UR5E1->getEnd();
		rw::kinematics::Frame* endFrame2 = UR5E2->getEnd();
		vector<rw::kinematics::Frame *> endFrames;
		endFrames.push_back(endFrame1);
		endFrames.push_back(endFrame2);
		string treeName = "robotTree";

		rw::models::TreeDevice::Ptr robotTree = new rw::models::TreeDevice(startFrame,endFrames,treeName,state);

		cout <<"str: "<< robotTree->getDOF() << endl;

		// Setting up collision detector
		rw::proximity::CollisionDetector::Ptr detector = new rw::proximity::CollisionDetector(wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy());

		// QConstraint::Ptr q_constraint = QConstraint::makeFixed(false); 
		QConstraint::Ptr q_constraint = QConstraint::make(detector, robotTree, state); 
		QEdgeConstraintIncremental::Ptr q_edge_constraint = QEdgeConstraintIncremental::makeDefault(q_constraint, robotTree);

		double expandRadius = 0.25; 
		double connectRadius = 0.6;
						
		SBLSetup setup = SBLSetup::make(q_constraint, q_edge_constraint, robotTree, expandRadius, connectRadius);
		QToQPlanner::Ptr sbl_planner = SBLPlanner::makeQToQPlanner(setup);


		if (!checkCollisions(robotTree, state, *detector, from))
			{
				cout << "Start Q in collision" << endl;
				throw exception();
			}
		if (!checkCollisions(robotTree, state, *detector, to))
			{
				cout << "End Q in collision" << endl;
				throw exception();
			}

		//cout << "Planning from " << from << " to " << to << endl;
		Timer t;
		t.resetAndResume();
		QPath sbl_path; 
		sbl_planner->query(from,to,sbl_path,MAXTIME);
		t.pause();
		path = sbl_path; 

		optimizePath(detector, robotTree);
		
		if (t.getTime() >= MAXTIME or !path.size()) {
			cout << "Notice: max time of " << MAXTIME << " seconds reached." << endl;
			return false; 
		}else{
			return true;
		}
		
	}
	double totalMotion(rw::trajectory::QPath& path1)
	{
		double totalMot = 0.0;
		for (size_t i = 0; i < path1.size()-1; i++)
		{
			totalMot += (path1[i]-path1[i+1]).norm1();
		}
		return totalMot;
	}
	void optimizePath(rw::proximity::CollisionDetector::Ptr detector, rw::models::Device::Ptr device)
	{	
	rw::pathplanning::PlannerConstraint constraint = rw::pathplanning::PlannerConstraint::make(detector,device,state);
	rw::math::QMetric::Ptr metric = rw::math::MetricFactory::makeInfinity<rw::math::Q>();
	rwlibs::pathoptimization::PathLengthOptimizer pathOptimizer(constraint,metric);
	Timer t;
	t.resetAndResume();
	path = pathOptimizer.partialShortCut(path);
	t.pause();
	cout << path.size() << ", " << t.getTime() << endl;
	}
	void savePath()
	{
		savePathToLua(path);
	}
	void saveLongPath()
	{
		savePathToLua(longPath);
	}

	QPath getPath(){
		return path;
	}

	bool testMove()
	{
		Q q1 = getQ1();
		Q q2 = getQ2();
		
		cout << "rob1: " << q1 << endl;
		cout << "rob2: " << q2 << endl;

		q1[0] = q1[0]-0.1;
		q2[0] = q2[0]-0.1;

		cout << "rob1 2: " << q1 << endl;
		cout << "rob2 2: " << q2 << endl;

		vector<double> qvector = {q1[0],q1[1],q1[2],q1[3],q1[4],q1[5],q2[0],q2[1],q2[2],q2[3],q2[4],q2[5]};
		Q qsamlet(qvector);
		cout << "samlet q: " << qvector << endl; 
		setQ(qsamlet);
		return 1;
	}

};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "URRobot");

	URRobot robot;

	//Q home(6, 0, -1.5707, 0, -1.5707, 0, 0);
	
	vector<double> QHome{3.28, -2.2, 2.4, -1.8, -1.5, 0, -0.13, -0.78, -2.2, -1.7, 1.5, 0};
	vector<double> QvecFrom{2.21, -0.89, 1.13, -1.80, -1.61, -0.14,-0.68, -2.27, -1.08, -1.35, 1.57, 0.02};
	vector<double> QvecTo{3.64, -0.71, 0.83, -1.68, -1.59, 1.26, 0.91, -2.38, -0.87, -1.48, 1.58, -1.48};

	rw::math::Q home(QHome);
	rw::math::Q from(QvecFrom);
	rw::math::Q to(QvecTo);

	while(true){
		std::cout << "Current joint config:" << std::endl << robot.getQ1() << std::endl << std::endl;
		cout << "Input command: " << endl; 
		string input; 
		cin >> input;
		if(input == "sbl1"){
			if(robot.calculateSBLPath(home, from)){
				cout << "Calculated SBL path!" << endl; 
			}else{
				cout << "Could not calculate SBL path!" << endl; 
			}
		}else if(input == "sbl2"){
			if(robot.calculateSBLPath(from, to)){
				cout << "Calculated SBL path!" << endl; 
			}else{
				cout << "Could not calculate SBL path!" << endl; 
			}
		}else if(input == "test"){
			if(robot.testMove()){
				cout << "test move" << endl; 
			}else{
				cout << "not test move" << endl; 
			}
		}else if(input == "home"){
			robot.home();
		}else if(input == "follow"){
			cout << "following" << endl; 
			robot.followTrajectory();

		}else if(input == "save"){
			robot.savePath();
			cout << "Saved" << endl; 
		}else if(input == "g1"){
			robot.graspGripper1();
		}else if(input == "g2"){
			robot.graspGripper2();
		}else if(input == "r1"){
			robot.releaseGripper1();
		}else if(input == "r2"){
			robot.releaseGripper2();
		}else if(input == "plan"){
			robot.plan();
		}else if(input == "plan2"){
			robot.plan2();
		}else if(input == "start"){
			QPath path = robot.getPath(); 
			if(path.size()){
				cout << "Itteration through path" << endl; 
				for (QPath::iterator it = path.begin(); it < path.end(); it++) {
					cout << *it << endl;
					if(robot.setQ(*it))
						cout << std::endl << "New joint config:" << endl << robot.getQ1() << endl;
					else
						cout << endl << "Failed to move robot" << endl;
				}
			}else{
				cout << "The path has no size!" << endl; 
			}
			
		}else if(input == "stop"){
			cout << "Ending program" << endl; 
			break; 
		}else{
			cout << "Unkown command " << input << endl; 
		}
		ros::spinOnce();
	}

	return 0; 

}
