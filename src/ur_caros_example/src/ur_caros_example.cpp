#include <iostream>
#include "rw/rw.hpp"
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include "caros/serial_device_si_proxy.h"
#include "ros/package.h"

// SBL includes
#include <rwlibs/pathplanners/sbl/SBLPlanner.hpp>
#include <rwlibs/pathplanners/sbl/SBLOptions.hpp>
#include <rwlibs/pathplanners/sbl/SBLSetup.hpp>
#include <rwlibs/pathplanners/sbl/SBLInternal.hpp>

#include <rw/pathplanning.hpp>

#include <caros_common_msgs/Q.h>
#include <caros/common_robwork.h>

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

#define MAXTIME 20

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

class URRobot {
	using Q = rw::math::Q;

private:
	ros::NodeHandle nh;
	WorkCell::Ptr wc;
	Device::Ptr UR5E1;
	Device::Ptr UR5E2; 
	State state;
	caros::SerialDeviceSIProxy* robot;
	QPath path; 

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
		robot = new caros::SerialDeviceSIProxy(nh, "caros_universalrobot");

		// Publisher for robwork plugin
		pub_multi_ur5e = nh.advertise<caros_control_msgs::RobotState>("/robot_state/multirobot/ur5e", 1);


		// Wait for first state message, to make sure robot is ready
		ros::topic::waitForMessage<caros_control_msgs::RobotState>("/caros_universalrobot/caros_serial_device_service_interface/robot_state", nh);
	    ros::spinOnce();
	}

	Q getQ()
	{
		// spinOnce processes one batch of messages, calling all the callbacks
	    ros::spinOnce();
	    Q q = robot->getQ();
		UR5E1->setQ(q, state);
	    return q;
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
        float speed = 0.5;
		rw::math::Q q1(6, q[0], q[1], q[2], q[3], q[4], q[5]); 
		if (robot->movePtp(q1, speed) && setDoubleQ(q)) { 
			return true;
		} else
			return false;
	}

	bool calculateRRTPath(Q from, Q to){/*
		const State state = wc->getDefaultState();

		CollisionDetector detector(wc, ProximityStrategyFactory::makeDefaultCollisionStrategy());
		PlannerConstraint constraint = PlannerConstraint::make(&detector,robotTree,state);

		QSampler::Ptr sampler = QSampler::makeConstrained(QSampler::makeUniform(robotTree),constraint.getQConstraintPtr());

		QMetric::Ptr metric = MetricFactory::makeEuclidean<Q>();
		double extend = 0.01;
		QToQPlanner::Ptr planner = RRTPlanner::makeQToQPlanner(constraint, sampler, metric, extend, RRTPlanner::RRTConnect);

		if (!checkCollisions(robotTree, state, detector, from))
			return 0;
		if (!checkCollisions(robotTree, state, detector, to))
			return 0;

		cout << "Planning from " << from << " to " << to << endl;
		QPath rrt_path;

		Timer t;
		t.resetAndResume();
		planner->query(from,to,rrt_path,MAXTIME);
		t.pause();
		path = rrt_path; 
		cout << "Path of length " << path.size() << " found in " << t.getTime() << " seconds." << endl;
		if (t.getTime() >= MAXTIME) {
			cout << "Notice: max time of " << MAXTIME << " seconds reached." << endl;
			return false; 
		}else{
			return true;
		}
	*/	
	}

	bool calculateSBLPath(Q from, Q to){
		const State state = wc->getDefaultState();

		// Setting up tree device 
		rw::kinematics::Frame* startFrame = wc->findFrame("WORLD");

		rw::kinematics::Frame* endFrame1 = UR5E1->getEnd();
		rw::kinematics::Frame* endFrame2 = UR5E2->getEnd();
		vector<rw::kinematics::Frame *> endFrames;
		endFrames.push_back(endFrame1);
		endFrames.push_back(endFrame2);
		string treeName = "robotTree";

		TreeDevice robotTree(startFrame,endFrames,treeName,state);

		cout <<"str: "<< robotTree.getDOF() << endl;

		// Setting up collision detector
		
		CollisionDetector detector(wc, ProximityStrategyFactory::makeDefaultCollisionStrategy());

	//	QConstraint::Ptr q_constraint = QConstraint::makeFixed(false); 
		QConstraint::Ptr q_constraint = QConstraint::make(&detector, &robotTree, state); 
		QEdgeConstraintIncremental::Ptr q_edge_constraint = QEdgeConstraintIncremental::makeDefault(q_constraint, &robotTree);

		double expandRadius = 0.01; 
		double connectRadius = 0.03; //0.01; 

		SBLSetup setup = SBLSetup::make(q_constraint, q_edge_constraint, &robotTree, expandRadius, connectRadius);
		QToQPlanner::Ptr sbl_planner = SBLPlanner::makeQToQPlanner(setup);


		if (!checkCollisions(&robotTree, state, detector, from))
			return 0;
		if (!checkCollisions(&robotTree, state, detector, to))
			return 0;

		cout << "Planning from " << from << " to " << to << endl;
		Timer t;
		t.resetAndResume();
		QPath sbl_path; 
		sbl_planner->query(from,to,sbl_path,MAXTIME);
		t.pause();
		path = sbl_path; 
		cout << "Path of length " << path.size() << " found in " << t.getTime() << " seconds." << endl;
		if (t.getTime() >= MAXTIME or !path.size()) {
			cout << "Notice: max time of " << MAXTIME << " seconds reached." << endl;
			return false; 
		}else{
			return true;
		}
		
	}

	QPath getPath(){
		return path;
	}

};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "URRobot");

	URRobot robot;

	Q home(6, 0, -1.5707, 0, -1.5707, 0, 0);

	vector<double> QvecFrom{2, -0.8, 1.0, -1.5, -1.5, 0, -1.0, -2.2, -1.2, -1.5, 1.5, 0};
	vector<double> QvecTo{1.2, -0.8, 1.0, -1.5, -1.5, 0, -2.0, -2.2, -1.2, -1.5, 1.5, 0};

	rw::math::Q from(QvecFrom);
	rw::math::Q to(QvecTo);

	while(true){
		std::cout << "Current joint config:" << std::endl << robot.getQ() << std::endl << std::endl;
		cout << "Input command: " << endl; 
		string input; 
		cin >> input;
		if(input == "sbl"){
			if(robot.calculateSBLPath(from, to)){
				cout << "Calculated SBL path!" << endl; 
			}else{
				cout << "Could not calculate SBL path!" << endl; 
			}
		}else if(input == "rrt"){
			if(robot.calculateRRTPath(from, to)){
				cout << "Calculated RRT path!" << endl; 
			}else{
				cout << "Could not calculate RRT path!" << endl; 
			}
		}else if(input == "start"){
			QPath path = robot.getPath(); 
			if(path.size()){
				cout << "Itteration through path" << endl; 
				for (QPath::iterator it = path.begin(); it < path.end(); it++) {
					cout << *it << endl;
					if(robot.setQ(*it))
						cout << std::endl << "New joint config:" << endl << robot.getQ() << endl;
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
