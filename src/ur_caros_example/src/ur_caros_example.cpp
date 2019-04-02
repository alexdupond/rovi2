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

#define MAXTIME 60

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
	Device::Ptr device;
	State state;
	caros::SerialDeviceSIProxy* robot;
	QPath path; 


public:
	URRobot()
	{
		auto packagePath = ros::package::getPath("ur_caros_example");
		wc = WorkCellLoader::Factory::load(packagePath + "/WorkCell/Scene.wc.xml");
		device = wc->findDevice("UR5");
		state = wc->getDefaultState();
		robot = new caros::SerialDeviceSIProxy(nh, "caros_universalrobot");

		// Setting up 

		// Wait for first state message, to make sure robot is ready
		ros::topic::waitForMessage<caros_control_msgs::RobotState>("/caros_universalrobot/caros_serial_device_service_interface/robot_state", nh);
	    ros::spinOnce();
	}

	Q getQ()
	{
		// spinOnce processes one batch of messages, calling all the callbacks
	    ros::spinOnce();
	    Q q = robot->getQ();
		device->setQ(q, state);
	    return q;
	}

	bool setQ(Q q)
	{
		// Tell robot to move to joint config q
        float speed = 0.5;
		if (robot->movePtp(q, speed)) { 
			return true;
		} else
			return false;
	}

	bool calculateRRTPath(Q from, Q to){
		const State state = wc->getDefaultState();

		CollisionDetector detector(wc, ProximityStrategyFactory::makeDefaultCollisionStrategy());
		PlannerConstraint constraint = PlannerConstraint::make(&detector,device,state);

		QSampler::Ptr sampler = QSampler::makeConstrained(QSampler::makeUniform(device),constraint.getQConstraintPtr());

		QMetric::Ptr metric = MetricFactory::makeEuclidean<Q>();
		double extend = 0.01;
		QToQPlanner::Ptr planner = RRTPlanner::makeQToQPlanner(constraint, sampler, metric, extend, RRTPlanner::RRTConnect);

		if (!checkCollisions(device, state, detector, from))
			return 0;
		if (!checkCollisions(device, state, detector, to))
			return 0;

		cout << "Planning from " << from << " to " << to << endl;
		QPath rrt_path;

		Timer t;
		t.resetAndResume();
		planner->query(from,to,path,MAXTIME);
		t.pause();
		path = rrt_path; 
		cout << "Path of length " << path.size() << " found in " << t.getTime() << " seconds." << endl;
		if (t.getTime() >= MAXTIME) {
			cout << "Notice: max time of " << MAXTIME << " seconds reached." << endl;
			return false; 
		}else{
			return true;
		}
		
	}

	bool calculateSBLPath(Q from, Q to){
		const State state = wc->getDefaultState();

		QConstraint::Ptr q_constraint = QConstraint::makeFixed(false); 
		QEdgeConstraintIncremental::Ptr q_edge_constraint = QEdgeConstraintIncremental::makeDefault(q_constraint, device);
		double expandRadius = 0.0001; 
		double connectRadius = 0.05; 
		SBLSetup setup = SBLSetup::make(q_constraint, q_edge_constraint, device, expandRadius, connectRadius);
		QToQPlanner::Ptr sbl_planner = SBLPlanner::makeQToQPlanner(setup);

		CollisionDetector detector(wc, ProximityStrategyFactory::makeDefaultCollisionStrategy());


		if (!checkCollisions(device, state, detector, from))
			return 0;
		if (!checkCollisions(device, state, detector, to))
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
	Q from(6, 0, -1.5707, 0, -1.5707, 0, 0);
	Q to(6, -0.5, -1.0, 0, -1.5707, 0.1, 0.9);

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
