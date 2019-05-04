#include "urRobot.h"

#define PUB_UR5E_1 "/robot_state/ur5e1"
#define PUB_UR5E_2 "/robot_state/ur5e2"

URRobot::URRobot(ros::NodeHandle* nodehandler):nh(*nodehandler)
{
	auto packagePath = ros::package::getPath("ur_caros_example");
	wc = rw::loaders::WorkCellLoader::Factory::load(packagePath + "/RoviScene/Scene.xml");
	UR5E1 = wc->findDevice("UR5e_1");
	UR5E2 = wc->findDevice("UR5e_2");	
	state = wc->getDefaultState();
	robot = new caros::SerialDeviceSIProxy(nh, "caros_universalrobot");

	// Publisher for robwork plugin
	pub_ur5e1 = nh.advertise<caros_control_msgs::RobotState>(PUB_UR5E_1, 1);
	pub_ur5e2 = nh.advertise<caros_control_msgs::RobotState>(PUB_UR5E_2, 1);



	// Wait for first state message, to make sure robot is ready
	ros::topic::waitForMessage<caros_control_msgs::RobotState>("/caros_universalrobot/caros_serial_device_service_interface/robot_state", nh);
	ros::spinOnce();
}

bool URRobot::calculatePrioritizedPath(vector<rw::math::Q>& robot1, vector<rw::math::Q>& robot2, rw::trajectory::QPath& result1, rw::trajectory::QPath& result2){
	double extend = 0.1; 
	PrioritizedPlanner planner(wc, UR5E1, UR5E2, extend); 

	cout << "Robot 1 size = " << robot1.size() << ", and Robot 2 size = " << robot2.size() << endl; 
	if(robot1.size() && robot2.size()){

		if(!planner.calculateRRTPath(robot1, result1))
			return false;

		cout << "Finished planning for robot 1 - Total size = " << result1.size() << endl; 
		
		if(planner.calculateDynamicRRTPath(robot2, result1, result2)){
			cout << "Finished planning for robot 2 " << endl; 
			return true; 
		}else{
			return false; 
		}
	}
	return false; 
};



bool URRobot::checkCollisions(rw::models::Device::Ptr device, const rw::kinematics::State &state, const rw::proximity::CollisionDetector &detector, const rw::math::Q &q) {
	rw::kinematics::State testState;
	rw::proximity::CollisionDetector::QueryResult data;
	bool colFrom;

	testState = state;
	device->setQ(q,testState);
	colFrom = detector.inCollision(testState,&data);
	if (colFrom) {
		cerr << "Configuration in collision: " << q << endl;
		cerr << "Colliding frames: " << endl;
		rw::kinematics::FramePairSet fps = data.collidingFrames;
		for (rw::kinematics::FramePairSet::iterator it = fps.begin(); it != fps.end(); it++) {
			cerr << (*it).first->getName() << " " << (*it).second->getName() << endl;
		}
		return false;
	}
	return true;
}

rw::math::Q URRobot::getQ()
{
	// spinOnce processes one batch of messages, calling all the callbacks
    ros::spinOnce();
    rw::math::Q q = robot->getQ();
	UR5E1->setQ(q, state);
    return q;
}

bool URRobot::setQRobot1(rw::math::Q q){
	// Creating  the message to be send to the robots
	caros_common_msgs::Q q_ros = caros::toRos(q);
	caros_control_msgs::RobotState msg_q;
	msg_q.q = q_ros; 
	pub_ur5e1.publish(msg_q); 
	return true; 
}

bool URRobot::setQRobot2(rw::math::Q q){
	// Creating  the message to be send to the robots
	caros_common_msgs::Q q_ros = caros::toRos(q);
	caros_control_msgs::RobotState msg_q;
	msg_q.q = q_ros; 
	pub_ur5e2.publish(msg_q); 
	return true; 
}

bool URRobot::setQ(rw::math::Q q1, rw::math::Q q2){
	// Setting robot speed
    float speed = 0.5;
    // Setting a configuration to send to the URSIM 

	if (/*robot->moveServoQ(q1, 0.18, 0.04, 100)*/robot->movePtp(q1, speed) && setQRobot1(q1) && setQRobot2(q2)) 
	{ 
		return true;
	} else
		return false;

}


bool URRobot::calculateSBLPath(rw::math::Q from, rw::math::Q to){
	//| ----------------- Setting up tree device ----------------------| 
	rw::kinematics::Frame* startFrame = wc->findFrame("WORLD");

	rw::kinematics::Frame* endFrame1 = UR5E1->getEnd();
	rw::kinematics::Frame* endFrame2 = UR5E2->getEnd();
	vector<rw::kinematics::Frame *> endFrames;
	endFrames.push_back(endFrame1);
	endFrames.push_back(endFrame2);

    rw::kinematics::State state = wc->getDefaultState();
	string treeName = "robotTree";

	rw::models::TreeDevice robotTree(startFrame,endFrames,treeName,state);

	cout <<"str: "<< robotTree.getDOF() << endl;

	//| ----------------- Setting up collision detector and constraint for SBLSetup ----------------------| 

	rw::proximity::CollisionDetector detector(wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy());

	rw::pathplanning::QConstraint::Ptr q_constraint = rw::pathplanning::QConstraint::make(&detector, &robotTree, state); 
	rw::pathplanning::QEdgeConstraintIncremental::Ptr q_edge_constraint = rw::pathplanning::QEdgeConstraintIncremental::makeDefault(q_constraint, &robotTree);

	double expandRadius = 0.01; 
	double connectRadius = 0.03; 

	rwlibs::pathplanners::SBLSetup setup = rwlibs::pathplanners::SBLSetup::make(q_constraint, q_edge_constraint, &robotTree, expandRadius, connectRadius);
	rw::pathplanning::QToQPlanner::Ptr sbl_planner = rwlibs::pathplanners::SBLPlanner::makeQToQPlanner(setup);

	//| ----------------- Checking for collision in start and end configuration ----------------------| 

	if (!checkCollisions(&robotTree, state, detector, from))
		return 0;
	if (!checkCollisions(&robotTree, state, detector, to))
		return 0;

 	//| ----------------- Planning path between from and to ----------------------| 

	cout << "Planning from " << from << " to " << to << endl;
	rw::common::Timer t;                                        // Timer to stop when planning takes too long
	t.resetAndResume();                                         // Starting timer                           
	sbl_planner->query(from,to,sbl_path,MAXTIME);               // Start planing
	t.pause();                                                  // Stopping timer                                          // Save path in class
	cout << "Path of length " << sbl_path.size() << " found in " << t.getTime() << " seconds." << endl;
	if (t.getTime() >= MAXTIME or !sbl_path.size()) {
		cout << "Notice: max time of " << MAXTIME << " seconds reached." << endl;
		return false; 
	}else{
        //| ----------------- Setting up path otimization to find better route ----------------------| 

		rw::pathplanning::PlannerConstraint constraint = rw::pathplanning::PlannerConstraint::make(&detector,&robotTree,state);
		rw::math::QMetric::Ptr metric = rw::math::MetricFactory::makeInfinity<rw::math::Q>();
		rwlibs::pathoptimization::PathLengthOptimizer pathOptimizer(constraint,metric);
		sbl_path = pathOptimizer.partialShortCut(sbl_path);
		cout << "optimized path lenght: " << sbl_path.size() << endl;
		return true;
	}
		
}

rw::trajectory::QPath URRobot::getPath(int ID){
	if(ID == 1){
		return path_UR5E1; 
	}else if(ID == 2){
		return path_UR5E2; 
	}
	return -1;
}

URRobot::~URRobot(){

}