#include "urRobot.h"

URRobot::URRobot(ros::NodeHandle* nodehandler):nh(*nodehandler)
{
	auto packagePath = ros::package::getPath("ur_caros_example");
	wc = rw::loaders::WorkCellLoader::Factory::load(packagePath + "/RoviScene/Scene.xml");
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

bool URRobot::calculatePrioritizedPath(vector<rw::math::Q> from, vector<rw::math::Q> to){
	PrioritizedPlanner planner(wc, UR5E1, UR5E2); 
	planner.calculateRRTPath(from[0], to[0]);
	path = planner.getPath();
	return true; 
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

bool URRobot::setDoubleQ(rw::math::Q q){
	// Creating  the message to be send to the robots
	caros_common_msgs::Q q_ros = caros::toRos(q);
	caros_control_msgs::RobotState msg_multi_q;
	msg_multi_q.q = q_ros; 
	pub_multi_ur5e.publish(msg_multi_q); 
	return true; 
}

bool URRobot::setQ(rw::math::Q q){
	// Setting robot speed
    float speed = 0.5;
    // Setting a configuration to send to the URSIM 
	rw::math::Q q1(6, q[0], q[1], q[2], q[3], q[4], q[5]); 
    // Setting both the robot in URSIM and the two robots in RobWorkStudio. This will return true, when the robot in URSIM is close to its goal position.
	if(q.size() == 12){
		if (robot->movePtp(q1, speed) && setDoubleQ(q)) { 
			return true;
		} else
			return false;
	}else{
		if (robot->movePtp(q1, speed)) { 
			return true;
		} else
			return false;
	}

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
	rw::trajectory::QPath sbl_path;                             
	sbl_planner->query(from,to,sbl_path,MAXTIME);               // Start planing
	t.pause();                                                  // Stopping timer 
	path = sbl_path;                                            // Save path in class
	cout << "Path of length " << path.size() << " found in " << t.getTime() << " seconds." << endl;
	if (t.getTime() >= MAXTIME or !path.size()) {
		cout << "Notice: max time of " << MAXTIME << " seconds reached." << endl;
		return false; 
	}else{
        //| ----------------- Setting up path otimization to find better route ----------------------| 

		rw::pathplanning::PlannerConstraint constraint = rw::pathplanning::PlannerConstraint::make(&detector,&robotTree,state);
		rw::math::QMetric::Ptr metric = rw::math::MetricFactory::makeInfinity<rw::math::Q>();
		rwlibs::pathoptimization::PathLengthOptimizer pathOptimizer(constraint,metric);
		path = pathOptimizer.partialShortCut(path);
		cout << "optimized path lenght: " << path.size() << endl;
		return true;
	}
		
}

rw::trajectory::QPath URRobot::getPath(){
	return path;
}

URRobot::~URRobot(){

}