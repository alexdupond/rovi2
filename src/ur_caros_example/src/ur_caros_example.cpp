#include <iostream>
#include "rw/rw.hpp"


#include "ros/package.h"


using namespace std;
using namespace rw::math;


#include "urRobot.h"
#include "prioritizedPlanner.h"


int main(int argc, char** argv)
{
	ros::init(argc, argv, "URRobot");
	
	ros::NodeHandle nodehandler; 

	URRobot robot(&nodehandler);

	vector<double> QrobotFrom1{2, -0.8, 1.0, -1.5, -1.5, 0};
	vector<double> QrobotFrom2{-1.0, -2.2, -1.2, -1.5, 1.5, 0};
	vector<double> QrobotTo1{0.5, -0.95, 0.95, -1.45, -1.0, 0};
	vector<double> QrobotTo2{-2.0, -2.2, -1.2, -1.5, 1.5, 0}; 

	vector<double> QvecFrom = QrobotFrom1; 
	QvecFrom.insert(QvecFrom.end(), QrobotFrom2.begin(), QrobotFrom2.end()); 

	vector<double> QvecTo = QrobotTo1;
	QvecTo.insert(QvecTo.end(), QrobotTo2.begin(), QrobotTo2.end());

	rw::math::Q from(QvecFrom);
	rw::math::Q to(QvecTo);

	vector<rw::math::Q> robot1Path;
	vector<rw::math::Q> robot2Path; 

	robot1Path.push_back(QrobotFrom1);
	robot1Path.push_back(QrobotTo1); 
	robot2Path.push_back(QrobotFrom2);
	robot2Path.push_back(QrobotTo2); 

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
			if(robot.calculatePrioritizedPath(robot1Path, robot2Path)){
				cout << "Calculated RRT path!" << endl; 
			}else{
				cout << "Could not calculate RRT path!" << endl; 
			}
		}else if(input == "start"){
			rw::trajectory::QPath path = robot.getPath(); 
			if(path.size()){
				cout << "Itteration through path" << endl; 
				for (rw::trajectory::QPath::iterator it = path.begin(); it < path.end(); it++) {
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
