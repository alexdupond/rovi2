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

	vector<double> QrobotFrom11{2, -0.8, 1.0, -1.5, -1.5, 0};
	vector<double> QrobotFrom21{-1.0, -2.2, -1.2, -1.5, 1.5, 0};

	vector<double> QrobotTo11{0.5, -0.95, 0.95, -1.45, -1.0, 0};
	vector<double> QrobotTo21{-2.0, -2.2, -1.2, -1.5, 1.5, 0};

	vector<double> QrobotTo12{1, -1.5, 1.9, -1.6, -1.8, 0}; 

	vector<double> QvecFrom = QrobotFrom11; 
	QvecFrom.insert(QvecFrom.end(), QrobotFrom21.begin(), QrobotFrom21.end()); 

	vector<double> QvecTo = QrobotTo11;
	QvecTo.insert(QvecTo.end(), QrobotTo21.begin(), QrobotTo21.end());

	rw::math::Q from(QvecFrom);
	rw::math::Q to(QvecTo);

	vector<rw::math::Q> robot1Path;
	vector<rw::math::Q> robot2Path; 

	robot1Path.push_back(QrobotFrom11);
	robot1Path.push_back(QrobotTo11);
	robot1Path.push_back(QrobotTo12);
	robot2Path.push_back(QrobotFrom21);
	robot2Path.push_back(QrobotTo21); 

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
				rw::trajectory::QPath path1;
				rw::trajectory::QPath path2;

				if(robot.getPath(1).size() > robot.getPath(2).size()){
					path1 = robot.getPath(1);
					path2 = robot.getPath(2); 
				}else{
					path2 = robot.getPath(1);
					path1 = robot.getPath(2); 
				}

				rw::trajectory::QPath::iterator it1; 
				rw::trajectory::QPath::iterator it2;

				if(path1.size() && path2.size()){
					cout << "Itteration through path" << endl; 
					it2 = path2.begin(); 
					for (it1 = path1.begin(); it1 < path1.end(); it1++) {
						
						cout << *it1 << " && " << *it2 << endl;
						if(robot.setQ(*it1, *it2))
							cout << std::endl << "New joint config:" << endl << robot.getQ() << endl;
						else
							cout << endl << "Failed to move robot" << endl;

						if(it2 < path2.end()-1){
							it2++; 
							cout << "Incrementing itterator" << endl; 
						}		
					}
				}else{
					cout << "The path has no size!" << endl; 
				}
			}else{
				cout << "Could not calculate RRT path!" << endl; 
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
