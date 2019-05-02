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

	vector<double> Qrob1Q1{2, -0.8, 1.0, -1.5, -1.5, 0};
	vector<double> Qrob1Q2{0.5, -0.95, 0.95, -1.45, -1.0, 0};
	vector<double> Qrob1Q3{-0.2, 0.0, 0.60, -1.45, -1.0, 0};

	vector<double> Qrob2Q1{-1.0, -2.2, -1.2, -1.5, 1.5, 0};
	vector<double> Qrob2Q2{-2.0, -2.2, -1.2, -1.5, 1.5, 0};
	vector<double> Qrob2Q3{0.0, -1.2, 1.0, -1.6, -1.0, 0}; 

	vector<rw::math::Q> robot1Path;
	vector<rw::math::Q> robot2Path; 

	robot1Path.push_back(Qrob1Q1);
	robot1Path.push_back(Qrob1Q2);
	robot1Path.push_back(Qrob1Q3);
	robot2Path.push_back(Qrob2Q1);
	robot2Path.push_back(Qrob2Q2);
	robot2Path.push_back(Qrob2Q3);  

	while(true){
		std::cout << "Current joint config:" << std::endl << robot.getQ() << std::endl << std::endl;
		cout << "Input command: " << endl; 
		string input; 
		cin >> input;
		if(input == "rrt"){
			rw::trajectory::QPath qPath1; 
			rw::trajectory::QPath qPath2;
			if(robot.calculatePrioritizedPath(robot1Path, robot2Path, qPath1, qPath2)){
				cout << "Calculated RRT path!" << endl; 

				int it =  qPath2.size();
				if(qPath1.size() > qPath2.size()){
					it =  qPath1.size();
				}
				cout << "Size of path 1 =" << qPath1.size() << ", and size of path 2 = " << qPath2.size() << endl; 

				rw::trajectory::QPath::iterator it1; 
				rw::trajectory::QPath::iterator it2;

				if(qPath1.size() && qPath2.size()){
					cout << "Itteration through path" << endl; 
					it1 = qPath1.begin();
					it2 = qPath2.begin(); 
					for (int i = 0 ; i < it; i++) {
						
						cout << *it1 << " && " << *it2 << endl;
						if(robot.setQ(*it1, *it2))
							cout << std::endl << "New joint config:" << endl << robot.getQ() << endl;
						else
							cout << endl << "Failed to move robot" << endl;

						if(it2 < qPath2.end()-1){
							it2++; 
						}
						if(it1 < qPath1.end()-1){
							it1++; 
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
