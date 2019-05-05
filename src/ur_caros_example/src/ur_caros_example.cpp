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

	vector<double> Qrob1Start{1.339, -2.324, 2.297, -1.544, -1.569, -1.017}; // Start
	vector<double> Qrob1Q1{2.009, -0.859, 1.164, -1.874, -1.571, -0.347}; // First point 
	vector<double> Qrob1Q2{0.698, -0.904, 1.289, -1.956, -1.574, -1.655}; // Pick point 
	vector<double> Qrob1Q3{0.698, -0.792, 1.374, -2.153, -1.574, -1.655}; // Go down
	vector<double> Qrob1Q4{0.698, -0.904, 1.289, -1.956, -1.574, -1.655}; // Go up

	vector<double> Qrob2Start{1.339, -2.324, 2.297, -1.544, -1.569, -1.017}; // Start
	vector<double> Qrob2Q1{2.098, -0.907, 1.300, -1.967, -1.571, 2.881}; // Pick point 
	vector<double> Qrob2Q2{2.098, -0.797, 1.383, -2.187, -1.568, 2.883}; // Go down
	vector<double> Qrob2Q3{2.098, -0.907, 1.300, -1.967, -1.571, 2.881}; // Go up
	vector<double> Qrob2Q4{0.667, -1.082, 1.540, -2.028, -1.569, 1.453}; // Last point 


	vector<rw::math::Q> robot1Path;
	vector<rw::math::Q> robot2Path; 

	robot1Path.push_back(Qrob1Start); 
	robot1Path.push_back(Qrob1Q1);
	robot1Path.push_back(Qrob1Q2);
	robot1Path.push_back(Qrob1Q3);
	robot1Path.push_back(Qrob1Q4);
	robot1Path.push_back(Qrob1Q1);
	robot1Path.push_back(Qrob1Q2);
	robot1Path.push_back(Qrob1Q3);
	robot1Path.push_back(Qrob1Q4);
	robot1Path.push_back(Qrob1Start); 

	robot2Path.push_back(Qrob2Start); 
	robot2Path.push_back(Qrob2Q1);
	robot2Path.push_back(Qrob2Q2);
	robot2Path.push_back(Qrob2Q3);
	robot2Path.push_back(Qrob2Q4);
	robot2Path.push_back(Qrob2Start);  

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
							rw::math::Q q2before = *it2; 
							it2++; 
							rw::math::Q q2after = *it2; 

							double maxLen = 0; 
							for(size_t j = 0; j < 6; j++)
							{
								double length = abs(q2before[j] -q2after[j]); 
								if(maxLen < length){
									maxLen = length; 
								}
							
							}
							cout << "Max length of path 2 = " << maxLen << endl; 
							
						}
						if(it1 < qPath1.end()-1){
							rw::math::Q q1before = *it1; 
							it1++; 
							rw::math::Q q1after = *it1; 
							double maxLen = 0; 
							for(size_t j = 0; j < 6; j++)
							{
								double length = abs(q1before[j] -q1after[j]); 
								if(maxLen < length){
									maxLen = length; 
								}
							
							}
							cout << "Max length of path 1 = " << maxLen << endl; 

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
