#include <iostream>
#include "rw/rw.hpp"


#include "ros/package.h"


using namespace std;
using namespace rw::math;

#include <fstream>

#include "urRobot.h"
#include "prioritizedPlanner.h"

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

int main(int argc, char** argv)
{
	ros::init(argc, argv, "URRobot");
	
	ros::NodeHandle nodehandler; 

	URRobot robot(&nodehandler);

	vector<double> testRob1Start{2.21, -0.89, 1.13, -1.80, -1.61, -0.14}; 
	vector<double> testRob1End{3.64, -0.71, 0.83, -1.68, -1.59, 1.26};
	vector<double> testRob2Start{-0.68, -2.27, -1.08, -1.35, 1.57, 0.02};
	vector<double> testRob2End{0.91, -2.38, -0.87, -1.48, 1.58, -1.48};   

	vector<double> Qrob1Start{1.339, -2.324, 2.297, -1.544, -1.569, -1.017}; // Start
	vector<double> Qrob1Q1{2.009, -0.859, 1.164, -1.874, -1.571, -0.347}; // Pick point
	vector<double> Qrob1Q2{2.009, -0.74, 1.273, -2.103, -1.571, -0.347}; // Go down
	vector<double> Qrob1Q3{0.698, -0.904, 1.289, -1.956, -1.574, -1.655}; // Pick point 
	vector<double> Qrob1Q4{0.698, -0.792, 1.374, -2.153, -1.574, -1.655}; // Go down


	vector<double> Qrob2Start{1.339, -2.324, 2.297, -1.544, -1.569, -1.017}; // Start
	vector<double> Qrob2Q1{2.098, -0.907, 1.300, -1.967, -1.571, -0.256}; // Pick point 
	vector<double> Qrob2Q2{2.098, -0.797, 1.383, -2.187, -1.568, -0.256}; // Go down
	vector<double> Qrob2Q3{0.667, -1.082, 1.540, -2.028, -1.569, -1.687}; // Last point 
	vector<double> Qrob2Q4{0.667, -0.933, 1.639, -2.277, -1.571, -1.687}; // Go down


	vector<rw::math::Q> robot1Path;
	vector<rw::math::Q> robot2Path; 

	robot1Path.push_back(testRob1Start); 
	robot1Path.push_back(testRob1End);

	robot2Path.push_back(testRob2Start); 
	robot2Path.push_back(testRob2End);  
/*
	robot1Path.push_back(Qrob1Start); 
	robot1Path.push_back(Qrob1Q1);
	robot1Path.push_back(Qrob1Q2);
	robot1Path.push_back(Qrob1Q1);
	robot1Path.push_back(Qrob1Q3);
	robot1Path.push_back(Qrob1Q4);
	robot1Path.push_back(Qrob1Q3);
	robot1Path.push_back(Qrob1Start); 

	robot2Path.push_back(Qrob2Start); 
	robot2Path.push_back(Qrob2Q1);
	robot2Path.push_back(Qrob2Q2);
	robot2Path.push_back(Qrob2Q1);
	robot2Path.push_back(Qrob2Q3);
	robot2Path.push_back(Qrob2Q4);
	robot2Path.push_back(Qrob2Q3);
	robot2Path.push_back(Qrob2Start);  
*/

	double extend = 0.306; 
	double aggressive = 72.5; 
	double extendMin = 0.01; 
	double extendMax = 0.5; 

	double aggressiveMin = 50;
	double aggressiveMax = 95;

	while(true){
		std::cout << "Current joint config:" << std::endl << robot.getQ() << std::endl << std::endl;
		cout << "Input command: " << endl; 
		string input; 
		cin >> input;
		if(input == "test")
		{
			double count = 10;
			string file_name = "./test.txt";
			fstream csvfile;
			csvfile.open(file_name, ios::out | ios::app);
			csvfile << "Extend value | Aggression value | path1 size | path1 time | path1 new size | path1 new time | path2 size | path2 time " << endl; 
			csvfile.close();

			for(size_t i = 0; i < count; i++)
			{
				double ext = extendMin + (extendMax - extendMin) *(i/count);
				for(size_t j = 0; j < count; j++)
				{	
					double aggr = aggressiveMin + (aggressiveMax-aggressiveMin) * (j/count);
					rw::trajectory::QPath qPath1; 
					rw::trajectory::QPath qPath2;
					// string file_name = "./test.txt";
					// fstream csvfile;
					// csvfile.open(file_name, ios::out | ios::app);
					// csvfile << ext << ", " << aggr << ", "; 
					// csvfile.close();
					if(robot.calculatePrioritizedPath(robot1Path, robot2Path, qPath1, qPath2, ext, aggr)){
						//cout << "Success!" << endl;
					}else{
						//cout << "Failed!" << endl; 
					}
				}
			}
			
		


		}else if(input == "rrt"){
			rw::trajectory::QPath qPath1; 
			rw::trajectory::QPath qPath2;

			rw::trajectory::QPath total; 
			if(robot.calculatePrioritizedPath(robot1Path, robot2Path, qPath1, qPath2, extend, aggressive)){
				cout << "Calculated RRT path!" << endl; 


				int it =  qPath2.size();
				if(qPath1.size() > qPath2.size()){
					it =  qPath1.size();
				}
				cout << "Size of path 1 =" << qPath1.size() << ", and size of path 2 = " << qPath2.size() << endl; 
				total = joinPaths(qPath1, qPath2); 
				cout << "Size of joint path = " << total.size() << ", and number og elements = " << total[0].size() << endl; 

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
