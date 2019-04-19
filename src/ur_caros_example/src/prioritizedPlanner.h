#ifndef _PRIORITIZEDPLANNER_H_
#define _PRIORITIZEDPLANNER_H_

#include <iostream>
#include "rw/rw.hpp"
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rwlibs/pathoptimization/pathlength/PathLengthOptimizer.hpp>

#include <rw/pathplanning.hpp>




using namespace std;


#define MAXTIME 50

class PrioritizedPlanner{
public:
	PrioritizedPlanner(rw::models::WorkCell::Ptr wc, rw::models::Device::Ptr device1, rw::models::Device::Ptr device2); 
    vector<double> calculateTimesteps(rw::trajectory::QPath path);
    rw::trajectory::QPath getPath(int ID); 
    bool calculateRRTPath(rw::math::Q from, rw::math::Q to);
    bool prioritizedPlanning(rw::math::Q from, rw::math::Q to, rw::trajectory::QPath &result);
    rw::trajectory::QPath optimizePath(rw::trajectory::QPath path, rw::models::Device::Ptr device);  
    bool checkCollisions(rw::models::Device::Ptr device, const rw::proximity::CollisionDetector &detector, const rw::math::Q &q);
    ~PrioritizedPlanner(); 


private:
	rw::models::WorkCell::Ptr _wc;
	rw::models::Device::Ptr _device1;
	rw::models::Device::Ptr _device2; 
	rw::kinematics::State _state;
	rw::trajectory::QPath _path_1;
    rw::trajectory::QPath _path_2; 
	vector<double> _timesteps; 

};

#endif // PRIORITIZEDMULTIPLANNER