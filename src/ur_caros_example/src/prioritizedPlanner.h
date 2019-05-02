#ifndef _PRIORITIZEDPLANNER_H_
#define _PRIORITIZEDPLANNER_H_

#include <iostream>
#include "rw/rw.hpp"
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rwlibs/pathoptimization/pathlength/PathLengthOptimizer.hpp>


#include <rw/pathplanning.hpp>

#include "rrt/RRTNode.hpp"
#include "rrt/RRTTree.hpp"


#include <cfloat>
#include <algorithm>
#include <boost/foreach.hpp>

#include <stdio.h>      /* printf, scanf, puts, NULL */
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */


using namespace std;
using namespace rw;
using namespace rw::math;
using namespace rw::common;
using namespace rw::pathplanning;
using namespace rw::trajectory;
using namespace rwlibs::pathplanners;


#define MAXTIME 50

typedef RRTNode<rw::math::Q> Node;
typedef RRTTree<rw::math::Q> Tree;

enum ExtendResult { Trapped, Reached, Advanced };

class PrioritizedPlanner{

public:
	PrioritizedPlanner(rw::models::WorkCell::Ptr wc, rw::models::Device::Ptr device1, rw::models::Device::Ptr device2, double extend); 
    bool calculateTimesteps(rw::trajectory::QPath& path);
    rw::trajectory::QPath getPath(int ID); 
    void setPath(rw::trajectory::QPath& path);
    bool calculateRRTPath(const rw::math::Q& from, const rw::math::Q& to, rw::trajectory::QPath& result);
    bool calculateDynamicRRTPath(const rw::math::Q &from, const rw::math::Q &to, rw::trajectory::QPath& path1, rw::trajectory::QPath& result);
    void optimizePath(rw::trajectory::QPath& path, rw::models::Device::Ptr device);  
    bool checkCollisions(rw::models::Device::Ptr device, const rw::proximity::CollisionDetector &detector, const rw::math::Q &q);
    ~PrioritizedPlanner(); 

    // For custom RRT planner with dynamic model
    Node* nearestNeighbor(const Tree& tree,const Q& q);
    ExtendResult extend(Tree& tree,const Q& q,Node* qNearNode, const Q& qGoal, const rw::proximity::CollisionDetector &detector);
    ExtendResult growTree(Tree& tree,const Q& q_rand, const Q& qGoal, const rw::proximity::CollisionDetector &detector);
    void getPathFromTree(const Tree& startTree, const Tree& goalTree, rw::trajectory::QPath& result);
    double multiplier(const rw::math::Q& q, int d); 
    double distance(const rw::math::Q& q); 

    // Colissions checking
    bool inCollision(const rw::proximity::CollisionDetector &detector, const rw::math::Q& q1, const rw::math::Q& q2); 
    bool inCollision(const rw::proximity::CollisionDetector &detector, const Q& qPrev1, const Q& qNext1, const Q& qPrev2, const Q& qNext2, float e);    







private:
	rw::models::WorkCell::Ptr _wc;
	rw::models::Device::Ptr _device1;
	rw::models::Device::Ptr _device2; 
	rw::kinematics::State _state;
	rw::trajectory::QPath _path_1;
    rw::trajectory::QPath _path_2; 
	vector<double> _timesteps; 
    double currentTimestep = 0;
    double _extend = 0; 
    int _depth = 1; 

};

#endif // PRIORITIZEDMULTIPLANNER