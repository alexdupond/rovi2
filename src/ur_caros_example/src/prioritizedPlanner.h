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

class RRTStruct
{
public:
    RRTStruct(PlannerConstraint& constraint,
        QSampler::Ptr sampler,
        QMetric::Ptr metric,
        double extend)
        :
        constraint(constraint),
        sampler(sampler),
        metric(metric),
        extend(extend)
    {
        RW_ASSERT(sampler);
        RW_ASSERT(metric);
    }

    void updateConstraint(PlannerConstraint q_constraint){ constraint = q_constraint;} 

    PlannerConstraint constraint;
    QSampler::Ptr sampler;
    QMetric::Ptr metric;
    double extend;
};


class PrioritizedPlanner{

public:
	PrioritizedPlanner(rw::models::WorkCell::Ptr wc, rw::models::Device::Ptr device1, rw::models::Device::Ptr device2); 
    bool calculateTimesteps(rw::trajectory::QPath path);
    rw::trajectory::QPath getPath(int ID); 
    void setPath(rw::trajectory::QPath);
    bool calculateRRTPath(rw::math::Q from, rw::math::Q to);
    bool calculateDynamicRRTPath(rw::math::Q &from, rw::math::Q &to);
    rw::trajectory::QPath optimizePath(rw::trajectory::QPath& path, rw::models::Device::Ptr device);  
    bool checkCollisions(rw::models::Device::Ptr device, const rw::proximity::CollisionDetector &detector, const rw::math::Q &q);
    ~PrioritizedPlanner(); 

    bool inCollision(const RRTStruct& rrt, const Q& q);
    bool inCollision(const RRTStruct& rrt, Node* a, const Q& b);
    Node* nearestNeighbor(const RRTStruct& rrt,const Tree& tree,const Q& q);
    ExtendResult extend(const RRTStruct& rrt,Tree& tree,const Q& q,Node* qNearNode);
    ExtendResult connect(const RRTStruct& rrt,Tree& tree,const Q& q);
    ExtendResult growTree(const RRTStruct& rrt,Tree& tree,const Q& q_rand, const Q& q_goal);
    void getPathFromTree(const Tree& startTree, const Tree& goalTree, rw::trajectory::QPath& result);
    void updateDevice(rw::math::Q q); 

    double multiplier(rw::math::Q q, int d); 


private:
	rw::models::WorkCell::Ptr _wc;
	rw::models::Device::Ptr _device1;
	rw::models::Device::Ptr _device2; 
	rw::kinematics::State _state;
	rw::trajectory::QPath _path_1;
    rw::trajectory::QPath _path_2; 
	vector<double> _timesteps; 
    double currentTimestep = 0; 

};

#endif // PRIORITIZEDMULTIPLANNER