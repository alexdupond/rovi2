#include "prioritizedPlanner.h"

#include "rrt/RRTNode.hpp"
#include "rrt/RRTTree.hpp"

#include <rw/pathplanning/PlannerConstraint.hpp>
#include <rw/pathplanning/QSampler.hpp>

#include <cfloat>
#include <algorithm>
#include <boost/foreach.hpp>

using namespace rw;
using namespace rw::math;
using namespace rw::common;
using namespace rw::pathplanning;
using namespace rw::trajectory;
using namespace rwlibs::pathplanners;

typedef RRTNode<rw::math::Q> Node;
typedef RRTTree<rw::math::Q> Tree;
//typedef rw::trajectory::QPath Path;

const Q& getQVal(Node* node) { return node->getValue();}

enum ExtendResult { Trapped, Reached, Advanced };

class RRTStruct
{
public:
    RRTStruct(const PlannerConstraint& constraint,
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

    PlannerConstraint constraint;
    QSampler::Ptr sampler;
    QMetric::Ptr metric;
    double extend;
};

bool inCollision(const RRTStruct& rrt, const Q& q);
bool inCollision(const RRTStruct& rrt, Node* a, const Q& b);
Node* nearestNeighbor(const RRTStruct& rrt,const Tree& tree,const Q& q);
ExtendResult extend(const RRTStruct& rrt,Tree& tree,const Q& q,Node* qNearNode);
ExtendResult connect(const RRTStruct& rrt,Tree& tree,const Q& q);
ExtendResult growTree(const RRTStruct& rrt,Tree& tree,const Q& q);
void getPath(const Tree& startTree, const Tree& goalTree, rw::trajectory::QPath& result);


PrioritizedPlanner::PrioritizedPlanner(rw::models::WorkCell::Ptr wc, rw::models::Device::Ptr device1, rw::models::Device::Ptr device2){
    _wc = wc;
    _state = wc->getDefaultState();
    _device1 = device1; 
    _device2 = device2;
}


bool PrioritizedPlanner::checkCollisions(rw::models::Device::Ptr device, const rw::proximity::CollisionDetector &detector, const rw::math::Q &q) {
	rw::proximity::CollisionDetector::QueryResult data;

	device->setQ(q,_state);
	if (detector.inCollision(_state,&data)) {
		cerr << "Configuration in collision: " << q << endl;
		return false;
	}
	return true;
}

bool PrioritizedPlanner::calculateRRTPath(rw::math::Q from, rw::math::Q to){
	rw::proximity::CollisionDetector detector(_wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy());
	rw::pathplanning::PlannerConstraint constraint = rw::pathplanning::PlannerConstraint::make(&detector,_device1,_state);

	rw::pathplanning::QSampler::Ptr sampler = rw::pathplanning::QSampler::makeConstrained(rw::pathplanning::QSampler::makeUniform(_device1),constraint.getQConstraintPtr());

	rw::math::QMetric::Ptr metric = rw::math::MetricFactory::makeEuclidean<rw::math::Q>();
	double extend = 0.005;
	rw::pathplanning::QToQPlanner::Ptr planner = rwlibs::pathplanners::RRTPlanner::makeQToQPlanner(constraint, sampler, metric, extend, rwlibs::pathplanners::RRTPlanner::RRTConnect);

	if (!checkCollisions(_device1, detector, from))
		return 0;
	if (!checkCollisions(_device1, detector, to))
		return 0;

	cout << "Planning from " << from << " to " << to << endl;
	rw::trajectory::QPath rrt_path;

	rw::common::Timer t;
	t.resetAndResume();
	planner->query(from,to,rrt_path,MAXTIME);
	t.pause();
	_path = rrt_path; 
	cout << "Path of length " << _path.size() << " found in " << t.getTime() << " seconds." << endl;
	if (t.getTime() >= MAXTIME) {
		cout << "Notice: max time of " << MAXTIME << " seconds reached." << endl;
		return false; 
	}else{	
		return true;
	}
	
}

bool PrioritizedPlanner::optimizePath(rw::trajectory::QPath path){
    rw::proximity::CollisionDetector detector(_wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy());
	rw::pathplanning::PlannerConstraint constraint = rw::pathplanning::PlannerConstraint::make(&detector,_device1,_state);
    rw::math::QMetric::Ptr metric = rw::math::MetricFactory::makeEuclidean<rw::math::Q>();
    rwlibs::pathoptimization::PathLengthOptimizer pathOptimizer(constraint,metric);
    _path = pathOptimizer.partialShortCut(_path);
	cout << "Optimized path lenght: " << _path.size() << endl;
}


vector<double> PrioritizedPlanner::calculateTimesteps(rw::trajectory::QPath path){
	vector<double> time; 
	if(path.size()){
		double currentTime = 0;
		time.push_back(currentTime);
		for (rw::trajectory::QPath::iterator it = path.begin(); it < path.end()-1; it++) {
			rw::math::Q q_now = *it;
			it++; 
			rw::math::Q q_next = *it;
			it--;
			double stepsize = 0; 
			for (int i = 0; i < q_now.size(); i++){
				if(stepsize < abs(q_now[i]-q_next[i])){
					stepsize = abs(q_now[i]-q_next[i]);	
				}
			}
			currentTime += stepsize; 
			cout << "Largest stepsize = " << stepsize << endl; 
			cout << "Current time = " << currentTime << endl; 
			time.push_back(currentTime); 
		}
	}else{
		cout << "The path has no size!" << endl; 
	}

	return time; 
}

rw::trajectory::QPath PrioritizedPlanner::getPath(){
    return _path; 
}


bool PrioritizedPlanner::prioritizedPlanning(vector<rw::math::Q> from, vector<rw::math::Q> to, rw::trajectory::QPath robotPath, vector<double> steplist){
    // SETTING UP RRT STRUCT FOR PLANNER
	rw::proximity::CollisionDetector detector(_wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy());
	rw::pathplanning::PlannerConstraint constraint = rw::pathplanning::PlannerConstraint::make(&detector,_device2,_state);

	rw::pathplanning::QSampler::Ptr sampler = rw::pathplanning::QSampler::makeConstrained(rw::pathplanning::QSampler::makeUniform(_device2),constraint.getQConstraintPtr());

	rw::math::QMetric::Ptr metric = rw::math::MetricFactory::makeEuclidean<rw::math::Q>();
	double extend = 0.001;

    RRTStruct _rrt(constraint, sampler, metric, extend); 

    // STARTING TO PLAN ROUTE
/*
    // Checking start and end configuration 
    if (inCollision(_rrt, start)) {
        std::cout<<"Start is in collision"<<std::endl;
        return false;
    }

    if (inCollision(_rrt, goal)) {
        std::cout<<"Goal is in collision"<<std::endl;
        return false;
    }

    if (!_rrt.constraint.getQEdgeConstraint().inCollision(start, goal)) {
        result.push_back(start);
        result.push_back(goal);
        return true;
    }

    Tree startTree(start);
    Tree goalTree(goal);

    while (!stop.stop()) {
        const Q qAttr = _rrt.sampler->sample();
        if (qAttr.empty()) RW_THROW("Sampler must always succeed.");

        // If both trees manage to connect, then return the resulting
        // path.
        if (growTree(_rrt, startTree, qAttr) == Reached)
        {
            getPath(startTree, goalTree, result);
            return true;
        }
    }
*/
    return false;
}


PrioritizedPlanner::~PrioritizedPlanner(){
    
}





bool inCollision(const RRTStruct& rrt, const Q& q)
{
    return rrt.constraint.getQConstraint().inCollision(q);
}

// 'node' is known to be collision free, but 'b' is not.
bool inCollision(const RRTStruct& rrt,
                    Node* a,
                    const Q& b)
{
    return
        rrt.constraint.getQConstraint().inCollision(b) ||
        rrt.constraint.getQEdgeConstraint().inCollision(getQVal(a), b);
}

// Brute-force nearest neighbor search.
Node* nearestNeighbor(
    const RRTStruct& rrt,
    const Tree& tree,
    const Q& q)
{
    const QMetric& metric = *rrt.metric;
    double minLength = DBL_MAX;
    Node* minNode = NULL;

    BOOST_FOREACH(Node* node, tree.getNodes()) {
        const double length = metric.distance(q, getQVal(node));
        if (length < minLength) {
            minLength = length;
            minNode = node;
        }
    }

    RW_ASSERT(minNode);
    return minNode;
}

ExtendResult extend(const RRTStruct& rrt,Tree& tree,const Q& q,Node* qNearNode)
{
    const Q& qNear = getQVal(qNearNode);
    const Q delta = q - qNear;
    const double dist = rrt.metric->distance(delta);

    if (dist <= rrt.extend) {
        if (!inCollision(rrt, qNearNode, q)) {                
            tree.add(q, qNearNode);
            return Reached;
        } else {
            return Trapped;
        }
    } else {
        const Q qNew = qNear + (rrt.extend / dist) * delta;
        if (!inCollision(rrt, qNearNode, qNew)) {
            tree.add(qNew, qNearNode);
            return Advanced;
        } else {
            return Trapped;
        }
    }
}

ExtendResult connect(const RRTStruct& rrt,Tree& tree,const Q& q)
{
    Node* qNearNode = nearestNeighbor(rrt, tree, q);

    ExtendResult s = Advanced;
    bool hasAdvanced = false;
    while (s == Advanced) {
        s = extend(rrt, tree, q, qNearNode);
        if (s == Advanced) {
            qNearNode = &tree.getLast();
            hasAdvanced = true;
        }
    } 

    if (s == Trapped && hasAdvanced)
        return Advanced;
    else
        return s;
}

ExtendResult growTree(const RRTStruct& rrt,Tree& tree,const Q& q)
{
    Node* qNearNode = nearestNeighbor(rrt, tree, q);
    return extend(rrt, tree, q, qNearNode);
}

// Assuming that both trees have just been extended, retrieve the resulting
// path.
void getPath(const Tree& startTree, const Tree& goalTree, rw::trajectory::QPath& result)
{
    rw::trajectory::QPath revPart;
    Tree::getRootPath(*startTree.getLast().getParent(), revPart);
    result.insert(result.end(), revPart.rbegin(), revPart.rend());
    Tree::getRootPath(goalTree.getLast(), result);
}