#include "prioritizedPlanner.h"

#include <rw/pathplanning/PlannerConstraint.hpp>
#include <rw/pathplanning/QSampler.hpp>


//typedef rw::trajectory::QPath Path;

const Q& getQVal(Node* node) { return node->getValue();}
double getDistanceVal(Node* node){ return node->getDistance(); }



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
    rw::math::Q q(6, 2, -2, -1.2, -1.5, 1.5, 0);
    _device2->setQ(q, _state);
	rw::proximity::CollisionDetector detector(_wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy());
	rw::pathplanning::PlannerConstraint constraint = rw::pathplanning::PlannerConstraint::make(&detector,_device1,_state);

	rw::pathplanning::QSampler::Ptr sampler = rw::pathplanning::QSampler::makeConstrained(rw::pathplanning::QSampler::makeUniform(_device1),constraint.getQConstraintPtr());

	rw::math::QMetric::Ptr metric = rw::math::MetricFactory::makeEuclidean<rw::math::Q>();
	double extend = 0.1;
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
	_path_1 = rrt_path; 
	cout << "Path of length " << _path_1.size() << " found in " << t.getTime() << " seconds." << endl;
	if (t.getTime() >= MAXTIME) {
		cout << "Notice: max time of " << MAXTIME << " seconds reached." << endl;
		return false; 
	}else{	
		return true;
	}
	
}

void PrioritizedPlanner::setPath(rw::trajectory::QPath path){
    _path_1 = path; 
}

rw::trajectory::QPath PrioritizedPlanner::optimizePath(rw::trajectory::QPath& path, rw::models::Device::Ptr device){
    rw::proximity::CollisionDetector detector(_wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy());
	rw::pathplanning::PlannerConstraint constraint = rw::pathplanning::PlannerConstraint::make(&detector,device,_state);
    rw::math::QMetric::Ptr metric = rw::math::MetricFactory::makeEuclidean<rw::math::Q>();
    rwlibs::pathoptimization::PathLengthOptimizer pathOptimizer(constraint,metric);
    path = pathOptimizer.partialShortCut(path);
	cout << "Optimized path lenght: " << path.size() << endl;
    return path; 
}


bool PrioritizedPlanner::calculateTimesteps(rw::trajectory::QPath path){ 
	if(path.size()){
		double currentTime = 0;
		_timesteps.push_back(currentTime);
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
			currentTime = stepsize; 
			cout << "Largest stepsize = " << stepsize << endl; 
			cout << "Current time = " << currentTime << endl; 
			_timesteps.push_back(currentTime); 
		}
        return true; 
	}else{
		cout << "The path has no size!" << endl; 
        return false; 
	}

}

rw::trajectory::QPath PrioritizedPlanner::getPath(int ID){
    if(ID == 1)
        return _path_1;
    if(ID == 2)
        return _path_2;

    return 0;
}


bool PrioritizedPlanner::calculateDynamicRRTPath(rw::math::Q &start, rw::math::Q &goal){
    // SETTING UP RRT STRUCT FOR PLANNER
	rw::proximity::CollisionDetector detector(_wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy());
	rw::pathplanning::PlannerConstraint constraint = rw::pathplanning::PlannerConstraint::make(&detector,_device2,_state);

	rw::pathplanning::QSampler::Ptr sampler = rw::pathplanning::QSampler::makeConstrained(rw::pathplanning::QSampler::makeUniform(_device2),constraint.getQConstraintPtr());

	rw::math::QMetric::Ptr metric = rw::math::MetricFactory::makeEuclidean<rw::math::Q>();
	double extend = 0.05;

    RRTStruct _rrt(constraint, sampler, metric, extend); 

    // STARTING TO PLAN ROUTE

    // Checking start and end configuration 
    cout << "Calculation path from: " << start << ", to: " << goal << endl; 
    if (inCollision(_rrt, start)) {
        std::cout<<"Start is in collision"<<std::endl;
        return false;
    }

    if (inCollision(_rrt, goal)) {
        std::cout<<"Goal is in collision"<<std::endl;
        return false;
    }

  /*  if (!_rrt.constraint.getQEdgeConstraint().inCollision(start, goal)) {
        result.push_back(start);
        result.push_back(goal);
        cout << "Path found - Size = " << result.size() << endl;
        return true;
    }
*/
   
    Tree startTree(start);
    Tree goalTree(goal);
    Tree* treeA = &startTree;
    Tree* treeB = &goalTree;

    rw::common::Timer t;
	t.resetAndResume();
    srand (time(NULL)); 
    while (t.getTime() < 400 ) {

        const Q qAttr = _rrt.sampler->sample();
        if (qAttr.empty()) RW_THROW("Sampler must always succeed.");

        if (growTree(_rrt, *treeA, qAttr, goal) == Reached )// != Trapped && connect(_rrt, *treeB, getQVal(&treeA->getLast())) == Reached)
        {
            getPathFromTree(startTree, goalTree, _path_2);
            cout << "Path found - Size = " << _path_2.size() << endl;
            return true;
        }

       // cout << "Tree A size = " << getDistanceVal(&treeA->getLast()) << ", Tree B size = " << getDistanceVal(&treeB->getLast()) << endl; 
        //std::swap(treeA, treeB);
    }
    
    cout << "TIMEOUT!" << endl; 

    return false;
}



bool PrioritizedPlanner::inCollision(const RRTStruct& rrt, const Q& q)
{
    return rrt.constraint.getQConstraint().inCollision(q);
}

// 'node' is known to be collision free, but 'b' is not.
bool PrioritizedPlanner::inCollision(const RRTStruct& rrt,
                    Node* a,
                    const Q& b)
{

    return
        rrt.constraint.getQConstraint().inCollision(b) ||
        rrt.constraint.getQEdgeConstraint().inCollision(getQVal(a), b);
}

// Brute-force nearest neighbor search.
Node* PrioritizedPlanner::nearestNeighbor(
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

void PrioritizedPlanner::updateDevice(rw::math::Q q){
    _device1->setQ(q, _state);
}

ExtendResult PrioritizedPlanner::extend(const RRTStruct& rrt,Tree& tree,const rw::math::Q& q,Node* qNearNode)
{
    int depth = getDistanceVal(qNearNode); 
    double multi = 0;
    cout << "Depth = " << depth << endl; 
    const rw::math::Q& qNear = getQVal(qNearNode);
    const rw::math::Q delta = q - qNear;
  
    if(depth < _path_1.size()-1){
        updateDevice(_path_1[depth]);
        multi = multiplier(delta, depth); 
    }
   
    const double dist = rrt.metric->distance(delta);
    cout << "Distance = " << dist << endl; 
    if (dist <= rrt.extend) {
        if (!inCollision(rrt, qNearNode, q)) {                
            tree.add(q, qNearNode, 1);
            return Reached;
        } else {
            cout << "Collision!" << endl; 
            return Trapped;
        }
    } else {
        Q qNew; 
        if(depth < _path_1.size()-1){
            qNew = qNear + multi * delta;
        }else{
            qNew = qNear + (rrt.extend/dist) * delta;
        }
        cout << "Configuration device 1 = " << _device1->getQ(_state) << endl; 
        cout << "Configuration device 2 = " << qNew << endl; 

        // SETTING UP RRT STRUCT FOR PLANNER
        rw::proximity::CollisionDetector detector(_wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy());
        rw::pathplanning::PlannerConstraint constraint = rw::pathplanning::PlannerConstraint::make(&detector,_device2,_state);

        rw::pathplanning::QSampler::Ptr sampler = rw::pathplanning::QSampler::makeConstrained(rw::pathplanning::QSampler::makeUniform(_device2),constraint.getQConstraintPtr());

        rw::math::QMetric::Ptr metric = rw::math::MetricFactory::makeEuclidean<rw::math::Q>();
        double extend = 0.05;

        RRTStruct _rrt(constraint, sampler, metric, extend); 


        if (!inCollision(_rrt, qNearNode, qNew)) {
            tree.add(qNew, qNearNode, 1);
            return Advanced;
        } else {
            cout << "Collision!" << endl; 
            return Trapped;
        }
    }
}

double PrioritizedPlanner::multiplier(rw::math::Q q, int d)
{
    int id = -1; 
    double value = 0; 
    cout << "Q for largest value test = " << q << endl; 
    for(size_t i = 0; i < q.size(); i++)
    {
        if(abs(q[i]) > value){
            id = i; 
            value = abs(q[i]);
        }
    }
    cout << "Timestep / largest value = multiplier " << abs(_timesteps[d]) << ", " << value << " = " << abs(_timesteps[d])/value << endl;
    return abs(_timesteps[d])/value; 
}
/*
ExtendResult PrioritizedPlanner::connect(const RRTStruct& rrt,Tree& tree,const Q& q)
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
*/


ExtendResult PrioritizedPlanner::growTree(const RRTStruct& rrt,Tree& tree,const Q& q_rand, const Q& q_goal)
{
    int val = rand()%100; 
    if(val > 40){
        cout << "Chose random!" << endl << endl; 
        Node* qNearNode = nearestNeighbor(rrt, tree, q_rand);
        return extend(rrt, tree, q_rand, qNearNode);
    }else{
        cout << "Chose goal!" << endl << endl; 
        Node* qNearNode = nearestNeighbor(rrt, tree, q_rand);
        return extend(rrt, tree, q_goal, qNearNode);
    }
}

// Assuming that both trees have just been extended, retrieve the resulting
// path.
void PrioritizedPlanner::getPathFromTree(const Tree& startTree, const Tree& goalTree, rw::trajectory::QPath& result)
{
    rw::trajectory::QPath revPart;
    Tree::getRootPath(*startTree.getLast().getParent(), revPart);
    result.insert(result.end(), revPart.rbegin(), revPart.rend());
    Tree::getRootPath(goalTree.getLast(), result);
}


PrioritizedPlanner::~PrioritizedPlanner(){
    
}