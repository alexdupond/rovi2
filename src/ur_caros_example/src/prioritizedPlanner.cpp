#include "prioritizedPlanner.h"

#include <rw/pathplanning/PlannerConstraint.hpp>
#include <rw/pathplanning/QSampler.hpp>


//typedef rw::trajectory::QPath Path;

const Q& getQVal(Node* node) { return node->getValue();}
double getDepth(Node* node){ return node->getDepth(); }



PrioritizedPlanner::PrioritizedPlanner(rw::models::WorkCell::Ptr wc, rw::models::Device::Ptr device1, rw::models::Device::Ptr device2, double extend){
    _wc = wc;
    _state = wc->getDefaultState();
    _extend = extend; 
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

bool PrioritizedPlanner::calculateRRTPath(const rw::math::Q& from, const rw::math::Q& to){
    // Setting device 2 to be as far from device 1 as possible - Hack to remove device 2 when planning for device 1
    rw::math::Q q(6, 2, -2, -1.2, -1.5, 1.5, 0);
    _device2->setQ(q, _state);
	rw::proximity::CollisionDetector detector(_wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy());
	rw::pathplanning::PlannerConstraint constraint = rw::pathplanning::PlannerConstraint::make(&detector,_device1,_state);

	rw::pathplanning::QSampler::Ptr sampler = rw::pathplanning::QSampler::makeConstrained(rw::pathplanning::QSampler::makeUniform(_device1),constraint.getQConstraintPtr());

	rw::math::QMetric::Ptr metric = rw::math::MetricFactory::makeEuclidean<rw::math::Q>();
	rw::pathplanning::QToQPlanner::Ptr planner = rwlibs::pathplanners::RRTPlanner::makeQToQPlanner(constraint, sampler, metric, _extend, rwlibs::pathplanners::RRTPlanner::RRTConnect);

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
        optimizePath(_path_1, _device1);
		return true;
	}
	
}

void PrioritizedPlanner::setPath(rw::trajectory::QPath& path){
    _path_1 = path; 
}

void PrioritizedPlanner::optimizePath(rw::trajectory::QPath& path, rw::models::Device::Ptr device){
    rw::proximity::CollisionDetector detector(_wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy());
	rw::pathplanning::PlannerConstraint constraint = rw::pathplanning::PlannerConstraint::make(&detector,device,_state);
    rw::math::QMetric::Ptr metric = rw::math::MetricFactory::makeEuclidean<rw::math::Q>();
    rwlibs::pathoptimization::PathLengthOptimizer pathOptimizer(constraint,metric);
    path = pathOptimizer.partialShortCut(path);
	cout << "Optimized path lenght: " << path.size() << endl; 
}


bool PrioritizedPlanner::calculateTimesteps(rw::trajectory::QPath& path){ 
	if(path.size()){
		double currentTime = 0;
		_timesteps.push_back(currentTime);
		for (rw::trajectory::QPath::iterator it = path.begin(); it < path.end()-1; it++) {
			rw::math::Q q_now = *it;
			it++; 
			rw::math::Q q_next = *it;
			it--;
			double stepsize = 0; 
			for (size_t i = 0; i < q_now.size(); i++){
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


bool PrioritizedPlanner::calculateDynamicRRTPath(const rw::math::Q &start, const rw::math::Q &goal){
    // SETTING UP RRT STRUCT FOR PLANNER
	rw::proximity::CollisionDetector detector(_wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy());
	rw::pathplanning::PlannerConstraint constraint = rw::pathplanning::PlannerConstraint::make(&detector,_device2,_state);

	rw::pathplanning::QSampler::Ptr sampler = rw::pathplanning::QSampler::makeConstrained(rw::pathplanning::QSampler::makeUniform(_device2),constraint.getQConstraintPtr());

    // Checking start and end configuration 
    cout << "Calculation path from: " << start << ", to: " << goal << endl; 
    if (inCollision(detector, start, _path_1[0])) {
        std::cout<<"Start is in collision"<<std::endl;
        return false;
    }
   
    Tree startTree(start);
    Tree goalTree(goal);
    Tree* treeA = &startTree;

    rw::common::Timer t;
	t.resetAndResume();
    srand (time(NULL)); 
    while (t.getTime() < 15 ) {

        const Q qAttr = sampler->sample();
        if (qAttr.empty()) RW_THROW("Sampler must always succeed.");

        if (growTree(*treeA, qAttr, goal, detector) == Reached )// != Trapped && connect(_rrt, *treeB, getQVal(&treeA->getLast())) == Reached)
        {
            getPathFromTree(startTree, goalTree, _path_2);
            cout << "Path found - Size = " << _path_2.size() << endl;
            cout << "Time to calculate path = " << t.getTime() << endl; 
            return true;
        }
    }
    
    cout << "TIMEOUT!" << endl; 

    return false;
}

bool PrioritizedPlanner::inCollision(const rw::proximity::CollisionDetector &detector, const rw::math::Q &q_new, const rw::math::Q &q_robot_1)
{
	rw::proximity::CollisionDetector::QueryResult data;
    
	_device1->setQ(q_robot_1,_state);
    _device2->setQ(q_new, _state);
	if (detector.inCollision(_state,&data)) {
		cout << "Configuration between " << q_new << ", and " << q_robot_1 << endl;
		return true;
	}
	return false;
}



// Brute-force nearest neighbor search.
Node* PrioritizedPlanner::nearestNeighbor(const Tree& tree, const Q& q)
{
    double minLength = DBL_MAX;
    Node* minNode = NULL;

    BOOST_FOREACH(Node* node, tree.getNodes()) {
        const double length = distance(q - getQVal(node)); 
        if (length < minLength) {
            minLength = length;
            minNode = node;
        }
    }

    RW_ASSERT(minNode);
    return minNode;
}

double PrioritizedPlanner::distance(const rw::math::Q& q){
    double sum = 0; 
    for(size_t i = 0; i < q.size(); i++)
    {
        sum += pow(q[i],2); 
    }
    return sqrt(sum); 
} 


ExtendResult PrioritizedPlanner::extend(Tree& tree,const rw::math::Q& q,Node* qNearNode, const Q& qGoal, const rw::proximity::CollisionDetector &detector)
{   
    // Init data needed
    int depth = getDepth(qNearNode); 
    double multi = 0;
    Q qNew;
    cout << "Depth = " << depth << endl; 
    const rw::math::Q& qNear = getQVal(qNearNode);

    // Calculate the difference between random q and the near node
    int val = rand()%100; 
    rw::math::Q delta; 
    if(val > 90){
        cout << "Moving towards random" << endl; 
        delta = q - qNear; 
    }else{
        delta = qGoal - qNear;
        cout << "Moving towards goal" << endl; 
    }
    

    // Convert to distance - This is calculated to estimate the step size
    const double dist = distance(delta);
    cout << "Distance = " << dist << endl;
  
    if(depth < int(_path_1.size()-1)){
        multi = multiplier(delta, depth);
        qNew = qNear + multi * delta;
        cout << "Distance to goal before = " << distance(qNear-qGoal) << endl;
        cout << "Distance to goal after = " << distance(qNew-qGoal) << endl; 

    }else{
        depth = _path_1.size() - 1; 
        qNew = qNear + (_extend/dist) * delta;
        cout << "Distance to goal before = " << distance(qNear-qGoal) << endl;
        cout << "Distance to goal after = " << distance(qNew-qGoal) << endl; 
    }
    cout << "Configuration device 1 = " << _device1->getQ(_state) << endl; 
    cout << "Configuration device 2 = " << qNew << endl; 

    // Calculating the distance to the goal configuration 
    double distGoal = distance(qNew - qGoal);
    cout << "Distance to goal = " << distGoal << endl;

    if (distGoal <= _extend) {
        if (!inCollision(detector, qNew, _path_1[depth])){//!inCollision(rrt, qNearNode, q)) {                
            tree.add(q, qNearNode, 1);
            return Reached;
        } else {
            cout << "Collision!" << endl; 
            return Trapped;
        }
    } else {
        if (!inCollision(detector, qNew, _path_1[depth])){//!inCollision(rrt, qNearNode, qNew)) {
            tree.add(qNew, qNearNode, 1);
            return Advanced;
        } else {
            cout << "Collision!" << endl; 
            return Trapped;
        }
    }
}

double PrioritizedPlanner::multiplier(const rw::math::Q& q, int d)
{
    double value = 0; 
    cout << "Q for largest value test = " << q << endl; 
    for(size_t i = 0; i < q.size(); i++)
    {
        if(abs(q[i]) > value){
            value = abs(q[i]);
        }
    }
    cout << "Timestep / largest value = multiplier " << abs(_timesteps[d]) << ", " << value << " = " << abs(_timesteps[d])/value << endl;
    return abs(_timesteps[d])/value; 
}

ExtendResult PrioritizedPlanner::growTree(Tree& tree,const Q& q_rand, const Q& q_goal, const rw::proximity::CollisionDetector &detector)
{
    Node* qNearNode = nearestNeighbor(tree, q_rand);
    return extend(tree, q_rand, qNearNode, q_goal, detector);

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