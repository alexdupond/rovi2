#include "prioritizedPlanner.h"

#include <rw/pathplanning/PlannerConstraint.hpp>
#include <rw/pathplanning/QSampler.hpp>


//typedef rw::trajectory::QPath Path;

const Q& getQVal(Node* node) { return node->getValue();}
double getDepth(Node* node){ return node->getDepth(); }



PrioritizedPlanner::PrioritizedPlanner(rw::models::WorkCell::Ptr wc, rw::models::Device::Ptr device1, rw::models::Device::Ptr device2, double extend, double aggressiveness){
    _wc = wc;
    _state = wc->getDefaultState();
    _extend = extend; 
    _aggressiveness = aggressiveness;
    _device1 = device1; 
    _device2 = device2;
}


bool PrioritizedPlanner::checkCollisions(rw::models::Device::Ptr device, const rw::proximity::CollisionDetector &detector, const rw::math::Q &q) {
	rw::proximity::CollisionDetector::QueryResult data;

	device->setQ(q,_state);
	if (detector.inCollision(_state,&data)) {
	//	cout << "Configuration in collision: " << q << endl;
		return false;
	}
	return true;
}


bool PrioritizedPlanner::calculateRRTPath(const vector<rw::math::Q>& qVec, rw::trajectory::QPath& result){
    // Setting device 2 to be as far from device 1 as possible - Hack to remove device 2 when planning for device 1
    rw::math::Q q(6, 2, -2, -1.2, -1.5, 1.5, 0);
    _device2->setQ(q, _state);
	rw::proximity::CollisionDetector detector(_wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy());
	rw::pathplanning::PlannerConstraint constraint = rw::pathplanning::PlannerConstraint::make(&detector,_device1,_state);

	rw::pathplanning::QSampler::Ptr sampler = rw::pathplanning::QSampler::makeConstrained(rw::pathplanning::QSampler::makeUniform(_device1),constraint.getQConstraintPtr());

	rw::math::QMetric::Ptr metric = rw::math::MetricFactory::makeEuclidean<rw::math::Q>();
	rw::pathplanning::QToQPlanner::Ptr planner = rwlibs::pathplanners::RRTPlanner::makeQToQPlanner(constraint, sampler, metric, _extend, rwlibs::pathplanners::RRTPlanner::RRTConnect);

	for(size_t i = 0; i < qVec.size()-1; i++)
    {
        rw::math::Q from = qVec[i]; 
        rw::math::Q to = qVec[i+1]; 
        rw::trajectory::QPath path; 


        if (!checkCollisions(_device1, detector, from))
            return 0;
        if (!checkCollisions(_device1, detector, to))
            return 0;

     //   cout << "Planning from " << from << " to " << to << endl;
        rw::common::Timer t;
        t.resetAndResume();
        planner->query(from,to,path,MAXTIME);
        t.pause(); 

        //cout << "Path of length " << path.size() << " found in " << t.getTime() << " seconds." << endl;
        // string file_name = "./test.txt";
		// fstream csvfile;
		// csvfile.open(file_name, ios::out | ios::app);
		// csvfile << path.size() << ", " << t.getTime() << ", ";
		// csvfile.close();
        
        if (t.getTime() >= MAXTIME) {
           // cout << "Notice: max time of " << MAXTIME << " seconds reached." << endl;
            return false; 
        }else{
            rwlibs::pathoptimization::PathLengthOptimizer pathOptimizer(constraint,metric);
            t.resetAndResume();
            path = pathOptimizer.partialShortCut(path);
            t.pause(); 
           // cout << "Optimized path lenght: " << path.size() << endl; 
		    // csvfile.open(file_name, ios::out | ios::app);
		    // csvfile << path.size() << ", " << t.getTime() << ", ";
		    // csvfile.close();
        }
        

        if(i != 0 && (path[0] == result[result.size()-1]) ){
            result.pop_back();
        }


        for(size_t j = 0; j < path.size(); j++)
        {
            result.push_back(path[j]);
        }
        
    }
 
    if(calculateTimesteps(result))
        return true;
    else 
        return false; 
	
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
//	cout << "Optimized path lenght: " << path.size() << endl;
    cout << path.size(); 
}


bool PrioritizedPlanner::calculateTimesteps(rw::trajectory::QPath& path){ 
	if(path.size()){
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
			_timesteps.push_back(stepsize);
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


bool PrioritizedPlanner::calculateDynamicRRTPath(const vector<rw::math::Q> &qVec, rw::trajectory::QPath& qPathRob1, rw::trajectory::QPath& result){
    _path_1 = qPathRob1; 
    // SETTING UP RRT STRUCT FOR PLANNER
	rw::proximity::CollisionDetector detector(_wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy());
	rw::pathplanning::PlannerConstraint constraint = rw::pathplanning::PlannerConstraint::make(&detector,_device2,_state);

	rw::pathplanning::QSampler::Ptr sampler = rw::pathplanning::QSampler::makeConstrained(rw::pathplanning::QSampler::makeUniform(_device2),constraint.getQConstraintPtr());

    // Checking start and end configuration 

    double timeout = 20;
    
   // cout << "Total path of path 1 is = " << qPathRob1.size() << " and number of timesteps = " << _timesteps.size() << endl; 

    if (inCollision(detector, qVec[0], qPathRob1[0])) {
       // std::cout<<"Start is in collision"<<std::endl;
        return false;
    }

    for(size_t i = 0; i < qVec.size()-1; i++)
    {
        rw::math::Q start = qVec[i]; 
        rw::math::Q goal = qVec[i+1]; 
        int depth = result.size(); 
        rw::trajectory::QPath qPathRob2;

   //     cout << "Calculation path from: " << start << ", to: " << goal << endl; 
    //    cout << "Depth initialized to = " << depth << endl;

        Tree startTree(start, depth);
        Tree goalTree(goal, 1);
        Tree* treeA = &startTree;

        fstream csvfile;

        rw::common::Timer t;
        t.resetAndResume();
        srand (time(NULL)); 

        while (true) {

            const Q qAttr = sampler->sample();
            if (qAttr.empty()) RW_THROW("Sampler must always succeed.");

            if (growTree(*treeA, qAttr, goal, detector) == Reached )
            {
                getPathFromTree(startTree, goalTree, qPathRob2);

             //   cout << "Path found - Size = " << qPathRob2.size() << endl;
              //  cout << "Time to calculate path = " << t.getTime() << endl; 
                // string file_name = "./test.txt";
                // fstream csvfile;
                // csvfile.open(file_name, ios::out | ios::app);
                // csvfile << qPathRob2.size() << ", " << t.getTime()  << endl; 
                // csvfile.close();

               // rw::trajectory::QPath qPath = optimizeDynamicPath(detector, qPathRob1, qPathRob2, _timesteps, result.size()); 
                break; 
            }

            if(t.getTime() > timeout){
                // string file_name = "./test.txt";
                // csvfile.open(file_name, ios::out | ios::app);
                // csvfile << 0 << ", " << 0  << endl; 
                // csvfile.close();
                return false; 
            }
        }


        if(i != 0 && (qPathRob2[0] == result[result.size()-1]) ){
            result.pop_back();
        }

        for(size_t j = 0; j < qPathRob2.size(); j++)
        {
            result.push_back(qPathRob2[j]);
        }
        

    }
    
    if(calculateTimesteps(result))
        return true; 
    return false;
}


bool PrioritizedPlanner::inCollision(const rw::proximity::CollisionDetector &detector, const rw::math::Q& q1, const rw::math::Q& q2) 
{
	rw::proximity::CollisionDetector::QueryResult data;
    
	_device1->setQ(q1,_state);
    _device2->setQ(q2, _state);
	if (detector.inCollision(_state,&data)) {
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
    const rw::math::Q& qNear = getQVal(qNearNode);

    // Calculate the difference between random q and the near node
    int val = rand()%100; 
    rw::math::Q delta; 
    if(val > _aggressiveness){
        delta = q - qNear; 
    }else{
        delta = qGoal - qNear;
    }

    // Convert to distance - This is calculated to estimate the step size
    const double dist = distance(delta);
    Q qNew;
    Q qRobPrev;
    Q qRobNext;
    if(depth < int(_path_1.size()-1)){
        multi = multiplier(delta, depth - 1);
        qNew = qNear + multi * delta;     
        qRobPrev = _path_1[depth-1]; 
        qRobNext = _path_1[depth];
    }else{
        depth = _path_1.size() - 1; 
        qNew = qNear + (_extend/dist) * delta;
        qRobPrev = _path_1[depth]; 
        qRobNext = _path_1[depth];
    }


    double distGoal = distance(qNew - qGoal);

    if (distGoal <= _extend) {
        if (!inCollision(detector, qNew, qRobNext) && !inCollision(detector, qRobPrev, qRobNext, qNear, qNew, 0.005))
        {             
            tree.add(qNew, qNearNode, 1);
            return Reached;
        } else {
            return Trapped;
        }
    } else {
        if (!inCollision(detector, qNew, qRobNext) && !inCollision(detector, qRobPrev, qRobNext, qNear, qNew, 0.005))
        {
            tree.add(qNew, qNearNode, 1);
            return Advanced;
        } else {
            return Trapped;
        }
    }
}

double PrioritizedPlanner::multiplier(const rw::math::Q& q, int d)
{
    double value = 0; 
    for(size_t i = 0; i < q.size(); i++)
    {
        if(abs(q[i]) > value){
            value = abs(q[i]);
        }
    }
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


double maxJoint(rw::math::Q& q){
    double maxJoint = 0;
    for(size_t i = 0; i < q.size(); i++)
    {
        double joint = abs(q[i]);
        if(joint > maxJoint ){
            maxJoint = joint; 
        }
    }
    return maxJoint;  
}

rw::trajectory::QPath PrioritizedPlanner::optimizeDynamicPath(const rw::proximity::CollisionDetector &detector, rw::trajectory::QPath& qFullPath, rw::trajectory::QPath& qPartialPath, vector<double> timesteps, int depth){
    int size = qPartialPath.size(); 
    rw::trajectory::QPath optimizedPath; 

    optimizedPath.push_back(qPartialPath[0]); 

    for(size_t i = 0; i < size - 2; i++)
    {

        rw::math::Q qRob1Old = qFullPath[depth + i]; 
        rw::math::Q qRob1New = qFullPath[depth + i + 2]; 
        rw::math::Q qRob2Old = qPartialPath[i]; 
        rw::math::Q qRob2New = qPartialPath[i + 2]; 

        rw::math::Q delta = qRob2New - qRob2Old;
     //   cout << "Time interval is equal to = " << timesteps[depth + i] << " + " << timesteps[depth + i + 1] << endl;   
        double timeInterval = timesteps[depth + i] + timesteps[depth + i + 1]; 
        double multi = multiplier(delta, depth);

       // cout << "Time interval = " << timeInterval << ", and max joint = " << maxJoint(delta) << endl;  
        if((timeInterval < maxJoint(delta)) && !inCollision(detector, qRob1Old, qRob1New, qRob2Old , qRob2New, 0.005)){
            optimizedPath.push_back(qRob2Old + multi * delta);
        }
    }

    cout << "Old size = " << size << " vs. new size = " << optimizedPath.size() << endl; 
    return optimizedPath; 
}


bool PrioritizedPlanner::inCollision(const rw::proximity::CollisionDetector &detector, const Q& qPrev1, const Q& qNext1, const Q& qPrev2, const Q& qNext2, float e)    
{
    double collisionDetectorCalls = 0;

    Q dq1 = qNext1 - qPrev1;
    Q dq2 = qNext2 - qPrev2; 
    int n = 0; 
 
    if((ceil(dq1.norm2() / e)-1) > (ceil(dq2.norm2() / e)-1))
        n = (ceil(dq1.norm2() / e)-1); 
    else
        n = (ceil(dq2.norm2() / e)-1); 

    Q step1 = (dq1/(n+1));
    Q step2 = (dq2/(n+1));
    for(int i = 1; i < n; i++)
    {
        Q qi1 = i * step1 + qPrev1;
        Q qi2 = i * step2 + qPrev2;
        collisionDetectorCalls++;
        if(inCollision(detector, qi1, qi2))
        {   
            return true;
        }
        
    }
    return false;
}




PrioritizedPlanner::~PrioritizedPlanner(){
    
}