#include "LowLevelAStar.h"
using namespace std;

/*
typedef tuple<int, int, int, int> Quad;
typedef pair<int, int> Pair;
typedef tuple<int,int,int> TimedLoc;
typedef tuple<Pair,Pair,int> Collision;
typedef tuple<int,int, Pair, Pair, int> TimedCol;
typedef tuple<int, Pair, Pair,  int> Constraint;
*/


////////////////////////////// Collision and Conflict handlers ///////////////////////////////////////////////////////////

Collision detectSingleCollision(vector<TimedLoc> path1, vector<TimedLoc> path2){
    int max_search_time = max(path1.size(), path2.size());

    for(int t=0; t < max_search_time; t++){
        TimedLoc xyt1 = getLoc(path1, t);
        TimedLoc xyt2 = getLoc(path2, t);
        TimedLoc xyt1p1 = getLoc(path1, t+1);
        TimedLoc xyt2p1 = getLoc(path2, t+1);
        if(get<0>(xyt1) == get<0>(xyt2) and get<1>(xyt1) == get<1>(xyt2))
            return make_tuple(make_pair(get<0>(xyt1), get<1>(xyt1)), make_pair(get<0>(xyt2), get<1>(xyt2)), t);
        if(get<0>(xyt1) == get<0>(xyt2p1) and get<1>(xyt1) == get<1>(xyt2p1)){
            if(get<0>(xyt2) == get<0>(xyt1p1) and get<1>(xyt2) == get<1>(xyt1p1)){
                return make_tuple(make_pair(get<0>(xyt1), get<1>(xyt1)), make_pair(get<0>(xyt1p1), get<1>(xyt1p1)), t+1);
            }
        }
    }
    return make_tuple(make_pair(-1,-1),make_pair(-1,-1),-1);
}

vector<TimedCol> detectCollisionsInPaths(vector<vector<TimedLoc>> paths){
    vector<TimedCol> collisions;

    for(int i = 0; i < paths.size(); i++){
        for(int j=i+1; j < paths.size(); j++){
            Collision colCheck = detectSingleCollision(paths.at(i), paths.at(j));
            if(get<2>(colCheck) == -1) { continue; }
            else{
                Pair loc1 = get<0>(colCheck);
                Pair loc2 = get<1>(colCheck);
                collisions.push_back(make_tuple(i, j, loc1, loc2, get<2>(colCheck)));
            }

        }
    }
    return collisions;
}

vector<Constraint> resolveCollision(TimedCol c){
    vector<Constraint> constraints;

    if(get<2>(c) == get<3>(c)){
        constraints.push_back(make_tuple(get<0>(c), get<2>(c), get<3>(c), get<4>(c)));
        constraints.push_back(make_tuple(get<1>(c), get<2>(c), get<3>(c), get<4>(c)));
    } else {
        constraints.push_back(make_tuple(get<0>(c), get<2>(c), get<3>(c), get<4>(c)));
        constraints.push_back(make_tuple(get<1>(c), get<3>(c), get<2>(c), get<4>(c)));
    }

    return constraints;
}

int getSumOfCosts(vector<vector<TimedLoc>> paths){
    int sum = 0;
    for(int i=0; i < paths.size(); i++){
        sum += paths.size();
    }
    return sum;
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////// CT Node ///////////////////////////////////////////////////////////////

class CTNode{
    public:
    vector<vector<TimedLoc>> paths;
    vector<TimedCol> collisions;
    vector<Constraint> constraints;
    int sum_of_costs;
    CTNode(){
        int sum_of_costs = 0;
    }

    void copyNode(CTNode ctn){
        this->paths = ctn.paths;
        this->collisions = ctn.collisions;
        this->constraints = ctn.constraints;
        this->sum_of_costs = ctn.sum_of_costs;
    }
};

struct CTNodeComparator{
    bool operator()(CTNode const& a, CTNode const& b) const {
        if(a.sum_of_costs > b.sum_of_costs){
            return true;
        }
        else if(a.sum_of_costs == b.sum_of_costs){
            return a.collisions.size() >= b.collisions.size();
        } else {
            return false;
        }
    }
};

//////////////////////////////////////////// Main Search Function //////////////////////////////////////////////////////////

vector<vector<TimedLoc>> cbsSearch(vector<Quad> tasks, Astar plannerObject){

    priority_queue<CTNode, vector<CTNode>, CTNodeComparator> open_queue;
    CTNode root;
    vector<vector<TimedLoc>> results;
    for(int i=0; i < tasks.size(); i++){
        vector<TimedLoc> agent_path = plannerObject.beginSearch(tasks.at(i), i, root.constraints);
        if(agent_path.empty()) {return results;}
        root.paths.push_back(agent_path);
    }
    root.sum_of_costs = getSumOfCosts(root.paths);
    root.collisions = detectCollisionsInPaths(root.paths);
    open_queue.push(root);
    int count = 0;

    while(!open_queue.empty()){
        count++;
        CTNode current_ct_node = open_queue.top();
        open_queue.pop();

        if(current_ct_node.collisions.empty()) { return current_ct_node.paths; }
        if(count == 5) {return current_ct_node.paths;}

        TimedCol curr_collision = current_ct_node.collisions.at(0);
        vector<Constraint> resolved_constraints = resolveCollision(curr_collision);

        for(int i=0; i < resolved_constraints.size(); i++){
            Constraint c = resolved_constraints.at(i);
            CTNode next_ct_node;
            next_ct_node.copyNode(current_ct_node);

            next_ct_node.constraints.push_back(resolved_constraints.at(i));
            int agent_to_replan = get<0>(resolved_constraints.at(i));
            vector<TimedLoc> replanned_path = plannerObject.beginSearch(tasks.at(agent_to_replan), 
                                                                        agent_to_replan, 
                                                                        next_ct_node.constraints);
            
            if(replanned_path.empty()) { continue; }

            next_ct_node.paths.at(agent_to_replan) = replanned_path;
            next_ct_node.collisions = detectCollisionsInPaths(next_ct_node.paths);
            next_ct_node.sum_of_costs = getSumOfCosts(next_ct_node.paths);
            open_queue.push(next_ct_node);
        }
    }
    return results;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



