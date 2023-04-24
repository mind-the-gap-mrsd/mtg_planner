#include "LowLevelSearch.h"
using namespace std;


/***
 * Conflict check: detecting first collision between two given agent paths
 *      a. Given set of endpoints [(x1, y1), (x2, y2)] and [(x3, y3), (x4, y4)] check the sign of
 *         [(y4-y1)/(y2-y1) - (x4-x1)/(x2-x1)] * [(y3-y1)/(y2-y1) - (x3-x1)/(x2-x1)]
 *          i. if above +eps -> no intersection -> no collision
 *         ii. if below -eps -> definite intersection
 *        iii. If within (-eps, +eps) -> one of the vertices is collinear: assume intersection at vertex
 *      
 *      b. If intersects -> Find point of intersection using substitution and calculate time taken to reach the point by
 *         both agents based on time to reach first endpoint + del(heading)/omega + dist/velocity
 * 
 *      c. Create a collision - (agent i, agent j, [float x_imp, float y_imp], float time_imp)
*/

tuple<float, float, float> detectFirstCollision(vector<TimedLoc> path1, vector<TimedLoc> path2){
    float max_time = max(get<2>(path1.back()), get<2>(path2.back()));
    float t_iter = 0;
    while(t_iter <= max_time){
        tuple<float, float, float> loc1 = getLoc(path1, t_iter);
        tuple<float, float, float> loc2 = getLoc(path2, t_iter);
        float distance = sqrt(pow(get<0>(loc2) - get<0>(loc1), 2) + pow(get<1>(loc2) - get<1>(loc1), 2));
        if(distance <= x_eps){
            float x_imp = 0.5*(get<0>(loc1) + get<0>(loc2));
            float y_imp = 0.5*(get<1>(loc1) + get<1>(loc2));
            return make_tuple(x_imp, y_imp, t_iter);
            
        }
        t_iter += delta_t;
    }
    return make_tuple(-1., -1., -1.);
}

vector<Collision> detectCollisionsInPaths(vector<vector<TimedLoc>> agent_paths){
    vector<Collision> collisions;
    for(int i=0; i < agent_paths.size(); i++){
        vector<TimedLoc> path_agent_i = agent_paths.at(i);

        for(int j = i+1; j < agent_paths.size(); j++){
            vector<TimedLoc> path_agent_j = agent_paths.at(j);
            tuple<float, float, float> collision_i_j = detectFirstCollision(path_agent_i, path_agent_j);
            if(get<2>(collision_i_j) >= 0){
                Collision col = make_tuple(i, j, 
                                           make_tuple(get<0>(collision_i_j), get<1>(collision_i_j)), 
                                           get<2>(collision_i_j));
                collisions.push_back(col);
            }
            
        }
    }
    return collisions;
}

/**
 * Collision resolution: From a conflict, we pick each agent and create a constraint
 * RETURNS: vector of two conflicts -> (agent i/j, [float x_imp, float y_imp], float time_imp )
*/

vector<Constraint> resolveCollision(Collision col){
    vector<Constraint> constraints;
    constraints.push_back(make_tuple(get<0>(col), get<2>(col), get<3>(col)));
    constraints.push_back(make_tuple(get<1>(col), get<2>(col), get<3>(col)));
    return constraints;
}

float getSumOfCosts(vector<vector<TimedLoc>> agent_paths){
    float cost = 0;
    for(int i=0; i < agent_paths.size(); i++){
        cost += get<2>(agent_paths.at(i).back());
    }
    return cost;
}

/**
 * Normal CBS High-level search
*/

class CTNode{
    public:
    vector<vector<TimedLoc>> paths;
    vector<Collision> collisions;
    vector<Constraint> constraints;
    float sum_of_costs;
    CTNode(){
        this->sum_of_costs = 0;
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
        else if(abs(a.sum_of_costs - b.sum_of_costs) < t_eps){
            return a.collisions.size() >= b.collisions.size();
        } else {
            return false;
        }
    }
};

//////////////////////////////////////////// Main Search Function //////////////////////////////////////////////////////////

vector<vector<TimedLoc>> cbsSearch(vector<Task> tasks, LowLevelPlanner plannerObject){

    priority_queue<CTNode, vector<CTNode>, CTNodeComparator> open_queue;
    CTNode root;
    vector<vector<TimedLoc>> results;
    for(int i=0; i < tasks.size(); i++){
        cout << "Planning agent " << i << endl;
        vector<TimedLoc> agent_path = plannerObject.beginSearch(tasks.at(i), i, root.constraints);
        if(agent_path.empty()) {return results;}
        root.paths.push_back(agent_path);
    }
    root.sum_of_costs = getSumOfCosts(root.paths);
    root.collisions = detectCollisionsInPaths(root.paths);
    open_queue.push(root);

    while(!open_queue.empty()){
        cout << "Number of Nodes: " << open_queue.size() << endl;
        CTNode current_ct_node = open_queue.top();
        open_queue.pop();

        if(current_ct_node.collisions.empty()) { return current_ct_node.paths; }

        Collision curr_collision = current_ct_node.collisions.at(0);
        vector<Constraint> resolved_constraints = resolveCollision(curr_collision);
        

        for(int i=0; i < resolved_constraints.size(); i++){
            Constraint c = resolved_constraints.at(i);
            CTNode next_ct_node;
            next_ct_node.copyNode(current_ct_node);
            cout << get<0>(c) << ", " << get<0>(get<1>(c)) << ", " << get<1>(get<1>(c)) << ", " << get<2>(c) << endl;
            next_ct_node.constraints.push_back(c);
            int agent_to_replan = get<0>(c);
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