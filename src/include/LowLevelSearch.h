/**
 * METHOD: Build constraint table for checking collisions
 * INPUTS: Vector of constraints -> (int agent_number, float pair Loc, float time_imp)
 * 
 * 1. If current agent != agent_number -> continue
 * 
 * 2. Create a map from pair of float time ranges to vector of int pairs blocked
 *    during that time duration. Vector of int pairs comes from taking a two agent
 *    sized square about point of impact and blocking all cells within it for a duration
 *    (t_imp - eps_t, t_imp + eps_t)
 * 
 *  eps_t -> time taken for agent to cover two body lengths - 2*agent_size/agent_vel
*/

/**
 * METHOD: Check if current location and time of agent is constrained from the 
 * built constraint table
 * 
 * INPUTS: Node being considered, constraint table
 * 
 * 1. Iterate over keys - if node.time is within the range then check if node
 *  location in the vector list. If so, return true, else false.
*/

#include<cmath>
#include<tuple>
#include<algorithm>
#include<queue>
#include<map>
#include "NavGrid.h"
#include "constants.h"
using namespace std;




float dist(int x1, int y1, int x2, int y2){
    return sqrt(pow(x2-x1, 2) + pow(y2-y1, 2));
}

class Node{
    public:
    int x, y;
    float t, heading;
    TimedLoc parent;

    float f,g,h;

    Node(int nx, int ny, Node n){
        this->x = nx;
        this->y = ny;
        this->heading = n.heading;
        this->t = n.t;
        this->g = n.g;
        this->h = 0;
        this->f = this->g + this-> h;
        this->parent = make_tuple(n.x, n.y, n.t);
    }
    Node(int nx, int ny){
        this->x = nx;
        this->y = ny;
        this->t = 0.0;
        this->heading = 0;
        this->g = 0;
        this->h = 0;
        this->f = this->g + this-> h;
        this->parent = make_tuple(-1, -1, -1.0);
    }
    Node(){
        this->x = -1;
        this->y = -1;
        this->t = -1;
        this->g = -1;
        this->heading = 0;
        this->h = -1;
        this->f = -1;
        this->parent = make_tuple(-1,-1, -1.0);
    }
    bool isLessThan(Node n2){
        return this->f < n2.f;
    }
};

struct NodeComparator{
    bool operator()(Node const& a, Node const& b) const {
        return a.f > b.f;
    }
};

class LowLevelPlanner{
    public:
    // start and goal locations
    int x0, y0, xf, yf, agent_id;
    // Planning grid object
    NavGrid planning_grid;
    float max_time;
    //  Movement directions
    int move[9][2] = {{0,0}, {0,1}, {0,-1}, {1, 0}, {-1, 0}, {-1,-1}, {1,-1}, {-1,1}, {1,1}};
    //STL implementations of 
    priority_queue<Node, vector<Node>, NodeComparator> open_queue;
    map<TimedLoc, Node> closed_list;
    map<tuple<float, float>, map<tuple<int, int>, int>> constraint_table;

    LowLevelPlanner(){
        this->x0 = -1;
        this->y0 = -1;
        this->xf = -1;
        this->yf = -1;
        this->max_time = 0.0;
        NavGrid temp;
        this->planning_grid = temp;
        this->agent_id = -1;
    }
    LowLevelPlanner(NavGrid map){
        this->planning_grid = map;
        this->agent_id = -1;
        this->max_time = 0.0;
    }

    float computeHeuristic(Node n){
        return sqrt(pow((n.x - xf),2) + pow((n.y-yf),2));
    }

    float computePathCost(Node curr_node, Node next_node){
        return dist(curr_node.x, curr_node.y, next_node.x, next_node.y);
    }

    bool inMap(int pos_x, int pos_y){
        if(pos_x < 0 || pos_y < 0)
            return false;
        
        if(pos_x >= this->planning_grid.width || pos_y >= this->planning_grid.height)
            return false;

        return true;
    }

    bool hitsObstacle(int pos_x, int pos_y){
        for(int i=-1*(agent_size/2 + 1); i <= (agent_size/2 + 1); i++){
            for(int j=-1*(agent_size/2 + 1); j <= (agent_size/2 + 1); j++){
                if(inMap(pos_x +i, pos_y + j)){
                    if(this->planning_grid.grid[pos_y + j][pos_x + i] == 1){
                        return true;
                    }
                }
            }
        }
        return false;
    }

    vector<tuple<int, int>> getConstrainedCells(tuple<float, float> impact_location){
        int x_imp = (int)round(get<0>(impact_location));
        int y_imp = (int)round(get<1>(impact_location));

        vector<tuple<int, int>> constrained_list;

        for(int i=-1*(3*agent_size); i <= (3*agent_size); i++){
            for(int j=-1*(3*agent_size); j <= (3*agent_size); j++){
                if(inMap(x_imp+i, y_imp + j)){
                    if(pow(i,2) + pow(j,2) <= blocking_radius_sq*pow(agent_size,2))
                        constrained_list.push_back(make_tuple(x_imp + i, y_imp + j)); 
                }
            }
        }

        return constrained_list;
    }

    // Constraint structure - (agent ID, float pair of location, float time)

    void buildConstraintTable(vector<Constraint> constraints){
        
        if(constraints.empty()) { return; }
        this->constraint_table.clear();

        for(int i=0; i < constraints.size(); i++){
            float c_time = get<2>(constraints.at(i));
            if(c_time > this->max_time) { this->max_time = c_time; }

            if(get<0>(constraints.at(i)) == this->agent_id){
                tuple<float, float> key = make_tuple(c_time - t_eps, c_time + t_eps);
                // cout << get<0>(key) <<  ", " << get<1>(key) << endl;
                vector<tuple<int, int>> constrained_list = getConstrainedCells(get<1>(constraints.at(i)));
                for(int j = 0; j < constrained_list.size(); j++){
                    // cout << get<0>(constrained_list.at(j)) << ", " << get<1>(constrained_list.at(j)) << endl;
                    constraint_table[key][constrained_list.at(j)] = 1;
                }
            }
        }
    }

    bool isConstrained(Node n){

        if(this->constraint_table.empty()) { return false; }

        float current_time = n.t;
        map<tuple<float, float>, map<tuple<int, int>, int>>::iterator iter;
        for(iter = this->constraint_table.begin(); iter != this->constraint_table.end(); ++iter){
            tuple<float, float> time_bound = iter->first;
            if(current_time < get<0>(time_bound) || current_time > get<1>(time_bound)){
                continue;
            }
            tuple<int, int> current_loc = make_tuple(n.x, n.y);
            if(iter->second.count(current_loc) > 0){
                return true;
            }
        }

        return false;
    }

    vector<TimedLoc> tracePath(Node n){
        vector<TimedLoc> result;
        result.push_back(make_tuple(n.x, n.y, n.t));
        while(get<0>(n.parent) != -1 and get<1>(n.parent) != -1){
            float time_to_turn = abs(n.heading - this->closed_list[n.parent].heading)/agent_ang_vel;
            result.push_back(make_tuple(get<0>(n.parent), get<1>(n.parent), get<2>(n.parent) + time_to_turn));
            result.push_back(n.parent);
            n = this->closed_list[n.parent];
        }
        reverse(result.begin(), result.end());
        return result;
    }

    bool lineOfSight(Node node1, Node node2){
         
        int x = node1.x;
        int y = node1.y;
        int delx = abs(node2.x - node1.x);
        int dely = abs(node2.y - node1.y);
        int s1 = (node2.x > node1.x) ?  1 : -1;
        int s2 = (node2.y > node1.y) ?  1 : -1;
        if(delx == 0.0) {s1 = 0;}
        if(dely == 0.0) {s2 = 0;}

        int interchange = 0;

        if (dely > delx){
        int temp = delx;
        delx = dely;
        dely = temp;
        interchange = 1;
        }

        int e = 2*dely - delx;
        int a = 2*dely;
        int b = 2*dely - 2*delx;
        for(int i=0; i < delx; i++){
            if(e < 0){
                if(interchange == 1) {y = y + s2;}
                else {x = x + s1;}
                e = e + a;
            }
            else {
            y = y + s2;
            x = x + s1;
            e = e + b;
            }
            if(inMap(x, y)){
                if(hitsObstacle(x, y)) {return false;}
                Node intermediate(x, y, node1);
                intermediate.t = node1.t + computePathCost(intermediate, node1)*this->planning_grid.resolution/agent_velocity + abs(node1.heading - node2.heading)/agent_ang_vel; 
                if(isConstrained(intermediate)) {return false;}
            }
        }
        return true;
    }

/**
 * 1. Node has int x,y float time, heading, Node parent
 * 2. During normal A-star, when creating child node do these checks
 *      a. If not in map -> continue
 *      b. If in obstacle -> continue
 * 
 *      c. If line of sight with current parent:
 *          i. child.heading <-- arctan[(y_child - y_parent)/(x_child - x_parent)]
 *         ii. child.time <-- Parent.time + |child.heading - parent.heading|/omega + dist(child, parent)/vel
 * 
 *      d. If not in line of sight:
 *          i. child.heading <- Direct based on move direction
 *         ii. child.time <- current.time + del(heading)/omega + dist(child, current)/vel
 * 
 *      e. If child.loc and child.time is constrained -> continue
 *      f. Else do duplicate check and add to open queue
 * 
 * RETURNS : vector of timestamped locations - (int x, int y, float time_to_reach)
*/

    vector<TimedLoc> beginSearch(Task task, int id, vector<Constraint> constraints){

        while(!this->open_queue.empty())
            this->open_queue.pop();
        this->closed_list.clear();
        
        this->x0 = get<0>(task);
        this->y0 = get<1>(task);
        this->xf = get<2>(task);
        this->yf = get<3>(task);
        this->agent_id = id;

        buildConstraintTable(constraints);

        Node start(x0, y0);
        start.h = computeHeuristic(start);
        start.f = start.g + start.h;
        open_queue.push(start);

        vector<TimedLoc> result;
        
        while(!this->open_queue.empty()){
            Node curr_node = this->open_queue.top();
            this->open_queue.pop();

            if(curr_node.x == xf && curr_node.y == yf){
                bool check_constraint_flag = false;
                if(curr_node.t <= this->max_time + t_eps)
                    check_constraint_flag = true;
                if(!check_constraint_flag){
                    result = tracePath(curr_node);
                    return result;
                }
            }

            for(int dir=0; dir < 9; dir++){
                int new_pos_x = curr_node.x + this->move[dir][0];
                int new_pos_y = curr_node.y + this->move[dir][1];

                if(!inMap(new_pos_x, new_pos_y)) { continue; }
                // if(this->planning_grid.grid[new_pos_y][new_pos_x] == 1){continue;}
                if(hitsObstacle(new_pos_x, new_pos_y)) { continue; }

                Node child(new_pos_x, new_pos_y, curr_node);

                Node parent_of_curr = this->closed_list[curr_node.parent];
                if(parent_of_curr.x == -1 and parent_of_curr.y == -1){
                    parent_of_curr = start;
                }

                if(lineOfSight(parent_of_curr, child)){
                    child.h = computeHeuristic(child);
                    if(dir != 0){
                        child.parent = make_tuple(parent_of_curr.x, parent_of_curr.y, parent_of_curr.t);
                        child.heading = atan2(child.y - parent_of_curr.y, child.x - parent_of_curr.x);
                        float parent_child_dist = computePathCost(parent_of_curr, child);
                        child.t = parent_of_curr.t + abs(child.heading - parent_of_curr.heading)/agent_ang_vel + parent_child_dist*this->planning_grid.resolution/agent_velocity;
                        child.g = parent_of_curr.g + parent_child_dist;
                        child.f = child.g + child.h;
                    }
                    else{
                        child.t = curr_node.t + 1;
                        child.g += agent_velocity/this->planning_grid.resolution;
                        child.f = child.g + child.h;
                    }
                }
                else {
                    child.h = computeHeuristic(child);
                    if(dir != 0){
                        child.heading = atan2(child.y - curr_node.y, child.x - curr_node.x);
                        float curr_child_dist = computePathCost(curr_node, child);
                        child.t = curr_node.t + abs(child.heading-curr_node.heading)/agent_ang_vel + curr_child_dist*this->planning_grid.resolution/agent_velocity;
                        child.g = curr_node.g + curr_child_dist;
                        child.f = child.g + child.h;
                    }
                    else {
                        child.t = curr_node.t + 1;
                        child.g += agent_velocity/this->planning_grid.resolution;
                        child.f = child.h + child.g;
                    }
                }
                
                if(isConstrained(child)){ continue; }

                map<TimedLoc,Node>::iterator iter = closed_list.find(make_tuple(child.x, child.y, child.t));
                if(iter != closed_list.end()){
                    if(child.isLessThan(iter->second)){
                        closed_list[make_tuple(child.x, child.y, child.t)] = child;
                        open_queue.push(child);
                    }

                }
                else{
                    closed_list[make_tuple(child.x, child.y, child.t)] = child;
                    open_queue.push(child);
                }    
            }
        }
        return result;       
    }
};





