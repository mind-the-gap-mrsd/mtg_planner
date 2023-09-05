#include "NavGrid.h"
using namespace std;
mutex mtx_lowlevel;




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
    // int move[5][2] = {{0,0}, {0,1}, {0,-1}, {1, 0}, {-1, 0}};
    //STL implementations of 
    priority_queue<Node, vector<Node>, NodeComparator> open_queue;
    map<TimedLoc, Node> closed_list;
    map<TimedLoc, Node> open_list;
    //constraint_matrix[i][j] -> vector of (float,float) constrained time intervals at location i,j
    vector<vector<vector<tuple<float, float>>>> constraint_matrix;

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
        this->constraint_matrix.resize(planning_grid.height, vector<vector<tuple<float, float>>>(planning_grid.width));
        this->agent_id = -1;
        this->max_time = 0.0;
    }

    float computeHeuristic(Node n){
        int nx = n.x;
        int ny = n.y;

        int dijkstra_cost = this->planning_grid.goal_dijkstra_map[make_tuple(xf, yf)].at(ny).at(nx);
        // Factor of 10 in denominator since dijkstra is run with 10 and 14 instead of 1 and 1.4 - easier to write :P
        return (float)dijkstra_cost*grid_resolution/(10.0*agent_velocity);
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

    bool hitsObstacle(int pos_x, int pos_y, char mdf){

        if(mdf == 'u'){
            for(int i=-2*(agent_size); i < 2*(agent_size); i++){
                for(int j=-2*(agent_size); j < 2*(agent_size); j++){
                    if(inMap(pos_x +i, pos_y + j)){
                        if(pow(i,2) + pow(j,2) <= pow(inflation_radius,2)){
                            if(this->planning_grid.grid[pos_y + j][pos_x + i] == 1){
                                return true;
                            }
                        }
                    }
                }
            }
            return false;
        }

        if(mdf == 'd'){
            if(this->planning_grid.grid[pos_y][pos_x] == 1)
                return true;
            else
                return false;
        }

        return false;
    }

    vector<tuple<int, int>> updateConstrainedCells(tuple<float, float> impact_location, float constrained_time){
        int x_imp = (int)round(get<0>(impact_location));
        int y_imp = (int)round(get<1>(impact_location));

        vector<tuple<int, int>> constrained_list;

        for(int i=-1*(3*agent_size); i <= (3*agent_size); i++){
            for(int j=-1*(3*agent_size); j <= (3*agent_size); j++){
                if(inMap(x_imp+i, y_imp + j)){
                    if(pow(i,2) + pow(j,2) <= blocking_radius_sq*pow(agent_size,2))
                        this->constraint_matrix[y_imp + j][x_imp + i].push_back(make_tuple(constrained_time - t_eps/2, constrained_time + t_eps/2)); 
                }
            }
        }

        return constrained_list;
    }

    void buildConstraintTable(vector<Constraint> constraints){
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
     *  eps_t -> time taken for agent to cover two body lengths - 2*agent_size*resolution/agent_vel
     * 
     * 3. Constraint structure - (agent ID, float pair of location, float time)
    */
        
        if(constraints.empty()) { return; }
        this->constraint_matrix.clear();
        this->constraint_matrix.resize(this->planning_grid.height, vector<vector<tuple<float, float>>>(this->planning_grid.width));

        for(int i=0; i < constraints.size(); i++){
            if(get<0>(constraints.at(i)) == this->agent_id){
                updateConstrainedCells(get<1>(constraints.at(i)), get<2>(constraints.at(i)));
            }
        }
        if(this->constraint_matrix[this->yf][this->xf].empty()) { this->max_time = 0; }

        for(int i = 0; i < this->constraint_matrix[this->yf][this->xf].size(); i++){
            tuple<float, float> time_bounds = this->constraint_matrix[this->yf][this->xf].at(i);
            if (get<1>(time_bounds) > this->max_time) {this->max_time = get<1>(time_bounds);}
        }
    }

    bool isConstrained(Node n){
    /**
     * METHOD: Check if current location and time of agent is constrained from the 
     * built constraint table
     * 
     * INPUTS: Node being considered, constraint table
     * 
     * 1. Iterate over keys - if node.time is within the range then check if node
     *  location in the vector list. If so, return true, else false.
    */
        vector<tuple<float, float>> time_bounds = this->constraint_matrix[n.y][n.x];
        if(time_bounds.empty()) { 
            return false; 
        }

        for(int i=0; i <time_bounds.size(); i++){
            float t_lower = get<0>(time_bounds.at(i));
            float t_upper = get<1>(time_bounds.at(i));
            if((n.t - t_lower)*(n.t-t_upper) <= 0) { 
                return true; } 
        }
        return false;
    }

    vector<TimedLoc> tracePath(Node n){
        vector<TimedLoc> result;
        result.push_back(make_tuple(n.x, n.y, n.t));
        while(get<0>(n.parent) != -1 and get<1>(n.parent) != -1){
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
                if(hitsObstacle(x, y, map_dilate_flag)) {return false;}
                Node intermediate(x, y, node1);
                intermediate.t = node1.t + computePathCost(intermediate, node1)*this->planning_grid.resolution/agent_velocity; 
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

    void searchNextNode_thread(int dir, Node& curr_node, Node& start){
        int new_pos_x = curr_node.x + this->move[dir][0];
        int new_pos_y = curr_node.y + this->move[dir][1];

        if(!inMap(new_pos_x, new_pos_y)) { return; }
        if(hitsObstacle(new_pos_x, new_pos_y, map_dilate_flag)) { return; }

        Node child(new_pos_x, new_pos_y, curr_node);

        Node parent_of_curr = this->closed_list[curr_node.parent];
        if(parent_of_curr.x == -1 and parent_of_curr.y == -1){
            parent_of_curr = start;
        }

        child.h = computeHeuristic(child);

        if(dir != 0){
            if(lineOfSight(parent_of_curr, child)){
                child.parent = make_tuple(parent_of_curr.x, parent_of_curr.y, parent_of_curr.t);
                child.heading = atan2(child.y - parent_of_curr.y, child.x - parent_of_curr.x);
                float parent_child_dist = computePathCost(parent_of_curr, child);
                child.t = parent_of_curr.t  + parent_child_dist*this->planning_grid.resolution/agent_velocity;
                child.g = child.t;
                child.f = child.g + child.h;
            }
            else {
                child.heading = atan2(child.y - curr_node.y, child.x - curr_node.x);
                float curr_child_dist = computePathCost(curr_node, child);
                child.t = curr_node.t  + curr_child_dist*this->planning_grid.resolution/agent_velocity;
                // child.g = curr_node.g + curr_child_dist;
                child.g = child.t;
                child.f = child.g + child.h;
            }

        } else {
            child.t = curr_node.t + 1.4*this->planning_grid.resolution/agent_velocity;
            child.g = child.t;
            child.f = child.g + child.h;
        }

        if(isConstrained(child)){ return; }

        map<TimedLoc,Node>::iterator iter = closed_list.find(make_tuple(child.x, child.y, child.t));
        if(iter != closed_list.end()){
            if(child.isLessThan(iter->second)){
                mtx_lowlevel.lock();
                closed_list[iter->first] = child;
                open_queue.push(child);
                mtx_lowlevel.unlock();
            }

        }
        else{
            mtx_lowlevel.lock();
            closed_list[make_tuple(child.x, child.y, child.t)] = child;
            open_queue.push(child);
            mtx_lowlevel.unlock();
        }
    }
    
    vector<TimedLoc> beginSearch(Task task, int id, vector<Constraint> constraints){
        auto start_time = chrono::high_resolution_clock::now();
        auto end_time = chrono::high_resolution_clock::now();
        chrono::duration<double> duration = end_time - start_time; 
        while(!this->open_queue.empty())
            this->open_queue.pop();
        this->closed_list.clear();

        std::ofstream logfile;
        logfile.open("log" + to_string(id) +".csv", std::ios::app);
        logfile << "-1,-1,-1" << endl;
        
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

        if(hitsObstacle(xf, yf, map_dilate_flag)){
            cout << "End point for Agent " << id << " is too close to obstacles" << endl;
            return result;
        }
        
        while(!this->open_queue.empty() && duration.count() < TIME_OUT){
            end_time = chrono::high_resolution_clock::now();
            duration = end_time - start_time;
            // cout << "Exec time is: " << duration.count() << endl;
            Node curr_node = this->open_queue.top();
            // Remove from open queue
            this->open_queue.pop();
            

            // Write current timestamped node to csv file for viz
            logfile << curr_node.x << "," << curr_node.y << "," << curr_node.t << endl;


            if(curr_node.x == xf && curr_node.y == yf){
                // TODO: Logic to check if goal is constrained in future states before returning
                bool goal_is_constrained = false;

                Node n(curr_node.x, curr_node.y, curr_node);
                while(n.t <= this->max_time + t_eps){
                    if(isConstrained(n)){
                        goal_is_constrained = true;
                        curr_node.t = n.t;
                        break;
                    }
                    n.t += t_eps;
                }
                if(!goal_is_constrained){ 
                    result = tracePath(curr_node);
                    return result;
                }
            }

            thread threads[9];
            for(int dir=0; dir < 9; dir++){
                threads[dir] = thread(&LowLevelPlanner::searchNextNode_thread, this, dir, std::ref(curr_node), std::ref(start));
            }

            for(int dir=0; dir < 9; dir++){
                threads[dir].join();
            }

                // int new_pos_x = curr_node.x + this->move[dir][0];
                // int new_pos_y = curr_node.y + this->move[dir][1];

                // if(!inMap(new_pos_x, new_pos_y)) { continue; }
                // if(hitsObstacle(new_pos_x, new_pos_y, map_dilate_flag)) { continue; }

                // Node child(new_pos_x, new_pos_y, curr_node);

                // Node parent_of_curr = this->closed_list[curr_node.parent];
                // if(parent_of_curr.x == -1 and parent_of_curr.y == -1){
                //     parent_of_curr = start;
                // }

                // child.h = computeHeuristic(child);

                // if(dir != 0){
                //     if(lineOfSight(parent_of_curr, child)){
                //         child.parent = make_tuple(parent_of_curr.x, parent_of_curr.y, parent_of_curr.t);
                //         child.heading = atan2(child.y - parent_of_curr.y, child.x - parent_of_curr.x);
                //         float parent_child_dist = computePathCost(parent_of_curr, child);
                //         child.t = parent_of_curr.t  + parent_child_dist*this->planning_grid.resolution/agent_velocity;
                //         child.g = child.t;
                //         child.f = child.g + child.h;
                //     }
                //     else {
                //         child.heading = atan2(child.y - curr_node.y, child.x - curr_node.x);
                //         float curr_child_dist = computePathCost(curr_node, child);
                //         child.t = curr_node.t  + curr_child_dist*this->planning_grid.resolution/agent_velocity;
                //         // child.g = curr_node.g + curr_child_dist;
                //         child.g = child.t;
                //         child.f = child.g + child.h;
                //     }

                // } else {
                //     child.t = curr_node.t + t_eps;
                //     child.g = child.t;
                //     child.f = child.g + child.h;
                // }

                // if(isConstrained(child)){ continue; }

                // map<TimedLoc,Node>::iterator iter = closed_list.find(make_tuple(child.x, child.y, child.t));
                // if(iter != closed_list.end()){
                //     if(child.isLessThan(iter->second)){
                //         closed_list[iter->first] = child;
                //         open_queue.push(child);
                //     }

                // }
                // else{
                //     closed_list[make_tuple(child.x, child.y, child.t)] = child;
                //     open_queue.push(child);
                // }
        }
        return result;       
    }
};





