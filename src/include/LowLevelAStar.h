#include<iostream>
#include<cmath>
#include<vector>
#include<string>
#include<fstream>
#include<tuple>
#include<algorithm>
#include<queue>
#include<map>
#include<array>
using namespace std;

typedef tuple<int, int, int, int> Quad;
typedef pair<int, int> Pair;
typedef tuple<int,int,int> TimedLoc;
typedef tuple<Pair,Pair,int> Collision;
typedef tuple<int,int, Pair, Pair, int> TimedCol;
typedef tuple<int, Pair, Pair,  int> Constraint;
typedef tuple<Pair, Pair> LocPair;

class Node{
    public:

    int x, y, t;
    TimedLoc parent;
    float f,g,h;

    Node(int nx, int ny, Node n){
        this->x = nx;
        this->y = ny;
        this->t = n.t + 1;
        this->g = n.g;
        this->h = 0;
        this->f = this->g + this-> h;
        this->parent = make_tuple(n.x, n.y, n.t);
    }
    Node(int nx, int ny){
        this->x = nx;
        this->y = ny;
        this->t = 0;
        this->g = 0;
        this->h = 0;
        this->f = this->g + this-> h;
        this->parent = make_tuple(-1, -1, -1);
    }
    Node(){
        this->x = -1;
        this->y = -1;
        this->t = -1;
        this->g = -1;
        this->h = -1;
        this->f = -1;
        this->parent = make_tuple(-1,-1, -1);
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

class Grid{
    public:
    int height, width;
    float resolution;
    vector<vector<int>> g;
    // Grid(string filename){
    //     ifstream obs_file;
    //     obs_file.open(filename);
    //     string line;
    //     int i  = 0;
    //     if(obs_file.is_open()){
    //         while(obs_file){
    //             getline(obs_file, line);
    //             if(i == 0){
    //                 this->height = stoi(line);
    //             }
    //             else if(i == 1){
    //                 this->width = stoi(line);
    //                 vector<vector<int>> temp(this->height, vector<int>(this->width, 0));
    //                 this->g = temp;
    //             }
    //             else{
    //                 int split = line.find(",");
    //                 int x = stoi(line.substr(0,split));
    //                 int y = stoi(line.substr(split+1));
    //                 cout << this->height << ", " << this->width << endl;
    //                 this->g[y][x] = 1;
    //             }
    //             i++;
    //         }
    //     }   

    // }

    Grid(int h, int w, float res, vector<int> data){
        this->height = h;
        this->width = w;
        this->resolution = res;
        vector<vector<int>> temp(h, vector<int>(w, 0));
        this->g = temp;
        for(int i=0; i < h; i++){
            for(int j=0; j < w; j++){
                if(data[i*w + j] == 100)
                    this->g[h-1-i][j] = 1;
                else
                    this->g[h-1-i][j] = 0;
            }
        }
    }
    Grid(){
        this->height = 0;
        this->width = 0;
        vector<vector<int>> temp;
        this->g = temp;
    }

    void printGrid(){
        if(!this->g.empty()){
            for(int i=0; i < this->height; i++){
                for(int j=0; j < this->width; j++){
                    cout << this->g[i][j];
                }
                cout << endl;
            }
        }
    }
};

class Astar{
    public:
    int x0, y0, xf, yf, agent_id;
    int max_time;
    Grid grid;
    // array<array<int, 2>, 9> move = {{{0,0}, {0,1}, {0,-1}, {1, 0}, {-1, 0}, {-1,-1}, {1,-1}, {-1,1}, {1,1}}};
    array<array<int, 2>, 5> move = {{{0,0}, {0,1}, {0,-1}, {1, 0}, {-1, 0}}};
    priority_queue<Node, vector<Node>, NodeComparator> open_queue;
    map<TimedLoc, Node> closed_list;
    map<int, map<LocPair, int>> constraint_table;

    Astar(){
        this->x0 = -1;
        this->y0 = -1;
        this->xf = -1;
        this->yf = -1;
        Grid temp;
        this->grid = temp;
        this->max_time = -1;
        this->agent_id = -1;
    }
    Astar(Grid map){
        this->grid = map;
        this->max_time = 0;
        this->agent_id = -1;
    }
    float computeHeuristic(Node n){
        return sqrt(pow((n.x - xf),2) + pow((n.y-yf),2));
    }
    float computePathCost(Node curr_node, Node next_node){
        if((next_node.x-curr_node.x)*(next_node.y - curr_node.y) == 0)
            return 1;
        else
            return 1.4;
    }

    bool inMap(int pos_x, int pos_y){
        if(pos_x < 0 || pos_y < 0)
            return false;
        
        if(pos_x >= this->grid.width || pos_y >= this->grid.height)
            return false;

        return true;
    }

    // Constraint structure -> (agentID, Loc1, Loc2, time)
    void buildConstraintTable(vector<Constraint> constraints){
        
        this->constraint_table.clear();

        if(constraints.empty()) { return; }

        for(int i=0; i < constraints.size(); i++){
            int c_time = get<3>(constraints.at(i));
            if(c_time > this->max_time) { this->max_time = c_time; }

            if(get<0>(constraints.at(i)) == this->agent_id){
                this->constraint_table[c_time][make_tuple(get<1>(constraints.at(i)), get<2>(constraints.at(i)))] = 1;
            }
        }
    }

    bool isConstrained(Node curr_node, Node next_node, int next_time){

        Pair curr_node_loc = make_pair(curr_node.x, curr_node.y);
        Pair next_node_loc = make_pair(next_node.x, next_node.y);
        LocPair edge = make_pair(curr_node_loc, next_node_loc);
        LocPair vertex = make_pair(next_node_loc, next_node_loc);

        if(this->constraint_table.count(next_time) == 0){
            return false;
        }

        if(this->constraint_table[next_time].count(vertex) != 0){ return true; }

        if(this->constraint_table[next_time].count(edge) != 0) { return true; }

        return false;
        
    }

    vector<TimedLoc> tracePath(Node n){
        vector<TimedLoc> result;
        result.push_back(make_tuple(n.x, n.y, n.t));
        while(n.parent != make_tuple(-1,-1,-1)){
            result.push_back(n.parent);
            n = this->closed_list[n.parent];
        }
        reverse(result.begin(), result.end());
        return result;
    }

    vector<TimedLoc> beginSearch(Quad task, int id, vector<Constraint> constraints){

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
               result = tracePath(curr_node);
               return result;
            }

            for(int dir=0; dir < this->move.size(); dir++){
                int new_pos_x = curr_node.x + this->move[dir][0];
                int new_pos_y = curr_node.y + this->move[dir][1];

                if(!inMap(new_pos_x, new_pos_y)) { continue; }

                if(this->grid.g[new_pos_y][new_pos_x] == 1) { continue; }

                Node child(new_pos_x, new_pos_y, curr_node);
                child.h = computeHeuristic(child);
                child.g = child.g + computePathCost(curr_node, child);
                child.f = child.h + child.g;

                if(isConstrained(curr_node, child, child.t)) { continue; }

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

TimedLoc getLoc(vector<TimedLoc> path, int time){
    if(time < 0)
        return path.at(0);
    else if(time < path.size()){
        return path.at(time);
    } else {
        return path.back();
    }
}
