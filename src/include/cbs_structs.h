#include "LowLevelAStar.h"
#include<sstream>
using namespace std;

typedef tuple<int, int, int, int> Conflict;

vector<Quad> ingestTasks(string filename){
    
    vector<Quad> inputTasks;
    ifstream taskFile;
    taskFile.open(filename);
    string line;
    if(taskFile.is_open()){
        while(taskFile){
            getline(taskFile, line);
            vector<int> v;
            string tmp;
            std::stringstream ss(line);
            while(getline(ss, tmp, ',')){
                v.push_back(stoi(tmp));
            }

            Quad agentTask = make_tuple(v[0], v[1], v[2], v[3]);
            inputTasks.push_back(agentTask);
        }
    }
    inputTasks.pop_back();
    return inputTasks;
}

Pair tripleToPair(Triple tr){
    return make_pair(get<0>(tr), get<1>(tr));
}


vector<Conflict> checkVertexCollisions(vector<Triple> agentLocsAtTime_t, int t){

    /**
    * Duplicate vertex detection at a fixed time instant across all agents
    * Pair-wise search in m^2
    */

    vector<Conflict> vertexCollisionVector;
    for(int i=0; i < agentLocsAtTime_t.size()-1; i++){
        Triple Loc_1 = agentLocsAtTime_t.at(i);
        for(int j=i+1; j < agentLocsAtTime_t.size(); j++){
            Triple Loc_2 = agentLocsAtTime_t.at(j);
            if(Loc_1 == Loc_2){
                vertexCollisionVector.push_back(make_tuple(i, j, t, 0));
            }
            else
                continue;
        }
    }
    return vertexCollisionVector;
}

vector<Conflict> checkEdgeCollisions(vector<Triple> agentLocs_t, vector<Triple> agentLocs_tp1, int t){

    /**
    * Checking across consecutive timesteps if two agents are exchanging positions
    * n^2 search with duplicate removal
    */

    vector<Conflict> edgeCollisionVector;

    for(int i=0; i < agentLocs_t.size(); i++){
        Pair agentNextLoc_i = tripleToPair(agentLocs_tp1.at(i));
        for(int j=0; j < agentLocs_tp1.size(); j++){
            if(j==i)
                continue;
            
            if(tripleToPair(agentLocs_t.at(j)) == agentNextLoc_i){
                if(tripleToPair(agentLocs_tp1.at(j)) == tripleToPair(agentLocs_t.at(i))){
                    Conflict collidingAgents = make_tuple(i, j, t, 1);
                    if(find(edgeCollisionVector.begin(), edgeCollisionVector.end(), make_tuple(j, i, t, 1)) != edgeCollisionVector.end())
                        edgeCollisionVector.push_back(collidingAgents);
                }
            }
        }
    }

    return edgeCollisionVector;
}

class CTNode{
    public:
    vector<vector<Triple>> agentPlans;
    vector<Quad> constraints;
    vector<Conflict> conflicts;
    Astar plannerObject;
    vector<Quad> tasks;
    bool possible;

    CTNode(OccupancyGrid g, vector<Quad> inputTasks){
        
        this->plannerObject = Astar(g);
        this->tasks = inputTasks;
        for(int i=0; i < tasks.size(); i++){
            int sx = get<0>(tasks.at(i));
            int sy = get<1>(tasks.at(i));
            int gx = get<2>(tasks.at(i));
            int gy = get<3>(tasks.at(i));
            
            
            vector<Triple> currAgentPlan = this->plannerObject.beginSearch(sx, sy, gx, gy, this->constraints);
            if(currAgentPlan.empty()){
                possible = false;
            }
            this->agentPlans.push_back(currAgentPlan);
        }
        if(possible)
            this->conflicts = findConflicts();
        
    };

    CTNode(){
        vector<vector<Triple>> aP;
        this->agentPlans = aP;
        vector<Quad> cons;
        this->constraints = cons;
        vector<Conflict> conf;
        this->conflicts = conf;
        this->plannerObject = Astar();
        vector<Quad> emptyTasks;
        this->tasks = emptyTasks;
        this->possible = true; 
    };

    bool replanAgent(int agentNumber);
    vector<Conflict> findConflicts();
    int nodeCost();
    vector<Quad> constraintFromConflict(Conflict conf);
    vector<vector<Triple>> HighLevelSearch(OccupancyGrid g, vector<Quad> inputTasks);
    void copyNode(CTNode N);
};

void CTNode::copyNode(CTNode N){

    this->agentPlans = N.agentPlans;
    this->constraints = N.constraints;
    this->conflicts = N.conflicts;
    this->plannerObject = N.plannerObject;
    this->tasks = N.tasks;
    this->possible = N.possible;

    return;
}

int CTNode::nodeCost(){

    int cost = 0;
    for(int i=0; i < agentPlans.size(); i++){
        vector<Triple> singlePlan = agentPlans.at(i);
        int ind_cost = singlePlan.size();
        Pair finalPos = tripleToPair(singlePlan.back());
        Pair compare = tripleToPair(singlePlan.at(ind_cost-1));
        while(compare == finalPos){
            ind_cost--;
            compare = tripleToPair(singlePlan.at(ind_cost-1));
        }
        
        cost += ind_cost;

    }
    return cost;
}

vector<Quad> CTNode::constraintFromConflict(Conflict conf){
    /**
     * Conflict is of the form -> {agent 1, agent 2, time, 0(vertex)/1(edge)}
     * Constraint is of the form -> {location (id), time, -1} for vertex
     * and {location 1 (id), location 2 (id), time} for edge
    */

   // Vertex -> constr k = {agent k at time t (id), time t, -1}; k = 1,2

   int flag = get<3>(conf);
   int width = plannerObject.og.width;

   int agent1 = get<0>(conf);
   int agent2 = get<1>(conf);
   int time = get<2>(conf);

   vector<Quad> constraints;
   
   if(flag == 0){
    Pair Agent1Loc = tripleToPair(agentPlans.at(agent1).at(time));
    Pair Agent2Loc = tripleToPair(agentPlans.at(agent2).at(time));

    int Agent1_id = Agent1Loc.second*width + Agent1Loc.first;
    int Agent2_id = Agent2Loc.second*width + Agent2Loc.first;

    constraints.push_back(make_tuple(agent1, Agent1_id, time, -1));
    constraints.push_back(make_tuple(agent2, Agent2_id, time, -1));

   }

   if(flag == 1){
    Pair Agent1Loc_t = tripleToPair(agentPlans.at(agent1).at(time-1));
    Pair Agent1Loc_tp1 = tripleToPair(agentPlans.at(agent1).at(time));

    Pair Agent2Loc_t = tripleToPair(agentPlans.at(agent2).at(time-1));
    Pair Agent2Loc_tp1 = tripleToPair(agentPlans.at(agent2).at(time));

    int Agent1_id_t = Agent1Loc_t.second*width + Agent1Loc_t.first;
    int Agent1_id_tp1 = Agent1Loc_tp1.second*width + Agent1Loc_tp1.first;

    int Agent2_id_t = Agent2Loc_t.second*width + Agent2Loc_t.first;
    int Agent2_id_tp1 = Agent2Loc_tp1.second*width + Agent2Loc_tp1.first;

    constraints.push_back(make_tuple(agent1, Agent1_id_t, Agent1_id_tp1, time));
    constraints.push_back(make_tuple(agent2, Agent2_id_t, Agent2_id_tp1, time));

   }

   return constraints;
}

vector<Conflict> CTNode::findConflicts(){

    //Extend agentPlans to repeat last element till all agent plans are defined for full makespan

    int numOfAgents = agentPlans.size();
    int makespan = 0;
    vector<vector<Triple>> extendedAgentPlans = agentPlans;

    for(int i=0; i < agentPlans.size(); i++){
        int agentTime = agentPlans.at(i).size();
        if(agentTime > makespan)
            makespan = agentTime;
    }

    for(int i=0; i < extendedAgentPlans.size(); i++){
        Triple goalPoint = extendedAgentPlans.at(i).back();
        int timeForAgent = agentPlans.at(i).size();
        for(int time = timeForAgent; time < makespan; time++){
            extendedAgentPlans.at(i).push_back(make_tuple(get<0>(goalPoint), get<1>(goalPoint), time));
        }
    }

    this->agentPlans = extendedAgentPlans;


    //Convert to time-indexed array of vector<Triple> to make collision check easier

    vector<Triple> timeIndexedPaths[makespan];

    for(int i =0; i < makespan; i++){
        for(int j=0; j < numOfAgents; j++){
            timeIndexedPaths[i].push_back(extendedAgentPlans.at(j).at(i));
        }
    }

    //Vertex collision - duplicates at a given index

    vector<Conflict> vertexCollisions;
    for(int t=0; t < makespan; t++){
        vector<Conflict> vertColl_t = checkVertexCollisions(timeIndexedPaths[t], t);
        vertexCollisions.insert(vertexCollisions.end(), vertColl_t.begin(), vertColl_t.end());
    }

    //Edge collision - (x,y) at t -> (p,q) at t+1 for one agent while another agent is (p,q) at t -> (x,y) at t+1 without duplicates

    vector<Conflict> edgeCollisions;
    for(int t=0; t < makespan-1 ; t++){
        vector<Conflict> edgeColl_t = checkEdgeCollisions(timeIndexedPaths[t], timeIndexedPaths[t+1], t+1);
        edgeCollisions.insert(edgeCollisions.end(), edgeColl_t.begin(), edgeColl_t.end());
    }

    //return as a vector of quadruples -> {agent1 index, agent2 index, time of collision, 0/1 for vertex edge}

    vector<Conflict> allCollisions = vertexCollisions;
    allCollisions.insert(allCollisions.end(), edgeCollisions.begin(), edgeCollisions.end());

    return allCollisions;
}

bool CTNode::replanAgent(int agentNumber){

    // Replan agent at index i with new constraint set keeping others constant since no new constraints means algo should give same output
    int sx = get<0>(tasks.at(agentNumber));
    int sy = get<1>(tasks.at(agentNumber));
    int gx = get<2>(tasks.at(agentNumber));
    int gy = get<3>(tasks.at(agentNumber));

    vector<Triple> replannedPath = plannerObject.beginSearch(sx, sy, gx, gy, constraints);
    if(replannedPath.empty())
        return false;
    else{
        agentPlans.at(agentNumber) = replannedPath;
        return true;
    }
}

vector<vector<Triple>> HighLevelSearch(OccupancyGrid g, vector<Quad> inputTasks){
    
    CTNode Root(g, inputTasks);
    

    vector<CTNode> openList;
    openList.push_back(Root);

    CTNode bestCTNode;

    vector<vector<Triple>> result;
    
    if(Root.possible == false){
        return result;
    }

    while(!openList.empty()){
        cout << openList.size() <<endl;
        
        int cost_min = INT16_MAX;
        int index = -1;
        int min_conf = INT16_MAX;

        for(int i=0; i<openList.size(); i++){
            int curr_cost = openList.at(i).nodeCost();
            if(curr_cost < cost_min){
                cost_min = curr_cost;
            } 
        }

        for(int i=0; i<openList.size(); i++){
            int curr_cost = openList.at(i).nodeCost();
            if(curr_cost == cost_min){
                if(openList.at(i).conflicts.size() < min_conf){
                    min_conf = openList.at(i).conflicts.size();
                    index = i;
                }
            }
        }

        bestCTNode.copyNode(openList.at(index));
        openList.erase(openList.begin() + index);

        if(bestCTNode.conflicts.empty()){
            result = bestCTNode.agentPlans;
            return result;
        }
        
        Conflict firstConflict = bestCTNode.conflicts.at(0);
        vector<Quad> newConstraints = bestCTNode.constraintFromConflict(firstConflict);
        for(int i=0; i < newConstraints.size() ;i++){
            CTNode N_prime;
            N_prime.copyNode(bestCTNode);
            N_prime.constraints.push_back(newConstraints.at(i));
            int agentToReplan = get<0>(newConstraints.at(i));
            bool pathFound = N_prime.replanAgent(agentToReplan);
            
            if(!pathFound)
                continue;
            
            N_prime.conflicts = N_prime.findConflicts();
            openList.push_back(N_prime);
        }
    }
    return result;
}
