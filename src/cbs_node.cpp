#include<ros/ros.h>
#include<nav_msgs/OccupancyGrid.h>
#include<nav_msgs/MapMetaData.h>
#include<nav_msgs/Path.h>
#include<geometry_msgs/PoseStamped.h>
#include<geometry_msgs/Pose.h>
#include "mtg_messages/mtg_controller.h"
#include "mtg_messages/controller_replan.h"
#include "mtg_messages/ta_out.h"
#include "mtg_messages/agent_route.h"
#include "mtg_messages/task.h"
#include<std_msgs/Bool.h>
#include "include/HighLevelSearch.h"

using namespace std;

typedef tuple<float, float, int> WorldLoc;





class mapReceiveClass{
    public:
    NavGrid planningGrid;
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Subscriber goal_set_subscriber;
    ros::ServiceClient task_alloc_client;
    ros::ServiceClient controller_client;
    ros::ServiceServer replan_service; 
    vector<queue<tuple<int, int>>> task_queues;
    vector<tuple<int, int>> start_locs;
    bool replan_flag{false};

    mapReceiveClass(){
        ros::NodeHandle n;
        this->nh = n;
        this->sub = this->nh.subscribe("/sim_map", 10, &mapReceiveClass::mapReceiveCallback, this);
        this->task_alloc_client = this->nh.serviceClient<mtg_messages::ta_out>("/ta_out");
        this->goal_set_subscriber = this->nh.subscribe<std_msgs::Bool>("goals_set", 10, &mapReceiveClass::goalSetCallback, this);
        this->controller_client = this->nh.serviceClient<mtg_messages::mtg_controller>("/mtg_controller/controller/");
        this->replan_service = this->nh.advertiseService("/mtg_planner/controller_replan", &mapReceiveClass::updateTaskQueue, this);
    }

    void mapReceiveCallback(const nav_msgs::OccupancyGrid& recvOG){
    
        nav_msgs::MapMetaData meta = recvOG.info;
        int mapWidth = meta.width;
        int mapHeight = meta.height;
        float mapResolution = meta.resolution;
        geometry_msgs::Pose p = meta.origin;

        vector<int> occupData;
        for(int i=0; i < recvOG.data.size(); i++){
            occupData.push_back((int)(unsigned char)recvOG.data[i]);
        }
        NavGrid grid(mapHeight, mapWidth, mapResolution, occupData);
        this->planningGrid = grid;

    }
    void goalSetCallback(const std_msgs::Bool::ConstPtr& goal_set){
        std::cout << "callback called" << std::endl;
        this->task_queues.clear();
        this->start_locs.clear();
        createTaskQueues();
        sendPaths();        
    }

    void createTaskQueues(){
        
        if(this->planningGrid.height == 0)
            return;

        mtg_messages::ta_out call;
        call.request.req = "please";
        this->task_alloc_client.call(call);
        while(!call.response.agent_routes.size()){cout << "Waiting for data" << endl;}

        cout << "Num of agents: " << call.response.agent_routes.size() << endl;

        for(int i=0; i < call.response.agent_routes.size(); i++){
            mtg_messages::agent_route agent_route_i  = call.response.agent_routes.at(i);
            vector<mtg_messages::task> agent_task_i = agent_route_i.goal_list;
            queue<tuple<int, int>> agent_i_queue;
            for(int j=0; j < agent_task_i.size(); j++){
                int x_i_j = (int)(agent_task_i.at(j).x/this->planningGrid.resolution);
                int y_i_j = this->planningGrid.height - (int)(agent_task_i.at(j).y/this->planningGrid.resolution);
                if(j == 0){
                    this->start_locs.push_back(make_tuple(x_i_j, y_i_j));

                }
                else {
                    agent_i_queue.push(make_tuple(x_i_j, y_i_j));
                }     
            }
            if(agent_task_i.size()==1){
                int x_i_j = (int)(agent_task_i.at(0).x/this->planningGrid.resolution);
                int y_i_j = this->planningGrid.height - (int)(agent_task_i.at(0).y/this->planningGrid.resolution);
                agent_i_queue.push(make_tuple(x_i_j, y_i_j));
            }
            this->task_queues.push_back(agent_i_queue);
        }

    }

    bool updateTaskQueue(mtg_messages::controller_replan::Request& controller_req,
                         mtg_messages::controller_replan::Response& controller_resp){
        // The request should contain IDs, poses and reached or not
        vector<int> goal_status_flags = controller_req.goal_status;
        // Go through list -> if reached, then set current taskQueue[0] as start_locs and pop queue
        for(int i=0; i < goal_status_flags.size(); i++){
            int px = (int)(controller_req.agent_locations.at(i).position.x/this->planningGrid.resolution);
            int py = this->planningGrid.height - (int)(controller_req.agent_locations.at(i).position.y/this->planningGrid.resolution);
            this->start_locs[i] = make_tuple(px, py);
            if(goal_status_flags.at(i) == 1){
                this->task_queues.at(i).pop();
                if(this->task_queues.at(i).empty()){
                    this->task_queues.at(i).push(make_tuple(px, py));
                }
            }
        }
        controller_resp.done_mf = true;
        cout << "Queues updated" << endl;
        this->replan_flag = true;
        return true;

    }
    
    vector<vector<TimedLoc>> findPaths(){

        vector<vector<TimedLoc>> results;
        
        if(this->planningGrid.height == 0)
            return results;

        vector<Task> inputTasks;
        for(int i=0; i < this->start_locs.size(); i++){
            int sx = get<0>(this->start_locs.at(i));
            int sy = get<1>(this->start_locs.at(i));
            int gx = get<0>(this->task_queues.at(i).front());
            int gy = get<1>(this->task_queues.at(i).front());
            inputTasks.push_back(make_tuple(sx, sy, gx, gy));
            cout << "Agent: " << i << "Start: " << sx << ", " << sy << " Goals: " << gx << ", " << gy << endl; 
        }

        LowLevelPlanner plannerObject(this->planningGrid);
        results = cbsSearch(inputTasks, plannerObject);
        return results;
    }

    vector<nav_msgs::Path> gridToWorldTransformAnyAngle(vector<vector<TimedLoc>> results, float timestep){


        nav_msgs::Path dummy_path;
        vector<nav_msgs::Path> world_result(results.size(), dummy_path);
        
        for(int i=0; i < results.size(); i++){
            vector<TimedLoc> agent_grid_coords = results.at(i);
            float max_time = get<2>(agent_grid_coords.back());
            geometry_msgs::PoseStamped dummy;
            vector<geometry_msgs::PoseStamped> agent_world_coords;

            if(get<0>(agent_grid_coords.at(0)) == get<0>(agent_grid_coords.back()) && get<1>(agent_grid_coords.at(0)) == get<1>(agent_grid_coords.back()))
                continue;


            float t = 0;
            while(t <= max_time + timestep){
                tuple<float, float, float> loc_agent_i = getLoc(agent_grid_coords, t);
                if(get<0>(loc_agent_i) == get<0>(getLoc(agent_grid_coords, t - timestep)) && get<1>(loc_agent_i) == get<1>(getLoc(agent_grid_coords, t - timestep))){
                    t += timestep;
                    continue;
                }
                float x = this->planningGrid.resolution*get<0>(loc_agent_i);
                float y = this->planningGrid.resolution*(this->planningGrid.height - get<1>(loc_agent_i));
                agent_world_coords.push_back(dummy);
                agent_world_coords[agent_world_coords.size()-1].pose.position.x = x;
                agent_world_coords[agent_world_coords.size()-1].pose.position.y = y;
                t += timestep;
            }

            world_result[i].poses = agent_world_coords;
        }
        return world_result;
    }

    vector<string> createAgentNames(vector<vector<TimedLoc>> results){
        vector<string> agent_name_list;
        int agent_total = results.size();
        for(int i =0; i < agent_total; i++){
            agent_name_list.push_back("agent_" + to_string(i));
        }
        return agent_name_list;
    }

    void sendPaths(){

        vector<vector<TimedLoc>> output;
        vector<nav_msgs::Path> paths_to_send;
        vector<string> agent_names;

        if(!this->planningGrid.grid.empty()){
            // Finding paths and logging output
            output = this->findPaths();
            this->logOutput(output);
            paths_to_send = this->gridToWorldTransformAnyAngle(output, 0.1);
            agent_names = this->createAgentNames(output);
            vector<int64_t> goal_ids(agent_names.size(), 1);
            vector<int64_t> goal_types(agent_names.size(), 1);
            
            // Calling controller with path data
            mtg_messages::mtg_controller call;
            call.request.stop_controller = false;
            call.request.agent_names = agent_names;
            call.request.paths = paths_to_send;
            call.request.goal_id = goal_ids;
            call.request.goal_type = goal_types;
            this->controller_client.call(call);
        }
        return;
    }


    void logOutput(vector<vector<TimedLoc>> results){
        if(results.empty()) {
            cout << "No Solution" << endl;
            return;
        }
        for(int i=0; i < results.size(); i++){
            vector<TimedLoc> result = results.at(i);
            cout << "-------------------------------------------------- Agent #" << i << "--------------------------------------" << endl;
            for(int j=0; j < result.size(); j++){
                cout <<"(" << get<0>(result.at(j)) << ", " << get<1>(result.at(j)) << ", " << get<2>(result.at(j)) << ")" << endl;
            }    
        }
    }
};



int main(int argc, char **argv){

    ros::init(argc, argv, "cbs_node");
    ros::NodeHandle n;

    mapReceiveClass mpC;

    while(1){
        if(mpC.replan_flag){
            mpC.replan_flag = false;
            cout << "Calling replanning of paths" << endl;
            mpC.sendPaths();
        }
        ros::spinOnce();
    }

    // ros::spin();

}