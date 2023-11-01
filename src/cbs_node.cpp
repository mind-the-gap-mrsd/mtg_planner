#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include "mtg_messages/mtg_controller.h"
#include "mtg_messages/controller_replan.h"
#include "mtg_messages/ta_out.h"
#include "mtg_messages/agent_route.h"
#include "mtg_messages/task.h"
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <signal.h>
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
    ros::Publisher plan_complete_publisher;
    map<int, ros::Publisher> rviz_path_pubs; 
    vector<queue<tuple<int, int>>> task_queues;
    vector<tuple<int, int>> start_locs;
    vector<tuple<float, float>> start_locs_float;
    vector<nav_msgs::Path> agent_current_paths;
    bool replan_flag{false};
    bool complete_flag{false};

    mapReceiveClass(){
        ros::NodeHandle n;
        this->nh = n;
        this->sub = this->nh.subscribe("/sim_map", 10, &mapReceiveClass::mapReceiveCallback, this);
        this->task_alloc_client = this->nh.serviceClient<mtg_messages::ta_out>("/hlp_task_server");
        this->goal_set_subscriber = this->nh.subscribe("/plan_set", 10, &mapReceiveClass::goalSetCallback, this);
        this->controller_client = this->nh.serviceClient<mtg_messages::mtg_controller>("/mtg_controller/controller/");
        this->replan_service = this->nh.advertiseService("/mtg_planner/controller_replan", &mapReceiveClass::updateTaskQueue, this);
        this->plan_complete_publisher = this->nh.advertise<std_msgs::Int32>("/plan_complete", 1);
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
    void goalSetCallback(const std_msgs::Int32& goal_set){
        std::cout << "callback called" << std::endl;
        this->task_queues.clear();
        this->start_locs.clear();
        createTaskQueues();
        sendPaths();
        return;        
    }

    void createTaskQueues(){
        
        if(this->planningGrid.height == 0)
            return;

        this->task_queues.clear();
        this->start_locs.clear();
        this->start_locs_float.clear();

        mtg_messages::ta_out call;
        call.request.req = "please";
        this->task_alloc_client.call(call);
        // while(!call.response.agent_routes.size()){cout << "Waiting for data" << endl;}

        cout << "Num of agents: " << call.response.agent_routes.size() << endl;

        for(int i=0; i < call.response.agent_routes.size(); i++){
            this->rviz_path_pubs[i] = this->nh.advertise<nav_msgs::Path>(std::string("/mtg_planner/agent") + to_string(i), 1000);
        }

        for(int i=0; i < call.response.agent_routes.size(); i++){
            mtg_messages::agent_route agent_route_i  = call.response.agent_routes.at(i);
            vector<mtg_messages::task> agent_task_i = agent_route_i.goal_list;
            queue<tuple<int, int>> agent_i_queue;
            for(int j=0; j < agent_task_i.size(); j++){
                int x_i_j = (int)(agent_task_i.at(j).x/this->planningGrid.resolution);
                int y_i_j = this->planningGrid.height - (int)(agent_task_i.at(j).y/this->planningGrid.resolution);
                float f_x_i_j = agent_task_i.at(j).x;
                float f_y_i_j = agent_task_i.at(j).y;
                if(j == 0){
                    this->start_locs.push_back(make_tuple(x_i_j, y_i_j));
                    this->start_locs_float.push_back(make_tuple(f_x_i_j, f_y_i_j));

                }
                else {
                    // To ensure that planning happens for susbequent goals even if agent starts at a goal location
                    if(x_i_j == get<0>(this->start_locs[i]) and y_i_j == get<1>(this->start_locs[i])){
                        if(agent_task_i.size() > 2)
                            continue;
                    }
                    agent_i_queue.push(make_tuple(x_i_j, y_i_j));
                }     
            }
            if(agent_task_i.size() == 1){
                int x_i_j = (int)(agent_task_i.at(0).x/this->planningGrid.resolution);
                int y_i_j = this->planningGrid.height - (int)(agent_task_i.at(0).y/this->planningGrid.resolution);
                agent_i_queue.push(make_tuple(x_i_j, y_i_j));
            }
            this->task_queues.push_back(agent_i_queue);
        }
        // Printing out task queue lengths for allotment issues
        for(int i = 0; i < task_queues.size(); i++){
            cout << "Task queue for agent " << i << " is: " << task_queues.at(i).size() << endl;
        }

    }

    bool updateTaskQueue(mtg_messages::controller_replan::Request& controller_req,
                         mtg_messages::controller_replan::Response& controller_resp){
        // The request should contain IDs, poses and reached or not
        vector<int> goal_status_flags = controller_req.goal_status;
        // Go through list -> if reached, then set current taskQueue[0] as start_locs and pop queue
        for(int i=0; i < goal_status_flags.size(); i++){
            int px = (int)(controller_req.agent_locations.at(i).position.x/this->planningGrid.resolution);
            int py = (int)this->planningGrid.height - (controller_req.agent_locations.at(i).position.y/this->planningGrid.resolution);
            float fx = controller_req.agent_locations.at(i).position.x;
            float fy = controller_req.agent_locations.at(i).position.y;
            this->start_locs[i] = make_tuple(px, py);
            this->start_locs_float[i] = make_tuple(fx, fy);

            cout << "Goal Status for Agent " << i << " is " << goal_status_flags.at(i) << endl;
            
            if(goal_status_flags.at(i) == 1){
                cout << "Task for agent " << i << " has been popped from the queue" << endl; 
                this->task_queues.at(i).pop();
                cout << "New size for agent " << i << " is " << task_queues.at(i).size() << endl;
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
            cout << "Agent: " << i << ", Start: " << sx << ", " << sy << " Goals: " << gx << ", " << gy << endl; 
        }
        
        this->planningGrid.populateGoalHeuristics(inputTasks);
        cout << "Calculated Goal Hueristics" << endl;
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

            if(get<0>(agent_grid_coords.at(0)) == get<0>(agent_grid_coords.back())&& get<1>(agent_grid_coords.at(0)) == get<1>(agent_grid_coords.back())) {
                if(agent_grid_coords.size() <= 2)
                    continue;
            }
                

            float t = 0;
            int start_index = 1;
            while(t <= max_time){
                TimedLoc next_anchor_point = agent_grid_coords.at(min(start_index, (int)agent_grid_coords.size()-1)); 
                tuple<float, float, float> loc_agent_i = getLoc(agent_grid_coords, t);
                // if(t > 0 && get<0>(loc_agent_i) == get<0>(getLoc(agent_grid_coords, t - timestep)) && get<1>(loc_agent_i) == get<1>(getLoc(agent_grid_coords, t - timestep))){
                //     t += timestep;
                //     continue;
                // }
                // If time is greater than next anchor point time, send anchor point instead and set t to that anchor point's timestamp
                if(t > get<2>(next_anchor_point)){
                    loc_agent_i = next_anchor_point;
                    t = get<2>(next_anchor_point);
                    start_index += 1;
                }
                float x = this->planningGrid.resolution*get<0>(loc_agent_i);
                float y = this->planningGrid.resolution*(this->planningGrid.height - get<1>(loc_agent_i));
                // cout << x << ", " << y << endl;
                agent_world_coords.push_back(dummy);
                agent_world_coords[agent_world_coords.size()-1].pose.position.x = x;
                agent_world_coords[agent_world_coords.size()-1].pose.position.y = y;
                t += timestep;
            }

            // Manually setting start of path to float location rather than grid coordinate to prevent snapping
            agent_world_coords[0].pose.position.x = get<0>(this->start_locs_float[i]);
            agent_world_coords[0].pose.position.y = get<1>(this->start_locs_float[i]);
            
            // Manually appending goal node
            agent_world_coords.push_back(dummy);
            agent_world_coords[agent_world_coords.size()-1].pose.position.x = this->planningGrid.resolution*get<0>(agent_grid_coords.back());
            agent_world_coords[agent_world_coords.size()-1].pose.position.y = this->planningGrid.resolution*(this->planningGrid.height - get<1>(agent_grid_coords.back()));

            world_result[i].poses = agent_world_coords;
            world_result[i].header.frame_id = "map";
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
            this->complete_flag = true;
            for(int i = 0; i < output.size(); i++){
                if(output.at(i).size() > 1){
                    this->complete_flag = false;
                    break;
                }
            }
            paths_to_send = this->gridToWorldTransformAnyAngle(output, 5*delta_t);
            this->agent_current_paths = paths_to_send;
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

            // for(int i= 0; i < paths_to_send.size(); i++){
            //     this->rviz_path_pubs[i].publish(paths_to_send.at(i));
            // }
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

void cleanExitCallback(int signum){
    cout << "Caught exit signal, terminating planner" << endl;
    exit(signum);
}



int main(int argc, char **argv){

    ros::init(argc, argv, "cbs_node");
    ros::NodeHandle n;

    mapReceiveClass mpC;
    signal(SIGINT, cleanExitCallback);

    while(1){
        if(mpC.replan_flag && !mpC.complete_flag){
            mpC.replan_flag = false;
            cout << "Calling replanning of paths" << endl;
            mpC.sendPaths();
        }
        if(mpC.agent_current_paths.size() > 0){
            for(int i=0; i < mpC.agent_current_paths.size(); i++){
                mpC.rviz_path_pubs[i].publish(mpC.agent_current_paths.at(i));
            }
        }
        if(mpC.complete_flag){
            //publish 1 to plan_complete topic
            mpC.complete_flag = false;
            std_msgs::Int32 msg;
            msg.data = 1;
            mpC.plan_complete_publisher.publish(msg);
        }
        ros::spinOnce();
        sleep(0.01);
    }
}