#include<ros/ros.h>
#include<nav_msgs/OccupancyGrid.h>
#include<nav_msgs/MapMetaData.h>
#include<nav_msgs/Path.h>
#include<geometry_msgs/PoseStamped.h>
#include<geometry_msgs/Pose.h>
#include "mtg_messages/mtg_controller.h"
#include "mtg_task_allocation/ta_out.h"
#include<std_msgs/Bool.h>
#include "include/cbs.h"

using namespace std;

typedef tuple<float, float, int> WorldLoc;





class mapReceiveClass{
    public:
    NavGrid planningGrid;
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Subscriber goal_set_subscriber;
    ros::ServiceClient cl; 

    mapReceiveClass(){
        ros::NodeHandle n;
        this->nh = n;
        this->sub = this->nh.subscribe("/sim_map", 10, &mapReceiveClass::mapReceiveCallback, this);
        this->cl = this->nh.serviceClient<mtg_task_allocation::ta_out>("/ta_out");
        this->goal_set_subscriber = this->nh.subscribe<std_msgs::Bool>("goals_set", 10, &mapReceiveClass::goal_set_callback, this);
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
    void goal_set_callback(const std_msgs::Bool::ConstPtr& goal_set){
        std::cout << "callback called" << std::endl;
        int occupied = 0;
        int free = 0;
        vector<vector<TimedLoc>> output;
        vector<nav_msgs::Path> paths_to_send;
        vector<string> agent_names;
        if(!this->planningGrid.grid.empty()){
            output = this->findPaths();
            paths_to_send = this->gridToWorldTransform(output);
            this->logOutput(output);
            agent_names = this->createAgentNames(output);
            vector<int64_t> goal_ids(agent_names.size(), 1);
            vector<int64_t> goal_types(agent_names.size(), 1);

            ros::ServiceClient client = this->nh.serviceClient<mtg_messages::mtg_controller>("/mtg_controller/controller/");

            mtg_messages::mtg_controller call;
            call.request.stop_controller = false;
            call.request.agent_names = agent_names;
            call.request.paths = paths_to_send;
            call.request.goal_id = goal_ids;
            call.request.goal_type = goal_types;

            client.call(call);

        }
    }
    vector<vector<TimedLoc>> findPaths(){

        vector<vector<TimedLoc>> results;
        
        if(this->planningGrid.height == 0)
            return results;

        vector<Task> inputTasks;
        mtg_task_allocation::ta_out call;
        call.request.req = "please";
        this->cl.call(call);
        while(!call.response.agent_numbers.size()){cout << "Waiting for data" << endl;}

        cout << "Num of agents: " << call.response.agent_numbers.size() << endl;

        for(int i=0; i < call.response.agent_start.size(); i++){
            int sx = (int)(call.response.agent_start[i].position.x/this->planningGrid.resolution);
            int sy = this->planningGrid.height - (int)(call.response.agent_start[i].position.y/this->planningGrid.resolution);
            int gx = (int)(call.response.agent_goals[i].position.x/this->planningGrid.resolution);
            int gy = this->planningGrid.height - (int)(call.response.agent_goals[i].position.y/this->planningGrid.resolution);
            cout << sx << ", " << sy << ", " << gx << ", " << gy << endl;
            inputTasks.push_back(make_tuple(sx, sy, gx, gy));
        }
        LowLevelPlanner plannerObject(this->planningGrid);
        results = cbsSearch(inputTasks, plannerObject);
        return results;
    }
    vector<nav_msgs::Path> gridToWorldTransform(vector<vector<TimedLoc>> results){


        nav_msgs::Path dummy_path;
        vector<nav_msgs::Path> world_result(results.size(), dummy_path);
        
        for(int i=0; i < results.size(); i++){
            vector<TimedLoc> agent_grid_coords = results.at(i);
            geometry_msgs::PoseStamped dummy;
            vector<geometry_msgs::PoseStamped> agent_world_coords(agent_grid_coords.size(), dummy);

            for(int j=0; j < agent_grid_coords.size(); j++){
                float x = this->planningGrid.resolution*get<0>(agent_grid_coords.at(j));
                float y = this->planningGrid.resolution*(this->planningGrid.height - get<1>(agent_grid_coords.at(j)));
                agent_world_coords[j].pose.position.x = x;
                agent_world_coords[j].pose.position.y = y;

            }

            world_result[i].poses = agent_world_coords;
        }
        return world_result;
    }

    vector<nav_msgs::Path> gridToWorldTransformAnyAngle(vector<vector<TimedLoc>> results, float timestep){


        nav_msgs::Path dummy_path;
        vector<nav_msgs::Path> world_result(results.size(), dummy_path);
        
        for(int i=0; i < results.size(); i++){
            vector<TimedLoc> agent_grid_coords = results.at(i);
            float max_time = get<2>(agent_grid_coords.back());
            geometry_msgs::PoseStamped dummy;
            vector<geometry_msgs::PoseStamped> agent_world_coords;

            float t = 0;
            while(t <= max_time){
                tuple<float, float, float> loc_agent_i = getLoc(agent_grid_coords, t);
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
    ros::spin();

}