#include "include/HighLevelSearch.h"
#include "include/test_cases.h"
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <geometry_msgs/Pose.h>
#include <signal.h>
using namespace std;

class testerClass {
    public:
    NavGrid planningGrid;
    ros::NodeHandle nh;
    ros::Subscriber map_sub;
    // ith index of each vector together forms a high level search task
    vector<tuple<int, int>> start_locs;
    vector<tuple<int, int>> goal_locs;
    bool test_complete_flag = false;

    testerClass(){
        ros::NodeHandle n;
        this->nh = n;
        this->map_sub = this->nh.subscribe("/sim_map", 10, &testerClass::mapReceiveCallback, this);
    }

    void mapReceiveCallback(const nav_msgs::OccupancyGrid& recvOG){
        cout << "Enterred map callback" << endl;
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

    bool test_case_index(int index){
        tuple<vector<tuple<int, int>>, vector<tuple<int, int>>> test_case = chooseTestCase(index);
        this->start_locs = get<0>(test_case);
        this->goal_locs = get<1>(test_case);
        vector<vector<TimedLoc>> results;

        vector<Task> inputTasks;
        for(int i=0; i < int(this->start_locs.size()); i++){
            int sx = get<0>(this->start_locs.at(i));
            int sy = get<1>(this->start_locs.at(i));
            int gx = get<0>(this->goal_locs.at(i));
            int gy = get<1>(this->goal_locs.at(i));
            inputTasks.push_back(make_tuple(sx, sy, gx, gy));
        }
        this->planningGrid.populateGoalHeuristics(inputTasks);
        LowLevelPlanner plannerObject(this->planningGrid);
        results = cbsSearch(inputTasks, plannerObject);
        if(results.empty()){ return false;}
        else {return true;}
    }

    void run_test_suite(){
        int passed_tests = 0;
        int failed_tests = 0;
        if(this->planningGrid.height == 0){
            return;
        }
        for(int i=0; i < TOTAL_TESTS; i++){
            cout << "------------ # " << i << " # -------------" << endl;
            ros::Time start = ros::Time::now();
            bool test_result = test_case_index(i);
            ros::Time end = ros::Time::now();
            if(test_result){
                cout << "[PASSED]" << endl;
                passed_tests += 1;
            }
            else{
                cout <<"[FAILED]" << endl;
                failed_tests += 1;
            }
            cout << "Time Taken: " << (end - start).toSec() << endl;
        }
        cout << "----------- Test Summary ------------" << endl;
        cout << "Total tests: " << TOTAL_TESTS << endl;
        cout << "Passed tests: " << passed_tests << endl;
        cout << "Failed tests: " << failed_tests << endl;
        this->test_complete_flag = true;
        return; 
    }


};

void cleanExitCallback(int signum){
    cout << "Caught exit signal, terminating planner" << endl;
    exit(signum);
}

int main(int argc, char **argv){

    ros::init(argc, argv, "planner_tester");
    ros::NodeHandle n;

    signal(SIGINT, cleanExitCallback);
    testerClass tc;
    while(!tc.test_complete_flag){
        tc.run_test_suite();
        ros::spinOnce();
        sleep(0.01);
    }

    return 0;
}