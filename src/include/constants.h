#include<vector>
#include<string>
#include<iostream>
#include<fstream>
#include<queue>
#include<algorithm>
#include<map>
#include <cmath>
#include <tuple>
#include <thread>
#include <mutex>
using namespace std;



float agent_velocity = 0.5;
//Angular velocity is linear velocity by the distance between wheels (10 cm)
float agent_ang_vel = 0.5*agent_velocity/0.05;
// Agent size in grid cell units
int agent_size = 3;
// resolution in m/ grid cell
float grid_resolution = 0.05;
// Threshold to detect collisions between paths in grid cell units
float x_eps = agent_size + 0.5;
//Squared of blocking radius in agent_size units
float blocking_radius_sq = 1.3;
// Max time duration to block out for a suspected collision
float t_eps = ((sqrt(blocking_radius_sq)*agent_size*grid_resolution)/agent_velocity);
// Inflation radius for collision check
float inflation_radius = 3.3;
// Discretization time step for planning
float delta_t = 0.2*(grid_resolution/agent_velocity);
// float delta_t = 0.1;
// Time out constant in seconds
double TIME_OUT = 60; 
// Map Dilate Flag
char map_dilate_flag = 'u';

typedef tuple<int, int, float> TimedLoc;
typedef tuple<int, tuple<float, float>, float> Constraint;
typedef tuple<int ,int, int, int> Task;
typedef tuple<int, int, tuple<float, float>, float> Collision;

void logOutput(vector<vector<TimedLoc>> results){
    for(int i=0; i < results.size(); i++){
        cout << "--------------------------- Agent #" << i << "---------------------------------" << endl;
        vector<TimedLoc> result = results.at(i);
        for(int j=0; j < result.size(); j++){
            cout << "(" << get<0>(result.at(j)) << ", " << get<1>(result.at(j)) << ", " << get<2>(result.at(j)) << ")" << endl;
        }
    }
}

tuple<float, float, float> getLoc(vector<TimedLoc> path, float time){
    
    if(time < 0)
        return make_tuple(get<0>(path.at(0))*1.0,get<1>(path.at(0))*1.0, get<2>(path.at(0)));
    if(time >= get<2>(path.back()))
        return make_tuple(get<0>(path.back())*1.0, get<1>(path.back())*1.0, time);

    float x_point = 0, y_point = 0;
    int index = -1;
    for(int i=0; i < path.size(); i++){
        if(time < get<2>(path.at(i))){
            index = i-1;
            break;
        }
    }

    int x1 = get<0>(path.at(index)), y1 = get<1>(path.at(index));
    int x2 = get<0>(path.at(index+1)), y2 = get<1>(path.at(index+1));

    if(x1 == x2 and y1 == y2)
        return make_tuple(x1*1.0, y1*1.0, time);
    
    float angle = atan2(y2-y1, x2-x1);
    float delt = time - get<2>(path.at(index));

    x_point = x1 + (agent_velocity/grid_resolution)*cos(angle)*delt;
    y_point = y1 + (agent_velocity/grid_resolution)*sin(angle)*delt;

    return make_tuple(x_point, y_point, time);
}