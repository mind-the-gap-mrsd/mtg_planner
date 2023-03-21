float agent_velocity = 0.8;
float agent_ang_vel = 0.5;
int agent_size = 5;
float grid_resolution = 0.05;
float x_eps = agent_size+1;
float t_eps = (0.5*(3.141/(agent_ang_vel) + (x_eps)/agent_velocity)+1);
float delta_t = 0.1;

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