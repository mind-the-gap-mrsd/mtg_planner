
#include "constants.h"
using namespace std;

const int INF = 1e9;

class NavGrid{
    public:
    int height, width;
    float resolution;
    vector<vector<int>> grid;
    map<tuple<int, int>, vector<vector<int>>> goal_dijkstra_map;
    NavGrid(string filename){
        ifstream obs_file;
        obs_file.open(filename);
        string line;
        int i  = 0;
        if(obs_file.is_open()){
            while(obs_file){
                getline(obs_file, line);
                if(i == 0){
                    this->height = stoi(line);
                }
                else if(i == 1){
                    this->width = stoi(line);
                    vector<vector<int>> temp(this->height, vector<int>(this->width, 0));
                    this->grid = temp;
                }
                else{
                    int split = line.find(",");
                    int x = stoi(line.substr(0,split));
                    int y = stoi(line.substr(split+1));
                    this->grid[y][x] = 1;
                }
                i++;
            }
        }   

    }

    NavGrid(int h, int w, float res, vector<int> data){
        this->height = h;
        this->width = w;
        this->resolution = res;
        vector<vector<int>> temp(h, vector<int>(w, 0));
        this->grid = temp;
        for(int i=0; i < h; i++){
            for(int j=0; j < w; j++){
                if(data[i*w + j] == 100)
                    this->grid[h-1-i][j] = 1;
                else
                    this->grid[h-1-i][j] = 0;
            }
        }
    }
    NavGrid(){
        this->height = 0;
        this->width = 0;
        vector<vector<int>> temp;
        this->grid = temp;
    }

    void printGrid(){
        if(!this->grid.empty()){
            for(int i=0; i < this->height; i++){
                for(int j=0; j < this->width; j++){
                    cout << this->grid[i][j];
                }
                cout << endl;
            }
        }
    }

    bool hitsObstacle(int pos_x, int pos_y){

        for(int i=-2*(agent_size); i < 2*(agent_size); i++){
            for(int j=-2*(agent_size); j < 2*(agent_size); j++){
                if(pos_x+i >= 0 || pos_y+j >= 0 || pos_x+i < width || pos_y+j < height){
                    if(pow(i,2) + pow(j,2) <= pow(inflation_radius,2)){
                        if(this->grid[pos_y + j][pos_x + i] == 1){
                            return true;
                        }
                    }
                }
            }
        }
        return false;
    }

    // Written by ChatGPT - Dijkstra C++ Algorithm given a goal point to return 2D vector of ints

    vector<vector<int>> dijkstra(tuple<int, int> goal_loc){
        int n = this->height;
        int m = this->width;

        int gx = get<0>(goal_loc); 
        int gy = get<1>(goal_loc);

        vector<vector<int>> d(n, vector<int>(m, INF));
        d[gy][gx] = 0;

        priority_queue<pair<int, pair<int, int>>, vector<pair<int, pair<int, int>>>, greater<pair<int, pair<int, int>>>> q;
        q.push(make_pair(0, make_pair(gx, gy)));

        int move[8][2] = {{-1,-1}, {0,-1}, {1, -1}, {-1,0}, {1,0}, {-1,1}, {0, 1}, {1, 1}};

        while (!q.empty()) {
            auto p = q.top();
            q.pop();

            int x = p.second.first;
            int y = p.second.second;
            int dist = p.first;

            if (dist > d[y][x]) continue;

            for (int i = 0; i < 8; i++) {
                int nx = x + move[i][0];
                int ny = y + move[i][1];

                if (nx < 0 || ny < 0 || nx >= m || ny >= n) continue;

                if (hitsObstacle(nx, ny)) continue;

                int delta_dist = ((nx-x)*(ny-y) == 0) ? 10 : 14;

                int ndist = dist + delta_dist;
                if (ndist < d[ny][nx]) {
                    d[ny][nx] = ndist;
                    q.push(make_pair(ndist, make_pair(nx, ny)));
                }
            }
        }

        return d;
    }

    void populateGoalHeuristics(vector<Task> inputTasks){
        for(int i = 0; i < inputTasks.size(); i++){
            tuple<int, int> ith_goal = make_tuple(get<2>(inputTasks.at(i)), get<3>(inputTasks.at(i)));
            map<tuple<int, int>, vector<vector<int>>>::iterator iter = this->goal_dijkstra_map.find(ith_goal);
            if(iter == goal_dijkstra_map.end()){
                this->goal_dijkstra_map[ith_goal] = this->dijkstra(ith_goal);
                cout << "Added " << get<0>(ith_goal) << ", " << get<1>(ith_goal) << " to dijkstra map" << endl;
                cout << "Map size is now " << this->goal_dijkstra_map.size() << endl;
            }
        }
    }
};