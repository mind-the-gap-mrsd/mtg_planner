#include<vector>
#include<string>
#include<iostream>
#include<fstream>
using namespace std;

class NavGrid{
    public:
    int height, width;
    float resolution;
    vector<vector<int>> grid;
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
};