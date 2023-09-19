using namespace std;

// Count of number of test cases 
int TOTAL_TESTS = 12;

tuple<vector<tuple<int, int>>, vector<tuple<int, int>>> chooseTestCase(int index){
    vector<tuple<int, int>> start_locs;
    vector<tuple<int, int>> goal_locs; 
    switch(index){
        case 0:
            cout << "Simple two-agent swap" << endl; 
            start_locs.push_back(make_tuple(66,71));
            start_locs.push_back(make_tuple(100, 73));
            goal_locs.push_back(make_tuple(97, 72));
            goal_locs.push_back(make_tuple(69, 71));
            return make_tuple(start_locs, goal_locs);
        case 1:
            cout << "Two agent swap around obstacle" << endl;
            start_locs.push_back(make_tuple(45,67));
            start_locs.push_back(make_tuple(67, 86));
            goal_locs.push_back(make_tuple(70, 87));
            goal_locs.push_back(make_tuple(45, 64));
            return make_tuple(start_locs, goal_locs);
        case 2:
            cout << "Two agent swap through corridor" << endl;
            start_locs.push_back(make_tuple(97, 73));
            start_locs.push_back(make_tuple(106, 83));
            goal_locs.push_back(make_tuple(109, 86));
            goal_locs.push_back(make_tuple(95, 71));
            return make_tuple(start_locs, goal_locs);
        case 3:
            cout << "Two agent case diagonal across map" << endl;
            start_locs.push_back(make_tuple(48, 57));
            start_locs.push_back(make_tuple(112, 58));
            goal_locs.push_back(make_tuple(109, 85));
            goal_locs.push_back(make_tuple(50, 85));
            return make_tuple(start_locs, goal_locs);
        case 4:
            cout << "One agent obstructs the other" << endl;
            start_locs.push_back(make_tuple(75, 74));
            start_locs.push_back(make_tuple(92, 74));
            goal_locs.push_back(make_tuple(83, 74));
            goal_locs.push_back(make_tuple(69, 74));
            return make_tuple(start_locs, goal_locs);
        case 5:
            cout << "Obstructing agent part - corridor" << endl;
            start_locs.push_back(make_tuple(92, 73));
            start_locs.push_back(make_tuple(100, 86));
            goal_locs.push_back(make_tuple(110, 84));
            goal_locs.push_back(make_tuple(100, 78));
            return make_tuple(start_locs, goal_locs);
        case 6:
            cout << "Three agent standard plan" << endl;
            start_locs.push_back(make_tuple(52, 60));
            start_locs.push_back(make_tuple(112, 87));
            start_locs.push_back(make_tuple(84, 57));
            goal_locs.push_back(make_tuple(74, 72));
            goal_locs.push_back(make_tuple(52, 85));
            goal_locs.push_back(make_tuple(110, 60));
            return make_tuple(start_locs, goal_locs);
        case 7:
            cout << "Three agents leaving corridor" << endl;
            start_locs.push_back(make_tuple(110, 87));
            start_locs.push_back(make_tuple(104, 82));
            start_locs.push_back(make_tuple(111, 80));
            goal_locs.push_back(make_tuple(105, 59));
            goal_locs.push_back(make_tuple(79, 86));
            goal_locs.push_back(make_tuple(96, 74));
            return make_tuple(start_locs, goal_locs);
        case 8:
            cout << "One enterring, two exiting corridor" << endl;
            start_locs.push_back(make_tuple(110, 87));
            start_locs.push_back(make_tuple(104, 82));
            start_locs.push_back(make_tuple(95, 74));
            goal_locs.push_back(make_tuple(105, 59));
            goal_locs.push_back(make_tuple(79, 86));
            goal_locs.push_back(make_tuple(107, 83));
            return make_tuple(start_locs, goal_locs);
        case 9:
            cout << "Two swap around obstacle, one agent blocking one path" << endl;
            start_locs.push_back(make_tuple(98, 68));
            start_locs.push_back(make_tuple(81, 68));
            start_locs.push_back(make_tuple(89, 58));
            goal_locs.push_back(make_tuple(80, 68));
            goal_locs.push_back(make_tuple(99, 69));
            goal_locs.push_back(make_tuple(89, 58));
            return make_tuple(start_locs, goal_locs);
        // case 10:
        //     cout << "Third agent at goal, swap other two" << endl;
        //     start_locs.push_back(make_tuple(95, 73));
        //     start_locs.push_back(make_tuple(71, 73));
        //     start_locs.push_back(make_tuple(83, 72));
        //     goal_locs.push_back(make_tuple(67, 73));
        //     goal_locs.push_back(make_tuple(99, 73));
        //     goal_locs.push_back(make_tuple(83, 73));
        //     return make_tuple(start_locs, goal_locs);
        case 10:
            cout << "Two diagonal swap, third opp diagonal (!)" << endl;
            start_locs.push_back(make_tuple(52, 60));
            start_locs.push_back(make_tuple(112, 87));
            start_locs.push_back(make_tuple(109, 58));
            goal_locs.push_back(make_tuple(112, 84));
            goal_locs.push_back(make_tuple(49, 58));
            goal_locs.push_back(make_tuple(53, 86));
            return make_tuple(start_locs, goal_locs);
        case 11:
            cout << "Swap with third agent in corridor entrance (!)" << endl;
            start_locs.push_back(make_tuple(107, 84));
            start_locs.push_back(make_tuple(89, 75));
            start_locs.push_back(make_tuple(100, 78));
            goal_locs.push_back(make_tuple(86, 76));
            goal_locs.push_back(make_tuple(110, 86));
            goal_locs.push_back(make_tuple(100, 77));
            return make_tuple(start_locs, goal_locs);

    }
};
