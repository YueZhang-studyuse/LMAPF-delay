#pragma once

#include "common.h"
#include "SharedEnv.h"
//#include "Instance.h"

//less delay first go
class SimulateLDF 
{
public:
    struct Agent
    {
        int id = -1;
        int delay = 0;
        int v_now = -1;
        int v_next = -1;

        Agent(int aid)
        {
            id = aid;
        }
    };

    vector<list<int>> unexecuted_path;
    vector<list<int>> simulated_path;

    //SimulateMCP(int map_size, int window_size):map_size(map_size),window_size(window_size){};
    SimulateLDF(int window_size):window_size(window_size){};

    void init(vector<Path> curr_commit);
    void clear()
    {
        unexecuted_path.clear();
        simulated_path.clear();
        for (auto a: agents)
        {
            delete a;
        }
        agents.clear();
    };
    void simulate(const vector<vector<bool>> & delays);
    //void simulate(vector<Path*>& paths, const vector<vector<bool>> & delays);

private:
    int simulate_time = 0; //we only simulate to the end of the next window
    int window_size = 1;
    vector<Agent*> agents;

    unordered_map<int,int> occupy_now;
    unordered_map<int,int> occupy_next;

    bool funcPIBT(Agent* ai);
    bool moveAgent(int a, int t, const vector<bool> & delay);
    void delayAgents(int a, int t);
    

    //bool moveAgent(vector<Path>& paths_copy, vector<Path*>& paths, list<int>::iterator& p, int t, const vector<bool> & delay);
};