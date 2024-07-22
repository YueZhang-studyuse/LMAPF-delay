#pragma once

#include "common.h"
#include "SharedEnv.h"
#include "Instance.h"


//casual pibt
class SimulatePIBTD
{
public:
    struct Agent
    {
        int id = -1;
        int delay = 0;
        int v_now = -1;
        int v_next = -1;
        int internal_time = 0;
        double priority = 0;

        Agent(int aid)
        {
            id = aid;
        }
    };

    //vector<list<int>> unexecuted_path;
    vector<vector<int>> time_dependent_path;
    vector<vector<int>> simulated_path;

    const Instance& instance;

    //SimulateMCP(int map_size, int window_size):map_size(map_size),window_size(window_size){};
    SimulatePIBTD(int window_size, const Instance& instance):window_size(window_size),instance(instance){};

    void init(vector<Path> curr_commit);
    void clear()
    {
        time_dependent_path.clear();
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

    bool funcPIBT(Agent* ai, bool first);
    bool moveAgent(int a, int t, const vector<bool> & delay);
    void delayAgents(int a, int t);
    

    //bool moveAgent(vector<Path>& paths_copy, vector<Path*>& paths, list<int>::iterator& p, int t, const vector<bool> & delay);
};