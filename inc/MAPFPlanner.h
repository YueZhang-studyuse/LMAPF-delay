#pragma once
#include <ctime>
#include "SharedEnv.h"
#include "ActionModel.h"
#include "Instance.h"
#include <LNS.h>

enum mapf_algo { LACAM, LACAMLNS, LACAMLNS2, ALGO_COUNT };

class MAPFPlanner
{
public:
    SharedEnvironment* env;

	MAPFPlanner(SharedEnvironment* env): env(env){};
    MAPFPlanner(){env = new SharedEnvironment();};
	virtual ~MAPFPlanner(){delete env; delete lns;};


    virtual void initialize(int preprocess_time_limit);

    // return next states for all agents
    virtual void plan(int time_limit);

    virtual void loadPaths();
    virtual void planner_commit(vector<Path>& curr_commits); //return additional wait when runtime > timelimit

    int commit = 1;

    Instance instance;
    LNS* lns = nullptr;

    vector<list<int>> commited_paths;
    vector<list<int>> future_paths;
    bool initial_run = true;
    bool initial_success = false;

    mapf_algo algo = mapf_algo::LACAMLNS2;

    int mapf_planner;

    //keep track of planning success/failure
    list<bool> plan_success;

    double decay_factor = 0;
    double increase_factor = 0;
    double current_window_factor = 1;
    int check_window = 20; //how many we check for decide the collision-free window
};
