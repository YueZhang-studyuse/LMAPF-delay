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
    virtual void plan(int time_limit, std::vector<Action> & plan);

    virtual void loadPaths();
    //virtual void plan_commit(vector<Action> & actions);

    int commit = 1;
    int remain_commit = 1;

    Instance instance;
    LNS* lns = nullptr;

    vector<list<int>> commited_paths;
    vector<list<int>> future_paths;
    bool initial_run = true;
    bool initial_success = false;

    mapf_algo algo = mapf_algo::LACAMLNS2;

    int mapf_planner;
};
