#pragma once
#include "BasicLNS.h"
#include "InitLNS.h"

//pibt related
//#include "simplegrid.h"
//#include "pibt_agent.h"
// #include "problem.h"
// #include "mapf.h"
// #include "pibt.h"
// #include "pps.h"
// #include "winpibt.h"

enum destroy_heuristic { RANDOMAGENTS, RANDOMWALK, INTERSECTION, DESTORY_COUNT };

// TODO: adaptively change the neighbor size, that is,
// increase it if no progress is made for a while
// decrease it if replanning fails to find any solutions for several times

class LNS : public BasicLNS
{
public:
    vector<Agent> agents;
    double preprocessing_time = 0;
    double initial_solution_runtime = 0;
    int initial_sum_of_costs = -1;
    int sum_of_costs_lowerbound = -1;
    int sum_of_distances = -1;
    int restart_times = 0;
    int complete_paths = 0;
    int delete_timesteps = 0;
    
    // LNS(const Instance& instance, double time_limit,
    //     string  init_algo_name, string  replan_algo_name, const string & destory_name,
    //     int neighbor_size, int num_of_iterations, bool init_lns, string  init_destory_name, bool use_sipp,
    //     bool truncate_initial_paths, int screen, PIBTPPS_option pipp_option);
    LNS(const Instance& instance, double time_limit,
        string  init_algo_name, string  replan_algo_name, const string & destory_name,
        int neighbor_size, int num_of_iterations, bool init_lns, string  init_destory_name, bool use_sipp,
        bool truncate_initial_paths, int screen);
    ~LNS()
    {
        delete init_lns;
    }
    bool fixInitialSolution();
    bool fixInitialSolutionWithLaCAM();
    bool getInitialSolution();
    bool run();
    void validateSolution() const;
    bool loadPaths(const string & file_name);
    void writeIterStatsToFile(const string & file_name) const;
    void writeResultToFile(const string & file_name) const;
    void writePathsToFile(const string & file_name) const;
    string getSolverName() const override { return "LNS(" + init_algo_name + ";" + replan_algo_name + ")"; }

    bool loadPaths(vector<list<int>> paths);
    bool loadPaths(vector<Path> paths);
    void commitPath(int step, vector<list<int>> &commit_path, vector<list<int>> &future_path,bool skip_start,int current_time);
    bool validateCommitSolution(vector<list<int>> commited_paths) const;

    void setIterations(int iterations){ num_of_iterations = iterations;}
    void setRuntimeLimit(int time){time_limit = time; replan_time_limit = time_limit / 100;}
    void clearAll(const string & destory_name);
    void setHasInitialSolution (bool has_initial){ has_initial_solution = has_initial;}
    bool fixInitialSolutionWithLNS2();

    bool target_considered = true;

    bool has_initial_solution = false;

    bool initial_collision = false;

    void setStartGoal();

    void checkReplan();

private:
    InitLNS* init_lns = nullptr;
    string init_algo_name;
    string replan_algo_name;
    bool use_init_lns; // use LNS to find initial solutions
    bool truncate_initial_paths;
    destroy_heuristic destroy_strategy = RANDOMWALK;
    int num_of_iterations;
    string init_destory_name;
    PIBTPPS_option pipp_option;


    PathTable path_table; // 1. stores the paths of all agents in a time-space table;
    // 2. avoid making copies of this variable as much as possible.
    unordered_set<int> tabu_list; // used by randomwalk strategy
    list<int> intersections;

    //for commitment
    vector<int> stay_target;

    bool runPP();
    bool runLACAM2(); 


    //MAPF preparePIBTProblem(vector<int>& shuffled_agents);
    //void updatePIBTResult(const PIBT_Agents& A, vector<int>& shuffled_agents);

    void chooseDestroyHeuristicbyALNS();

    bool generateNeighborByRandomWalk();
    bool generateNeighborByIntersection();

    int findMostDelayedAgent();
    int findRandomAgent() const;
    void randomWalk(int agent_id, int start_location, int start_timestep,
                    set<int>& neighbor, int neighbor_size, int upperbound);

    void randomWalkwithStayTarget(int agent_id, int start_location, int start_timestep,
                     set<int>& conflicting_agents, int neighbor_size, int upperbound);

    void truncatePaths();
    void deleteRepeatedStates();
};
