#include <MAPFPlanner.h>
#include <random>
#include "lacam2/lacam2.hpp"


void MAPFPlanner::initialize(int preprocess_time_limit)
{
    instance.initMap(env);
    instance.computeAllPair(); //all pair heuristic 
    
    lns = new LNS(instance, preprocess_time_limit,
                "LACAM",
                "PP",
                "Adaptive",
                8,
                MAX_TIMESTEP,
                true,
                "Adaptive",
                true,
                true,0);

    commited_paths.resize(env->num_of_agents);
    future_paths.resize(env->num_of_agents);

    lns->commit = commit;

    if (mapf_planner == 1)
        algo = mapf_algo::LACAM;
    else if (mapf_planner == 2)
        algo = mapf_algo::LACAMLNS;
    else if (mapf_planner == 3)
        algo = mapf_algo::LACAMLNS2;
}

void MAPFPlanner::loadPaths()
{
    if (initial_success && !initial_run)
    {
        lns->clearAll("Adaptive");
        // //current we simply load future path without push back the unexeuted part
        // lns->loadPaths(future_paths);
        // lns->checkReplan();
    }
}

void MAPFPlanner::plan(int time_limit) 
{
    if (algo == mapf_algo::LACAM)
    {
        lns->setRuntimeLimit(time_limit);
        lns->setIterations(0);

        if (initial_run)
        {
            instance.prepareDummy();
            lns->setRuntimeLimit(1); //1s for initial run
            initial_success = lns->getInitialSolution();
            initial_run = false;
        }
        else if (!initial_success) //restart if current no initional solution
        {
            lns->clearAll("Adaptive");
            cout<<"initial not success"<<endl;
            lns->setRuntimeLimit(time_limit);
            initial_success = lns->getInitialSolution();
        }
        else 
        {
            lns->setRuntimeLimit(time_limit);
            lns->setIterations(0); //lacam only
            lns->fixInitialSolutionWithLaCAM();
        }
    }

    if (algo == mapf_algo::LACAMLNS)
    {
        lns->setRuntimeLimit(time_limit);
        lns->setIterations(MAX_TIMESTEP);

        if (initial_run)
        {
            instance.prepareDummy();
            lns->clearAll("Adaptive");
            lns->setRuntimeLimit(1); //1s for initial run
            lns->setIterations(MAX_TIMESTEP);
            initial_success = lns->run();
            initial_run = false;
        }
        else if (!initial_success) //restart if current no initional solution
        {
            lns->clearAll("Adaptive");
            cout<<"initial not success"<<endl;
            lns->setRuntimeLimit(time_limit);
            lns->setIterations(MAX_TIMESTEP);
            initial_success = lns->run();
        }
        else 
        {
            lns->setRuntimeLimit(time_limit);
            lns->fixInitialSolutionWithLaCAM();
            lns->has_initial_solution = true;
            lns->setIterations(MAX_TIMESTEP); 
            lns->run();
        }
    }

    if (algo == mapf_algo::LACAMLNS2)
    {
        lns->setRuntimeLimit(time_limit);
        lns->setIterations(MAX_TIMESTEP);

        if (initial_run)
        {
            instance.prepareDummy();
            lns->clearAll("Adaptive");
            lns->setRuntimeLimit(1); //1s for initial run
            lns->setIterations(MAX_TIMESTEP);
            initial_success = lns->run();
            initial_run = false;
        }
        else if (!initial_success) //restart if current no initional solution
        {
            lns->clearAll("Adaptive");
            cout<<"initial not success"<<endl;
            lns->setRuntimeLimit(time_limit);
            lns->setIterations(MAX_TIMESTEP);
            initial_success = lns->run();
        }
        else 
        {
            //lns->clearAll("Adaptive");
            //lns->loadPaths(future_paths);
            lns->setRuntimeLimit(time_limit);
            lns->fixInitialSolutionWithLNS2();
            lns->has_initial_solution = true;
            lns->setIterations(MAX_TIMESTEP); 
            lns->run();
        }
    }
}
void MAPFPlanner::planner_commit(vector<Path>& curr_commits)
{
    //int time_exceed = (int)(lns->runtime - time_limit);
    cout<<"time "<<lns->runtime<<endl;

    for (int i = 0; i < env->num_of_agents; i++)
    {
        future_paths[i].clear();
        commited_paths[i].clear();
    }

    lns->commitPath(commit,commited_paths,future_paths,true,env->curr_timestep);
    if (!lns->validateCommitSolution(commited_paths)) //current window has collisions --this should not happen, because we use mcp in lns2
    {
        cerr<<"errors"<<endl;
        exit(-1);
    }
    
    for (int i = 0; i < commited_paths.size(); i++)
    {
        for (auto loc: commited_paths[i])
        {
            curr_commits[i].push_back(PathEntry(loc));
        }
    }
}

// void MAPFPlanner::plan_commit(vector<Action> & actions) 
// {
//     actions = std::vector<Action>(env->curr_states.size(), Action::WA);
//     if (commited_paths[0].empty())
//     {
//         for (int i = 0; i < env->num_of_agents; i++)
//         {
//             future_paths[i].clear();
//             commited_paths[i].clear();
//         }

//         lns->commitPath(commit,commited_paths,future_paths,true,env->curr_timestep);
//         if (!lns->validateCommitSolution(commited_paths)) //current window has collisions --this should not happen, because we use mcp in lns2
//         {
//             cerr<<"errors"<<endl;
//             exit(-1);
//         }
//     }

//     //trans to actions
//     for (int agent = 0; agent < env->num_of_agents; agent++)
//     {
//         int next_loc = commited_paths[agent].front();
//         int diff = next_loc - env->curr_states[agent].location;
//         if (diff == 1)
//             actions[agent] = Action::E;
//         else if (diff == -1)
//             actions[agent] = Action::WE;
//         else if (diff > 0)
//             actions[agent] = Action::S;
//         else if (diff == 0)
//             actions[agent] = Action::WA;
//         else
//             actions[agent] = Action::N;
//         commited_paths[agent].pop_front();
//     }

//     return;

// }

