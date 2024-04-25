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
        //current we simply load future path without push back the unexeuted part
        lns->loadPaths(future_paths);
        lns->checkReplan();
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
            lns->collision_clear_window = MAX_TIMESTEP;
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
            cout<<"current planner widnow "<<lns->collision_clear_window<<endl;
            //set one number for exp
            //lns->collision_clear_window = MAX_TIMESTEP;
            lns->setRuntimeLimit(time_limit);
            plan_success.push_back(lns->fixInitialSolutionWithLNS2());
            
            lns->has_initial_solution = true;
            lns->setIterations(MAX_TIMESTEP); 
            lns->run();

            if (plan_success.back())
                cum_success++;
            else
                cum_fail++;
            //choose whether to decrease the window
            int ran_result = rand()%(cum_fail+cum_success)+1;
            cout<<"random result "<<ran_result<<" cum succss "<<cum_success<<" cum fail "<<cum_fail<<" iterations "<<lns->iteration_stats.size()+lns->num_of_failures<<endl;
            if (ran_result > cum_success) //decrease 
            {
                cout<<"decrease "<<endl;
                current_window_factor = decay_factor;
                cum_success = 0;
                cum_fail = 0;
            }
            else if (current_window_factor < 1)
            {
                //check if increase
                if ((lns->iteration_stats.size()+lns->num_of_failures)*lns->get_neighbor_size() >= env->num_of_agents) //simply increase if we can run enough iterations
                {
                    cout<<"increase "<<endl;
                    current_window_factor = 1+increase_factor;
                    cum_fail=0;
                    cum_success=0;
                }
            }
            // if (current_window_factor >= 1)
            // {
            //     current_window_factor = 1;
            //     lns->collision_clear_window = MAX_TIMESTEP;
            // }
            // else
            // {
                if (lns->collision_clear_window == MAX_TIMESTEP)
                {
                    lns->collision_clear_window = (int) (((double)lns->sum_of_costs/env->num_of_agents)*current_window_factor);
                }
                else
                {
                    lns->collision_clear_window = (int) (lns->collision_clear_window*current_window_factor);
                }
            //}
            if (lns->collision_clear_window < 10)
                lns->collision_clear_window = 10;
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



