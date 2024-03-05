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
    if (initial_success && !initial_run && commited_paths[0].empty())
    {
        lns->clearAll("Adaptive");
        //change future_paths to the mcped version
        future_paths.clear();
        future_paths.resize(env->num_of_agents);
        for (int agent = 0; agent < env->num_of_agents; agent++)
        {
            for (int t = 1; t < env->planned_paths[agent].size();t++)
            {
                future_paths[agent].push_back(env->planned_paths[agent][t].location);
            }
        }
        lns->loadPaths(future_paths);
        lns->checkReplan();
    }
}

void MAPFPlanner::plan(int time_limit,vector<Action> & actions) 
{
    actions = std::vector<Action>(env->curr_states.size(), Action::WA);

    if (!commited_paths[0].empty())
    {
        //trans to actions
        for (int agent = 0; agent < env->num_of_agents; agent++)
        { 
            int next_loc = commited_paths[agent].front();
            int diff = next_loc - env->curr_states[agent].location;
            if (diff == 1)
                actions[agent] = Action::E;
            else if (diff == -1)
                actions[agent] = Action::WE;
            else if (diff > 0)
                actions[agent] = Action::S;
            else if (diff == 0)
                actions[agent] = Action::WA;
            else
                actions[agent] = Action::N;
            commited_paths[agent].pop_front();
        }
        cout<<"commit directly"<<endl;
        return;
    }

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
            //lns->clearAll("Adaptive");
            lns->setRuntimeLimit(time_limit);
            lns->setIterations(0); //lacam only
            //lns->loadPaths(future_paths);
            lns->fixInitialSolutionWithLaCAM();
        }

        // for (int i = 0; i < env->num_of_agents; i++)
        // {
        //     future_paths[i].clear();
        //     commited_paths[i].clear();
        // }


        // lns->commitPath(commit,commited_paths,future_paths,true,env->curr_timestep);
        // if (!lns->validateCommitSolution(commited_paths)) //current window has collisions --this should not happen in lacam
        // {
        //     cout<<"lacam produce errors"<<endl;
        //     for (int i = 0; i < env->num_of_agents; i++)
        //     {
        //         future_paths[i].clear();
        //         commited_paths[i].clear();
        //     }
        //     return;
        // }
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
            //lns->clearAll("Adaptive");
            //lns->loadPaths(future_paths);
            lns->setRuntimeLimit(time_limit);
            lns->fixInitialSolutionWithLaCAM();
            lns->has_initial_solution = true;
            lns->setIterations(MAX_TIMESTEP); 
            lns->run();
        }

        // for (int i = 0; i < env->num_of_agents; i++)
        // {
        //     future_paths[i].clear();
        //     commited_paths[i].clear();
        // }

        // actions = std::vector<Action>(env->curr_states.size(), Action::WA);

        // lns->commitPath(commit,commited_paths,future_paths,true,env->curr_timestep);
        // if (!lns->validateCommitSolution(commited_paths)) //current window has collisions --this should not happen in lacam-lns1
        // {
        //     cerr<<"lacam lns produce errors"<<endl;
        //     exit(-1);
        // }
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

        // for (int i = 0; i < env->num_of_agents; i++)
        // {
        //     future_paths[i].clear();
        //     commited_paths[i].clear();
        // }

        // actions = std::vector<Action>(env->curr_states.size(), Action::WA);

        // lns->commitPath(commit,commited_paths,future_paths,false,env->curr_timestep);
        // if (!lns->validateCommitSolution(commited_paths)) //current window has collisions --this should not happen, because we use mcp in lns2
        // {
        //     cerr<<"errors"<<endl;
        //     exit(-1);
        // }
        // else
        // {
        //     for (int i = 0; i < env->num_of_agents; i++)
        //     {
        //         commited_paths[i].pop_front(); //skip starting locations
        //     }
        // }
    }

    int time_exceed = (int)(lns->runtime - time_limit);
    cout<<"time "<<lns->runtime<<endl;


    actions = std::vector<Action>(env->curr_states.size(), Action::WA);
    if (commited_paths[0].empty())
    {
        for (int i = 0; i < env->num_of_agents; i++)
        {
            future_paths[i].clear();
            commited_paths[i].clear();
        }

        lns->commitPath(commit,commited_paths,future_paths,true,env->curr_timestep);
        // int time_exceed = (int)(lns->runtime - time_limit);
        if (time_exceed > 0)
        {
            cout<<"time out for "<<time_exceed<<" s"<<endl;
            for (int i = 0; i < time_exceed; i++)
            {
                for (int i = 0; i < env->num_of_agents; i++)
                {
                    commited_paths[i].push_front(env->curr_states[i].location); //for timeouts
                }
            }
        }
        if (!lns->validateCommitSolution(commited_paths)) //current window has collisions --this should not happen, because we use mcp in lns2
        {
            cerr<<"errors"<<endl;
            exit(-1);
        }
        //put all current paths to shared env
        env->planned_paths.clear();
        env->planned_paths.resize(env->num_of_agents);
        for (int i = 0; i < env->num_of_agents; i++)
        {
            env->planned_paths[i].resize(commited_paths[i].size()+future_paths[i].size()+1); //current implement for commit window = 1
            env->planned_paths[i][0].location = env->curr_states[i].location;
            int cnt = 1;
            for (auto loc: commited_paths[i])
            {
                env->planned_paths[i][cnt].location = loc;
                cnt++;
            }
            for (auto loc: future_paths[i])
            {
                env->planned_paths[i][cnt].location = loc;
                cnt++;
            }
        }
    }

    // //trans to actions
    for (int agent = 0; agent < env->num_of_agents; agent++)
    { 
        int next_loc = commited_paths[agent].front();
        int diff = next_loc - env->curr_states[agent].location;
        if (diff == 1)
            actions[agent] = Action::E;
        else if (diff == -1)
            actions[agent] = Action::WE;
        else if (diff > 0)
            actions[agent] = Action::S;
        else if (diff == 0)
            actions[agent] = Action::WA;
        else
            actions[agent] = Action::N;
        commited_paths[agent].pop_front();
    }
    return;
    // }
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

