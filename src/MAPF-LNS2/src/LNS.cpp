#include "LNS.h"
#include <queue>
#include<boost/tokenizer.hpp>
#include <utility>
#include "lacam2/lacam2.hpp"

LNS::LNS(const Instance& instance, double time_limit, string init_algo_name, string replan_algo_name,
         const string & destory_name, int neighbor_size, int num_of_iterations, bool use_init_lns,
         string init_destory_name, bool use_sipp, bool truncate_initial_paths,
         int screen) :
         BasicLNS(instance, time_limit, neighbor_size, screen),
         init_algo_name(std::move(init_algo_name)),  replan_algo_name(std::move(replan_algo_name)),
         num_of_iterations(num_of_iterations),
         use_init_lns(use_init_lns), init_destory_name(std::move(init_destory_name)),
         truncate_initial_paths(truncate_initial_paths),
         path_table(instance.env->map.size())
{
    start_time = Time::now();
    //replan_time_limit = time_limit / 100;
    if (destory_name == "Adaptive")
    {
        ALNS = true;
        destroy_weights.assign(DESTORY_COUNT, 1);
        decay_factor = 0.01;
        reaction_factor = 0.01;
    }
    else if (destory_name == "RandomWalk")
        destroy_strategy = RANDOMWALK;
    else if (destory_name == "Intersection")
        destroy_strategy = INTERSECTION;
    else if (destory_name == "Random")
        destroy_strategy = RANDOMAGENTS;
    else
    {
        cerr << "Destroy heuristic " << destory_name << " does not exists. " << endl;
        exit(-1);
    }

    int N = instance.getDefaultNumberOfAgents();
    agents.reserve(N);
    stay_target.resize(N);

    for (int i = 0; i < N; i++)
    {
        agents.emplace_back(instance, i, use_sipp);
        stay_target[i] = 0;
    }

    preprocessing_time = ((fsec)(Time::now() - start_time)).count();
    if (screen >= 2)
        cout << "Pre-processing time = " << preprocessing_time << " seconds." << endl;
    
}


bool LNS::run()
{
    // only for statistic analysis, and thus is not included in runtime
    sum_of_distances = 0;
    for (const auto & agent : agents)
    {
        sum_of_distances += instance.getAllpairDistance(instance.env->curr_states[agent.id].location, instance.env->goal_locations[agent.id][0].first);
        //agent.path_planner->my_heuristic[agent.path_planner->start_location];
    }

    initial_solution_runtime = 0;
    bool succ = true;
    // if (has_initial_solution)
    //     succ = fixInitialSolution();
    // else
    if (!has_initial_solution)
    {
        start_time = Time::now();
        succ = getInitialSolution(); 
        if (!succ) //we do not have a initial solution when reaching the runtime limit, commit what it have anyway
            return false;
    }
    initial_solution_runtime = ((fsec)(Time::now() - start_time)).count();

    iteration_stats.emplace_back(neighbor.agents.size(),
                                 initial_sum_of_costs, initial_solution_runtime, init_algo_name);
    runtime = initial_solution_runtime;
    if (succ)
    {
        if (screen >= 1)
            cout << "Initial solution cost = " << initial_sum_of_costs << ", "
                 << "runtime = " << initial_solution_runtime << endl;
    }
    else
    {
        cout << "Failed to find an initial solution in "
             << runtime << " seconds after  " << restart_times << " restarts" << endl;
        return false; // terminate because no initial solution is found
    }

    while (runtime < time_limit && iteration_stats.size() <= num_of_iterations && !timeout_flag)
    {
        runtime =((fsec)(Time::now() - start_time)).count();

        ALNS = false;

        if (ALNS)
            chooseDestroyHeuristicbyALNS();

        switch (destroy_strategy)
        {
            case RANDOMWALK:
                succ = generateNeighborByRandomWalk();
                break;
            case INTERSECTION:
                succ = generateNeighborByIntersection();
                break;
            case RANDOMAGENTS:
                neighbor.agents.resize(agents.size());
                for (int i = 0; i < (int)agents.size(); i++)
                    neighbor.agents[i] = i;
                if (neighbor.agents.size() > neighbor_size)
                {
                    std::random_shuffle(neighbor.agents.begin(), neighbor.agents.end());
                    neighbor.agents.resize(neighbor_size);
                }
                succ = true;
                break;
            default:
                cerr << "Wrong neighbor generation strategy" << endl;
                exit(-1);
        }

        if(!succ)
            continue;

        // store the neighbor information
        neighbor.old_paths.resize(neighbor.agents.size());
        neighbor.old_sum_of_costs = 0;
        for (int i = 0; i < (int)neighbor.agents.size(); i++)
        {
            if (replan_algo_name == "PP")
                neighbor.old_paths[i] = agents[neighbor.agents[i]].path;
            path_table.deletePath(neighbor.agents[i], agents[neighbor.agents[i]].path);
            neighbor.old_sum_of_costs += agents[neighbor.agents[i]].path.size() - 1;
        }
        
        if (replan_algo_name == "PP")
            succ = runPP();
        else
        {
            cerr << "Wrong replanning strategy" << endl;
            exit(-1);
        }

        if (ALNS) // update destroy heuristics
        {
            if (neighbor.old_sum_of_costs > neighbor.sum_of_costs )
            {
                destroy_weights[selected_neighbor] =
                        reaction_factor * (neighbor.old_sum_of_costs - neighbor.sum_of_costs) / neighbor.agents.size()
                        + (1 - reaction_factor) * destroy_weights[selected_neighbor];
            }
            else
            {
                destroy_weights[selected_neighbor] =
                        (1 - decay_factor) * destroy_weights[selected_neighbor];
            }
        }
        //cout<<"neighbor test "<<neighbor.sum_of_costs <<" "<<neighbor.old_sum_of_costs<<" "<<sum_of_costs<<endl;
        runtime = ((fsec)(Time::now() - start_time)).count();
        sum_of_costs += neighbor.sum_of_costs - neighbor.old_sum_of_costs;
        if (screen >= 1)
            cout << "Iteration " << iteration_stats.size() << ", "
                 << "group size = " << neighbor.agents.size() << ", "
                 << "solution cost = " << sum_of_costs << ", "
                 << "remaining time = " << time_limit - runtime << endl;
        iteration_stats.emplace_back(neighbor.agents.size(), sum_of_costs, runtime, replan_algo_name);
    }


    average_group_size = - iteration_stats.front().num_of_agents;
    for (const auto& data : iteration_stats)
        average_group_size += data.num_of_agents;
    if (average_group_size > 0)
        average_group_size /= (double)(iteration_stats.size() - 1);

    cout << getSolverName() << ": "
         << "runtime = " << runtime << ", "
         << "deleted timesteps = " << delete_timesteps << ","
         << "iterations = " << iteration_stats.size() << ", "
         << "solution cost = " << sum_of_costs << ", "
         << "initial solution cost = " << initial_sum_of_costs << ", "
         << "failed iterations = " << num_of_failures << endl;
        
    cout<<"iterations details"<<endl;
    int time = 0;
    for (auto i: iteration_stats)
    {
        if (i.runtime >= time)
        {
            cout<<i.runtime<<" ";
            time++;
        }
    }
    cout<<endl;
    time = 0;
    for (auto i: iteration_stats)
    {
        if (i.runtime >= time)
        {
            cout<<i.sum_of_costs<<" ";
            time++;
        }
    }
    cout<<endl;
    return true;
}

void LNS::truncatePaths() // truncate paths to maximize the number of agents at goal locations
{
    int best_t = 0;
    int best_finished = 0;
    int makespan = 0;
    for (int t = 0;; t++)
    {
        int finished = 0;
        bool all_finished = true;
        for (const auto& agent : agents)
        {
            if (!agent.path.empty() and
                agent.path[min(t, (int)agent.path.size() - 1)].location == agent.path_planner->goal_location)
                finished++;
            if ((int)agent.path.size() > t + 1) all_finished = false;
        }
        if (finished > best_finished)
        {
            best_t = t;
            best_finished = finished;
        }
        if (all_finished)
        {
            makespan = t;
            if (screen == 2)
                cout << finished << " finished agents at the LAST timestep of " << t << endl;
            break;
        }
    }
    if (screen == 2)
        cout << best_finished << " finished agents at the BEST timestep of " << best_t << endl;
    if (best_t < makespan)
    {
        delete_timesteps += makespan - best_t;
        for (auto& agent : agents)
        {
            if ((int)agent.path.size() > best_t + 1) agent.path.resize(best_t + 1);
        }
    }
}
void LNS::deleteRepeatedStates() // if there are two timesteps when locations of all agents are the same, delete the subpaths in between
{
    unordered_map<vector<int>, int, container_hash<vector<int>>> states;
    for (int t = 0;; t++)
    {
        bool all_finished = true;
        vector<int> state(agents.size(), -1);
        for (const auto& agent : agents)
        {
            if (!agent.path.empty())
                state[agent.id] =  agent.path[min(t, (int)agent.path.size() - 1)].location;
            if ((int)agent.path.size() > t + 1) all_finished = false;
        }
        if (states.find(state) != states.end())
        {
            auto t1 = states[state];
            if (screen > 1)
                cout << "Delete paths between timesteps " << t1 << " and " << t << endl;
            delete_timesteps += t - t1;
            for (auto& agent : agents)
            {
                if ((int)agent.path.size() <= t1 + 1) continue;
                else if (agent.path.size() <= t) agent.path.resize(t1 + 1);
                else
                {
                    for (int t2 = 1; t2 < agent.path.size() - t; t2++)
                        agent.path[t1+t2] = agent.path[t+t2];
                    agent.path.resize(agent.path.size() - t + t1);
                }
                for (auto i = states.begin(), last = states.end(); i != last; )
                {
                    if (i->second > t1) i = states.erase(i);
                    else ++i;
                }
            }
            t = t1;
        }
        else
            states[state] = t;
        if (all_finished)
        {
            if (screen > 1)
            {
                cout << "Makespan is " << t << endl;
                for (const auto& a1_ : agents)
                {
                    for (int t = 1; t < (int) a1_.path.size(); t++ )
                    {
                        //cout<<a1_.path[t - 1].location<<" "<<endl;
                        if (!instance.validMove(a1_.path[t - 1].location, a1_.path[t].location))
                        {
                            cerr << "The path of agent " << a1_.id << " jump from "
                                 << a1_.path[t - 1].location << " to " << a1_.path[t].location
                                 << " between timesteps " << t - 1 << " and " << t << endl;
                            exit(-1);
                        }
                        //cout<<endl;
                    }
                }
            }
            break;
        }
    }
}
bool LNS::fixInitialSolution()
{
    neighbor.agents.clear();
    initial_sum_of_costs = 0;
    list<int> complete_agents; // subsets of agents who have complete and collision-free paths
    // if (truncate_initial_paths)
    // {
    //     //truncatePaths();
    //     //deleteRepeatedStates();
    // }
    int makespan = 0;
    for (auto& agent : agents)
    {
        if (agent.path.empty() or agent.path.back().location != agent.path_planner->goal_location)
        {
            neighbor.agents.emplace_back(agent.id);
            agent.path.clear();
        }
        else
        {
            bool has_collision = false;
            for (auto other : complete_agents)
            {
                has_collision = instance.hasCollision(agent.path, agents[other].path);
                if (has_collision)
                {
                    neighbor.agents.emplace_back(agent.id);
                    break;
                }
            }
            if (!has_collision)
            {
                path_table.insertPath(agent.id, agent.path);
                complete_paths++;
                initial_sum_of_costs += (int)agent.path.size() - 1;
                complete_agents.emplace_back(agent.id);
                makespan = max(makespan, (int)agent.path.size() - 1);
            }
            instance.existing_path[agent.id].resize(agent.path.size());
            for (int i = 0; i < (int)agent.path.size(); i++)
            {
                instance.existing_path[agent.id][i] = agent.path[i].location;
            }
        }
    }
    if (screen == 2)
        cout << complete_agents.size() << " collision-free agents at timestep " << makespan << endl;
    neighbor.old_sum_of_costs = MAX_COST;
    neighbor.sum_of_costs = 0;
    auto succ = runPP();
    //auto succ = runLACAM2();
    if (succ)
    {
        initial_sum_of_costs += neighbor.sum_of_costs;
        sum_of_costs = initial_sum_of_costs;
        return true;
    }
    else
    {
        cout<<"pp failed"<<endl;
        return false;
    }
}

void LNS::checkReplan()
{
    neighbor.agents.clear();
    initial_sum_of_costs = 0;
    list<int> complete_agents; // subsets of agents who have complete and collision-free paths
    int makespan = 0;

    initial_collision = false;

    for (auto& agent : agents) //also include delaied agent
    {
        if (agent.path.empty() || agent.path.front().location != agent.path_planner->start_location)
        {
            if (!agent.path.empty())
            {
                cout<<"replan for agent "<<agent.id<<" due to delaies "<<agent.path.front().location<<" "<<agent.path_planner->start_location<<endl;
            }
            neighbor.agents.emplace_back(agent.id);
            agent.path.clear();
        }
        else //check if plan is reached goal or not
        {
            bool reached_goal = false;
            for (auto p: agent.path)
            {
                if (p.location == agent.path_planner->goal_location)
                {
                    reached_goal = true;
                    break;
                }
            }
            if (!reached_goal) 
            {
                neighbor.agents.emplace_back(agent.id);
                agent.path.clear();
            }
            bool has_collision = false;
            for (auto other : complete_agents)
            {
                has_collision = instance.hasCollision(agent.path, agents[other].path);
                if (has_collision)
                {
                    //neighbor.agents.emplace_back(agent.id);
                    initial_collision = true;
                    //agent.path.clear(); do not clear path, use this as initial solution to lns2
                    break;
                }
            }
            if (!has_collision)
            {
                path_table.insertPath(agent.id, agent.path);
                complete_paths++;
                initial_sum_of_costs += (int)agent.path.size() - 1;
                complete_agents.emplace_back(agent.id);
                makespan = max(makespan, (int)agent.path.size() - 1);
            }
        }
    }
    if (screen == 2)
        cout << complete_agents.size() << " collision-free agents at timestep " << makespan << endl;
}

bool LNS::fixInitialSolutionWithLNS2()
{
    if (!neighbor.agents.empty() || initial_collision) //replan is need
    {
        cout<<"Fix Solution with PP, neighbor size: "<<neighbor.agents.size()<<endl;
        start_time = Time::now();
        neighbor.old_sum_of_costs = MAX_COST;
        neighbor.sum_of_costs = 0;
        bool succ = false;
        if (!neighbor.agents.empty()) //only run pp for those who does not have a path yet
        {
            succ = runPP();
            if (succ && !initial_collision)
            {
                initial_sum_of_costs += neighbor.sum_of_costs;
                sum_of_costs = initial_sum_of_costs;
                return true;
            }
        }
        //not succ or initial_collision
        cout<<"Fix Solution with LNS2"<<endl;
        //we need lns2 to fix path even if runtime out
        init_lns = new InitLNS(instance, agents, time_limit - ((fsec)(Time::now() - start_time)).count(),
                replan_algo_name,init_destory_name, neighbor_size, screen);
        init_lns->commit = commit;

        succ = init_lns->run();
        path_table.reset();
        for (const auto & agent : agents)
        {
            path_table.insertPath(agent.id, agent.path);
        }
        init_lns->clear();
        initial_sum_of_costs = init_lns->sum_of_costs;
        sum_of_costs = initial_sum_of_costs;
        initial_solution_runtime = ((fsec)(Time::now() - start_time)).count();
        if (succ)
            return true;
        else
        {
            // cout<<"after post processing: "<<endl;
            // validateSolution();
            return false;
        }
    }
    sum_of_costs = initial_sum_of_costs;
    start_time = Time::now();
    return true;
}

bool LNS::fixInitialSolutionWithLaCAM()
{
    if (!neighbor.agents.empty() || initial_collision)
    {
        start_time = Time::now();
        clearAll("Adaptive");
        cout<<"Fix Solution with LACAM"<<endl;
        auto succ = getInitialSolution();
        if (succ)
        {
            initial_sum_of_costs += neighbor.sum_of_costs;
            sum_of_costs = initial_sum_of_costs;
            return true;
        }
        else
        {
            initial_sum_of_costs += neighbor.sum_of_costs;
            sum_of_costs = initial_sum_of_costs;
            cout<<"lacam failed"<<endl;
            return false;
        }
    }
    sum_of_costs = initial_sum_of_costs;
    start_time = Time::now();
    return true;
}

bool LNS::getInitialSolution()
{
    neighbor.agents.resize(agents.size());
    for (int i = 0; i < (int)agents.size(); i++)
        neighbor.agents[i] = i;
    neighbor.old_sum_of_costs = MAX_COST;
    neighbor.sum_of_costs = 0;
    bool succ = false;
    succ = runLACAM2();
    if (succ)
    {
        initial_sum_of_costs = neighbor.sum_of_costs;
        sum_of_costs = neighbor.sum_of_costs;
        return true;
    }
    else
    {
        return false;
    }
}

bool LNS::runPP()
{
    auto shuffled_agents = neighbor.agents;
    std::random_shuffle(shuffled_agents.begin(), shuffled_agents.end());
    if (screen >= 2) 
    {
        for (auto id : shuffled_agents)
        {
            cout << id << "(" << instance.getAllpairDistance(agents[id].path_planner->start_location, agents[id].path_planner->goal_location) <<
                "->" <<agents[id].path.size()-1<< "), ";
        }
        cout << endl;
    }

    int remaining_agents = (int)shuffled_agents.size();
    auto p = shuffled_agents.begin();
    neighbor.sum_of_costs = 0;
    runtime = ((fsec)(Time::now() - start_time)).count();
    double T = time_limit - runtime; // time limit
    if (instance.env->map.size() > 9000)
        T-=0.1;
    if (instance.env->map.size() > 50000)
        T-=0.1;
    auto time = Time::now();
    ConstraintTable constraint_table(instance.env->cols, instance.env->map.size(), &path_table);

    while (p != shuffled_agents.end() && ((fsec)(Time::now() - time)).count() < T)
    {
        //smarter time control
        if (remaining_agents < (int)shuffled_agents.size())
        {
            auto remain_time = T - ((fsec)(Time::now() - time)).count();
            auto avg_single = ((fsec)(Time::now() - time)).count()/((int)shuffled_agents.size()-remaining_agents);
            if (avg_single > remain_time)
            {
                timeout_flag = true;
                break;
            }
        }

        int id = *p;
        if (screen >= 3)
            cout << "Remaining agents = " << remaining_agents <<
                 ", remaining time = " << T - ((fsec)(Time::now() - time)).count() << " seconds. " << endl
                 << "Agent " << agents[id].id << endl;
        agents[id].path_planner->commit_window = commit;
        //agents[id].path = agents[id].path_planner->findPath(constraint_table);
        agents[id].path = agents[id].path_planner->findPath(constraint_table, T - ((fsec)(Time::now() - time)).count(),timeout_flag);
        if (timeout_flag)
            break;
        if (agents[id].path.empty())
        {
            if (screen >= 3)
                cout<<"sipp failed"<<endl;
            break;
        } 
        if (screen >= 3)
            cout<<"current cost "<<(int)agents[id].path.size() - 1;
        neighbor.sum_of_costs += (int)agents[id].path.size() - 1;
        if (neighbor.sum_of_costs >= neighbor.old_sum_of_costs)
        {
            break;
        }
        remaining_agents--;
        path_table.insertPath(agents[id].id, agents[id].path);
        ++p;
        if (screen >= 3)
            cout<<endl;
    }
    if (remaining_agents == 0 && !timeout_flag && neighbor.sum_of_costs <= neighbor.old_sum_of_costs) // accept new paths
    {
        return true;
    }
    else // stick to old paths
    {
        if (neighbor.sum_of_costs > neighbor.old_sum_of_costs && screen >= 3)
            cout<<"solution worse"<<endl;
        if (p != shuffled_agents.end())
            num_of_failures++;
        auto p2 = shuffled_agents.begin();
        while (p2 != p)
        {
            int a = *p2;
            path_table.deletePath(agents[a].id, agents[a].path);
            ++p2;
        }
        if (!neighbor.old_paths.empty())
        {
            p2 = neighbor.agents.begin();
            for (int i = 0; i < (int)neighbor.agents.size(); i++)
            {
                int a = *p2;
                agents[a].path = neighbor.old_paths[i];
                path_table.insertPath(agents[a].id, agents[a].path);
                ++p2;
            }
            neighbor.sum_of_costs = neighbor.old_sum_of_costs;
        }
        return false;
    }
}



bool LNS::runLACAM2() 
{

    LACAMInstance ins = LACAMInstance(instance.env);
    //ins.update_dummygoals(instance.getDummyGoals());
    string verbose = "";
    auto MT = std::mt19937(0);
    const auto deadline = Deadline((time_limit-0.1) * 1000);

    const Objective objective = static_cast<Objective>(0);
    const float restart_rate = 0.01;
    const auto solution = solve(instance, ins, verbose, commit, 0, &deadline, &MT, objective, restart_rate);

    auto succ = true;

    if (solution.empty()) 
    {
        cout<<"empty"<<endl;
        return false;
    }

    int soc = 0;

    for (int agent = 0; agent < instance.getDefaultNumberOfAgents(); agent++)
    {
        int reached_goal_time = -1;
        for (size_t t = 0; t < solution.size(); t++)
        {
            auto curr_loc = solution[t][agent]->index;
            // agents[agent].path[t].location = curr_loc;
            if (curr_loc == instance.env->goal_locations[agent][0].first)
            {
                reached_goal_time = t;
                break;
            }
        }

        if (reached_goal_time == -1 || reached_goal_time < commit)
        {
            agents[agent].path.resize(solution.size());
            for (size_t t = 0; t <= solution.size() - 1; t++)
            {
                auto curr_loc = solution[t][agent]->index;
                agents[agent].path[t].location = curr_loc;
            }
            path_table.insertPath(agents[agent].id, agents[agent].path);
            soc+=agents[agent].path.size()-1;
            if (reached_goal_time == -1)
                succ = false;
        }
        else
        {
            agents[agent].path.resize(reached_goal_time+1);
            for (size_t t = 0; t <= reached_goal_time; t++)
            {
                auto curr_loc = solution[t][agent]->index;
                agents[agent].path[t].location = curr_loc;
            }
            path_table.insertPath(agents[agent].id, agents[agent].path);
            soc+=agents[agent].path.size()-1;
        }

    }
    neighbor.sum_of_costs =soc;

    return succ;
}

void LNS::chooseDestroyHeuristicbyALNS()
{
    rouletteWheel();
    switch (selected_neighbor)
    {
        case 0 : destroy_strategy = RANDOMWALK; break;
        case 1 : destroy_strategy = INTERSECTION; break;
        case 2 : destroy_strategy = RANDOMAGENTS; break;
        default : cerr << "ERROR" << endl; exit(-1);
    }
}

bool LNS::generateNeighborByIntersection()
{
    if (intersections.empty())
    {
        for (int i = 0; i < instance.env->map.size(); i++)
        {
            if (!instance.isObstacle(i) && instance.getDegree(i) > 2)
                intersections.push_back(i);
        }
    }

    set<int> neighbors_set;
    auto pt = intersections.begin();
    std::advance(pt, rand() % intersections.size());
    int location = *pt;
    path_table.get_agents(neighbors_set, neighbor_size, location);
    if (neighbors_set.size() < neighbor_size)
    {
        set<int> closed;
        closed.insert(location);
        std::queue<int> open;
        open.push(location);
        while (!open.empty() && (int) neighbors_set.size() < neighbor_size)
        {
            int curr = open.front();
            open.pop();
            for (auto next : instance.getNeighbors(curr))
            {
                if (closed.count(next) > 0)
                    continue;
                open.push(next);
                closed.insert(next);
                if (instance.getDegree(next) >= 3)
                {
                    path_table.get_agents(neighbors_set, neighbor_size, next);
                    if ((int) neighbors_set.size() == neighbor_size)
                        break;
                }
            }
        }
    }
    neighbor.agents.assign(neighbors_set.begin(), neighbors_set.end());
    if (neighbor.agents.size() > neighbor_size)
    {
        std::random_shuffle(neighbor.agents.begin(), neighbor.agents.end());
        neighbor.agents.resize(neighbor_size);
    }
    if (screen >= 2)
        cout << "Generate " << neighbor.agents.size() << " neighbors by intersection " << location << endl;
    return true;
}

bool LNS::generateNeighborByRandomWalk()
{
    if (neighbor_size >= (int)agents.size())
    {
        neighbor.agents.resize(agents.size());
        for (int i = 0; i < (int)agents.size(); i++)
            neighbor.agents[i] = i;
        return true;
    }

    int a = findMostDelayedAgent();

    if (a < 0)
        return false;
    
    set<int> neighbors_set;
    neighbors_set.insert(a);
    randomWalk(a, agents[a].path[0].location, 0, neighbors_set, neighbor_size, (int) agents[a].path.size() - 1);
    int count = 0;
    while (neighbors_set.size() < neighbor_size && count < 10)
    {
        int t = rand() % agents[a].path.size();
    
        // if (target_considered)
        //     randomWalkwithStayTarget(a, agents[a].path[t].location, t, neighbors_set, neighbor_size, (int) agents[a].path.size() - 1);
        //     //randomWalkwithStayTarget(a, agents[a].path[t].location, t, neighbors_set, neighbor_size, 2);
        // else
        randomWalk(a, agents[a].path[t].location, t, neighbors_set, neighbor_size, (int) agents[a].path.size() - 1);
        count++;
        // select the next agent randomly
        int idx = rand() % neighbors_set.size();
        int i = 0;
        for (auto n : neighbors_set)
        {
            if (i == idx)
            {
                a = i;
                break;
            }
            i++;
        }
    }
    if (neighbors_set.size() < 2)
        return false;
    neighbor.agents.assign(neighbors_set.begin(), neighbors_set.end());
    if (screen >= 2)
        cout << "Generate " << neighbor.agents.size() << " neighbors by random walks of agent " << a<<endl;
            //  << "(" << instance.getAllpairDistance(agents[a].path_planner)
            //  //agents[a].path_planner->my_heuristic[agents[a].path_planner->start_location]
            //  << "->" << agents[a].path.size() - 1 << ")" << endl;


    return true;
}

int LNS::findMostDelayedAgent()
{
    int a = -1;
    int max_delays = -1;
    for (int i = 0; i < agents.size(); i++)
    {
        if (tabu_list.find(i) != tabu_list.end())
            continue;
        if ((int) agents[i].path.size() -1 <= commit) //if size equal to commit, this means we at least need to have this path length
            continue;
        int delays = (int) agents[i].path.size() - 1 -  instance.getAllpairDistance(agents[i].path_planner->start_location, agents[i].path_planner->goal_location);
        if (max_delays < delays)
        {
            a = i;
            max_delays = delays;
        }
    }
    if (max_delays == 0)
    {
        tabu_list.clear();
        return -1;
    }
    tabu_list.insert(a);
    if (tabu_list.size() == agents.size())
        tabu_list.clear();
    return a;
}

int LNS::findRandomAgent() const
{
    int a = 0;
    int pt = rand() % (sum_of_costs - sum_of_distances) + 1;
    int sum = 0;
    for (; a < (int) agents.size(); a++)
    {
        sum += agents[a].getNumOfDelays();
        if (sum >= pt)
            break;
    }
    assert(sum >= pt);
    return a;
}

// a random walk with path that is shorter than upperbound and has conflicting with neighbor_size agents
void LNS::randomWalk(int agent_id, int start_location, int start_timestep,
                     set<int>& conflicting_agents, int neighbor_size, int upperbound)
{
    int loc = start_location;
    for (int t = start_timestep; t < upperbound; t++)
    {
        auto next_locs = instance.getNeighbors(loc);
        next_locs.push_back(loc);
        while (!next_locs.empty())
        {
            int step = rand() % next_locs.size();
            auto it = next_locs.begin();
            advance(it, step);
            // int next_h_val = agents[agent_id].path_planner->my_heuristic[*it];
            int next_h_val = instance.getAllpairDistance(agents[agent_id].path_planner->goal_location,*it);
            //agents[agent_id].path_planner->my_heuristic[*it];
            if (t + 1 + next_h_val < upperbound) // move to this location
            {
                path_table.getConflictingAgents(agent_id, conflicting_agents, loc, *it, t + 1);
                loc = *it;
                break;
            }
            next_locs.erase(it);
        }
        if (next_locs.empty() || conflicting_agents.size() >= neighbor_size)
        {
            break;
        }
    }
}

void LNS::randomWalkwithStayTarget(int agent_id, int start_location, int start_timestep,
                     set<int>& conflicting_agents, int neighbor_size, int upperbound)
{
    int loc = start_location;
    for (int t = start_timestep; t < upperbound; t++)
    {
        auto next_locs = instance.getNeighbors(loc);
        next_locs.push_back(loc);
        while (!next_locs.empty())
        {
            int step = rand() % next_locs.size();
            auto it = next_locs.begin();
            advance(it, step);
            int next_h_val = agents[agent_id].path_planner->my_heuristic[*it];
            if (t + 1 + next_h_val < upperbound) // move to this location
            {
                //path_table.getConflictingAgents(agent_id, conflicting_agents, loc, *it, t + 1);
                path_table.getConflictingAgentswithStayTarget(agent_id, conflicting_agents,stay_target, loc, *it, t + 1);
                loc = *it;
                break;
            }
            next_locs.erase(it);
        }
        if (next_locs.empty() || conflicting_agents.size() >= neighbor_size)
            break;
    }
}

void LNS::validateSolution() const
{
    int sum = 0;
    for (const auto& a1_ : agents)
    {
        if (a1_.path.empty())
        {
            cout << "No solution for agent " << a1_.id << endl;
            //exit(-1);
        }
        else if (a1_.path_planner->start_location != a1_.path.front().location)
        {
            cout << "The path of agent " << a1_.id << " starts from location " << a1_.path.front().location
                << ", which is different from its start location " << a1_.path_planner->start_location << endl;
            //exit(-1);
        }
        else if (a1_.path_planner->goal_location != a1_.path.back().location)
        {
            cout << "The path of agent " << a1_.id << " ends at location " << a1_.path.back().location
                 << ", which is different from its goal location " << a1_.path_planner->goal_location << endl;
            //exit(-1);
        }
        for (int t = 1; t < (int) a1_.path.size(); t++ )
        {
            if (!instance.validMove(a1_.path[t - 1].location, a1_.path[t].location))
            {
                cout << "The path of agent " << a1_.id << " jump from "
                     << a1_.path[t - 1].location << " to " << a1_.path[t].location
                     << " between timesteps " << t - 1 << " and " << t << endl;
                //exit(-1);
            }
        }
        sum += (int) a1_.path.size() - 1;
        for (const auto  & a2_: agents)
        {
            if (a1_.id >= a2_.id || a2_.path.empty())
                continue;
            const auto & a1 = a1_.path.size() <= a2_.path.size()? a1_ : a2_;
            const auto & a2 = a1_.path.size() <= a2_.path.size()? a2_ : a1_;
            int t = 1;
            // for (; t < (int) a1.path.size(); t++)
            for (; t < (int) a1.path.size() && t <= commit; t++)
            {
                if (a1.path[t].location == a2.path[t].location) // vertex conflict
                {
                    cout << "Find a vertex conflict between agents " << a1.id << " and " << a2.id <<
                            " at location " << a1.path[t].location << " at timestep " << t << endl;
                    //exit(-1);
                }
                else if (a1.path[t].location == a2.path[t - 1].location &&
                        a1.path[t - 1].location == a2.path[t].location) // edge conflict
                {
                    cout << "Find an edge conflict between agents " << a1.id << " and " << a2.id <<
                         " at edge (" << a1.path[t - 1].location << "," << a1.path[t].location <<
                         ") at timestep " << t << endl;
                    //exit(-1);
                }
            }
            // int target = a1.path.back().location;
            // for (; t < (int) a2.path.size(); t++)
            // {
            //     if (a2.path[t].location == target)  // target conflict
            //     {
            //         cerr << "Find a target conflict where agent " << a2.id << " (of length " << a2.path.size() - 1<<
            //              ") traverses agent " << a1.id << " (of length " << a1.path.size() - 1<<
            //              ")'s target location " << target << " at timestep " << t << endl;
            //         exit(-1);
            //     }
            // }
        }
    }
    if (sum_of_costs != sum)
    {
        cout << "The computed sum of costs " << sum_of_costs <<
             " is different from the sum of the paths in the solution " << sum << endl;
        //exit(-1);
    }
}

void LNS::writeIterStatsToFile(const string & file_name) const
{
    if (init_lns != nullptr)
    {
        init_lns->writeIterStatsToFile(file_name + "-initLNS.csv");
    }
    if (iteration_stats.size() <= 1)
        return;
    string name = file_name;
    if (use_init_lns or num_of_iterations > 0)
        name += "-LNS.csv";
    else
        name += "-" + init_algo_name + ".csv";
    std::ofstream output;
    output.open(name);
    // header
    output << "num of agents," <<
           "sum of costs," <<
           "runtime," <<
           "cost lowerbound," <<
           "sum of distances," <<
           "MAPF algorithm" << endl;

    for (const auto &data : iteration_stats)
    {
        output << data.num_of_agents << "," <<
               data.sum_of_costs << "," <<
               data.runtime << "," <<
               max(sum_of_costs_lowerbound, sum_of_distances) << "," <<
               sum_of_distances << "," <<
               data.algorithm << endl;
    }
    output.close();
}

void LNS::writeResultToFile(const string & file_name) const
{
    if (init_lns != nullptr)
    {
        init_lns->writeResultToFile(file_name + "-initLNS.csv", sum_of_distances, preprocessing_time);
    }
    string name = file_name;
    if (use_init_lns or num_of_iterations > 0)
        name += "-LNS.csv";
    else
        name += "-" + init_algo_name + ".csv";
    std::ifstream infile(name);
    bool exist = infile.good();
    infile.close();
    if (!exist)
    {
        ofstream addHeads(name);
        addHeads << "runtime,solution cost,initial solution cost,lower bound,sum of distance," <<
                 "initial collision-free paths,delete timesteps,iterations," <<
                 "group size," <<
                 "runtime of initial solution,restart times,area under curve," <<
                 "LL expanded nodes,LL generated,LL reopened,LL runs," <<
                 "preprocessing runtime,solver name,instance name" << endl;
        addHeads.close();
    }
    uint64_t num_LL_expanded = 0, num_LL_generated = 0, num_LL_reopened = 0, num_LL_runs = 0;
    for (auto & agent : agents)
    {
        agent.path_planner->reset();
        num_LL_expanded += agent.path_planner->accumulated_num_expanded;
        num_LL_generated += agent.path_planner->accumulated_num_generated;
        num_LL_reopened += agent.path_planner->accumulated_num_reopened;
        num_LL_runs += agent.path_planner->num_runs;
    }
    double auc = 0;
    if (!iteration_stats.empty())
    {
        auto prev = iteration_stats.begin();
        auto curr = prev;
        ++curr;
        while (curr != iteration_stats.end() && curr->runtime < time_limit)
        {
            auc += (prev->sum_of_costs - sum_of_distances) * (curr->runtime - prev->runtime);
            prev = curr;
            ++curr;
        }
        auc += (prev->sum_of_costs - sum_of_distances) * (time_limit - prev->runtime);
    }
    ofstream stats(name, std::ios::app);
    stats << runtime << "," << sum_of_costs << "," << initial_sum_of_costs << "," <<
          max(sum_of_distances, sum_of_costs_lowerbound) << "," << sum_of_distances << "," <<
          complete_paths << "," << delete_timesteps << "," <<
          iteration_stats.size() << "," << average_group_size << "," <<
          initial_solution_runtime << "," << restart_times << "," << auc << "," <<
          num_LL_expanded << "," << num_LL_generated << "," << num_LL_reopened << "," << num_LL_runs << "," <<
          preprocessing_time << "," << getSolverName() << endl;
    stats.close();
}

bool LNS::loadPaths(const string & file_name)
{
    using namespace std;

    string line;
    ifstream myfile (file_name.c_str());
    if (!myfile.is_open())
        return false;

    while(getline(myfile, line))
    {
        boost::char_separator<char> sep(":()-> ,");
        boost::tokenizer< boost::char_separator<char> > tok(line, sep);
        auto beg = tok.begin();
        beg++; // skip "Agent"
        int agent_id = atoi((*beg).c_str());
        assert(0 <= agent_id < agents.size());
        beg++;
        while (beg != tok.end())
        {
            int col = atoi((*beg).c_str());
            beg++;
            int row = atoi((*beg).c_str());
            beg++;
            agents[agent_id].path.emplace_back(instance.linearizeCoordinate(row, col));
        }
        if (agents[agent_id].path.front().location != agents[agent_id].path_planner->start_location)
        {
            cerr << "Agent " << agent_id <<"'s path starts at " << agents[agent_id].path.front().location
            << "=(" << instance.getColCoordinate(agents[agent_id].path.front().location)
            << "," << instance.getRowCoordinate(agents[agent_id].path.front().location)
            << "), which is different from its start location " << agents[agent_id].path_planner->start_location << endl
            << "=(" << instance.getColCoordinate(agents[agent_id].path_planner->start_location)
            << "," << instance.getRowCoordinate(agents[agent_id].path_planner->start_location)
            << ")" << endl;
            exit(-1);
        }
    }
    myfile.close();
    has_initial_solution = true;
    return true;
}

bool LNS::loadPaths(vector<list<int>> paths)
{
    for (int agent_id = 0; agent_id < instance.getDefaultNumberOfAgents(); agent_id++)
    {
        if (paths[agent_id].empty())
            continue;
        for(auto location: paths[agent_id])
        {
            agents[agent_id].path.emplace_back(location);
        }
        for (int i = paths[agent_id].size(); i <= commit; i++) //we ensure enough locations in commit window
        {
            agents[agent_id].path.emplace_back(paths[agent_id].back());
        }
        //path_table.insertPath(agent_id, agents[agent_id].path);
        initial_sum_of_costs+=agents[agent_id].path.size()-1;
        // if (agents[agent_id].path.front().location != agents[agent_id].path_planner->start_location)
        // {
        //     cerr << "Agent " << agent_id <<"'s path starts at " << agents[agent_id].path.front().location
        //     << "=(" << instance.getColCoordinate(agents[agent_id].path.front().location)
        //     << "," << instance.getRowCoordinate(agents[agent_id].path.front().location)
        //     << "), which is different from its start location " << agents[agent_id].path_planner->start_location << endl
        //     << "=(" << instance.getColCoordinate(agents[agent_id].path_planner->start_location)
        //     << "," << instance.getRowCoordinate(agents[agent_id].path_planner->start_location)
        //     << ")" << endl;
        //     exit(-1);
        // }
    }
    has_initial_solution = true;
    return true;
}

bool LNS::loadPaths(vector<Path> paths)
{
    for (int agent_id = 0; agent_id < instance.getDefaultNumberOfAgents(); agent_id++)
    {
        if (paths[agent_id].empty())
            continue;
        for(auto location: paths[agent_id])
        {
            agents[agent_id].path.emplace_back(location);
        }
        for (int i = paths[agent_id].size(); i <= commit; i++) //we ensure enough locations in commit window
        {
            agents[agent_id].path.emplace_back(paths[agent_id].back());
        }
        //path_table.insertPath(agent_id, agents[agent_id].path);
        initial_sum_of_costs+=agents[agent_id].path.size()-1;
        // if (agents[agent_id].path.front().location != agents[agent_id].path_planner->start_location)
        // {
        //     cerr << "Agent " << agent_id <<"'s path starts at " << agents[agent_id].path.front().location
        //     << "=(" << instance.getColCoordinate(agents[agent_id].path.front().location)
        //     << "," << instance.getRowCoordinate(agents[agent_id].path.front().location)
        //     << "), which is different from its start location " << agents[agent_id].path_planner->start_location << endl
        //     << "=(" << instance.getColCoordinate(agents[agent_id].path_planner->start_location)
        //     << "," << instance.getRowCoordinate(agents[agent_id].path_planner->start_location)
        //     << ")" << endl;
        //     exit(-1);
        // }
    }
    has_initial_solution = true;
    return true;
}

void LNS::writePathsToFile(const string & file_name) const
{
    std::ofstream output;
    output.open(file_name);
    // header
    // output << agents.size() << endl;

    for (const auto &agent : agents)
    {
        output << "Agent " << agent.id << ":";
        for (const auto &state : agent.path)
            output << "(" << instance.getColCoordinate(state.location) << "," <<
                            instance.getRowCoordinate(state.location) << ")->";
        output << endl;
    }
    output.close();
}

void LNS::commitPath(int commit_step, vector<list<int>> &commit_path, vector<list<int>> &future_path,bool skip_start,int current_time)
{
    for (const auto &agent : agents)
    {
        // if (agent.path.empty())
        //     cout<<"error "<<agent.id<<endl;
        if (screen == 3)
            cout<<"Commiting: "<<agent.id<<" current location "<<agent.path.front().location<<" gola location "<<instance.env->goal_locations[agent.id][0].first<<endl;
        //agent reach target before, but need to de-tour due to resolving conflict, so we add the time reach target as waiting
        // if (current_time != 0 && agent.path.size() > 1 && commit_path[agent.id].size() <= current_time)
        // {
        //     commit_path[agent.id].emplace_back(commit_path[agent.id].back());
        //     if (screen == 3)
        //         cout<< "(" << instance.getColCoordinate(commit_path[agent.id].back()) << "," <<
        //                         instance.getRowCoordinate(commit_path[agent.id].back()) << ")->";
        // }
        // if (agent.path.size() == 1)
        // {
        //     cout<<"path empty"<<endl;
        // }
        if (agent.path.size() > commit_step)
        {
            if (stay_target[agent.id] != 0)
            {
                stay_target[agent.id] = 0;
            }
            int step = 0;
            for (const auto &state : agent.path)
            {
                if (step == 0)
                {
                    if (skip_start)
                    {
                        step++;
                        continue;
                    }
                }
                if(step <commit_step)
                {
                    commit_path[agent.id].emplace_back(state.location);
                    if (screen == 3)
                        // cout<< "(" << instance.getColCoordinate(state.location) << "," <<
                        //         instance.getRowCoordinate(state.location) << ")->";
                        cout<<state.location << ")->";
                }
                else if (step == commit_step)
                {
                    commit_path[agent.id].emplace_back(state.location);
                    if (screen == 3)
                    {
                        // cout<< "(" << instance.getColCoordinate(state.location) << "," <<
                        //             instance.getRowCoordinate(state.location) << ")"<<endl;
                        cout<<state.location<<endl;
                        cout<<"Remaining: "<<endl;
                    }
                        
                    future_path[agent.id].emplace_back(state.location);
                    if (screen == 3)
                        // cout<< "(" << instance.getColCoordinate(state.location) << "," <<
                        //         instance.getRowCoordinate(state.location) << ")->";
                        cout<<state.location << ")->";
                }
                else
                {
                    future_path[agent.id].emplace_back(state.location);
                    if (screen == 3)
                        // cout<< "(" << instance.getColCoordinate(state.location) << "," <<
                        //         instance.getRowCoordinate(state.location) << ")->";
                        cout<<state.location << ")->";
                }
                step++;
            }

        }
        else
        {
            int step = 0;
            for (const auto &state : agent.path)
            {
                if (step == 0)
                {
                    if (skip_start)
                    {
                        step++;
                        continue;
                    }
                }
                commit_path[agent.id].emplace_back(state.location);
                if (screen == 3)
                    // cout<< "(" << instance.getColCoordinate(state.location) << "," <<
                    //         instance.getRowCoordinate(state.location) << ")->";
                    cout<<state.location << ")->";
                step++;
            }
            for (;step<=commit_step;step++)
            {
                commit_path[agent.id].emplace_back(commit_path[agent.id].back());
                stay_target[agent.id]++;
                if (screen == 3)
                    // cout<< "(" << instance.getColCoordinate(state.location) << "," <<
                    //         instance.getRowCoordinate(state.location) << ")->";
                    cout<<commit_path[agent.id].back() << ")->";
            }
            if (screen == 3)
            {
                cout<<endl;
                cout<<"Remaining: "<<endl;
            }
            future_path[agent.id].emplace_back(commit_path[agent.id].back());
            if (screen == 3)
                // cout<< "(" << instance.getColCoordinate(commit_path[agent.id].back()) << "," <<
                //         instance.getRowCoordinate(commit_path[agent.id].back()) << ")->";
                cout<<commit_path[agent.id].back() << ")->";
        }
        if (screen == 3)
            cout<<endl;
    }
}

void LNS::setStartGoal()
{
    auto starts = instance.env->curr_states;
    auto goals = instance.env->goal_locations;
    auto dummy_goals = instance.getDummyGoals();
    for (auto& a: agents)
    {
        a.path_planner->start_location = starts[a.id].location;
        a.path_planner->goal_location = goals[a.id][0].first;
        a.path_planner->dummy_goal = dummy_goals[a.id];
        cout<<"start "<< a.path_planner->start_location<<endl;
    }
}

void LNS::clearAll(const string & destory_name)
{
    path_table.reset();
    //tabu_list.clear();
    //intersections.clear();
    auto starts = instance.env->curr_states;
    auto goals = instance.env->goal_locations;
    auto dummy_goals = instance.getDummyGoals();
    for (auto& a: agents)
    {
        a.path.clear();
        a.path_planner->start_location = starts[a.id].location;
        a.path_planner->goal_location = goals[a.id][0].first;
        a.path_planner->dummy_goal = dummy_goals[a.id];
        a.path_planner->commit_window = commit;
    }

    //start_time = Time::now(); 
    neighbor.agents.clear();
    neighbor.sum_of_costs = 0;
    neighbor.old_sum_of_costs = 0;
    neighbor.colliding_pairs.clear();
    neighbor.old_colliding_pairs.clear();
    neighbor.old_paths.clear();

    num_of_failures = 0;
    iteration_stats.clear();
    average_group_size = -1;
    sum_of_costs = 0;
    timeout_flag = false;

    // decay_factor = -1;
    // reaction_factor = -1;
    // destroy_weights.clear();
    // selected_neighbor = 0;

    preprocessing_time = 0;
    initial_solution_runtime = 0;
    initial_sum_of_costs = -1;
    sum_of_costs_lowerbound = -1;
    sum_of_distances = -1;
    restart_times = 0;
    complete_paths = 0;
    delete_timesteps = 0;

    num_of_iterations = 0;

    if (destory_name == "Adaptive")
    {
        ALNS = true;
        //destroy_weights.clear();
        //destroy_weights.assign(DESTORY_COUNT, 1);
        decay_factor = 0.01;
        reaction_factor = 0.01;
    }
    else if (destory_name == "RandomWalk")
        destroy_strategy = RANDOMWALK;
    else if (destory_name == "Intersection")
        destroy_strategy = INTERSECTION;
    else if (destory_name == "Random")
        destroy_strategy = RANDOMAGENTS;
    else
    {
        cerr << "Destroy heuristic " << destory_name << " does not exists. " << endl;
        exit(-1);
    }
}


bool LNS::validateCommitSolution(vector<list<int>> commited_paths) const
{
    vector<Agent> commited_agents;
    int N = instance.getDefaultNumberOfAgents();
    commited_agents.reserve(N);
    for (int i = 0; i < N; i++)
        commited_agents.emplace_back(instance, i, false);
    for (int agent_id = 0; agent_id < instance.getDefaultNumberOfAgents(); agent_id++)
    {
        for(auto location: commited_paths[agent_id])
        {
            commited_agents[agent_id].path.emplace_back(location);
        }
    }
    int sum = 0;
    for (const auto& a1_ : commited_agents)
    {
        for (int t = 1; t < (int) a1_.path.size(); t++ )
        {
            if (!instance.validMove(a1_.path[t - 1].location, a1_.path[t].location))
            {
                cout<<"invalid move "<<a1_.id<<endl;
                return false;
            }
        }
        sum += (int) a1_.path.size() - 1;
        for (const auto  & a2_: commited_agents)
        {
            if (a1_.id >= a2_.id || a2_.path.empty())
                continue;
            const auto & a1 = a1_.path.size() <= a2_.path.size()? a1_ : a2_;
            const auto & a2 = a1_.path.size() <= a2_.path.size()? a2_ : a1_;
            int t = 1;
            for (; t < (int) a1.path.size(); t++)
            {
                if (a1.path[t].location == a2.path[t].location) // vertex conflict
                {
                    cout<<"vertex conflict"<<a1.id<<" "<<a2.id<<endl;
                    return false;
                }
                else if (a1.path[t].location == a2.path[t - 1].location &&
                        a1.path[t - 1].location == a2.path[t].location) // edge conflict
                {
                    cout<<"edge conflict "<<a1.id<<" "<<a2.id<<endl;
                    return false;
                }
            }
        }
    }
    return true;
}



