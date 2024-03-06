#include <cmath>
#include "CompetitionSystem.h"
#include <boost/tokenizer.hpp>
#include "nlohmann/json.hpp"
#include <functional>
#include <Logger.h>
#include <DelaySimulation.h>

using json = nlohmann::ordered_json;


list<Task> BaseSystem::move(vector<Action>& actions)
{
    for (int k = 0; k < num_of_agents; k++)
    {
        if (k >= actions.size()){
            fast_mover_feasible = false;
            planner_movements[k].push_back(Action::NA);
        }
        else
        {
            planner_movements[k].push_back(actions[k]);
        }
    }

    list<Task> finished_tasks_this_timestep; // <agent_id, task_id, timestep>
    if (!valid_moves(curr_states, actions))
    {
        fast_mover_feasible = false;
        actions = std::vector<Action>(num_of_agents, Action::WA);
        curr_states = model->result_states(curr_states, actions);
    }
    else //we simulate delays
    {
        vector<bool> delay;
        delay.resize(num_of_agents);
        int current_time = curr_states[0].timestep;
        for (int i = 0; i < num_of_agents; i++)
        {
            delay[i] = simulation_delay[i][current_time];
        }

        curr_states = model->result_states(curr_states, actions);
        
        SimulateMCP postmcp(map.map.size(),1);
        {
            vector<Path*> temp;
            temp.resize(env->num_of_agents);
            for (int a = 0; a < env->planned_paths.size(); a++)
            {
                temp[a] = &(env->planned_paths[a]);
            }
            postmcp.build(temp);
            postmcp.simulate(temp,delay);
        }
        postmcp.clear();

        for (int i = 0; i < env->planned_paths.size();i++)
        {
            curr_states[i].location = env->planned_paths[i][1].location;
        }
    }

    // agents do not move
    for (int k = 0; k < num_of_agents; k++)
    {
        if (!assigned_tasks[k].empty() && curr_states[k].location == assigned_tasks[k].front().location)
        {
            Task task = assigned_tasks[k].front();
            assigned_tasks[k].pop_front();
            task.t_completed = timestep;
            finished_tasks_this_timestep.push_back(task);
            events[k].push_back(make_tuple(task.task_id, timestep,"finished"));
            log_event_finished(k, task.task_id, timestep);
        }
        paths[k].push_back(curr_states[k]);
        actual_movements[k].push_back(actions[k]);
    }

    return finished_tasks_this_timestep;
}


// This function might not work correctly with small map (w or h <=2)
bool BaseSystem::valid_moves(vector<State>& prev, vector<Action>& action)
{
    return model->is_valid(prev, action);
}


void BaseSystem::sync_shared_env() 
{
    env->goal_locations.resize(num_of_agents);
    for (size_t i = 0; i < num_of_agents; i++)
    {
        env->goal_locations[i].clear();
        //check if current assigned task will be finished or not
        list<Task> copy_tasks;
        for(auto& task: assigned_tasks[i])
        {
            copy_tasks.push_back({task.location, task.t_assigned });
        }
        //dummy simulation of task finishing with inf assign system
        for (auto loc: curr_commits[i])
        {
            if (loc.location == copy_tasks.front().location)
                copy_tasks.pop_front();
            int counter = task_counter[k];
            int id = 0;
            while (copy_tasks.size() < num_tasks_reveal)
            {
                int c = counter * num_of_agents + i;
                int loc = tasks[c%tasks_size];
                Task task(task_id+id,loc,timestep,i);
                copy_tasks.push_back(task);
                id++;
                counter++;
            }
        }
        for(auto& task: copy_tasks[i])
        {
            env->goal_locations[i].push_back({task.location, task.t_assigned });
        }
    }
    env->curr_timestep = timestep;
    env->curr_states = curr_states;
}


bool BaseSystem::planner_initialize()
{
    using namespace std::placeholders;
    std::packaged_task<void(int)> init_task(std::bind(&MAPFPlanner::initialize, planner, _1));
    auto init_future = init_task.get_future();
    
    auto init_td = std::thread(std::move(init_task), preprocess_time_limit);
    if (init_future.wait_for(std::chrono::seconds(preprocess_time_limit)) == std::future_status::ready)
    {
        init_td.join();
        return true;
    }

    init_td.detach();
    return false;
}


void BaseSystem::log_preprocessing(bool succ)
{
    if (logger == nullptr)
        return;
    if (succ)
    {
        logger->log_info("Preprocessing success", timestep);
    } 
    else
    {
        logger->log_fatal("Preprocessing timeout", timestep);
    }
}


void BaseSystem::log_event_assigned(int agent_id, int task_id, int timestep)
{
    logger->log_info("Task " + std::to_string(task_id) + " is assigned to agent " + std::to_string(agent_id), timestep);
}


void BaseSystem::log_event_finished(int agent_id, int task_id, int timestep) 
{
    logger->log_info("Agent " + std::to_string(agent_id) + " finishes task " + std::to_string(task_id), timestep);
}

void BaseSystem::execution_with_delay(vector<Path> curr_commits)
{
        vector<vector<bool>> delay; // delay within window
        delay.resize(num_of_agents);
        int current_time = curr_states[0].timestep;
        for (int i = 0; i < num_of_agents; i++)
        {
            delay[i].resize(commit_window);
            for (int t = 0; t < commit_window; t++)
            {
                delay[i][t] = simulation_delay[i][current_time+t];
            }
        }
        
        SimulateMCP postmcp(map.map.size(),1);
        {
            vector<Path*> temp;
            temp.resize(env->num_of_agents);
            for (int a = 0; a < env->planned_paths.size(); a++)
            {
                temp[a] = &(curr_commits[a]);
            }
            postmcp.build(temp);
            postmcp.simulate(temp,delay);
        }
        postmcp.clear();
}

void BaseSystem::simulate(int simulation_time)
{
    //init logger
    initialize(); //funciontal initiliazing (maps, pre-process, etc.)
    int num_of_tasks = 0;

    cout << "----------------------------" << std::endl;
    cout << "Timestep " << timestep << std::endl;
    //add an initial planning here
    //plan
    sync_shared_env();
    auto start = std::chrono::steady_clock::now();
    planner->loadPaths(); //we assume time on loading path is free for analysis
    planner->plan(init_time_limit);
    auto end = std::chrono::steady_clock::now();
    auto diff = end-start; //actual planning time
    planner_times.push_back(std::chrono::duration<double>(diff).count());
    //commit k
    planner->commit(curr_commits); //push back the curr commits


    for (; timestep < simulation_time; ) //start execution
    {
        cout << "----------------------------" << std::endl;
        cout << "Timestep " << timestep << std::endl;

        // //if simulation time is over, don't plan
        // if (timestep + commit_window < simulation_time)
        // {
        //*** dummy sync component ***
        // predict the moves, give new goals if will finsihed, return moves will not be executed to planner
        sync_shared_env(); // need to extend to a dummy simulater

        //*** plan component ***  
        // plan for window (i)     
        planner->loadPaths(); //we assume time on loading path is free for analysis
        auto start = std::chrono::steady_clock::now();
        planner->plan(plan_time_limit);
        auto end = std::chrono::steady_clock::now();
        auto diff = end-start; //actual planning time
        planner_times.push_back(std::chrono::duration<double>(diff).count());
        //commit k
        planner->commit(curr_commits); //push back the curr commits
        //}

        //*** execution component ***
        // execute for window (i-1)
        // todo: add a validation component (current it's fine because the solver always return conflict-free solution)
        // execution with real delays w(i-1) and simulate future moves of w(i)
        execution_sync(); 
        for (int i = 1; i <= commit_window; i++)
        {
            list<Task> new_finished_tasks = move(actions); //record task finishes (from real exe)
            cout << new_finished_tasks.size() << " tasks has been finished in this timestep" << std::endl;
            for (auto task : new_finished_tasks)
            {
                // int id, loc, t;
                // std::tie(id, loc, t) = task;
                finished_tasks[task.agent_assigned].emplace_back(task);
                num_of_tasks++;
                num_of_task_finish++;
            }
            cout << num_of_tasks << " tasks has been finished by far in total" << std::endl;
            update_tasks();
            if (timestep >= simulation_time)
            {
                break;
            }
        }
    }

    cout << std::endl << "Done!" << std::endl;
}


void BaseSystem::initialize()
{
    paths.resize(num_of_agents);
    events.resize(num_of_agents);
    env->num_of_agents = num_of_agents;
    env->rows = map.rows;
    env->cols = map.cols;
    env->map = map.map;
    finished_tasks.resize(num_of_agents);
    timestep = 0;
    curr_states = starts;
    assigned_tasks.resize(num_of_agents);
    curr_commits.resize(num_of_agents);

    //planner initilise before knowing the first goals
    auto planner_initialize_success= planner_initialize();
    
    log_preprocessing(planner_initialize_success);
    if (!planner_initialize_success)
        return;

    // initialize_goal_locations();
    update_tasks();

    //sync_shared_env();

    actual_movements.resize(num_of_agents);
    planner_movements.resize(num_of_agents);
}


void BaseSystem::saveResults(const string &fileName, const bool outputSimple) const
{
    json js;
    // Save action model
    js["actionModel"] = "MAPF";

    std::string feasible = fast_mover_feasible ? "Yes" : "No";
    js["AllValid"] = feasible;

    js["teamSize"] = num_of_agents;

    // Save start locations[x,y,orientation]

    if (!outputSimple)
    {
        json start = json::array();
        for (int i = 0; i < num_of_agents; i++)
        {
            json s = json::array();
            s.push_back(starts[i].location/map.cols);
            s.push_back(starts[i].location%map.cols);
            switch (starts[i].orientation)
            {
            case 0:
                s.push_back("E");
                break;
            case 1:
                s.push_back("S");
            case 2:
                s.push_back("W");
                break;
            case 3:
                s.push_back("N");
                break;
            }
            start.push_back(s);
        }
        js["start"] = start;
    }

    js["numTaskFinished"] = num_of_task_finish;
    js["makespan"] = actual_movements[0].size();
  
    if (!outputSimple)
    {
    // Save actual paths
    json apaths = json::array();
    for (int i = 0; i < num_of_agents; i++)
    {
        std::string path;
        bool first = true;
        for (const auto action : actual_movements[i])
        {
            if (!first)
            {
                path+= ",";
            }
            else
            {
                first = false;
            }

            // if (action == Action::FW)
            // {
            //     path+="F";
            // }
            // else if (action == Action::CR)
            // {
            //     path+="R";
            // } 
            // else if (action == Action::CCR)
            // {
            //     path+="C";
            // }
            // else if (action == Action::NA)
            // {
            //     path+="T";
            // }
            // else
            // {
            //     path+="W";
            // }
            if (action == Action::N)
            {
                path+="NO";
            }
            else if (action == Action::E)
            {
                path+="EA";
            } 
            else if (action == Action::S)
            {
                path+="SO";
            }
            else if (action == Action::WE)
            {
                path+="WE";
            }
            else if (action == Action::NA)
            {
                path+="T";
            }
            else
            {
                path+="W";
            }
        }
        apaths.push_back(path);
    }
    js["actualPaths"] = apaths;

    //planned paths
    json ppaths = json::array();
    for (int i = 0; i < num_of_agents; i++)
    {
        std::string path;
        bool first = true;
        for (const auto action : planner_movements[i])
        {
            if (!first)
            {
                path+= ",";
            } 
            else 
            {
                first = false;
            }

            // if (action == Action::FW)
            // {
            //     path+="F";
            // }
            // else if (action == Action::CR)
            // {
            //     path+="R";
            // } 
            // else if (action == Action::CCR)
            // {
            //     path+="C";
            // } 
            // else if (action == Action::NA)
            // {
            //     path+="T";
            // }
            // else
            // {
            //     path+="W";
            // }
            if (action == Action::N)
            {
                path+="NO";
            }
            else if (action == Action::E)
            {
                path+="EA";
            } 
            else if (action == Action::S)
            {
                path+="SO";
            }
            else if (action == Action::WE)
            {
                path+="WE";
            }
            else if (action == Action::NA)
            {
                path+="T";
            }
            else
            {
                path+="W";
            }
        }  
        ppaths.push_back(path);
    }
    js["plannerPaths"] = ppaths;

    json planning_times = json::array();
    for (double time: planner_times)
        planning_times.push_back(time);
    js["plannerTimes"] = planning_times;

    // Save errors
    json errors = json::array();
    for (auto error: model->errors)
    {
        std::string error_msg;
        int agent1;
        int agent2;
        int timestep;
        std::tie(error_msg,agent1,agent2,timestep) = error;
        json e = json::array();
        e.push_back(agent1);
        e.push_back(agent2);
        e.push_back(timestep);
        e.push_back(error_msg);
        errors.push_back(e);

    }
    js["errors"] = errors;
  
    // Save events
    json events_json = json::array();
    for (int i = 0; i < num_of_agents; i++)
    {
        json event = json::array();
        for(auto e: events[i])
        {
            json ev = json::array();
            std::string event_msg;
            int task_id;
            int timestep;
            std::tie(task_id,timestep,event_msg) = e;
            ev.push_back(task_id);
            ev.push_back(timestep);
            ev.push_back(event_msg);
            event.push_back(ev);
        }
        events_json.push_back(event);
    }
    js["events"] = events_json;

    // Save all tasks
    json tasks = json::array();
    for (auto t: all_tasks)
    {
        json task = json::array();
        task.push_back(t.task_id);
        task.push_back(t.location/map.cols);
        task.push_back(t.location%map.cols);
        tasks.push_back(task);
    }
    js["tasks"] = tasks;
    }

    std::ofstream f(fileName,std::ios_base::trunc |std::ios_base::out);
    f << std::setw(4) << js;

}

bool FixedAssignSystem::load_agent_tasks(string fname)
{
    string line;
    std::ifstream myfile(fname.c_str());
    if (!myfile.is_open()) return false;

    getline(myfile, line);
    while (!myfile.eof() && line[0] == '#') {
        getline(myfile, line);
    }

    boost::char_separator<char> sep(",");
    boost::tokenizer<boost::char_separator<char>> tok(line, sep);
    boost::tokenizer<boost::char_separator<char>>::iterator beg = tok.begin();

    num_of_agents = atoi((*beg).c_str());
    int task_id = 0;
    // My benchmark
    if (num_of_agents == 0) {
        //issue_logs.push_back("Load file failed");
        std::cerr << "The number of agents should be larger than 0" << endl;
        exit(-1);
    }
    starts.resize(num_of_agents);
    task_queue.resize(num_of_agents);
  
    for (int i = 0; i < num_of_agents; i++)
    {
        cout << "agent " << i << ": ";

        getline(myfile, line);
        while (!myfile.eof() && line[0] == '#')
            getline(myfile, line);

        boost::tokenizer<boost::char_separator<char>> tok(line, sep);
        boost::tokenizer<boost::char_separator<char>>::iterator beg = tok.begin();
        // read start [row,col] for agent i
        int num_landmarks = atoi((*beg).c_str());
        beg++;
        auto loc = atoi((*beg).c_str());
        // agent_start_locations[i] = {loc, 0};
        starts[i] = State(loc, 0, 0);
        cout << loc;
        beg++;
        for (int j = 0; j < num_landmarks; j++, beg++)
        {
            auto loc = atoi((*beg).c_str());
            task_queue[i].emplace_back(task_id++, loc, 0, i);
            cout << " -> " << loc;
        }
        cout << endl;
    }
    myfile.close();

    return true;
}


void FixedAssignSystem::update_tasks()
{
    for (int k = 0; k < num_of_agents; k++)
    {
        while (assigned_tasks[k].size() < num_tasks_reveal && !task_queue[k].empty())
        {
            Task task = task_queue[k].front();
            task_queue[k].pop_front();
            assigned_tasks[k].push_back(task);
            events[k].push_back(make_tuple(task.task_id,timestep,"assigned"));
            all_tasks.push_back(task);
            log_event_assigned(k, task.task_id, timestep);
        }
    }
}


void TaskAssignSystem::update_tasks()
{
    for (int k = 0; k < num_of_agents; k++)
    {
        while (assigned_tasks[k].size() < num_tasks_reveal && !task_queue.empty())
        {
            std::cout << "assigned task " << task_queue.front().task_id <<
                " with loc " << task_queue.front().location << " to agent " << k << std::endl;
            Task task = task_queue.front();
            task.t_assigned = timestep;
            task.agent_assigned = k;
            task_queue.pop_front();
            assigned_tasks[k].push_back(task);
            events[k].push_back(make_tuple(task.task_id,timestep,"assigned"));
            all_tasks.push_back(task);
            log_event_assigned(k, task.task_id, timestep);
        }
    }
}


void InfAssignSystem::update_tasks(){
    for (int k = 0; k < num_of_agents; k++)
    {
        while (assigned_tasks[k].size() < num_tasks_reveal) 
        {
            int i = task_counter[k] * num_of_agents + k;
            int loc = tasks[i%tasks_size];
            Task task(task_id,loc,timestep,k);
            assigned_tasks[k].push_back(task);
            events[k].push_back(make_tuple(task.task_id,timestep,"assigned"));
            log_event_assigned(k, task.task_id, timestep);
            all_tasks.push_back(task);
            task_id++;
            task_counter[k]++;
        }
    }
}

