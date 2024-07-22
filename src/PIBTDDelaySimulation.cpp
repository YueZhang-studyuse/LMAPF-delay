#include "PIBTDDelaySimulation.h"



void SimulatePIBTD::init(vector<Path> curr_commit)  
{
    cout<<"Simulate PIBT window size:"<< window_size <<endl;
    simulated_path.resize(curr_commit.size());
    time_dependent_path.resize(curr_commit.size());
    agents.resize(curr_commit.size());

    for (int agent = 0; agent < curr_commit.size(); agent++)
    {
        if (simulate_time < curr_commit[agent].size()-1)
            simulate_time = curr_commit[agent].size()-1; //only simulate to the maximum timestep of the original path
        
        for (auto p: curr_commit[agent])
        {
            time_dependent_path[agent].push_back(p.location); //read every loc of path
        }
        simulated_path[agent].push_back(time_dependent_path[agent].front()); //start locations
        agents[agent] = new Agent(agent);
    }
}

void SimulatePIBTD::simulate(const vector<vector<bool>> & delays)
{
    cout<<"Start "<<(float)clock()/(float)CLOCKS_PER_SEC<<endl;
    // for (int a = 0; a < agents.size(); a++)
    // {
    //     cout<<"agent a "<<a<<" path: ";
    //     for (auto p: unexecuted_path[a])
    //     {
    //         cout<<p<<" ";
    //     }
    //     cout<<endl;
    // }
    for (int t = 0; t < simulate_time; t++)
    {
        cout<<"simulating t = "<<t<<endl;
        //update agent information
        occupy_now.clear();
        occupy_next.clear();
        for (int a = 0; a < agents.size(); a++)//clear previous information
        {
            agents[a]->v_now = simulated_path[a].back();
            agents[a]->v_next = -1;
            agents[a]->internal_time = t;
            occupy_now[simulated_path[a].back()] = a;
        }

        vector<int> order;
        order.resize(agents.size());
        // set order
        std::iota(order.begin(), order.end(), 0);
        std::sort(order.begin(), order.end(),
            [&](int i, int j) 
            {
                if (instance.getAllpairDistance(agents[i]->v_now,time_dependent_path[agents[i]->id][t]) == instance.getAllpairDistance(agents[j]->v_now,time_dependent_path[agents[j]->id][t]))
                    return agents[i]->delay < agents[j]->delay; 
                return instance.getAllpairDistance(agents[i]->v_now,time_dependent_path[agents[i]->id][t]) < instance.getAllpairDistance(agents[j]->v_now,time_dependent_path[agents[j]->id][t]);
            });

        //exec policy
        for (auto k : order) 
        {
            auto a = agents[k];
            if (a->v_next == -1)
            {
                funcPIBT(a,true);
            }
        }

        //simulate moves
        //simulate without delay, simply add decesion
        if (t >= delays.size())
        {
            for (int a = 0; a < agents.size(); a++)
            {
                simulated_path[a].push_back(agents[a]->v_next);
            }
        }
        else
        {
            for (int a = 0; a < agents.size(); a++) //we can simply skip direct delay/direct wait from execution
            {
                if (agents[a]->v_next == agents[a]->v_now)
                {
                    simulated_path[a].push_back(agents[a]->v_next);
                }
            }
            for (int a = 0; a < agents.size(); a++) //delay all affect agents
            {
                if (delays[t][a] && simulated_path[a].size()-1 <= t)
                {
                    //cout<<"delay exe "<<a<<endl;
                    delayAgents(a,t);
                }
            }
            for (int a = 0; a < agents.size(); a++)
            {
                if (simulated_path[a].size()-1 <= t) //not affected agents, go ahead
                {
                    //cout<<"move exe"<<endl;
                    simulated_path[a].push_back(agents[a]->v_next);
                }
            }
        }
    }
    // for (int a = 0; a < agents.size(); a++)
    // {
    //     cout<<"agent a "<<a<<" path: ";
    //     for (auto p: simulated_path[a])
    //     {
    //         cout<<p<<" ";
    //     }
    //     cout<<endl;
    // }

    cout<<"Simulation done at "<<(float)clock()/(float)CLOCKS_PER_SEC << endl;
    return;
}


bool SimulatePIBTD::funcPIBT(Agent* ai, bool first) //return move or not
{
    if (ai->id <= 5)
        cout<<"pibt for "<<ai->id<<" curr "<<ai->v_now<<endl;
    int from = ai->v_now;
    auto neighbor = instance.getNeighbors(from);
    vector<int> next;
    for (auto loc: neighbor)
        next.push_back(loc);
    if (first)
        next.push_back(ai->v_now);
    int loc_on_guidance = time_dependent_path[ai->id][ai->internal_time+1];

    std::sort(next.begin(), next.end(),
        [&](int i, int j) 
        {
            if (instance.getAllpairDistance(i,loc_on_guidance) != instance.getAllpairDistance(j,loc_on_guidance))
                return instance.getAllpairDistance(i,loc_on_guidance) < instance.getAllpairDistance(j,loc_on_guidance); 
            else
                return instance.getAllpairDistance(i,time_dependent_path[ai->id].back()) < instance.getAllpairDistance(j,time_dependent_path[ai->id].back());
        });

    for (int i = 0; i < next.size(); i++)
    {
        int to = next[i];
        // if (ai->id <= 5)
        //     cout<<"considering "<<to<<endl;
        if (occupy_next.find(to) != occupy_next.end()) //vertex conflict
        {
            // if (ai->id <= 5)
            //     cout<<"vertex conflict with "<<occupy_next[to]<<endl;
            continue;
        }
        if (occupy_now.find(to) != occupy_now.end() && agents[occupy_now[to]]->v_next == from) //swap conflict
        {
            // if (ai->id <= 5)
            //     cout<<"edge conflict with "<<occupy_now[to]<<endl;
            continue;
        }

        occupy_next[to] = ai->id;
        ai->v_next = to;

        if (to == from)
        {
            return false;
        }

        if (occupy_now.find(to) == occupy_now.end() || to == from)
            return true;
        if (occupy_now.find(to) != occupy_now.end() && occupy_now[to] != ai->id && agents[occupy_now[to]]->v_next == -1 && !funcPIBT(agents[occupy_now[to]],false))
        {
            // if (ai->id <= 5)
            //     cout<<"push faied "<<endl;
            continue;
        }
        return true;
    }

    //aj cannot move after backtracking
    ai->v_next = from;
    occupy_next[from] = ai->id;
    // if (ai->id <= 5)
    //     cout<<"false "<<ai->id<<endl;
    return false;
}

void SimulatePIBTD::delayAgents(int a, int t)
{
    cout<<"delaying "<<a<<" at "<<t<<endl;
    //delay itself
    agents[a]->v_next = agents[a]->v_now;
    simulated_path[a].push_back(agents[a]->v_next);
    agents[a]->delay++;
    //delay other agent that want a's waiting location
    if (occupy_next.find(agents[a]->v_next) != occupy_next.end() && simulated_path[occupy_next[agents[a]->v_next]].size()-1 <= t)
    {
        delayAgents(occupy_next[agents[a]->v_next],t);
    }
    occupy_next[agents[a]->v_next] = a;
    return;
}