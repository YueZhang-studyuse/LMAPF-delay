#include "PIBTDelaySimulation.h"



void SimulatePIBT::init(vector<Path> curr_commit)  
{
    cout<<"Simulate PIBT window size:"<< window_size <<endl;
    simulated_path.resize(curr_commit.size());
    time_independent_path.resize(curr_commit.size());
    agents.resize(curr_commit.size());

    for (int agent = 0; agent < curr_commit.size(); agent++)
    {
        if (simulate_time < curr_commit[agent].size()-1)
            simulate_time = curr_commit[agent].size()-1; //only simulate to the maximum timestep of the original path
        
        for (auto p: curr_commit[agent])
        {
            if (!time_independent_path[agent].empty() && p.location == time_independent_path[agent].back()) //only time independent path
                continue;
            time_independent_path[agent].push_back(p.location); //read every loc of path
        }
        simulated_path[agent].push_back(time_independent_path[agent].front()); //start locations
        agents[agent] = new Agent(agent);
    }
}

void SimulatePIBT::simulate(const vector<vector<bool>> & delays)
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
            occupy_now[simulated_path[a].back()] = a;
            agents[a]->priority++;
            if (agents[a]->v_now == time_independent_path[a].back())
                agents[a]->priority = 0;
        }

        vector<int> order;
        order.resize(agents.size());
        // set order
        std::iota(order.begin(), order.end(), 0);
        std::sort(order.begin(), order.end(),
            [&](int i, int j) 
            { 
                if (agents[i]->priority == agents[i]->priority)
                    return agents[i]->delay < agents[j]->delay; 
                return agents[i]->delay > agents[j]->delay;
            });

        //exec policy
        for (auto k : order) 
        {
            auto a = agents[k];
            if (a->v_next == -1)
            {
                funcPIBT(a);
            }
        }

        //simulate moves
        //simulate without delay, simply add decesion
        if (t >= delays.size())
        {
            for (int a = 0; a < agents.size(); a++)
            {
                simulated_path[a].push_back(agents[a]->v_next);
                agents[a]->internal_time = agents[a]->proposed_time;
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
                    agents[a]->internal_time = agents[a]->proposed_time;
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


bool SimulatePIBT::funcPIBT(Agent* ai) //return move or not
{
    // if (ai->id <= 5)
    //     cout<<"pibt for "<<ai->id<<" curr "<<ai->v_now<<endl;
    int from = ai->v_now;
    auto next = instance.getNeighbors(from);
    vector<pair<int,int>> next_with_time; //loc, time on the time independent path

    //sort based on sic
    if (ai->internal_time >= time_independent_path[ai->id].size()-1)
    {
        for (auto loc: next)
            next_with_time.push_back(make_pair(loc,ai->internal_time+1));
        int goal = time_independent_path[ai->id].back();
        std::sort(next_with_time.begin(), next_with_time.end(),
            [&](pair<int,int> i, pair<int,int> j) 
            {
                if (instance.getAllpairDistance(i.first,goal) != instance.getAllpairDistance(j.first,goal))
                    return instance.getAllpairDistance(i.first,goal) < instance.getAllpairDistance(j.first,goal); 
                else
                    return i.second < j.second;
            });
    }
    else //next loc on the path > nearest node on the rest of path
    {
        for (auto loc: next)
            next_with_time.push_back(make_pair(loc,ai->internal_time+1));
        unordered_map<int,int> dis_to_path;
        for (int i = 0; i < next_with_time.size(); i++)
        {
            int location = next_with_time[i].first;
            if (time_independent_path[ai->id][ai->internal_time] == ai->v_now && time_independent_path[ai->id][ai->internal_time+1] == location) // on guidance path
            {
                dis_to_path[location] = -1; // to make it very high priority
                next_with_time[i].second = ai->internal_time+1;
            }
            // else if (location == from) //stay at current
            // {
            //     dis_to_path[location] = MAX_TIMESTEP;
            //     next_with_time[i].second = ai->internal_time;
            // }
            else //check the nearest goal 
            {
                dis_to_path[location] = MAX_TIMESTEP;
                for (int t = ai->internal_time+1; t < time_independent_path[ai->id].size(); t++)
                {
                    int dis = instance.getAllpairDistance(time_independent_path[ai->id][t],location);
                    if (dis < dis_to_path[location])
                    {
                        dis_to_path[location] = dis;
                        next_with_time[i].second = t;
                    }
                }
            }
            // if (ai->id <= 5)
            //     cout<<"loc "<<location<<" dis "<<dis_to_path[location]<<", ";
        }
        // if (ai->id <= 5)
        //     cout<<endl;
        std::sort(next_with_time.begin(), next_with_time.end(),
            [&](pair<int,int> i, pair<int,int> j) 
            {
                if (dis_to_path[i.first] != dis_to_path[j.first])
                    return dis_to_path[i.first] < dis_to_path[j.first];
                else
                    return i.second < j.second;
            });
    }
    // if (ai->id <= 5)
    // {
    //     for (int i = 0; i < next_with_time.size(); i++)
    //     {
    //             cout<<"considering "<<next_with_time[i].first<<" "<<next_with_time[i].second<<",";
    //     }
    //     cout<<endl;
    // }
    for (int i = 0; i < next_with_time.size(); i++)
    {
        int to = next_with_time[i].first;
        //cout<<"considering "<<to<<endl;
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
        ai->proposed_time = next_with_time[i].second;

        if (occupy_now.find(to) == occupy_now.end() || to == from)
            return true;
        if (occupy_now.find(to) != occupy_now.end() && occupy_now[to] != ai->id && agents[occupy_now[to]]->v_next == -1 && !funcPIBT(agents[occupy_now[to]]))
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
    ai->proposed_time = ai->internal_time;
    //cout<<"false "<<ai->id<<endl;
    return false;
}

void SimulatePIBT::delayAgents(int a, int t)
{
    //cout<<"delaying "<<a<<" at "<<t<<endl;
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