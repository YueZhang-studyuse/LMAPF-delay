#include "DelaySimulation.h"



void SimulateLDF::init(vector<Path> curr_commit)  
{
    cout<<"Simulate LDF window size:"<< window_size <<endl;
    unexecuted_path.resize(curr_commit.size());
    simulated_path.resize(curr_commit.size());
    agents.resize(curr_commit.size());

    for (int agent = 0; agent < unexecuted_path.size(); agent++)
    {
        if (simulate_time < curr_commit[agent].size()-1)
            simulate_time = curr_commit[agent].size()-1; //only simulate to the maximum timestep of the original path
        
        for (auto p: curr_commit[agent])
        {
            unexecuted_path[agent].push_back(p.location); //read every loc of path
        }
        simulated_path[agent].push_back(unexecuted_path[agent].front()); //start locations
        unexecuted_path[agent].pop_front();
        agents[agent] = new Agent(agent);
    }
}

void SimulateLDF::simulate(const vector<vector<bool>> & delays)
{
    cout<<"Start "<<(float)clock()/(float)CLOCKS_PER_SEC<<endl;
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
        }
        vector<int> order;
        order.resize(agents.size());
        // set order
        std::iota(order.begin(), order.end(), 0);
        std::sort(order.begin(), order.end(),
            [&](int i, int j) { return agents[i]->delay < agents[j]->delay; });
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
                //proceed
                if (agents[a]->v_next == unexecuted_path[a].front())
                {
                    simulated_path[a].push_back(agents[a]->v_next);
                    unexecuted_path[a].pop_front();
                }
                else
                {
                    simulated_path[a].push_back(agents[a]->v_next);
                    agents[a]->delay++;
                }
            }
        }
        else
        {
            for (int a = 0; a < agents.size(); a++) //we can simply skip direct delay/direct wait from execution
            {
                if (agents[a]->v_next != unexecuted_path[a].front())
                {
                    //cout<<"delay exe 1 "<<a<<endl;
                    simulated_path[a].push_back(agents[a]->v_next);
                    agents[a]->delay++;
                }
                else if (agents[a]->v_next == agents[a]->v_now)
                {
                    //cout<<"wait exe 1 "<<a<<endl;
                    simulated_path[a].push_back(agents[a]->v_next);
                    unexecuted_path[a].pop_front();
                }
            }
            for (int a = 0; a < agents.size(); a++) //delay all affect agents
            {
                if (delays[t][a] && simulated_path[a].size()-1 <= t)
                {
                    //cout<<"delay exe"<<endl;
                    delayAgents(a,t);
                }
            }
            for (int a = 0; a < agents.size(); a++)
            {
                if (simulated_path[a].size()-1 <= t) //not affected agents, go ahead
                {
                    //cout<<"move exe"<<endl;
                    simulated_path[a].push_back(agents[a]->v_next);
                    unexecuted_path[a].pop_front();
                }
            }
        }
    }

    cout<<"Simulation done at "<<(float)clock()/(float)CLOCKS_PER_SEC << endl;
    return;
}


bool SimulateLDF::funcPIBT(Agent* ai) //return move or not
{
    int from = ai->v_now;
    int to;
    if (unexecuted_path[ai->id].empty())
        to = ai->v_now; //stay at target
    else
        to = unexecuted_path[ai->id].front();
    //cout<<"check for "<<ai->id<<" "<<from<<" "<<to<<endl;
    if (occupy_next.find(to) != occupy_next.end()) //higher agent already deceide to occupy
    {
        //cannot move, we make the decesion of wait
        ai->v_next = from;
        occupy_next[from] = ai->id;
        //cout<<"delay"<<endl;
        return false;
    }
    if (from == to) //agent direct wait
    {
        ai->v_next = from;
        occupy_next[from] = ai->id;
        //cout<<"direct wait"<<endl;
        return false;
    }
    if (occupy_now.find(to) == occupy_now.end()) //can simply move
    {
        ai->v_next = to;
        occupy_next[to] = ai->id;
        return true;
    }
    //try to move to location other agent occupy now
    int aj = occupy_now[to];

    //if edge conflict, wait
    if (agents[aj]->v_next == -1 && unexecuted_path[aj].front() == from)
    {
        //wait
        ai->v_next = from;
        occupy_next[from] = ai->id;
        //cout<<"edge conflict"<<endl;
        return false;
    }
    //secure move
    ai->v_next = to;
    occupy_next[to] = ai->id;
    //aj can move in eailier decesion
    if (agents[aj]->v_next != -1) 
    {
        return true;
    }
    //aj can move by current checking
    if (funcPIBT(agents[aj]))
    {
        return true;
    } 
    //aj cannot move after backtracking
    ai->v_next = from;
    occupy_next[from] = ai->id;
    //cout<<"cannot move"<<endl;
    return false;
}

void SimulateLDF::delayAgents(int a, int t)
{
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