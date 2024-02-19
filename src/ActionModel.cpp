#include "ActionModel.h"


std::ostream& operator<<(std::ostream &stream, const Action &action)
{
    // if (action == Action::FW) {
    //     stream << "F";
    // } else if (action == Action::CR) {
    //     stream << "R";
    // } else if (action == Action::CCR) {
    //     stream << "C";
    // } else {
    //     stream << "W";
    // }
    if (action == Action::N) {
        stream << "NO";
    } else if (action == Action::E) {
        stream << "EA";
    } else if (action == Action::S) {
        stream << "SO";
    } else if (action == Action::WE) {
        stream << "WE";
    } else {
        stream << "WA";
    }

    return stream;
}


bool ActionModel::is_valid(const vector<State>& prev, const vector<Action> & actions)
{
    if (prev.size() != actions.size())
    {
        errors.push_back(make_tuple("incorrect vector size",-1,-1,prev[0].timestep+1));
        return false;
    }

    vector<State> next = result_states(prev, actions);
    unordered_map<int, int> vertex_occupied;
    unordered_map<pair<int, int>, int> edge_occupied;

    for (int i = 0; i < prev.size(); i ++) 
    {
        //cout<<"locations validation "<<i<<" "<<prev[i].location<<" "<<next[i].location<<endl;
        if (next[i].location < 0 || next[i].location >= grid.map.size() || 
            (abs(next[i].location / cols - prev[i].location/cols) + abs(next[i].location % cols - prev[i].location %cols) > 1 ))
        {
            cout << "ERROR: agent " << i << " moves out of map size. " << endl;
            errors.push_back(make_tuple("unallowed move",i,-1,next[i].timestep));
            return false;
        }

        if (grid.map[next[i].location] == 1)
        {
            cout << "ERROR: agent " << i << " moves to an obstacle. " << endl;
            errors.push_back(make_tuple("unallowed move",i,-1,next[i].timestep));
            return false;
        }


        if (vertex_occupied.find(next[i].location) != vertex_occupied.end()) {
            cout << "ERROR: agents " << i << " and " << vertex_occupied[next[i].location] << " have a vertex conflict. " << endl;
            errors.push_back(make_tuple("vertex conflict",i,vertex_occupied[next[i].location], next[i].timestep));
            return false;
        }

        int edge_idx = (prev[i].location + 1) * rows * cols +  next[i].location;

        if (edge_occupied.find({prev[i].location, next[i].location}) != edge_occupied.end()) {
            cout << "ERROR: agents " << i << " and " << edge_occupied[{prev[i].location, next[i].location}] << " have an edge conflict. " << endl;
            errors.push_back(make_tuple("edge conflict", i, edge_occupied[{prev[i].location, next[i].location}], next[i].timestep));
            return false;
        }
        

        vertex_occupied[next[i].location] = i;
        int r_edge_idx = (next[i].location + 1) * rows * cols +  prev[i].location;
        edge_occupied[{next[i].location, prev[i].location}] = i;
    }

    return true;
}

vector<State> ActionModel::result_states_with_delays(const vector<State>& prev, const vector<Action> & action, const vector<bool> & delays) //simple mcp with only one step
{
    unordered_map<int,bool> simulated_agents; //to store agents that already simulated and can make move or not
    vector<list<int>> orders;
    orders.resize(grid.map.size());
    for(size_t i = 0 ; i < prev.size(); i ++)
    {
        orders[prev[i].location].push_back(i);
    }

    vector<State> next(prev.size());
    for (size_t i = 0 ; i < prev.size(); i ++)
    {
        next[i] = result_state(prev[i], action[i]); 
        orders[next[i].location].push_back(i);
    }

    for (size_t i = 0 ; i < next.size(); i++)
    {
        if (simulated_agents.find(i) != simulated_agents.end())
            continue;
        mcp_simulate(i,prev,next,orders,simulated_agents,delays);
    }
    for (size_t i = 0 ; i < next.size(); i++)
    {
        if (!simulated_agents[i])
        {
            cout<<"delaied "<<i<<endl;
            next[i].location = prev[i].location;
        }
    }
    return next;
}

bool ActionModel::mcp_simulate(int agent, const vector<State>& prev, vector<State>& next, vector<list<int>> & orders, unordered_map<int,bool> & simulated_agents,const vector<bool> & delays)
{
    int from = prev[agent].location;
    int to = next[agent].location;

    if (delays[agent]) //getting delaied directly, cannot make this move
    {
        cout<<"mcp delay "<<agent<<endl;
        simulated_agents[agent] = false; //also do not clear the order because we wait
        return false;
    }
    if (orders[to].front() == agent) //do not delaied and can make the move directly 
    {
        //cout<<"mcp go "<<agent<<endl;
        simulated_agents[agent] = true;
        if (orders[from].front() != agent)
        {
            cout<<"wrong first order"<<endl;
        }
        orders[from].pop_front(); //clear from
        return true;
    }
    if (orders[to].front() != agent && simulated_agents.find(orders[to].front())!= simulated_agents.end()) //the occupied agents simulated already
    {
        cout<<"mcp result delay "<<agent<<" due to "<<orders[to].front()<<endl;
        simulated_agents[agent] = false;
        return false;
    }
    //no recursive now
    // else
    // {
    //     cout<<"mcp result delay "<<agent<<" due to "<<orders[to].front()<<endl;
    //     simulated_agents[agent] = false;
    //     return;
    // }
    if (orders[to].front() != agent && simulated_agents.find(orders[to].front())== simulated_agents.end()) //the occupied agents not simulated yet
    {
        //pretend this agent can move
        cout<<"mcp recursive from agent "<<agent<<" to "<<orders[to].front()<<endl;
        orders[from].pop_front(); //clear from
        auto succ = mcp_simulate(orders[to].front(),prev,next,orders,simulated_agents,delays);
        if (succ)
        {
            simulated_agents[agent] = true;
            return true;
        }
        else
        {
            simulated_agents[agent] = false;
            orders[from].push_back(agent);
            return false;
        }
    }
}
