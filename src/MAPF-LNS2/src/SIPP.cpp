#include "SIPP.h"

void SIPP::updatePath(const LLNode* goal, vector<PathEntry> &path)
{
    num_collisions = goal->num_of_conflicts;
	path.resize(max(goal->timestep + 1,commit_window+1));
	// num_of_conflicts = goal->num_of_conflicts;

	const auto* curr = goal;
	while (curr->parent != nullptr) // non-root node
	{
		const auto* prev = curr->parent;
		int t = prev->timestep + 1;
		while (t < curr->timestep)
		{
			path[t].location = prev->location; // wait at prev location
			t++;
		}
		path[curr->timestep].location = curr->location; // move to curr location
		curr = prev;
	}
	assert(curr->timestep == 0);
    for (int i = goal->timestep + 1; i <= commit_window; i++) //fill path within window
    {
        path[i].location = goal->location;
    }
	path[0].location = curr->location;
}

Path SIPP::findPath(const ConstraintTable& constraint_table, double timeout, bool &timeout_flag)
{
    auto start_time = Time::now();
    reset();
    //ReservationTable reservation_table(constraint_table, goal_location);
    ReservationTable reservation_table(constraint_table, goal_location);
    Path path;

    Interval interval = reservation_table.get_first_safe_interval(start_location);
    if (get<0>(interval) > 0) //cannot hold this location at timestep 0
        return path;

    //auto last_target_collision_time = constraint_table.getLastCollisionTimestep(goal_location); only need touched goal, no requirement for stay at goal
    // generate start and add it to the OPEN & FOCAL list
    auto holding_time = 0;
    auto h = get_heuristic(start_location,goal_location);
    for (int i = 0; i < other_goal_locations.size(); i++)
    {
        if (i == 0)
            h+=get_heuristic(goal_location,other_goal_locations[i]);
        else
            h+=get_heuristic(other_goal_locations[i-1],other_goal_locations[i]);
    }

    auto start = new SIPPNode(start_location, 0, h, nullptr, 0, get<1>(interval), get<1>(interval),
                                get<2>(interval), get<2>(interval), (start_location == goal_location));
    pushNodeToFocal(start);

    //cout<<"goal holding time: "<<constraint_table.getHoldingTimeForWindow(goal_location, constraint_table.length_min,commit_window)<<endl; // the agent can hold the this location afterward until window

    while (!focal_list.empty())
    {
        if (((fsec)(Time::now() - start_time)).count() >= timeout)
        {
            timeout_flag = true;
            break;
        }

        auto* curr = focal_list.top();
        focal_list.pop();
        curr->in_openlist = false;
        num_expanded++;
        assert(curr->location >= 0);
        // check if the popped node is a goal
        if (curr->is_goal) //happends in lns2
        {
            updatePath(curr, path);
            break;
        }
        else if ((curr->goal_index > 0 || curr->location == goal_location) && // arrive at the goal location or reach goal before
                //!curr->wait_at_goal && // not wait at the goal location
                //curr->timestep >= constraint_table.getHoldingTimeForWindow(curr->location, constraint_table.length_min,commit_window) //maybe not stay at current location for larger commit
                curr->timestep >= commit_window) // the agent can hold the this location afterward until window
        {
            int future_collisions = 0; //disappear does not require to check future collisions
            if (future_collisions == 0) //agent can stay at goal location
            {
                updatePath(curr, path);
                break;
            }
            // generate a goal node
            auto goal = new SIPPNode(*curr);
            goal->is_goal = true;
            goal->h_val = 0;
            goal->num_of_conflicts += future_collisions;
            // try to retrieve it from the hash table
            if (dominanceCheck(goal))
                pushNodeToFocal(goal);
            else
                delete goal;
        }

        for (int next_location : instance.getNeighbors(curr->location)) // move to neighboring locations
        {
            for (auto & i : reservation_table.get_safe_intervals(curr->location, next_location, curr->timestep + 1, curr->high_expansion + 1))
            {
                int next_high_generation, next_timestep, next_high_expansion;
                bool next_v_collision, next_e_collision;
                tie(next_high_generation, next_timestep, next_high_expansion, next_v_collision, next_e_collision) = i;
                auto holding_time = 0;
                auto next_collisions = curr->num_of_conflicts + (int)next_v_collision + (int)next_e_collision;
                auto next_h_val = max(get_heuristic(next_location,goal_location), (next_collisions > 0? holding_time : curr->getFVal()) - next_timestep); // path max
                
                int num_goal_reached = curr->goal_index;
                
                if (curr->goal_index > 0)
                {
                    next_h_val = get_heuristic(next_location,other_goal_locations[curr->goal_index-1]);
                    if (next_location == other_goal_locations[curr->goal_index-1])
                        num_goal_reached++;
                }
                else if (next_location == goal_location)
                    num_goal_reached++;
                
                //add heursitics for all other goals
                for (int i = 0; i < other_goal_locations.size(); i++)
                {
                    if (curr->goal_index > i)
                        continue;
                    if (i == 0)
                        next_h_val+=get_heuristic(goal_location,other_goal_locations[i]);
                    else
                        next_h_val+=get_heuristic(other_goal_locations[i-1],other_goal_locations[i]);
                }

                int goal_reached = curr->goal_index;
                
                // generate (maybe temporary) node
                auto next = new SIPPNode(next_location, next_timestep, next_h_val, curr, next_timestep,
                                         next_high_generation, next_high_expansion, next_v_collision, next_collisions, num_goal_reached);
                // try to retrieve it from the hash table
                if (dominanceCheck(next))
                {
                    pushNodeToFocal(next);
                    //cout<<"pushed "<<next->reached_goal_at<<endl;
                }
                else
                {
                    delete next;
                }
            }
        }  // end for loop that generates successors
        
        // wait at the current location
        if (curr->high_expansion == curr->high_generation and
            reservation_table.find_safe_interval(interval, curr->location, curr->high_expansion) and
                get<0>(interval) + curr->h_val <= reservation_table.constraint_table.length_max)
        {
            auto next_timestep = get<0>(interval);
            auto next_h_val = max(get_heuristic(curr->location,goal_location), (get<2>(interval) ? holding_time : curr->getFVal()) - next_timestep); // path max
            int num_goal_reached = curr->goal_index;
                
            if (curr->goal_index > 0)
            {
                next_h_val = get_heuristic(curr->location,other_goal_locations[curr->goal_index-1]);
            }
            
            //add heursitics for all other goals
            for (int i = 0; i < other_goal_locations.size(); i++)
            {
                if (curr->goal_index-1 >= i)
                    continue;
                if (i == 0)
                    next_h_val+=get_heuristic(goal_location,other_goal_locations[i]);
                else
                    next_h_val+=get_heuristic(other_goal_locations[i-1],other_goal_locations[i]);
            }

            auto next_collisions = curr->num_of_conflicts +
                    // (int)curr->collision_v * max(next_timestep - curr->timestep - 1, 0) +
		    (int)get<2>(interval);
            auto next = new SIPPNode(curr->location, next_timestep, next_h_val, curr, next_timestep,
                                     get<1>(interval), get<1>(interval), get<2>(interval),
                                     next_collisions, curr->goal_index);
            //next->wait_at_goal = (curr->location == goal_location);
            next->wait_at_goal = (curr->location == goal_location);
            if (dominanceCheck(next))
                pushNodeToFocal(next);
            else
                delete next;
        }
    }  // end while loop
    releaseNodes();
    return path;
}

Path SIPP::findPath(const ConstraintTable& constraint_table)
{
    reset();
    //ReservationTable reservation_table(constraint_table, goal_location);
    ReservationTable reservation_table(constraint_table, goal_location);
    Path path;

    Interval interval = reservation_table.get_first_safe_interval(start_location);
    if (get<0>(interval) > 0) //cannot hold this location at timestep 0
        return path;

    //auto last_target_collision_time = constraint_table.getLastCollisionTimestep(goal_location); only need touched goal, no requirement for stay at goal
    // generate start and add it to the OPEN & FOCAL list
    auto holding_time = 0;
    auto h = get_heuristic(start_location,goal_location);

    auto start = new SIPPNode(start_location, 0, h, nullptr, 0, get<1>(interval), get<1>(interval),
                                get<2>(interval), get<2>(interval), (start_location == goal_location));
    pushNodeToFocal(start);

    //cout<<"goal holding time: "<<constraint_table.getHoldingTimeForWindow(goal_location, constraint_table.length_min,commit_window)<<endl; // the agent can hold the this location afterward until window

    while (!focal_list.empty())
    {
        auto* curr = focal_list.top();
        focal_list.pop();
        curr->in_openlist = false;
        num_expanded++;
        assert(curr->location >= 0);
        // check if the popped node is a goal
        if (curr->is_goal) //happends in lns2
        {
            updatePath(curr, path);
            break;
        }
        else if ((curr->goal_index > 0 || curr->location == goal_location) && // arrive at the goal location or reach goal before
                !curr->wait_at_goal && // not wait at the goal location
                curr->timestep >= constraint_table.getHoldingTimeForWindow(curr->location, constraint_table.length_min,commit_window)) // the agent can hold the this location afterward until window
        {
            int future_collisions = 0; //disappear does not require to check future collisions
            if (future_collisions == 0) //agent can stay at goal location
            {
                updatePath(curr, path);
                break;
            }
            // generate a goal node
            auto goal = new SIPPNode(*curr);
            goal->is_goal = true;
            goal->h_val = 0;
            goal->num_of_conflicts += future_collisions;
            // try to retrieve it from the hash table
            if (dominanceCheck(goal))
                pushNodeToFocal(goal);
            else
                delete goal;
        }

        for (int next_location : instance.getNeighbors(curr->location)) // move to neighboring locations
        {
            for (auto & i : reservation_table.get_safe_intervals(
                    curr->location, next_location, curr->timestep + 1, curr->high_expansion + 1))
            {
                int next_high_generation, next_timestep, next_high_expansion;
                bool next_v_collision, next_e_collision;
                tie(next_high_generation, next_timestep, next_high_expansion, next_v_collision, next_e_collision) = i;
                auto holding_time = 0;
                auto next_collisions = curr->num_of_conflicts +
                                      (int)next_v_collision + (int)next_e_collision;

                auto next_h_val = max(get_heuristic(next_location,goal_location), (next_collisions > 0?
                    holding_time : curr->getFVal()) - next_timestep); // path max
                
                int num_goal_reached = curr->goal_index;
                
                if (curr->goal_index > 0)
                {
                    next_h_val = 0; //go to any location is fine
                    if (next_location == other_goal_locations[curr->goal_index-1])
                        num_goal_reached++;
                }
                else if (next_location == goal_location)
                    num_goal_reached++;
                
                //add heursitics for all other goals
                for (int i = 0; i < other_goal_locations.size(); i++)
                {
                    if (curr->goal_index-1 >= i)
                        continue;
                    next_h_val+=get_heuristic(next_location,other_goal_locations[i]);
                }

                int goal_reached = curr->goal_index;
                
                // generate (maybe temporary) node
                auto next = new SIPPNode(next_location, next_timestep, next_h_val, curr, next_timestep,
                                         next_high_generation, next_high_expansion, next_v_collision, next_collisions, num_goal_reached);
                // try to retrieve it from the hash table
                if (dominanceCheck(next))
                {
                    pushNodeToFocal(next);
                    //cout<<"pushed "<<next->reached_goal_at<<endl;
                }
                else
                {
                    delete next;
                }
            }
        }  // end for loop that generates successors
        
        // wait at the current location
        if (curr->high_expansion == curr->high_generation and
            reservation_table.find_safe_interval(interval, curr->location, curr->high_expansion) and
                get<0>(interval) + curr->h_val <= reservation_table.constraint_table.length_max)
        {
            auto next_timestep = get<0>(interval);
            auto next_h_val = max(get_heuristic(curr->location,goal_location), (get<2>(interval) ? holding_time : curr->getFVal()) - next_timestep); // path max
            int num_goal_reached = curr->goal_index;
                
            if (curr->goal_index > 0)
            {
                next_h_val = get_heuristic(curr->location,other_goal_locations[curr->goal_index-1]);
            }
            
            //add heursitics for all other goals
            for (int i = 0; i < other_goal_locations.size(); i++)
            {
                if (curr->goal_index-1 >= i)
                    continue;
                if (i == 0)
                    next_h_val+=get_heuristic(goal_location,other_goal_locations[i]);
                else
                    next_h_val+=get_heuristic(other_goal_locations[i-1],other_goal_locations[i]);
            }

            auto next_collisions = curr->num_of_conflicts +
                    // (int)curr->collision_v * max(next_timestep - curr->timestep - 1, 0) +
		    (int)get<2>(interval);
            auto next = new SIPPNode(curr->location, next_timestep, next_h_val, curr, next_timestep,
                                     get<1>(interval), get<1>(interval), get<2>(interval),
                                     next_collisions, curr->goal_index);
            next->wait_at_goal = (curr->location == goal_location);
            if (dominanceCheck(next))
                pushNodeToFocal(next);
            else
                delete next;
        }
    }  // end while loop
    releaseNodes();
    return path;
}


// // find path by A*
// // Returns a path that minimizes the collisions with the paths in the path table, breaking ties by the length
// Path SIPP::findPath(const ConstraintTable& constraint_table)
// {
//     reset();
//     ReservationTable reservation_table(constraint_table, goal_location);
//     Path path;
//     Interval interval = reservation_table.get_first_safe_interval(start_location);
//     if (get<0>(interval) > 0)
//     {
//         return path;
//     }
//     auto holding_time = 0;
//     //constraint_table.getHoldingTimeForWindow(goal_location, constraint_table.length_min,commit_window);

//     auto last_target_collision_time = constraint_table.getLastCollisionTimestep(goal_location);
//     // generate start and add it to the OPEN & FOCAL list
//     //auto h = max(max(my_heuristic[start_location], holding_time), last_target_collision_time + 1);
//     auto h = max(max(get_heuristic(start_location,goal_location), holding_time), last_target_collision_time + 1);

//     auto start = new SIPPNode(start_location, 0, h, nullptr, 0, get<1>(interval), get<1>(interval),
//                                 get<2>(interval), get<2>(interval));
//     pushNodeToFocal(start);

//     while (!focal_list.empty())
//     {
//         auto* curr = focal_list.top();
//         focal_list.pop();
//         curr->in_openlist = false;
//         num_expanded++;
//         assert(curr->location >= 0);
//         // check if the popped node is a goal
//         if (curr->is_goal)
//         {
//             updatePath(curr, path);
//             break;
//         }
//         else if ((curr->reached_goal || curr->location == goal_location) && // arrive at the goal location or reach goal before
//                 !curr->wait_at_goal && // not wait at the goal location
//                 curr->timestep >= constraint_table.getHoldingTimeForWindow(curr->location, constraint_table.length_min,commit_window)) // the agent can hold the this location afterward until window
//         {
//             int future_collisions = 0; //disappear does not require to check future collisions

//             if (future_collisions == 0)
//             {
//                 updatePath(curr, path);
//                 break;
//             }
//             // generate a goal node
//             auto goal = new SIPPNode(*curr);
//             goal->is_goal = true;
//             goal->h_val = 0;
//             goal->num_of_conflicts += future_collisions;
//             // try to retrieve it from the hash table
//             if (dominanceCheck(goal))
//                 pushNodeToFocal(goal);
//             else
//                 delete goal;
//         }

//         for (int next_location : instance.getNeighbors(curr->location)) // move to neighboring locations
//         {
//             for (auto & i : reservation_table.get_safe_intervals(
//                     curr->location, next_location, curr->timestep + 1, curr->high_expansion + 1))
//             {
//                 int next_high_generation, next_timestep, next_high_expansion;
//                 bool next_v_collision, next_e_collision;
//                 tie(next_high_generation, next_timestep, next_high_expansion, next_v_collision, next_e_collision) = i;

//                 //if (next_timestep + my_heuristic[next_location] > constraint_table.length_max)
//                 if (next_timestep + get_heuristic(next_location,goal_location) > constraint_table.length_max)
//                     break;

//                 auto next_collisions = curr->num_of_conflicts +
//                                       (int)next_v_collision + (int)next_e_collision;
//                 //auto next_h_val = max(my_heuristic[next_location], (next_collisions > 0?
//                 auto next_h_val = max(get_heuristic(next_location,goal_location), (next_collisions > 0?
//                     holding_time : curr->getFVal()) - next_timestep); // path max
//                 // generate (maybe temporary) node
//                 auto next = new SIPPNode(next_location, next_timestep, next_h_val, curr, next_timestep,
//                                          next_high_generation, next_high_expansion, next_v_collision, next_collisions);
//                 // try to retrieve it from the hash table
//                 if (dominanceCheck(next))
//                 {
//                     pushNodeToFocal(next);
//                 }
//                 else
//                     delete next;
//             }
//         }  // end for loop that generates successors
        
//         // wait at the current location
//         if (curr->high_expansion == curr->high_generation and
//             reservation_table.find_safe_interval(interval, curr->location, curr->high_expansion) and
//                 get<0>(interval) + curr->h_val <= reservation_table.constraint_table.length_max)
//         {
//             auto next_timestep = get<0>(interval);
//             //auto next_h_val = max(my_heuristic[curr->location], (get<2>(interval) ? holding_time : curr->getFVal()) - next_timestep); // path max
//             auto next_h_val = max(get_heuristic(curr->location,goal_location), (get<2>(interval) ? holding_time : curr->getFVal()) - next_timestep); // path max
//             auto next_collisions = curr->num_of_conflicts +
//                     // (int)curr->collision_v * max(next_timestep - curr->timestep - 1, 0) +
// 		    (int)get<2>(interval);
//             auto next = new SIPPNode(curr->location, next_timestep, next_h_val, curr, next_timestep,
//                                      get<1>(interval), get<1>(interval), get<2>(interval),
//                                      next_collisions);
//             next->wait_at_goal = (curr->location == goal_location);
//             if (dominanceCheck(next))
//                 pushNodeToFocal(next);
//             else
//                 delete next;
//         }
//     }  // end while loop
//     releaseNodes();
//     return path;
// }

// TODO:: currently this is implemented in SIPP inefficiently
int SIPP::getTravelTime(int start, int end, const ConstraintTable& constraint_table, int upper_bound)
{
    reset();
    min_f_val = -1; // this disables focal list
    int length = MAX_TIMESTEP;
    //auto root = new SIPPNode(start, 0, compute_heuristic(start, end), nullptr, 0, 1, 1, 0, 0);
    auto root = new SIPPNode(start, 0, get_heuristic(start, end), nullptr, 0, 1, 1, 0, 0);
    pushNodeToOpenAndFocal(root);
    auto static_timestep = constraint_table.getMaxTimestep(); // everything is static after this timestep
    while (!open_list.empty())
    {
        auto curr = open_list.top(); open_list.pop();
        if (curr->location == end)
        {
            length = curr->g_val;
            break;
        }
        list<int> next_locations = instance.getNeighbors(curr->location);
        next_locations.emplace_back(curr->location);
        for (int next_location : next_locations)
        {
            int next_timestep = curr->timestep + 1;
            int next_g_val = curr->g_val + 1;
            if (static_timestep <= curr->timestep)
            {
                if (curr->location == next_location)
                {
                    continue;
                }
                next_timestep--;
            }
            if (!constraint_table.constrained(next_location, next_timestep) &&
                !constraint_table.constrained(curr->location, next_location, next_timestep))
            {  // if that grid is not blocked
                int next_h_val = get_heuristic(next_location, end);
                if (next_g_val + next_h_val >= upper_bound) // the cost of the path is larger than the upper bound
                    continue;
                auto next = new SIPPNode(next_location, next_g_val, next_h_val, nullptr, next_timestep,
                                         next_timestep + 1, next_timestep + 1, 0, 0);
                if (dominanceCheck(next))
                    pushNodeToOpenAndFocal(next);
                else
                    delete next;
            }
        }
    }
    releaseNodes();
    num_expanded = 0;
    num_generated = 0;
    num_reopened = 0;
    return length;
}

void SIPP::updateFocalList()
{
	auto open_head = open_list.top();
	if (open_head->getFVal() > min_f_val)
	{
		int new_min_f_val = (int)open_head->getFVal();
		for (auto n : open_list)
		{
			if (n->getFVal() > w * min_f_val && n->getFVal() <= w * new_min_f_val)
				n->focal_handle = focal_list.push(n);
		}
		min_f_val = new_min_f_val;
	}
}

inline void SIPP::pushNodeToOpenAndFocal(SIPPNode* node)
{
    num_generated++;
	node->open_handle = open_list.push(node);
	node->in_openlist = true;
	if (node->getFVal() <= w * min_f_val)
		node->focal_handle = focal_list.push(node);
    allNodes_table[node].push_back(node);
}
inline void SIPP::pushNodeToFocal(SIPPNode* node)
{
    num_generated++;
    allNodes_table[node].push_back(node);
    node->in_openlist = true;
    node->focal_handle = focal_list.push(node); // we only use focal list; no open list is used
}
inline void SIPP::eraseNodeFromLists(SIPPNode* node)
{
    if (open_list.empty())
    { // we only have focal list
        focal_list.erase(node->focal_handle);
    }
    else if (focal_list.empty())
    {  // we only have open list
        open_list.erase(node->open_handle);
    }
    else
    { // we have both open and focal
        open_list.erase(node->open_handle);
        if (node->getFVal() <= w * min_f_val)
            focal_list.erase(node->focal_handle);
    }
}
void SIPP::releaseNodes()
{
    open_list.clear();
    focal_list.clear();
    for (auto & node_list : allNodes_table)
        for (auto n : node_list.second)
        {
            delete n;
        }
    allNodes_table.clear();
    for (auto n : useless_nodes)
        delete n;
    useless_nodes.clear();
}

void SIPP::printSearchTree() const
{
    vector<list<SIPPNode*>> nodes;
    for (const auto & node_list : allNodes_table)
    {
        for (const auto & n : node_list.second)
        {
            if (nodes.size() <= n->timestep)
                nodes.resize(n->timestep + 1);
            nodes[n->timestep].emplace_back(n);
        }
    }
    cout << "Search Tree" << endl;
    for(int t = 0; t < nodes.size(); t++)
    {
        cout << "t=" << t << ":\t";
        for (const auto & n : nodes[t])
            cout << *n << "[" << n->timestep << "," << n->high_expansion << "),c=" << n->num_of_conflicts << "\t";
        cout << endl;
    }
}

// // return true iff we the new node is not dominated by any old node
// bool SIPP::dominanceCheck(SIPPNode* new_node)
// {
//     auto ptr = allNodes_table.find(new_node);
//     if (ptr == allNodes_table.end())
//         return true;
//     for (auto & old_node : ptr->second)
//     {
//         if (old_node->timestep <= new_node->timestep and
//             old_node->num_of_conflicts <= new_node->num_of_conflicts)
//         { // the new node is dominated by the old node
//             return false;
//         }
//         else if (old_node->timestep >= new_node->timestep and
//                 old_node->num_of_conflicts >= new_node->num_of_conflicts) // the old node is dominated by the new node
//         { // delete the old node
//             if (old_node->in_openlist) // the old node has not been expanded yet
//                 eraseNodeFromLists(old_node); // delete it from open and/or focal lists
//             else // the old node has been expanded already
//                 num_reopened++; //re-expand it
//             useless_nodes.push_back(old_node);
//             ptr->second.remove(old_node);
//             num_generated--; // this is because we later will increase num_generated when we insert the new node into lists.
//             return true;
//         }
//         else if(old_node->timestep < new_node->high_expansion and new_node->timestep < old_node->high_expansion)
//         { // intervals overlap --> we need to split the node to make them disjoint
//             if (old_node->timestep <= new_node->timestep)
//             {
//                 assert(old_node->num_of_conflicts > new_node->num_of_conflicts);
//                 old_node->high_expansion = new_node->timestep;
//             }
//             else // i.e., old_node->timestep > new_node->timestep
//             {
//                 assert(old_node->num_of_conflicts <= new_node->num_of_conflicts);
//                 new_node->high_expansion = old_node->timestep;
//             }
//         }
//     }
//     return true;
// }

// return true iff we the new node is not dominated by any old node
bool SIPP::dominanceCheck(SIPPNode* new_node)
{
    auto ptr = allNodes_table.find(new_node);
    if (ptr == allNodes_table.end())
        return true;
    for (auto & old_node : ptr->second)
    {
        // cout<<"old node: "<<old_node->location<<" "<<old_node->reached_goal_at<<" "<<old_node->timestep<<endl;
        // cout<<"new node: "<<new_node->reached_goal_at<<" "<<new_node->timestep<<endl;
        if (old_node->reached_goal_at <= new_node->reached_goal_at and
            old_node->timestep <= new_node->timestep and
            old_node->num_of_conflicts <= new_node->num_of_conflicts)
        { // the new node is dominated by the old node
            //cout<<"dominate"<<endl;
            return false;
        }
        else if (old_node->reached_goal_at >= new_node->reached_goal_at and
                old_node->timestep >= new_node->timestep and
                old_node->num_of_conflicts >= new_node->num_of_conflicts) // the old node is dominated by the new node
        { // delete the old node
            if (old_node->in_openlist) // the old node has not been expanded yet
                eraseNodeFromLists(old_node); // delete it from open and/or focal lists
            else // the old node has been expanded already
                num_reopened++; //re-expand it
            useless_nodes.push_back(old_node);
            ptr->second.remove(old_node);
            num_generated--; // this is because we later will increase num_generated when we insert the new node into lists.

            return true;
        }
        else if( old_node->reached_goal_at >= new_node->reached_goal_at and
            old_node->timestep < new_node->high_expansion and new_node->timestep < old_node->high_expansion)
        { // intervals overlap --> we need to split the node to make them disjoint
            if (old_node->timestep <= new_node->timestep)
            {
                assert(old_node->num_of_conflicts > new_node->num_of_conflicts);
                old_node->high_expansion = new_node->timestep;
            }
            else // i.e., old_node->timestep > new_node->timestep
            {
                assert(old_node->num_of_conflicts <= new_node->num_of_conflicts);
                new_node->high_expansion = old_node->timestep;
            }
        }
    }
    return true;
}
