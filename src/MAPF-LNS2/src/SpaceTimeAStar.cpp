#include "SpaceTimeAStar.h"


void SpaceTimeAStar::updatePath(const LLNode* goal, vector<PathEntry> &path)
{
    num_collisions = goal->num_of_conflicts;
    const LLNode* curr = goal;
    if (curr->is_goal)
        curr = curr->parent;
	path.reserve(curr->g_val + 1);
	while (curr != nullptr) 
	{
		path.emplace_back(curr->location);
		curr = curr->parent;
	}
    std::reverse(path.begin(),path.end());
}



// find path by time-space A* search
// Returns a path that minimizes the collisions with the paths in the path table, breaking ties by the length
Path SpaceTimeAStar::findPath(const ConstraintTable& constraint_table)
{
    reset();
    Path path;
    if (constraint_table.constrained(start_location, 0))
    {
		//cout<<"start constrainted"<<endl;
        return path;
    }
    auto holding_time = constraint_table.getHoldingTime(goal_location, constraint_table.length_min); // the earliest timestep that the agent can hold its goal location. The length_min is considered here.
	//cout<<"holding time "<<holding_time<<endl;
    auto static_timestep = constraint_table.getMaxTimestep() + 1; // everything is static after this timestep
	//cout<<"static "<<static_timestep<<endl;

    auto last_target_collision_time = constraint_table.getLastCollisionTimestep(goal_location);
	//cout<<"last target collision "<<last_target_collision_time<<endl;
    // generate start and add it to the OPEN & FOCAL list
    auto h = get_heuristic(start_location,goal_location);
	//max(max(my_heuristic[start_location], holding_time), last_target_collision_time + 1);
    auto start = new AStarNode(start_location, 0, h, nullptr, 0, 0, (start_location == goal_location));
    num_generated++;
    start->in_openlist = true;
    start->focal_handle = focal_list.push(start); // we only use focal list; no open list is used
    allNodes_table.insert(start);


    while (!focal_list.empty())
    {
        auto* curr = focal_list.top();
        focal_list.pop();
        curr->in_openlist = false;
        num_expanded++;
        assert(curr->location >= 0);
		auto temp = curr->parent;
        // check if the popped node is a goal
        if (curr->is_goal)
        {
            updatePath(curr, path);
            break;
        }
        else if (curr->location == goal_location) // the agent can hold the goal location afterward
        {
			curr->is_goal = true;
			updatePath(curr, path);
            break;
            // int future_collisions = constraint_table.getFutureNumOfCollisions(curr->location, curr->timestep);
            // if (future_collisions == 0)
            // {
            //     updatePath(curr, path);
            //     break;
            // }
            // // generate a goal node
            // auto goal = new AStarNode(*curr);
            // goal->is_goal = true;
            // goal->parent = curr;
            // goal->num_of_conflicts += future_collisions;
            // goal->h_val = 0;
            // // try to retrieve it from the hash table
            // auto it = allNodes_table.find(goal);
            // if (it == allNodes_table.end())
            // {
            //     goal->focal_handle = focal_list.push(goal);
            //     goal->in_openlist = true;
            //     num_generated++;
            //     allNodes_table.insert(goal);
            // }
            // else // update existing node's if needed (only in the open_list)
            // {
            //     auto existing_next = *it;
            //     if (existing_next->num_of_conflicts > goal->num_of_conflicts ||
			// 		//existing_next->reached_goal_at > next->reached_goal_at ||
            //        (//existing_next->reached_goal_at > goal->reached_goal_at &&
			// 		existing_next->num_of_conflicts == goal->num_of_conflicts &&
            //         existing_next->getFVal() > goal->getFVal()))
            //     {
            //         assert(existing_next->in_openlist);
            //         existing_next->copy(*goal);	// update existing node
            //         focal_list.update(existing_next->focal_handle);
            //         num_generated++; // reopen is considered as a new node
            //     }
            //     delete (goal);
            // }
        }
        if (curr->timestep >= constraint_table.length_max)
            continue;

        auto next_locations = instance.getNeighbors(curr->location);
        next_locations.emplace_back(curr->location);
        for (int next_location : next_locations)
        {
            int next_timestep = curr->timestep + 1;
            if (static_timestep < next_timestep)
            { // now everything is static, so switch to space A* where we always use the same timestep
                if (next_location == curr->location)
                {
                    continue;
                }
                next_timestep--;
            }

            if (constraint_table.constrained(next_location, next_timestep) ||
                constraint_table.constrained(curr->location, next_location, next_timestep))
                continue;

            // compute cost to next_id via curr node
            int next_g_val = curr->g_val + 1;
            //int next_h_val = my_heuristic[next_location];
			int next_h_val = 0;

            if (next_g_val + next_h_val > constraint_table.length_max)
                continue;
            auto num_conflicts = curr->num_of_conflicts +
                    constraint_table.getNumOfConflictsForStep(curr->location, next_location, next_timestep);
            // if (num_conflicts == 0)
            //     next_h_val = max(next_h_val, curr->getFVal() - next_g_val);  // path max
            // else
            //     next_h_val = max(next_h_val, holding_time - next_g_val); // path max
			next_h_val = get_heuristic(next_location,goal_location);
			
            // generate (maybe temporary) node
            auto next = new AStarNode(next_location, next_g_val, next_h_val,
                                      curr, next_timestep, num_conflicts, curr->goal_index+(next_location == goal_location));
			

            // if (next_location == goal_location && curr->location == goal_location)
            //     next->wait_at_goal = true;

            // try to retrieve it from the hash table
            auto it = allNodes_table.find(next);
            if (it == allNodes_table.end())
            {
                next->focal_handle = focal_list.push(next);
                next->in_openlist = true;
                num_generated++;
                allNodes_table.insert(next);
                continue;
            }
            // update existing node's if needed (only in the open_list)

            auto existing_next = *it;

            if (existing_next->num_of_conflicts > next->num_of_conflicts  || 
				//existing_next->reached_goal_at >  next->reached_goal_at ||
                (existing_next->num_of_conflicts == next->num_of_conflicts &&
                existing_next->getFVal() > next->getFVal()))
            {
                if (!existing_next->in_openlist) // if its in the closed list (reopen)
                {
                    existing_next->copy(*next);
                    existing_next->focal_handle = focal_list.push(existing_next);
                    existing_next->in_openlist = true;
                    num_generated++; // reopen is considered as a new node
                }
                else
                {
                    existing_next->copy(*next);	// update existing node
                    focal_list.update(existing_next->focal_handle);
                }
            }

            delete(next);  // not needed anymore -- we already generated it before
        }  // end for loop that generates successors
    }  // end while loop

    releaseNodes();
    return path;
}


int SpaceTimeAStar::getTravelTime(int start, int end, const ConstraintTable& constraint_table, int upper_bound)
{
    reset();
	int length = MAX_TIMESTEP;
    auto static_timestep = constraint_table.getMaxTimestep() + 1; // everything is static after this timestep
	auto root = new AStarNode(start, 0, compute_heuristic(start, end), nullptr, 0, 0);
	root->open_handle = open_list.push(root);  // add root to heap
	allNodes_table.insert(root);       // add root to hash_table (nodes)
	AStarNode* curr = nullptr;
	while (!open_list.empty())
	{
		curr = open_list.top(); open_list.pop();
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
			if (static_timestep < next_timestep)
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
				int next_h_val = compute_heuristic(next_location, end);
				if (next_g_val + next_h_val >= upper_bound) // the cost of the path is larger than the upper bound
					continue;
				auto next = new AStarNode(next_location, next_g_val, next_h_val, nullptr, next_timestep, 0);
				auto it = allNodes_table.find(next);
				if (it == allNodes_table.end())
				{  // add the newly generated node to heap and hash table
					next->open_handle = open_list.push(next);
					allNodes_table.insert(next);
				}
				else {  // update existing node's g_val if needed (only in the heap)
					delete(next);  // not needed anymore -- we already generated it before
					auto existing_next = *it;
					if (existing_next->g_val > next_g_val)
					{
						existing_next->g_val = next_g_val;
						existing_next->timestep = next_timestep;
						open_list.increase(existing_next->open_handle);
					}
				}
			}
		}
	}
	releaseNodes();
    num_expanded = 0;
    num_generated = 0;
    num_reopened = 0;
	return length;
}

inline AStarNode* SpaceTimeAStar::popNode()
{
	auto node = focal_list.top(); focal_list.pop();
	open_list.erase(node->open_handle);
	node->in_openlist = false;
	num_expanded++;
	return node;
}


inline void SpaceTimeAStar::pushNode(AStarNode* node)
{
	node->open_handle = open_list.push(node);
	node->in_openlist = true;
	num_generated++;
	if (node->getFVal() <= w * min_f_val)
		node->focal_handle = focal_list.push(node);		
}


void SpaceTimeAStar::updateFocalList()
{
	auto open_head = open_list.top();
	if (open_head->getFVal() > min_f_val)
	{
		int new_min_f_val = (int)open_head->getFVal();
		for (auto n : open_list)
		{
			if (n->getFVal() >  w * min_f_val && n->getFVal() <= w * new_min_f_val)
				n->focal_handle = focal_list.push(n);
		}
		min_f_val = new_min_f_val;
	}
}


void SpaceTimeAStar::releaseNodes()
{
	open_list.clear();
	focal_list.clear();
	for (auto node: allNodes_table)
		delete node;
	allNodes_table.clear();
}

