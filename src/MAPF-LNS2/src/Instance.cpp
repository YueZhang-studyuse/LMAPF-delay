#include<boost/tokenizer.hpp>
#include <algorithm>    // std::shuffle
#include <random>      // std::default_random_engine
#include <chrono>       // std::chrono::system_clock
#include"Instance.h"

void Instance::initMap(SharedEnvironment* simulate_env)
{
    env = simulate_env;
    existing_path.resize(env->num_of_agents);
    time_independent_path.resize(env->num_of_agents);
}

void Instance::prepareDummy()
{
    //prepare dummy goals
    vector<list<int>> loc_degrees;
    loc_degrees.resize(8); 

    for (int i = 0; i < env->map.size(); i++)
    {
        if (env->map[i] == 1)
            continue;
        int d = getDegreeAdvanced(i);
        if (d == 0)
            continue;
        if (getAllpairDistance(env->curr_states[0].location, i) == MAX_TIMESTEP) //do not belongs to the current connectted components
            continue;
        loc_degrees[d-1].push_back(i);
    }

    unordered_set<int> candidates;
    for (int j = 7; j >= 0; j--)
    {
        for (auto loc: loc_degrees[j])
        {
            candidates.insert(loc);
        }
        if (candidates.size() >= env->num_of_agents)
        {
            break;
        }
    }

    dummy_goals.resize(candidates.size());
    int index = 0;
    for (auto loc: candidates)
    {
        dummy_goals[index] = loc;
        index++;
    }
    std::random_shuffle(dummy_goals.begin(), dummy_goals.end());
    dummy_goals.resize(env->num_of_agents);
}


void Instance::printMap() const
{
	for (int i = 0; i< env->rows; i++)
	{
		for (int j = 0; j < env->cols;j++)
		{
			if (this->env->map[linearizeCoordinate(i, j)] == 1)
				cout << '@';
			else
				cout << '.';
		}
		cout << endl;
	}
}


void Instance::saveMap() const
{
	ofstream myfile;
	myfile.open(map_fname);
	if (!myfile.is_open())
	{
		cout << "Fail to save the map to " << map_fname << endl;
		return;
	}
	myfile << env->rows << "," << env->cols << endl;
	for (int i = 0; i < env->rows; i++)
	{
		for (int j = 0; j < env->cols; j++)
		{
			if (env->map[linearizeCoordinate(i, j)] == 1)
				myfile << "@";
			else
				myfile << ".";
		}
		myfile << endl;
	}
	myfile.close();
}


list<int> Instance::getNeighbors(int curr) const
{
	list<int> neighbors;
	int candidates[4] = {curr + 1, curr - 1, curr + env->cols, curr - env->cols};
	for (int next : candidates)
	{
		if (validMove(curr, next))
			neighbors.emplace_back(next);
	}
	return neighbors;
}

void Instance::savePaths(const string & file_name, const vector<Path*>& paths) const
{
    std::ofstream output;
    output.open(file_name);

    for (auto i = 0; i < paths.size(); i++)
    {
        output << "Agent " << i << ":";
        for (const auto &state : (*paths[i]))
        {
            if (nathan_benchmark)
                output << "(" << getColCoordinate(state.location) << "," << getRowCoordinate(state.location) << ")->";
            else
                output << "(" << getRowCoordinate(state.location) << "," << getColCoordinate(state.location) << ")->";
        }
        output << endl;
    }
    output.close();
}

void Instance::computeAllPair()
{
    int h_size  = 0;
    cout<<"computing all pair"<<endl;
    heuristic.resize(env->map.size());

    for (int i = 0; i < heuristic.size(); i++)
    {
        if (env->map[i] == 1)
            continue;
        //heuristic[i] = std::vector<int>(heuristic.size()-i, MAX_TIMESTEP);
        heuristic[i] = std::vector<int>(heuristic.size(), MAX_TIMESTEP);
        h_size++;
    }
    for (int i = 0; i < heuristic.size(); i++)
    {
        if (env->map[i] == 1)
            continue;
        // generate a heap that can save nodes (and a open_handle)
        boost::heap::pairing_heap< Node, boost::heap::compare<Node::compare_node> > heap;

        Node root(i, 0); //compute every node to i
        heuristic[i][i] = 0;

        heap.push(root);  // add root to heap
        while (!heap.empty())
        {
            Node curr = heap.top();
            heap.pop();
            for (int next_location : getNeighbors(curr.location))
            {
                if (heuristic[i][next_location] > curr.value + 1)
                {
                    heuristic[i][next_location] = curr.value + 1;
                    Node next(next_location, curr.value + 1);
                    heap.push(next);
                }
            }
        }
    }

    // //save heuristic
    // std::ofstream myfile;
    // myfile.open ("Paris_1_256_heuristics_table.txt");
	// myfile << "table_size" << std::endl << 
    //     h_size << "," << env->map.size() << std::endl;
	// for (int i = 0; i < heuristic.size(); i++) 
	// {
    //     if (heuristic[i].empty())
    //         continue;

    //     myfile << i << std::endl;
	// 	for (int h : heuristic[i]) 
	// 	{
    //         myfile << h << ",";
	// 	}
	// 	myfile << std::endl;
	// }
	// myfile.close();
}

int Instance::getTimeIndependentHeuristics(int agent, int loc) const
{
	if (guidance_heuristic[agent].empty()) //no guidance path
        return MAX_TIMESTEP;
	if (guidance_heuristic[agent][loc] < MAX_TIMESTEP)
		return guidance_heuristic[agent][loc];
	//expand by bfs
	while (!OPEN[agent].empty()) 
    {
        Node n = OPEN[agent].front();
		//cout<<"expanding "<<n.location<<endl;
        OPEN[agent].pop();
		int d_n = guidance_heuristic[agent][n.location];
		for (int next_location : getNeighbors(n.location))
		{
			int d_m = guidance_heuristic[agent][next_location];
			if (d_n + 1 >= d_m) continue;
			guidance_heuristic[agent][next_location] = d_n + 1;
			Node next(next_location, d_n + 1);
			OPEN[agent].push(next);
		}
        if (n.location == loc)
			return d_n;
    }
    return MAX_TIMESTEP;
}

void Instance::initGuidanceHeuristics() const
{
	guidance_heuristic.clear();
	guidance_heuristic.resize(env->num_of_agents);
	OPEN.clear();
	for (size_t i = 0; i < env->num_of_agents; i++) 
	{
		if (time_independent_path[i].empty())
            continue;
		guidance_heuristic[i] = std::vector<int>(env->map.size(), MAX_TIMESTEP);

		OPEN.push_back(std::queue<Node>());
		for (int t = 0; t < time_independent_path[i].size();t++)
		{
			int loc = time_independent_path[i][t];
			//cout<<"loc "<<loc;
			Node root(loc, 0); //compute every node to i
        	guidance_heuristic[i][loc] = abs((int)time_independent_path[i].size()-1-t);
        	OPEN[i].push(root);  // add root to heap
		}
	}
}

// bool Instance::validateSolution(const vector<Path*>& paths, int sum_of_costs, int num_of_colliding_pairs) const
// {
//     cout << "Validate solution ..." << endl;
//     if (paths.size() != start_locations.size())
//     {
//         cerr << "We have " << paths.size() << " for " << start_locations.size() << " agents." << endl;
//         exit(-1);
//     }
//     int sum = 0;
//     for (auto i = 0; i < start_locations.size(); i++)
//     {
//         if (paths[i] == nullptr or paths[i]->empty())
//         {
//             cerr << "No path for agent " << i << endl;
//             exit(-1);
//         }
//         else if (start_locations[i] != paths[i]->front().location)
//         {
//             cerr << "The path of agent " << i << " starts from location " << paths[i]->front().location
//                  << ", which is different from its start location " << start_locations[i] << endl;
//             exit(-1);
//         }
//         else if (goal_locations[i] != paths[i]->back().location)
//         {
//             cerr << "The path of agent " << i << " ends at location " << paths[i]->back().location
//                  << ", which is different from its goal location " << goal_locations[i] << endl;
//             exit(-1);
//         }
//         for (int t = 1; t < (int) paths[i]->size(); t++ )
//         {
//             if (!validMove(paths[i]->at(t - 1).location, paths[i]->at(t).location))
//             {
//                 cerr << "The path of agent " << i << " jumps from "
//                      << paths[i]->at(t - 1).location << " to " << paths[i]->at(t).location
//                      << " between timesteps " << t - 1 << " and " << t << endl;
//                 exit(-1);
//             }
//         }
//         sum += (int) paths[i]->size() - 1;
//     }
//     if (sum_of_costs != sum)
//     {
//         cerr << "The computed sum of costs " << sum_of_costs <<
//              " is different from that of the solution " << sum << endl;
//         exit(-1);
//     }
//     // check for colliions
//     int collisions = 0;
//     for (auto i = 0; i < start_locations.size(); i++)
//     {
//         for (auto j = i + 1; j < start_locations.size(); j++)
//         {
//             bool found_collision = false;
//             const auto a1 = paths[i]->size() <= paths[j]->size()? i : j;
//             const auto a2 = paths[i]->size() <= paths[j]->size()? j : i;
//             int t = 1;
//             for (; t < (int) paths[a1]->size(); t++)
//             {
//                 if (paths[a1]->at(t).location == paths[a2]->at(t).location) // vertex conflict
//                 {
//                     if (num_of_colliding_pairs == 0)
//                     {
//                         cerr << "Find a vertex conflict between agents " << a1 << " and " << a2 <<
//                              " at location " << paths[a1]->at(t).location << " at timestep " << t << endl;
//                         exit(-1);
//                     }
//                     collisions++;
//                     found_collision = true;
//                     break;
//                 }
//                 else if (paths[a1]->at(t).location == paths[a2]->at(t-1).location &&
//                         paths[a1]->at(t-1).location == paths[a2]->at(t).location) // edge conflict
//                 {
//                     if (num_of_colliding_pairs == 0)
//                     {
//                         cerr << "Find an edge conflict between agents " << a1 << " and " << a2 <<
//                              " at edge (" << paths[a1]->at(t-1).location << "," << paths[a1]->at(t).location <<
//                              ") at timestep " << t << endl;
//                         exit(-1);
//                     }
//                     collisions++;
//                     found_collision = true;
//                     break;
//                 }
//             }
//             if (!found_collision)
//             {
//                 auto target = paths[a1]->back().location;
//                 for (; t < (int) paths[a2]->size(); t++)
//                 {
//                     if (paths[a2]->at(t).location == target)  // target conflict
//                     {
//                         if (num_of_colliding_pairs == 0)
//                         {
//                             cerr << "Find a target conflict where agent " << a2 << " (of length " << paths[a2]->size() - 1 <<
//                                  ") traverses agent " << a1 << " (of length " << paths[a1]->size() - 1<<
//                                  ")'s target location " << target << " at timestep " << t << endl;
//                             exit(-1);
//                         }
//                         collisions++;
//                         break;
//                     }
//                 }
//             }
//         }
//     }
//     if (collisions != num_of_colliding_pairs)
//     {
//         cerr << "The computed number of colliding pairs " << num_of_colliding_pairs <<
//              " is different from that of the solution " << collisions << endl;
//         exit(-1);
//     }
//     cout << "Done!" << endl;
//     return true;
// }

// void Instance::assignDummyGoals(int agent) const
// {
//     vector<int> sum_degree = {0,0,0,0};
//     unordered_set<int> current_goals;

//     for (int i = 0; i < env->num_of_agents; i++)
//     {
//         if (i == agent)
//             continue;
//         int goal = (dummy_goals[i] == -1 ? env->goal_locations[i][0].first : dummy_goals[i]);
//         sum_degree[getDegree(goal)-1]++;
//     }

//     //assign dummy goals according to a start location
//     int accept_degree = 4;
//     while (degrees[accept_degree-1] <= sum_degree[accept_degree-1])
//     {
//         accept_degree--;
//     }
//     //use bfs to find nearst dummy goals (including current location)
//     std::queue<int> open;
//     unordered_set<int> close;
//     open.push(env->goal_locations[agent][0].first);
//     while(!open.empty())
//     {
//         int curr = open.front();
//         open.pop();
//         if (current_goals.find(curr) == current_goals.end() && env->map[curr] != 1 && 
//         getDegree(curr) >= accept_degree)
//         {
//             //return curr;
//             dummy_goals[agent] = curr;
//             return;
//         }
//         if (close.find(curr) != close.end())
//         {
//             continue;
//         }
//         close.insert(curr);
//         auto neighbor = getNeighbors(curr);
//         for (auto location: neighbor)
//         {
//             if (close.find(location) == close.end())
//                 open.push(location);
//         }
//     }
//     dummy_goals[agent] = -1;
// }

bool Instance::hasCollision(const Path& p1, const Path& p2) const
{
    int t = 1;
    for (; t < (int) min(p1.size(), p2.size()); t++)
    {
        if (p1[t].location == p2[t].location) // vertex conflict
        {
            return true;
        }
        else if (p1[t].location == p2[t-1].location and p1[t-1].location == p2[t].location) // edge conflict
        {
            return true;
        }
    }
    // if (p1.size() == p2.size()) return false;

    // auto p = p1.size() > p2.size()? p1 : p2;
    // auto target = p1.size() < p2.size()? p1.back().location : p2.back().location;
    // for (; t < (int) p.size(); t++)
    // {
    //     if (p[t].location == target)  // target conflict
    //     {
    //         return true;
    //     }
    // }
    return false;
}

// void Instance::createDummyGoals()
// {
//     unordered_set<int> dummy_goals_set;
//     unordered_set<int> goals;
//     dummy_goal_accpetance = 0;
//     for (int agent = 0; agent < num_of_agents; agent++)
//     {
//         // if (dummy_goals[agent] != -1)
//         // {
//         //     dummy_goals_set.insert(dummy_goals[agent]);
//         // }
//         goals.insert(env->goal_locations[agent][0].first);
//     }

//     for (int agent = 0; agent < num_of_agents; agent++)
//     {
//         if (goals.find(dummy_goals[agent]) != goals.end())
//             dummy_goals[agent] = -1;
//         if (dummy_goals[agent] != -1)
//         {
//             dummy_goals_set.insert(dummy_goals[agent]);
//         }
//     }
//     for (int agent = 0; agent < num_of_agents; agent++)
//     {
//         if (dummy_goals[agent] != -1)
//             continue;
//         //use bfs to find nearst dummy goals (including current location)
//         std::queue<int> open;
//         unordered_set<int> close;
//         open.push(goal_location_seqs[agent].back());
//         while(!open.empty())
//         {
//             int curr = open.front();
//             open.pop();
//             if (dummy_goals_set.find(curr) == dummy_goals_set.end() && goals.find(curr) == goals.end() && getDegree(curr) >= dummy_goal_accpetance)
//             {
//                 dummy_goals[agent] = curr;
//                 dummy_goals_set.insert(curr);
//                 break;
//             }
//             if (close.find(curr) != close.end())
//             {
//                 continue;
//             }
//             close.insert(curr);
//             auto neighbor = getNeighbors(curr);
//             for (auto loc: neighbor)
//             {
//                 if (close.find(loc) == close.end())
//                     open.push(loc);
//             }
//         }
//     }
// }