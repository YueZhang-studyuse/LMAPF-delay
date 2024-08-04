#pragma once
#include"lns_common.h"
#include "SharedEnv.h"

//instance: to save all the instance, heuristic related functions


// Currently only works for undirected unweighted 4-nighbor grids
class Instance 
{
public:
	// some information not needed -> access from env
	// int num_of_cols;
	// int num_of_rows;
	// int map_size;
	struct Node
	{
		int location;
		int value;

		Node() = default;
		Node(int location, int value) : location(location), value(value) {}
		// the following is used to compare nodes in the OPEN list
		struct compare_node
		{
			// returns true if n1 > n2 (note -- this gives us *min*-heap).
			bool operator()(const Node& n1, const Node& n2) const
			{
				return n1.value >= n2.value;
			}
		};  // used by OPEN (heap) to compare nodes (top of the heap has min f-val, and then highest g-val)
	};

	SharedEnvironment* env;
	vector<vector<int>> heuristic;

	mutable vector<vector<int>> guidance_heuristic;
	mutable vector<std::queue<Node>> OPEN;

	//save map degree - for assigning goals
	vector<int> degrees = {0,0,0,0};

	mutable vector<vector<int>> existing_path;
	mutable vector<vector<int>> time_independent_path;
	mutable vector<vector<int>> second_guidance_heuristic; //agent id <loc>

	//for allpair heuristics
	void computeAllPair();
	int getTimeIndependentHeuristics(int agent, int loc) const;
	void initGuidanceHeuristics() const;
	int getSecondGuidanceDistance(int agent, int loc) const
	{
		return second_guidance_heuristic[agent][loc];
	}

	int getTimeDepdentHeuristics(int agent, int loc, int t) const
	{
		// if (t < existing_path[agent].size())
		// {
		// 	cout<<"existing "<<existing_path[agent][t]<<" current "<<loc<<" agent "<<agent <<endl;
		// }
		// if (t >= existing_path[agent].size())
		// 	return MAX_TIMESTEP;
		// if (existing_path[agent][t] == loc)
		// 	return 0;
		return MAX_TIMESTEP;
		// //if (existing_path[agent][t] == loc)
		// 	// getAllpairDistance(existing_path[agent][t],loc);
		// return getAllpairDistance(existing_path[agent][t],loc);;
	}

	tuple<int,int,int> getGuidanceDistance(int agent, int loc, int goal, int t) const
	{
		int h_1 = getTimeDepdentHeuristics(agent,loc,t);
		// int h_2 = 0;
		// if (goal != env->goal_locations[agent].front().first) //not the first goal, then it's free to go to other locations
		// {
		// 	h_2 = 0;
		// }
		// else
		// {
		int h_2 = getTimeIndependentHeuristics(agent,loc);
		// }
		if (goal != env->goal_locations[agent].front().first)
			h_2 = 0;
		int h_3 = getAllpairDistance(loc,goal);
		return make_tuple(h_1,h_2,h_3);
	}

	void prepareDummy();

	// enum valid_moves_t { NORTH, EAST, SOUTH, WEST, WAIT_MOVE, MOVE_COUNT };  // MOVE_COUNT is the enum's size

	Instance()=default;

	void initMap(SharedEnvironment* simulate_env);
	//bool updateStartGoals();

	//void printAgents() const;
    // vector<int> getStarts() const {return start_locations;};
    // vector<list<int>> getGoals() const {return goal_location_seqs;};


    inline bool isObstacle(int loc) const 
	{ 
		//return my_map[loc]; 
		return (env->map[loc] == 1);
	}
    inline bool validMove(int curr, int next) const
    {
        if (next < 0 || next >= env->map.size())
            return false;
        if (env->map[next] == 1)
            return false;
        return getManhattanDistance(curr, next) < 2;
    }
    list<int> getNeighbors(int curr) const;


    inline int linearizeCoordinate(int row, int col) const { return ( this->env->cols * row + col); }
    inline int getRowCoordinate(int id) const { return id / this->env->cols; }
    inline int getColCoordinate(int id) const { return id % this->env->cols; }
    inline pair<int, int> getCoordinate(int id) const { return make_pair(id / this->env->cols, id % this->env->cols); }
    inline int getCols() const { return env->cols; }

    inline int getManhattanDistance(int loc1, int loc2) const
    {
        int loc1_x = getRowCoordinate(loc1);
        int loc1_y = getColCoordinate(loc1);
        int loc2_x = getRowCoordinate(loc2);
        int loc2_y = getColCoordinate(loc2);
        return abs(loc1_x - loc2_x) + abs(loc1_y - loc2_y);
    }

    static inline int getManhattanDistance(const pair<int, int>& loc1, const pair<int, int>& loc2)
    {
        return abs(loc1.first - loc2.first) + abs(loc1.second - loc2.second);
    }

	int getAllpairDistance(int loc1, int loc2) const
	{
		return heuristic[loc1][loc2];
	}

	int getDegree(int loc) const
	{
		assert(loc >= 0 && loc < env->map.size() && env->map[loc] != 1);
		int degree = 0;
		if (0 <= loc - env->cols && env->map[loc - env->cols] != 1)
			degree++;
		if (loc + env->cols < env->map.size() && env->map[loc + env->cols] != 1)
			degree++;
		if (loc % env->cols > 0 && env->map[loc - 1] != 1)
			degree++;
		if (loc % env->cols < env->cols - 1 && env->map[loc + 1] != 1)
			degree++;
		return degree;
	}

	int getDegreeAdvanced(int loc) const //more details, up to 8 degrees (including diagonal neighbors)
	{
		assert(loc >= 0 && loc < env->map.size() && env->map[loc] != 1);
		int degree = 0;
		if (0 < loc - env->cols && env->map[loc - env->cols] != 1)
			degree++;
		if (loc + env->cols < env->map.size() && env->map[loc + env->cols] != 1)
			degree++;
		if (loc % env->cols > 0 && env->map[loc - 1] != 1)
			degree++;
		if (loc % env->cols < env->cols - 1 && env->map[loc + 1] != 1)
			degree++;
		//diagonal
		if (0 <= loc - env->cols - 1 && loc % env->cols > 0 && env->map[loc - env->cols - 1] != 1)
			degree++;
		if (loc + env->cols + 1 < env->map.size() && loc % env->cols < env->cols - 1 && env->map[loc + env->cols + 1] != 1)
			degree++;
		if (0 <= loc - env->cols + 1 && loc % env->cols < env->cols - 1 && env->map[loc - env->cols + 1] != 1)
			degree++;
		if (loc + env->cols - 1 < env->map.size() && loc % env->cols > 0 && env->map[loc + env->cols + 1] != 1)
			degree++;
		return degree;
	}

	int getDefaultNumberOfAgents() const { return env->num_of_agents; }
    void savePaths(const string & file_name, const vector<Path*>& paths) const;
    //bool validateSolution(const vector<Path*>& paths, int sum_of_costs, int num_of_colliding_pairs) const;
    bool hasCollision(const Path& p1, const Path& p2) const;

	//void setStart(int agent, int location){start_locations[agent] = location;}

	//void createDummyGoals();
	//void assignDummyGoals (int agent) const;

	vector<int> getDummyGoals() const { return dummy_goals;};

	void updateDummyGoal(int agent, int location) const {dummy_goals[agent] = location;}
	
private:
	  // int moves_offset[MOVE_COUNT];
	  //vector<bool> my_map;
	  string map_fname;
	  string agent_fname;

	  //int num_of_agents;
	  //vector<int> start_locations;
	  //vector<int> goal_locations;
	  //vector<list<int>> goal_location_seqs;
	  mutable vector<int> dummy_goals;

	  bool nathan_benchmark = true;
	  void printMap() const;
	  void saveMap() const;

	  // Class  SingleAgentSolver can access private members of Node 
	  friend class SingleAgentSolver;
};