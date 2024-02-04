/*
 * instance definition
 */
#pragma once
#include <random>

#include "graph.hpp"
#include "utils.hpp"
#include "SharedEnv.h"

struct LACAMInstance 
{
    const Graph G;
    Config starts;
    Config goals;
    Config dummy_goals;

    const uint N;  // number of agents

    // for testing
    LACAMInstance(const std::string& map_filename,
                  const std::vector<uint>& start_indexes,
                  const std::vector<uint>& goal_indexes,
                  const std::vector<uint>& dummy_goal_indexes);
    // for MAPF benchmark
    LACAMInstance(const std::string& scen_filename, const std::string& map_filename,
                  const uint _N = 1);
    // random instance generation
    LACAMInstance(const std::string& map_filename, std::mt19937* MT,
                  const uint _N = 1);

    LACAMInstance(SharedEnvironment* env);

    ~LACAMInstance() {}

    void set_dummygoals(const std::vector<uint>& dummy_goal_indexes);

    

    // simple feasibility check of instance
    bool is_valid(const int verbose = 0) const;
};

// solution: a sequence of configurations
using Solution = std::vector<Config>;
std::ostream& operator<<(std::ostream& os, const Solution& solution);
