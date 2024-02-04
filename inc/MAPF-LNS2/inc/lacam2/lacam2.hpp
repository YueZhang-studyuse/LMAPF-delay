#pragma once

//#include "dist_table.hpp"
#include "graph.hpp"
#include "./lacam2/lacam_instance.hpp"
#include "planner.hpp"
#include "utils.hpp"

// main function
Solution solve(const Instance& instance,const LACAMInstance& ins, std::string& additional_info, int commit = 1, 
               const int verbose = 0, const Deadline* deadline = nullptr,
               std::mt19937* MT = nullptr, const Objective objective = OBJ_NONE,
               const float restart_rate = 0.00);
