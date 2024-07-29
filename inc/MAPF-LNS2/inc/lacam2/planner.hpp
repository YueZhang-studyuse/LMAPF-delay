/*
 * lacam-star
 */

#pragma once

//#include "dist_table.hpp"
#include "graph.hpp"
#include "lacam_instance.hpp"
#include "utils.hpp"
#include "Instance.h"

// objective function
enum Objective { OBJ_NONE, OBJ_MAKESPAN, OBJ_SUM_OF_LOSS };
std::ostream& operator<<(std::ostream& os, const Objective objective);

// PIBT agent
struct LACAMAgent {
  const uint id;
  Vertex* v_now;   // current location
  Vertex* v_next;  // next location
  int goal_index = 0; //for reach goal disappear
  int curr_timestep = 0;
  LACAMAgent(uint _id) : id(_id), v_now(nullptr), v_next(nullptr) {}
};
using LACAMAgents = std::vector<LACAMAgent*>;

// low-level node
struct LNode {
  std::vector<uint> who;
  Vertices where;
  const uint depth;
  LNode(LNode* parent = nullptr, uint i = 0,
        Vertex* v = nullptr);  // who and where
};

// high-level node
struct HNode {
  static uint HNODE_CNT;  // count #(high-level node)
  const Config C;

  // tree
  HNode* parent;
  std::set<HNode*> neighbor;

  // costs
  uint g;        // g-value (might be updated)
  const uint h;  // h-value
  uint f;        // g + h (might be updated)

  int curr_time = 0;

  //reached goal count
  //std::vector<bool> reach_goal;
  std::vector<int> reach_goal;

  int num_agent_reached = 0;

  // for low-level search
  std::vector<float> priorities;
  std::vector<uint> order;
  std::queue<LNode*> search_tree;

  int depth = 0;

  // HNode(const Config& _C, DistTable& D, HNode* _parent, const uint _g,
  //       const uint _h);
  HNode(const Config& _C, const Instance& I, HNode* _parent, const uint _g,
        const uint _h);
  ~HNode();
};
using HNodes = std::vector<HNode*>;

struct Planner {
  const Instance& instance;
  const LACAMInstance* ins;
  const Deadline* deadline;
  std::mt19937* MT;
  const int verbose;

  // hyper parameters
  const Objective objective;
  const float RESTART_RATE;  // random restart

  // solver utils
  const uint N;       // number of agents
  const uint V_size;  // number o vertices
  //DistTable D; //use my all pair heuristic
  uint loop_cnt;      // auxiliary

  int commit_window = 1;

  // used in PIBT
  std::vector<std::array<Vertex*, 5> > C_next;  // next locations, used in PIBT
  std::vector<float> tie_breakers;              // random values, used in PIBT
  LACAMAgents A;
  LACAMAgents occupied_now;                          // for quick collision checking
  LACAMAgents occupied_next;                         // for quick collision checking

  Planner(const Instance& _instance, const LACAMInstance* _ins, const Deadline* _deadline, std::mt19937* _MT,
          const int _verbose = 0,
          // other parameters
          const Objective _objective = OBJ_NONE,
          const float _restart_rate = 0.001);
  ~Planner();
  Solution solve(std::string& additional_info);
  void expand_lowlevel_tree(HNode* H, LNode* L);
  void rewrite(HNode* H_from, HNode* T, HNode* H_goal,
               std::stack<HNode*>& OPEN);
  uint get_edge_cost(const Config& C1, const Config& C2);
  uint get_edge_cost(HNode* H_from, HNode* H_to);
  uint get_h_value(const Config& C);
  bool get_new_config(HNode* H, LNode* L);
  bool funcPIBT(LACAMAgent* ai, bool first);

  // swap operation
  LACAMAgent* swap_possible_and_required(LACAMAgent* ai);
  bool is_swap_required(const uint pusher, const uint puller,
                        Vertex* v_pusher_origin, Vertex* v_puller_origin);
  bool is_swap_possible(Vertex* v_pusher_origin, Vertex* v_puller_origin);

  // utilities
  template <typename... Body>
  void solver_info(const int level, Body&&... body)
  {
    if (verbose < level) return;
    std::cout << "elapsed:" << std::setw(6) << elapsed_ms(deadline) << "ms"
              << "  loop_cnt:" << std::setw(8) << loop_cnt
              << "  node_cnt:" << std::setw(8) << HNode::HNODE_CNT << "\t";
    info(level, verbose, (body)...);
  }
};
