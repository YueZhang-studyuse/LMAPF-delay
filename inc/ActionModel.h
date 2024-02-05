#pragma once
#include <string>
#include "Grid.h"
#include "States.h"
#include "Logger.h"

/*
  NO  - north
  EA  - east
  SO - south
  WE   - West
  WA   - Wait
  NA  - Not applicable
*/

//enum Action {FW, CR, CCR, W, NA};
enum Action {N, E, S, WE, WA, NA};

std::ostream& operator<<(std::ostream &stream, const Action &action);

// class ActionModelWithRotate
// {
// public:
//     list<std::tuple<std::string,int,int,int>> errors;

//     ActionModelWithRotate(Grid & grid): grid(grid), rows(grid.rows), cols(grid.cols){
//         moves[0] = 1;
//         moves[1] = cols;
//         moves[2] = -1;
//         moves[3] = -cols;

//     };

//     bool is_valid(const vector<State>& prev, const vector<Action> & action);
//     void set_logger(Logger* logger){this->logger = logger;}

//     vector<State> result_states(const vector<State>& prev, const vector<Action> & action){
//         vector<State> next(prev.size());
//         for (size_t i = 0 ; i < prev.size(); i ++){
//             next[i] = result_state(prev[i], action[i]);
//         }
//         return next;
//     };


// protected:
//     const Grid& grid;
//     int rows;
//     int cols;
//     int moves[4];
//     Logger* logger = nullptr;

//     State result_state(const State & prev, Action action)
//     {
//         int new_location = prev.location;
//         int new_orientation = prev.orientation;
//         if (action == Action::FW)
//         {
//             new_location = new_location += moves[prev.orientation];
//         }
//         else if (action == Action::CR)
//         {
//             new_orientation = (prev.orientation + 1) % 4;
      
//         }
//         else if (action == Action::CCR)
//         {
//             new_orientation = (prev.orientation - 1) % 4;
//             if (new_orientation == -1)
//                 new_orientation = 3;
//         }

//         return State(new_location, prev.timestep + 1, new_orientation);
//     }
// };

class ActionModel
{
public:
    list<std::tuple<std::string,int,int,int>> errors;

    ActionModel(Grid & grid): grid(grid), rows(grid.rows), cols(grid.cols){
        moves[0] = -cols;
        moves[1] = 1;
        moves[2] = cols;
        moves[3] = -1;

    };

    bool is_valid(const vector<State>& prev, const vector<Action> & action);
    void set_logger(Logger* logger){this->logger = logger;}

    vector<State> result_states(const vector<State>& prev, const vector<Action> & action){
        vector<State> next(prev.size());
        for (size_t i = 0 ; i < prev.size(); i ++){
            next[i] = result_state(prev[i], action[i]);
        }
        return next;
    };

    vector<State> result_states_with_delays(const vector<State>& prev, const vector<Action> & action, const vector<bool> & delays);
    void mcp_simulate(int agent, const vector<State>& prev, vector<State>& next, vector<list<int>> orderes, unordered_map<int,bool> simulated_agents,const vector<bool> & delays);


protected:
    const Grid& grid;
    int rows;
    int cols;
    int moves[4];
    Logger* logger = nullptr;

    State result_state(const State & prev, Action action)
    {
        int new_location = prev.location;
        if (action == Action::N)
        {
            new_location = new_location += moves[0];
        }
        else if (action == Action::E)
        {
            new_location = new_location += moves[1];
      
        }
        else if (action == Action::S)
        {
            new_location = new_location += moves[2];
        }
        else if (action == Action::WE)
        {
            new_location = new_location += moves[3];
        }

        return State(new_location, prev.timestep + 1);
    }
};
