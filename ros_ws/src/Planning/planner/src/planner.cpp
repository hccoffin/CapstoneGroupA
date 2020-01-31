#include "ros/ros.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "queue"
#include "vector"
#include "set"
#include "planner/planner_node.h"

using namespace std;

vector<int> xy_to_rowcol(
    vector<vector<int>>& obstacle_map, vector<double>& map_bounds, 
    vector<double>& xy
) {
    double row_prop = (xy[0] - map_bounds[0]) / (map_bounds[1] - map_bounds[0]);
    double col_prop = (xy[1] - map_bounds[2]) / (map_bounds[3] - map_bounds[2]);
    int row = (int) round(row_prop * (obstacle_map.size() - 1));
    int col = (int) round(col_prop * (obstacle_map[0].size() - 1));
    return (vector<int>){row, col};
}

vector<double> rowcol_to_xy(
    vector<vector<int>>& obstacle_map, vector<double>& map_bounds, 
    vector<double>& rowcol
) {
    double x = (double)rowcol[0] / (obstacle_map.size() - 1) * (map_bounds[1] - map_bounds[0]);
    double y = (double)rowcol[1] / (obstacle_map[0].size() - 1) * (map_bounds[3] - map_bounds[2]);
    return (vector<double>){x, y};
}

// State and comparison functions for priority queue
struct EightConnectedState {
    int row;
    int col;
    double distance;
};
struct CompareDistance { 
    bool operator()(EightConnectedState const& s1, EightConnectedState const& s2) { 
        return s1.distance > s2.distance; 
    } 
};

vector<vector<double>> eight_connected_distance(
    vector<vector<int>>& obstacle_map, vector<double>& map_bounds,
    vector<double>& target_xy
) {
    priority_queue<EightConnectedState, vector<EightConnectedState>, CompareDistance> pq;
    vector<vector<int>> seen(obstacle_map.size(), vector<int> (obstacle_map[0].size(), 0));

    vector<vector<double>> distances(obstacle_map.size(), vector<double> (obstacle_map[0].size(), 0));
    for (int r = 0; r < obstacle_map.size(); r++) {
        for (int c = 0; c < obstacle_map[0].size(); c++) {
            if (obstacle_map[r][c] == 1) {
                distances[r][c] = numeric_limits<double>::infinity();
            }
        }
    }

    // intialize the priority queue and seen set with the target point
    vector<int> target_rowcol = xy_to_rowcol(
        obstacle_map, map_bounds, target_xy
    );
    EightConnectedState target_state;
    target_state.row = target_rowcol[0];
    target_state.col = target_rowcol[1];
    target_state.distance = 0;
    pq.push(target_state);
    seen[target_rowcol[0]][target_rowcol[1]] = 1;

    // find the distances to all other points in the map
    EightConnectedState cur_state;
    while (!pq.empty()) {
        cur_state = pq.top();
        pq.pop();
        distances[cur_state.row][cur_state.col] = cur_state.distance;
        for (const int delta_row : {-1, 0, 1}) {
            for (const int delta_col : {-1, 0, 1}) {
                int new_row = cur_state.row + delta_row;
                int new_col = cur_state.col + delta_col;
                double x_dist_sqrd = pow(delta_row * (map_bounds[1] - map_bounds[0]) / (obstacle_map.size() - 1), 2);
                double y_dist_sqrd = pow(delta_col * (map_bounds[3] - map_bounds[2]) / (obstacle_map[0].size() - 1), 2);
                double new_distance = cur_state.distance + sqrt(x_dist_sqrd + y_dist_sqrd);
                EightConnectedState new_state;
                new_state.row = new_row;
                new_state.col = new_col;
                new_state.distance = new_distance;
                if (
                    new_row >= 0 && new_row < obstacle_map.size() &&
                    new_col >= 0 && new_col < obstacle_map[0].size() &&
                    obstacle_map[new_row][new_col] == 0 &&
                    seen[new_row][new_col] == 0
                ) {
                    seen[new_row][new_col] = 1;
                    pq.push(new_state);
                }
            }
        }
    }

    return distances;
}

struct State {
    double x;
    double y;
    double theta;
    double v;
    double omega;
    double t;
};
struct MotionPrimitive {
    vector<State> states;
};
/*
struct PrimitiveGenerator {
    int n_a; // number of different accelerations
    int n_alpha; // number of different angular accelerations
    int n_t; // number of time steps (number of states in the primitive)
    double max_v; // max velocity
    double max_omega; // max angular velocity
    double max_a; // max acceleration (assume only forward)
    double max_alpha; // max angular acceleration (either to the left or right)
    double duration; // total time of the primitive
    vector<MotionPrimitive> operator()(
        vector<vector<int>>& obstacle_map, vector<double>& map_bounds, State s
    ) {
        vector<MotionPrimitive> primitives;
        for (int i = 0; i < n_a; i++) {
            for (int j = 0; j < n_alpha; j++) {
                MotionPrimitive prim;
                prim.states.push_back(s);
                double a = i * max_a / (n_a - 1);
                double alpha = j * max_alpha / (n_alpha - 1);
                bool prim_valid = true; 
                for (int k = 0; k < n_t; k++) {
                    double t = k * duration / (n_t - 1);
                    double v = s.v + (a * t);
                    double omega = s.omega + (alpha * t);
                    double theta = s.theta + (s.omega * t) + (alpha * t * t / 2);

                    // midpoint integration to find x and y
                    double prev_v = prim.states[prim.states.size() - 1].v;
                    double prev_theta = prim.states[prim.states.size() - 1].theta;
                    double dx = ((prev_v + v) / 2) * cos((theta + prev_theta) / 2);
                    double dy = ((prev_v + v) / 2) * sin((theta + prev_theta) / 2);
                    double x = prim.states[prim.states.size() - 1].x + dx;
                    double y = prim.states[prim.states.size() - 1].y + dy;

                    s_t_rowcol = xy_to_rowcol(obstacle_map, map_bounds, vector<double>{x, y});
                    if (obstacle_map[s_t_rowcol[0]][s_t_rowcol[1]] == 0) {
                        State s_t = {x, y, theta, v, omega, t};
                        prim.push_back(s_t);
                    } else {
                        prim_valid = false;
                        break;
                    }
                }
                if (prim_valid) {
                    primitives.push_back(prim);
                }
            }
        }
        return primitives;
    }
};
*/
vector<MotionPrimitive> generate_path(
    vector<vector<int>>& obstacle_map, vector<double>& map_bounds,
    State target_state, State initial_state//,
    //PrimitiveGenerator primitive_generator
){

}

int main(int argc, char **argv) {
    // vector<vector<int>> obstacle_map (11, vector<int> (10, 0));
    // obstacle_map[8][5] = 1;
    // obstacle_map[7][5] = 1;
    // obstacle_map[6][5] = 1;
    // obstacle_map[5][5] = 1;
    // obstacle_map[4][5] = 1;
    // obstacle_map[3][5] = 1;
    // obstacle_map[2][5] = 1;
    // obstacle_map[1][5] = 1;
    // obstacle_map[0][5] = 1;

    // vector<double> map_bounds = {0, 10, 0, 9};
    // vector<double> target = {0, 9};
    // vector<vector<double>> heuristic = eight_connected_distance(obstacle_map, map_bounds, target);

    // std::cout << std::setprecision(2) << std::fixed;
    // for (int r = 0; r < obstacle_map.size(); r++) {
    //     for (int c = 0; c < obstacle_map[0].size(); c++) {
    //         cout << heuristic[r][c] << "\t";
    //     }
    //     cout << endl;
    // }
    /////// Planner constructs plan here /////

      std::vector<double> px, py, theta, vx, vy, w, ts; // Store positions and velocities for each point here
      PlanMsg plan_msg;
      VizMsg viz_msg;
      if(!make_plan_msg(px, py, theta, vx, vy, w, ts, plan_msg, viz_msg)){
        std::cout << "Bad stuff!" << std::endl;
        return -1;
      }
      // Publisher plan_msg to topic

      return 0;
}