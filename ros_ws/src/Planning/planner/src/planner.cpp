#include "ros/ros.h"
#include "stdlib.h"
#include "stdio.h"
#include "math.h"
#include "numeric"
#include "queue"
#include "vector"
#include "set"
#include "planner/planner_node.h"

using namespace std;

vector<int> xy_to_rowcol(
    vector<size_t>& map_dims, vector<double>& map_bounds, 
    vector<double>& xy
) {
    double row_prop = (xy[0] - map_bounds[0]) / (map_bounds[1] - map_bounds[0]);
    double col_prop = (xy[1] - map_bounds[2]) / (map_bounds[3] - map_bounds[2]);
    int row = (int) round(row_prop * (map_dims[0] - 1));
    int col = (int) round(col_prop * (map_dims[1] - 1));
    return (vector<int>){row, col};
}

vector<double> rowcol_to_xy(
    vector<size_t>& map_dims, vector<double>& map_bounds, 
    vector<int>& rowcol
) {
    double x = (double)rowcol[0] / (map_dims[0] - 1) * (map_bounds[1] - map_bounds[0]);
    double y = (double)rowcol[1] / (map_dims[1] - 1) * (map_bounds[3] - map_bounds[2]);
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
    vector<size_t> map_dims = {obstacle_map.size(), obstacle_map[0].size()};
    vector<int> target_rowcol = xy_to_rowcol(
        map_dims, map_bounds, target_xy
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

double interpolator_weight(
    vector<size_t>& map_dims, vector<double>& map_bounds,
    vector<double>& xy, vector<int>& rowcol
) {
    vector<double> xy_gridpoint = rowcol_to_xy(map_dims, map_bounds, rowcol);
    double distance = sqrt(
        pow(xy_gridpoint[0] - xy[0], 2) + pow(xy_gridpoint[1] - xy[1], 2)
    );
    return exp(-distance);
}
double interpolator(
    vector<vector<double>>& distances, vector<double>& map_bounds, 
    vector<double> xy
) {
    vector<size_t> map_dims = {distances.size(), distances[0].size()};
    if (
        xy[0] < map_bounds[0] || xy[0] > map_bounds[1] || 
        xy[1] < map_bounds[2] || xy[1] > map_bounds[3]
    ) {
        return numeric_limits<double>::infinity();;
    } 
    vector<int> rowcol = xy_to_rowcol(map_dims, map_bounds, xy);
    int r = rowcol[0];
    int c = rowcol[1];
    vector<double> values;
    vector<double> weights;

    values.push_back(distances[r][c]);
    weights.push_back(interpolator_weight(map_dims, map_bounds, xy, rowcol));
    if ((r - 1) >= 0) {
        values.push_back(distances[r - 1][c]);
        vector<int> new_rowcol = {r-1, c};
        weights.push_back(interpolator_weight(map_dims, map_bounds, xy, new_rowcol));
    }
    if ((c - 1) >= 0) {
        values.push_back(distances[r][c - 1]);
        vector<int> new_rowcol = {r, c-1};
        weights.push_back(interpolator_weight(map_dims, map_bounds, xy, new_rowcol));
    }
    if ((r + 1) < map_dims[0]) {
        values.push_back(distances[r + 1][c]);
        vector<int> new_rowcol = {r+1, c};
        weights.push_back(interpolator_weight(map_dims, map_bounds, xy, new_rowcol));
    }
    if ((c + 1) < map_dims[1]) {
        values.push_back(distances[r][c + 1]);
        vector<int> new_rowcol = {r, c+1};
        weights.push_back(interpolator_weight(map_dims, map_bounds, xy, new_rowcol));
    }
    double weights_sum = 0;
    for (int i = 0; i < weights.size(); i++) {
        weights_sum += weights[i];
    }

    double interpolated_value = 0;
    for (int i = 0; i < values.size(); i++) {
        interpolated_value += weights[i] * values[i] / weights_sum;
    }
    return interpolated_value;
}

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
        vector<vector<double>>& distances, vector<double>& map_bounds, State& s
    ) {
        vector<MotionPrimitive> primitives;
        vector<size_t> map_dims = {distances.size(), distances[0].size()};
        for (int i = 0; i < n_a; i++) {
            for (int j = 0; j < n_alpha; j++) {
                MotionPrimitive prim;
                double a = (i * 2 * max_a / (n_a - 1)) - max_a;
                double alpha = (j * 2 * max_alpha / (n_alpha - 1)) - max_alpha;
                bool prim_valid = true; 
                double dt = duration / n_t;
                for (int k = 0; k < n_t; k++) {
                    double t = (k + 1) * duration / n_t;
                    double v = s.v + (a * t);
                    if (v > max_v) {
                        v = max_v;
                    }
                    if (v < 0) {
                        v = 0;
                    }
                    double omega = s.omega + (alpha * t);
                    if (omega > max_omega) {
                        omega = max_omega;
                    }
                    if (omega < -max_omega) {
                        omega = -max_omega;
                    }

                    // midpoint integration to find theta, x and y
                    State prev_state;
                    if (prim.states.size() == 0) {
                        prev_state = s;
                    } else {
                        prev_state = prim.states[prim.states.size() - 1];
                    }
                    double dtheta = dt * (prev_state.omega + omega) / 2;
                    double theta = prev_state.theta + dtheta;
                    if (theta < -M_PI) {
                        theta = theta - floor((theta + M_PI) / (2 * M_PI)) * (2 * M_PI);
                    } else if (theta > M_PI) {
                        theta = theta - ceil((theta - M_PI) / (2 * M_PI)) * (2 * M_PI);
                    }
                    double dx = ((prev_state.v + v) / 2) * cos((theta + prev_state.theta) / 2) * dt;
                    double dy = ((prev_state.v + v) / 2) * sin((theta + prev_state.theta) / 2) * dt;
                    double x = prev_state.x + dx;
                    double y = prev_state.y + dy;

                    vector<double> xy = {x, y};
                    if (
                        isinf(interpolator(distances, map_bounds, xy))
                    ) {
                        prim_valid = false;
                        break;
                    } 

                    vector<int> s_t_rowcol = xy_to_rowcol(map_dims, map_bounds, xy);
                    State s_t = {x, y, theta, v, omega, s.t + t};
                    prim.states.push_back(s_t);
                }
                if (prim_valid) {
                    primitives.push_back(prim);
                }
            }
        }
        return primitives;
    }
};

struct PlanningState {
    MotionPrimitive mp;
    double cost;
    double heuristic;
    int depth;
    int backpointer;
};
struct CompareValue { 
    bool operator()(PlanningState const& s1, PlanningState const& s2) { 
        return s1.cost + s1.heuristic > s2.cost + s2.heuristic; 
    } 
};
double heuristic(
    State end_state, State target_state, vector<vector<double>> distances, 
    vector<double> map_bounds, PrimitiveGenerator primitive_generator
) {
    double heur = interpolator(
        distances, map_bounds, {end_state.x, end_state.y}
    );
    heur = max(heur, abs(end_state.v - target_state.v) / primitive_generator.max_a);
    double theta_diff = end_state.theta - target_state.theta;
    if (theta_diff < -M_PI) {
        theta_diff = theta_diff - floor((theta_diff + M_PI) / (2 * M_PI)) * (2 * M_PI);
    } else if (theta_diff > M_PI) {
        theta_diff = theta_diff - ceil((theta_diff - M_PI) / (2 * M_PI)) * (2 * M_PI);
    }
    heur = max(heur, abs(theta_diff) / primitive_generator.max_omega);
    heur = max(heur, abs(end_state.omega - target_state.omega) / primitive_generator.max_alpha);
    return heur;
}
vector<MotionPrimitive> generate_path(
    vector<vector<int>>& obstacle_map, vector<double>& map_bounds,
    State target_state, State initial_state,
    PrimitiveGenerator primitive_generator, int max_depth, double weight
){
    vector<double> target_xy = {target_state.x, target_state.y};
    vector<vector<double>> distances = eight_connected_distance(
        obstacle_map, map_bounds, target_xy
    );
    for (int i = 0; i < distances.size(); i++) {
        for (int j = 0; j < distances[0].size(); j++) {
            distances[i][j] = distances[i][j] / primitive_generator.max_v;
        }
    }

    double init_heuristic = heuristic(
        initial_state, target_state, distances, map_bounds, primitive_generator
    );

    primitive_generator.duration = min(
        primitive_generator.duration, 2 * init_heuristic / max_depth
    );
    cout << primitive_generator.duration << endl;
    primitive_generator.n_t = ceil(20 * primitive_generator.duration);

    priority_queue<PlanningState, vector<PlanningState>, CompareValue> pq;
    vector<PlanningState> all_states;
    MotionPrimitive start_prim;
    start_prim.states.push_back(initial_state);
    PlanningState start;
    start.mp = start_prim;
    start.cost = 0;
    start.depth = 0;
    start.heuristic = weight * init_heuristic;
    start.backpointer = -1;

    PlanningState cur_state = start;
    while (cur_state.depth < max_depth) {
        vector<MotionPrimitive> possible_prims = primitive_generator(
            distances, map_bounds, cur_state.mp.states[cur_state.mp.states.size() - 1]
        );
        all_states.push_back(cur_state);
        for (MotionPrimitive mp: possible_prims) {
            PlanningState new_state;
            new_state.mp = mp;
            double end_t = mp.states[mp.states.size() - 1].t;
            new_state.cost = cur_state.cost + end_t;
            State end_state = mp.states[mp.states.size() - 1];
            new_state.heuristic = weight * heuristic(
                end_state, target_state, distances, map_bounds, primitive_generator
            );
            new_state.depth = cur_state.depth + 1;
            new_state.backpointer = all_states.size() - 1;
            pq.push(new_state);
        }
        if (pq.empty()) {
            cout << "Planner ran out of possible primitives" << endl;
            primitive_generator.duration = primitive_generator.duration / 2;
            primitive_generator.n_t = ceil(20 * primitive_generator.duration);
            continue;
        }
        cur_state = pq.top();
        pq.pop();
        
    }

    // backtrack to get the full path_reversed
    vector<MotionPrimitive> path;
    while (cur_state.backpointer != -1) {
        path.push_back(cur_state.mp);
        cur_state = all_states[cur_state.backpointer];
    }

    reverse(path.begin(), path.end());
    if (path.size() != 0) {
        for (State s : path[0].states) {
            cout << "x: " << s.x << "\t";
            cout << "y: " << s.y << "\t";
            cout << "theta: " << s.theta << "\t";
            cout << "v: " << s.v << "\t";
            cout << "omega: " << s.omega << "\t";
            cout << "t: " << s.t << "\t";
            cout << endl;
        }
    }
    return path;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "planner_node");
    ros::NodeHandle n;

    vector<vector<int>> obstacle_map (11, vector<int> (10, 0));
    obstacle_map[8][5] = 1;
    obstacle_map[7][5] = 1;
    obstacle_map[6][5] = 1;
    obstacle_map[5][5] = 1;
    obstacle_map[4][5] = 1;
    obstacle_map[3][5] = 1;
    obstacle_map[2][5] = 1;
    obstacle_map[1][5] = 1;
    obstacle_map[0][5] = 1;

    vector<double> map_bounds = {0, 10, 0, 9}; 

    std::cout << std::setprecision(2) << std::fixed;
    for (int r = 0; r < obstacle_map.size(); r++) {
        for (int c = 0; c < obstacle_map[0].size(); c++) {
            cout << obstacle_map[r][c] << "\t";
        }
        cout << endl;
    }

    PrimitiveGenerator primitive_generator;
    primitive_generator.duration = 2;
    primitive_generator.n_a = 5;
    primitive_generator.n_alpha = 5;
    primitive_generator.max_v = 2;
    primitive_generator.max_a = 1;
    primitive_generator.max_omega = M_PI / 4;
    primitive_generator.max_alpha = M_PI / 8;

    State initial_state = {0, 0, 0, 0, 0, 0};
    State target_state = {3, 9, -M_PI / 2, 0, 0, 0};

    double weight = 5;
    int max_depth = 3;

    vector<MotionPrimitive> mps = generate_path(
        obstacle_map, map_bounds, target_state, initial_state, 
        primitive_generator, max_depth, weight
    );

    /////// Planner constructs plan here /////
    std::vector<double> px, py, theta, vx, vy, w, ts;
    for (MotionPrimitive mp : mps) {
        for (State s : mp.states) {
            px.push_back(s.x);
            py.push_back(s.y);
            theta.push_back(s.theta);
            vx.push_back(s.v * cos(s.theta));
            vy.push_back(s.v * sin(s.theta));
            w.push_back(s.omega);
            ts.push_back(s.t);
        }
    }

    PlanMsg plan_msg;
    VizMsg viz_msg;
    if(!make_plan_msg(px, py, theta, vx, vy, w, ts, plan_msg, viz_msg)){
        std::cout << "Bad stuff!" << std::endl;
        return -1;
    }
    // Publisher plan_msg to topic
    ros::Publisher planner_pub = n.advertise<PlanMsg>("planner", 1000);
    ros::Rate loop_rate(10);

    int count = 0;
    while (ros::ok()) {
        planner_pub.publish(plan_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}