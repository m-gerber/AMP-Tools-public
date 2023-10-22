#include "MyPointWFAlgo.h"
#include "MyGridCSpace.h"
#include "tools/Path.h"
#include <queue>

/// @brief Create a discretized planning space of an environment (point agent). If you abstracted your GridCSpace2DConstructor, you may be
/// able to use that code here to construct a discretized C-space for a point agent.
/// @param environment Workspace and/or C-space (point agent)
/// @return Unique pointer to a GridCSpace2D object (see HW4)
std::unique_ptr<amp::GridCSpace2D> amp::MyPointWFAlgo::constructDiscretizedWorkspace(const amp::Environment2D& env) {
    std::unique_ptr<amp::MyGridCSpace> ptr(new MyGridCSpace(round((env.x_max-env.x_min)*4), round((env.y_max-env.y_min)*4), env.x_min, env.x_max, env.y_min, env.y_max));
    ptr->buildPointCSpace(env);
    return ptr;
}

int amp::MyPointWFAlgo::isValid(std::pair<int, int> inds, std::vector<std::vector<amp::cell>> graph, const amp::GridCSpace2D& grid_cspace) {
    if (inds.first < 0 || inds.second < 0 || inds.first > graph.size()-1 || inds.second > graph[0].size()-1) return 0;
    if (grid_cspace.operator()(inds.first,inds.second)) return -1;
    if (graph[inds.first][inds.second].visited) return 0;
    return 1;
}

amp::Path2D amp::MyPointWFAlgo::planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace) {
    std::pair<double, double> x0_bounds = grid_cspace.x0Bounds();
    std::pair<double, double> x1_bounds = grid_cspace.x1Bounds();

    std::pair<int, int> num_cells = grid_cspace.size();
    std::pair<int, int> goal_cell = grid_cspace.getCellFromPoint(q_goal[0], q_goal[1]);
    std::pair<int, int> init_cell = grid_cspace.getCellFromPoint(q_init[0], q_init[1]);

    double cell_size_x0 = (x0_bounds.second-x0_bounds.first) / static_cast<double>(num_cells.first);
    double cell_size_x1 = (x1_bounds.second-x1_bounds.first) / static_cast<double>(num_cells.second);

    std::pair<int, int> curr_pos = goal_cell;
    std::vector<std::vector<amp::cell>> graph;
    graph.resize(num_cells.first);
    for (int i = 0; i < num_cells.first; i++) graph[i].resize(num_cells.second);
    graph[curr_pos.first][curr_pos.second].visited = 1;
    graph[curr_pos.first][curr_pos.second].val = 2;
    graph[curr_pos.first][curr_pos.second].parent = {-1,-1};

    std::queue<std::pair<int, int>> q;
    q.push(curr_pos);

    int dx[] = { -1, 0, 1,  0 };
    int dy[] = {  0, 1, 0, -1 };
    std::pair<int, int> new_pos;

    int iter1 = 0;

    int wrap_ind;

    while((curr_pos != init_cell) && !q.empty()) {
        iter1++;
        curr_pos = q.front();
        q.pop();

        for (int i = 0; i < 4; i++) {
            new_pos.first = curr_pos.first + dx[i];
            new_pos.second = curr_pos.second + dy[i];
            if (isValid({new_pos.first, new_pos.second}, graph, grid_cspace) == 1) {
                q.push({new_pos.first, new_pos.second});
                graph[new_pos.first][new_pos.second].visited = true;
                graph[new_pos.first][new_pos.second].val = graph[curr_pos.first][curr_pos.second].val + 1;
                graph[new_pos.first][new_pos.second].parent = curr_pos;
                graph[curr_pos.first][curr_pos.second].child = new_pos;
            } else if (isValid({new_pos.first, new_pos.second}, graph, grid_cspace) == -1) {
                graph[new_pos.first][new_pos.second].visited = true;
                graph[new_pos.first][new_pos.second].val = 1;
            }
            
        }
    }

    amp::Path2D path;
    int iter2 = 0;

    path.waypoints.push_back(Eigen::Vector2d(q_init[0], q_init[1]));

    while (curr_pos != goal_cell || iter2 == iter1) {
        iter2++;
        path.waypoints.push_back(Eigen::Vector2d(curr_pos.first*cell_size_x0 + x0_bounds.first,curr_pos.second*cell_size_x1 + x1_bounds.first));
        curr_pos = graph[curr_pos.first][curr_pos.second].parent;
    }
    path.waypoints.push_back(Eigen::Vector2d(q_goal[0], q_goal[1]));

    return path;
}