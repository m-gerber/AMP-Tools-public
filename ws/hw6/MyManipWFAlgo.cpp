#include "MyManipWFAlgo.h"
#include <queue>

amp::Path2D amp::MyManipWFAlgo::planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace) {
    std::pair<double, double> x0_bounds = grid_cspace.x0Bounds();
    std::pair<double, double> x1_bounds = grid_cspace.x1Bounds();

    std::pair<int, int> num_cells = grid_cspace.size();
    std::pair<int, int> goal_cell = grid_cspace.getCellFromPoint(q_goal[0], q_goal[1]);
    std::pair<int, int> init_cell = grid_cspace.getCellFromPoint(q_init[0], q_init[1]);
    double cell_size_x0 = (x0_bounds.second-x0_bounds.first) / static_cast<double>(num_cells.first);
    double cell_size_x1 = (x1_bounds.second-x1_bounds.first) / static_cast<double>(num_cells.second);

    std::pair<int, int> curr_pos = goal_cell;
    std::vector<std::vector<amp::cell2>> graph;
    graph.resize(num_cells.first+1);
    for (int i = 0; i < num_cells.first+1; i++) graph[i].resize(num_cells.second+1);

    graph[curr_pos.first][curr_pos.second].visited = 1;
    graph[curr_pos.first][curr_pos.second].val = 2;
    graph[curr_pos.first][curr_pos.second].parent = {-1,-1};

    std::queue<std::pair<int, int>> q;
    q.push(curr_pos);

    int dx[] = { -1, 0, 1,  0 };
    int dy[] = {  0, 1, 0, -1 };
    int dxr[] = { -1, 0, 1,  0, 1, -1, -1,  1 };
    int dyr[] = {  0, 1, 0, -1, 1,  1, -1, -1 };
    std::pair<int, int> new_pos, reach_pos;

    int iter1 = 0;

    int wrap_ind;

    if (!grid_cspace(init_cell.first, init_cell.second) && !grid_cspace(goal_cell.first, goal_cell.second)) {
        while((curr_pos != init_cell) && !q.empty()) {
            iter1++;
            curr_pos = q.front();
            q.pop();

            for (int i = 0; i < 4; i++) {
                new_pos.first = curr_pos.first + dx[i];
                new_pos.second = curr_pos.second + dy[i];
                if (new_pos.first > num_cells.first) new_pos.first = 0;
                if (new_pos.first < 0) new_pos.first = num_cells.first;
                if (new_pos.second > num_cells.second) new_pos.second = 0;
                if (new_pos.second < 0) new_pos.second = num_cells.second;
                for (int j = 0; j < 8; j++) {
                    reach_pos.first = new_pos.first + 2*dxr[j];
                    reach_pos.second = new_pos.second + 2*dyr[j];
                    if (reach_pos.first >= num_cells.first) reach_pos.first = 0;
                    if (reach_pos.first < 0) reach_pos.first = num_cells.first - 1;
                    if (reach_pos.second >= num_cells.second) reach_pos.second = 0;
                    if (reach_pos.second < 0) reach_pos.second = num_cells.second - 1;
                    if (reach_pos == init_cell) break;
                    if (grid_cspace(reach_pos.first, reach_pos.second)) {
                        graph[new_pos.first][new_pos.second].visited = true;
                        graph[new_pos.first][new_pos.second].val = 1;
                        break;
                    }
                }
                if (graph[new_pos.first][new_pos.second].val == 0) {
                    if (grid_cspace(reach_pos.first, reach_pos.second)) {
                        graph[new_pos.first][new_pos.second].visited = true;
                        graph[new_pos.first][new_pos.second].val = 1;
                    } else {
                        q.push({new_pos.first, new_pos.second});
                        graph[new_pos.first][new_pos.second].visited = true;
                        graph[new_pos.first][new_pos.second].val = graph[curr_pos.first][curr_pos.second].val + 1;
                        graph[new_pos.first][new_pos.second].parent = curr_pos;
                        graph[curr_pos.first][curr_pos.second].child = new_pos;
                    }
                }
            }
        }
    } else return amp::Path2D();

    amp::Path2D path;
    int iter2 = 0;

    path.waypoints.push_back(Eigen::Vector2d(q_init[0], q_init[1]));

    while (curr_pos != goal_cell || iter2 == iter1) {
        iter2++;
        path.waypoints.push_back(Eigen::Vector2d(curr_pos.first*cell_size_x0 + x0_bounds.first,curr_pos.second*cell_size_x1 + x1_bounds.first));
        curr_pos = graph[curr_pos.first][curr_pos.second].parent;
    }
    path.waypoints.push_back(Eigen::Vector2d(q_goal[0], q_goal[1]));

    amp::unwrapPath(path, Eigen::Vector2d(grid_cspace.x0Bounds().first,grid_cspace.x1Bounds().first), Eigen::Vector2d(grid_cspace.x0Bounds().second,grid_cspace.x1Bounds().second));

    return path;
}