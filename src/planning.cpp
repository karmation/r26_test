#include "planning.h"
#include <vector>
#include <queue>
#include <map>
#include <algorithm>
#include <cmath>

using namespace std;

// Node structure for A* algorithm.
// Stores position, costs (g, h, f), and parent pointer.
struct Node {
    int x, y;
    double g_cost, h_cost, f_cost;
    int parent_x, parent_y;

    // Overload the > operator for the priority queue.
    // The priority queue is a max-heap by default, so we use >
    // to make it behave like a min-heap for f_cost.
    bool operator>(const Node& other) const {
        return f_cost > other.f_cost;
    }
};

Planner::Planner(const vector<vector<bool>> &grid) : grid(grid) {
  rows = grid.size();
  if (rows > 0) {
    cols = grid[0].size();
  } else {
    cols = 0;
  }
}

bool Planner::isvalid(int x, int y) const {
  // Check if the cell is within grid bounds and is not an obstacle.
  // In the given grid, 'false' means traversable, 'true' means obstacle.
  return (x >= 0 && x < rows && y >= 0 && y < cols && !grid[x][y]);
}

double Planner::heuristic(int x1, int y1, int x2, int y2) const {
  // Euclidean distance heuristic.
  return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}

vector<pair<int, int>> Planner::pathplanning(pair<int, int> start, pair<int, int> goal) {
    vector<pair<int, int>> path;
    
    // A* implementation
    priority_queue<Node, vector<Node>, greater<Node>> open_list;
    map<pair<int, int>, Node> all_nodes;

    Node start_node;
    start_node.x = start.first;
    start_node.y = start.second;
    start_node.g_cost = 0;
    start_node.h_cost = heuristic(start.first, start.second, goal.first, goal.second);
    start_node.f_cost = start_node.g_cost + start_node.h_cost;
    start_node.parent_x = -1;
    start_node.parent_y = -1;

    open_list.push(start_node);
    all_nodes[{start.first, start.second}] = start_node;

    while (!open_list.empty()) {
        Node current_node = open_list.top();
        open_list.pop();

        // If we reached the goal, reconstruct the path
        if (current_node.x == goal.first && current_node.y == goal.second) {
            pair<int, int> current_point = {current_node.x, current_node.y};
            while (current_point.first != -1) {
                path.push_back(current_point);
                Node& n = all_nodes[current_point];
                current_point = {n.parent_x, n.parent_y};
            }
            reverse(path.begin(), path.end());
            return path;
        }

        // Explore neighbors (8 directions)
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                if (dx == 0 && dy == 0) continue;

                int neighbor_x = current_node.x + dx;
                int neighbor_y = current_node.y + dy;

                if (isvalid(neighbor_x, neighbor_y)) {
                    double move_cost = (dx != 0 && dy != 0) ? 1.414 : 1.0; // Diagonal vs. Cardinal
                    double new_g_cost = current_node.g_cost + move_cost;

                    if (all_nodes.find({neighbor_x, neighbor_y}) == all_nodes.end() || new_g_cost < all_nodes[{neighbor_x, neighbor_y}].g_cost) {
                        Node neighbor_node;
                        neighbor_node.x = neighbor_x;
                        neighbor_node.y = neighbor_y;
                        neighbor_node.g_cost = new_g_cost;
                        neighbor_node.h_cost = heuristic(neighbor_x, neighbor_y, goal.first, goal.second);
                        neighbor_node.f_cost = neighbor_node.g_cost + neighbor_node.h_cost;
                        neighbor_node.parent_x = current_node.x;
                        neighbor_node.parent_y = current_node.y;
                        
                        open_list.push(neighbor_node);
                        all_nodes[{neighbor_x, neighbor_y}] = neighbor_node;
                    }
                }
            }
        }
    }

    return path; // Return empty path if goal is not reachable
}
