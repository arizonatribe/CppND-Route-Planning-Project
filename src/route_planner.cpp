#include "route_planner.h"
#include <algorithm>
using std::sort;
using std::reverse;

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    RouteModel::Node *start = &model.FindClosestNode(start_x, start_y);
    RouteModel::Node *goal = &model.FindClosestNode(end_x, end_y);

    start_node = start;
    end_node = goal;
}


float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}


void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    // Populate the list of current neighbors
    current_node->FindNeighbors();

    // Iterate through the list of unvisited neighhbors
    for (auto* ptr_neighbor : current_node->neighbors) {
        ptr_neighbor->parent = current_node;
        ptr_neighbor->h_value = CalculateHValue(ptr_neighbor);
        ptr_neighbor->g_value = current_node->g_value + ptr_neighbor->distance(*current_node);
        ptr_neighbor->visited = true;
        open_list.emplace_back(ptr_neighbor);
    }
}


RouteModel::Node *RoutePlanner::NextNode() {
    if (open_list.size() > 1) {
        sort(
            open_list.begin(),
            open_list.end(),
            [] (const RouteModel::Node* a, const RouteModel::Node* b) {
                return a->h_value + a->g_value > b->h_value + b->g_value;
            }
        );
    }
    RouteModel::Node *lowest_cost_node = open_list.back();
    open_list.pop_back();
    return lowest_cost_node;
}


std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    RouteModel::Node *parent = current_node->parent;
    path_found.push_back(*current_node);

    while (current_node != start_node) {
        distance += current_node->distance(*parent);
        current_node = parent;
        path_found.push_back(*current_node);
        parent = current_node->parent;
    }

    // The path of nodes should be ordered from starting node to ending node
    reverse(path_found.begin(), path_found.end());

    // Multiply the distance by the scale of the map to get meters.
    distance *= m_Model.MetricScale();
    return path_found;

}


void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    // First, seed the open list with the starting node.
    // This is because if `start_node` is the same as `end_node`,
    // adding neighboring nodes will be skipped.
    open_list.emplace_back(start_node);

    // Mark the starting node as visited,
    // all the neighboring nodes will themselves
    // be marked visited as they're expanded
    start_node->visited = true;

    while (!open_list.empty()) {
        // Pops the lowest cost node from the open list
        current_node = NextNode();

        // Once the goal is reached, map the final route
        if (current_node == end_node) {
            m_Model.path = ConstructFinalPath(end_node);
        } else {
            // Otherwise, retrieve any unvisited neighbors of the current node
            AddNeighbors(current_node);
        }
    }
}