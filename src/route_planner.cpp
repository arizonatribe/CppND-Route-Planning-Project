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

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    RouteModel::Node start = model.FindClosestNode(start_x, start_y);
    RouteModel::Node goal = model.FindClosestNode(end_x, end_y);

    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    this->start_node = &start;
    this->end_node = &goal;
}


// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    // Question: Would writing `*end_node` be better than writing `*(this->end_node)` ?
    return node->distance(*(this->end_node));
}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    // Populate the list of current neighbors
    current_node->FindNeighbors();

    // Iterate through the list of unvisited neighhbors
    for (RouteModel::Node* ptr_neighbor : current_node->neighbors) {
        ptr_neighbor->parent = current_node;
        ptr_neighbor->h_value = CalculateHValue(ptr_neighbor);
        ptr_neighbor->g_value = current_node->g_value + ptr_neighbor->distance(*current_node);
        ptr_neighbor->visited = true;
        this->open_list.push_back(ptr_neighbor);
    }
}

bool Compare(RouteModel::Node* a, RouteModel::Node* b) {
    return (a->h_value + a->g_value) > (b->h_value + b->g_value);
}

// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

RouteModel::Node *RoutePlanner::NextNode() {
    if (open_list.size() > 1) {
        sort(open_list.begin(), open_list.end(), Compare);
    }
    RouteModel::Node *lowest_cost_node = open_list.back();
    open_list.pop_back();
    return lowest_cost_node;
}


// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // TODO: Implement your solution here.

    RouteModel::Node *parent = current_node->parent;
    path_found.push_back(*current_node);

    while (parent != nullptr) {
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


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    // TODO: Implement your solution here.

    // First, seed the open list with the starting node.
    // This is because if `start_node` is the same as `end_node`,
    // adding neighboring nodes will be skipped.
    this->open_list.push_back(this->start_node);

    // As long as there are unvisited nodes in the open list...
    while (this->open_list.size() > 0) {
        // Pops the lowest cost node from the open list
        current_node = NextNode();

        // Once the goal is reached, map the final route
        if (current_node == this->end_node) {
            m_Model.path = ConstructFinalPath(this->end_node);
        } else {
            // Otherwise, retrieve any unvisited neighbors of the current node
            AddNeighbors(current_node);
        }
    }
}