#include "route_planner.h"
#include <algorithm>


RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
return node->distance(*end_node);
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
current_node->FindNeighbors();
for (const auto neighbour : current_node->neighbors)
{
    neighbour->parent = current_node; 
    neighbour->g_value = current_node->g_value+current_node->distance(*neighbour);
    neighbour->h_value = CalculateHValue(neighbour);
    open_list.push_back(neighbour);
    neighbour->visited = true;
}
}


bool Sum_h_g(const RouteModel::Node* x, const RouteModel::Node* y) 
{
    float comp1 = x->g_value + x->h_value;
    float comp2 = y->g_value + y->h_value;
    return comp1 > comp2;
} 
RouteModel::Node *RoutePlanner::NextNode() {
   
std::sort(open_list.begin(), open_list.end(), Sum_h_g);
RouteModel::Node* lowest_Sum;
lowest_Sum = open_list.back();
open_list.pop_back();
return lowest_Sum;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // TODO: Implement your solution here.
    while(current_node!= start_node)
    {
        
        distance += current_node->distance(*current_node->parent);
        path_found.push_back(*current_node);
      current_node = current_node->parent;
    }
    path_found.push_back(*current_node);
   std::reverse(path_found.begin(), path_found.end()); 
   
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
 

}  

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;
    
    start_node->visited = true;
    open_list.push_back(start_node); 

    while (open_list.size() != 0)
    {
       
        current_node = NextNode();
        if(current_node->distance(*end_node) == NULL)
        {
            m_Model.path = ConstructFinalPath(current_node);
            return;
        }
           AddNeighbors(current_node);  

    }
}       
