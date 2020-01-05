/*=================================================================
 *
 * PRM.cpp
 *
 *=================================================================*/
#include "sbp_prm_planner.h"

#include <iostream>
#include <algorithm>
#include <math.h>
#include <vector>
#include <unordered_map>
#include <random>
#include <chrono>
#include <cassert>
#include <float.h>
#include <cfloat>
#include <utility>
#include <cmath>
#include <set> 
#include <iterator> 

using namespace std;

#define PI 3.141592654

unsigned seed_prm = time(0);
int IsValidArmConfiguration_RRT(vector<double> angles, int numofDOFs, double*	map,
		   int x_size, int y_size);

SBP_PRM::SBP_PRM(double* map, vector<double> start_position, vector<double> goal_position,
    int x_size, int y_size, int numDOFS, int total_nodes, int k_nearest,
        double neighbor_radius)
        {
            m_map = map;
            m_start_position = start_position;
            m_goal_position = goal_position;
            m_x_size = x_size;
            m_y_size = y_size;
            m_numDOFs = numDOFS;
            m_total_nodes = total_nodes;
            m_k_nearest = k_nearest;
            m_neighbor_radius = neighbor_radius;
            m_solution_vertices = 0;
            srand(seed_prm);
        }
// Make member functions
double SBP_PRM::find_distance(vector<double> joint_config_1,vector<double> joint_config_2)
{
    double distance(0);
    for (int i=0; i<joint_config_1.size();i++)
    {
        distance += (joint_config_1[i] - joint_config_2[i])*(joint_config_1[i] - joint_config_2[i]);
    }
    distance = sqrt(distance);
    return distance;
}
// Helper function to check if edge connection can be made
int SBP_PRM::obstacle_free(int sample_1, int sample_2)
{
    double dj = 0.05;
    double total_dist = find_distance(prm_graph[sample_1].joint_config, prm_graph[sample_2].joint_config);
    vector<double> config_1 = prm_graph[sample_1].joint_config;
    vector<double> config_2 = prm_graph[sample_2].joint_config;
    while(dj<=total_dist)
    {
        vector<double> new_config;
        for(int i=0; i<m_numDOFs;i++)
        {
            new_config.push_back(config_1[i] + dj*(config_2[i]-config_1[i])/total_dist);
        }
        if (!IsValidArmConfiguration_RRT(new_config,m_numDOFs,m_map,m_x_size,m_y_size))
            return 0;
        dj +=dj;
    }
    return 1;
}
// Function to get random configuration
vector <double> SBP_PRM::get_random_config()
{
    
    vector <double> random_config;
    // Get dof number of configs
    for(int i=0; i<m_numDOFs; i++)
    {
        // double a_random_double = unif(myRandomEngine); 
        double a_random_double = (double)(rand()%100)/100*2*PI;
        random_config.push_back(a_random_double);
    }
    return random_config;
}
// Function to construct roadmap
void SBP_PRM::construct_roadmap()
{
    // Sample total nodes
    int num_nodes(0);
    while (num_nodes <= m_total_nodes)
    {
        vector<double> random_config; // Define vector for holding joint config
        if (num_nodes==0)
            random_config = m_start_position;
        else if (num_nodes==1)
            random_config = m_goal_position;
        else
            random_config = get_random_config();

        if(IsValidArmConfiguration_RRT(random_config,m_numDOFs,m_map,m_x_size,m_y_size))
        {
            prm_graph[num_nodes].joint_config = random_config; // Add config to graph
            num_nodes++; // Increment number of nodes in graph

        // Make edges by finding nearest neighbors
        // Last node added will have index num_nodes-1. We need to find neighbors of num_nodes-1.
        // So iterate till before num_nodes-1 so you don't add yourself as neighbor
            for (int i=0; i<num_nodes-1;i++) 
            {
                double neighbor_distance = find_distance(prm_graph[num_nodes-1].joint_config, prm_graph[i].joint_config);
                if (neighbor_distance < m_neighbor_radius)
                {
                    if (obstacle_free(i,num_nodes-1))
                    {
                        // Add edge from current node to iter node
                        prm_graph[num_nodes-1].edges.push_back(i);
                        prm_graph[num_nodes-1].edge_costs.push_back(neighbor_distance);
                        // Add edge from iter node to current node
                        prm_graph[i].edges.push_back(num_nodes-1);
                        prm_graph[i].edge_costs.push_back(neighbor_distance);
                    }
                }
            }
        }

    }
}

// Functions for graph search

void SBP_PRM::solutionPath(int goal, double*** plan, int* planlength)
{ 
    vector<int> planned_path;
    int current_index = goal;
    // Solution reached when parent index is equal to current index
    while (prm_graph[current_index].parent != current_index)
    {   
        planned_path.push_back(current_index);
        current_index = prm_graph[current_index].parent;
    }
    planned_path.push_back(current_index);
    reverse(planned_path.begin(),planned_path.end());

    m_solution_vertices = planned_path.size(); 

    *planlength = planned_path.size();
    *plan = NULL; 
    *plan = (double**) malloc(planned_path.size()*sizeof(double*));
    // Iterate over planned path
    for (int j=0;j<planned_path.size();j++)
    {
        (*plan)[j] = (double*) malloc(m_numDOFs*sizeof(double)); 
        for(int k=0;k<m_numDOFs;k++)
        {
            (*plan)[j][k] = prm_graph[planned_path[j]].joint_config[k];
        }
    }
}

// Dijkstra search to find the path
void SBP_PRM::dijkstra_search(int start_node, int goal_node, double*** plan, int* planlength)
{
    // Initialize the closed list to keep track of expanded nodes
    unordered_map <int, bool> closed_list;

    // Inititalize the start cell. It has no parents and its f, g & h values are 0
    prm_graph[start_node].f = 0;
    prm_graph[start_node].g = 0;
    prm_graph[start_node].h = 0;
    prm_graph[start_node].parent = start_node; // start cell as its parent as itself


    // Implement the open list to keep track of states to expand using a set
    // It is a set of f_COORINATE, i.e it has location of state and its f value
    set<f_COORDINATE> open_list; 
    // Add my start cell to my open list
    open_list.insert(make_pair (0.0, start_node)); 


    // Expand till open list is not empty
    while(!open_list.empty())
    {   
        // Pick index with lowest f value from open list. Set will help in doing this as it is ordered.
        //Put this in closed list.
        // Find neighbors of my current index and find f values only if they are not in closed list.
        // If they are not in closed list, find their f-values. If they are in the open list with a larger
        // f-value then update it, otherwise add this index to the open list. 
        // Loop till goal state has not been expanded.

        // Get index from openlist. Pop the first value from the open list.
        f_COORDINATE q = *open_list.begin();
        // Remove it from the open list
        open_list.erase(open_list.begin());
        // Get index of this node
        int q_current = q.second;
        closed_list[q_current] = true; 

        // Loop through neighbors
        for (int edg = 0; edg < prm_graph[q_current].edges.size(); edg++)
        {
            int q_new = prm_graph[q_current].edges[edg]; // Get successor
    
            double fNew, gNew, hNew; // Variables used to find f, g & h values
            
            // Only proceed if it is not in closed list
            if (closed_list[q_new] != true)
            {
                // Compute fNew, gNew, hNew.
                gNew = prm_graph[q_current].g + prm_graph[q_current].edge_costs[edg];
                hNew = 0;
                fNew = gNew + hNew;

                if (prm_graph[q_new].f == DBL_MAX || prm_graph[q_new].f > fNew)
                {
                    open_list.insert(make_pair (fNew, q_new));
                    prm_graph[q_new].f = fNew;
                    prm_graph[q_new].g = gNew;
                    prm_graph[q_new].h = hNew;
                    prm_graph[q_new].parent = q_current;
                }
            }
        }    
    }
    if (closed_list[goal_node])
    {
        solutionPath(goal_node,plan,planlength);
        return;
    }
    else
    {
        printf("No solution found");
        solutionPath(start_node,plan,planlength);
        return;
    }
}
// Member function to calculate quality of path
double SBP_PRM::cost_from_start(int sample_node)
{
    double cost(0);
    int vertex = sample_node;
    while (vertex!=0)
    {
        cost += prm_graph[vertex].f;
        vertex = prm_graph[vertex].parent;
    }
    return cost;
}
// Member function to check quality of path
double SBP_PRM::path_quality()
{
    double total_cost(0);
    if (m_solution_vertices > 0)
    {
        total_cost = cost_from_start(1); // cost of path
    }
    return total_cost;
}  



