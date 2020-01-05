/*=================================================================
 *
 * RRT Connect.cpp
 *
 *=================================================================*/
#include "sbp_rrt_connect_planner.h"

#include <iostream>
#include <math.h>
#include <algorithm>
#include <vector>
#include <unordered_map>
#include <random>
#include <chrono>
#include <cassert>
#include <float.h>
#include <cmath>
#include<ctime>
using namespace std;

#define PI 3.141592654

unsigned seed_rrt_connect = time(0);
int IsValidArmConfiguration_RRT(vector<double> angles, int numofDOFs, double*	map,
		   int x_size, int y_size);
   
SBP_RRT_Connect::SBP_RRT_Connect(double* map, int x_size, int y_size, vector<double> start_position, 
vector<double> goal_position, int numDOF, double epsilon, double dist_goal, int samplenum,
double sample_lowerbound, double sample_upperbound)
{
    m_map = map;
    m_x_size = x_size;
    m_y_size = y_size;
    m_start_position = start_position;
    m_goal_position = goal_position;
    m_numDOF = numDOF;
    m_total_vertices_T1 = 0;
    m_total_vertices_T2 = 0;
    m_epsilon = epsilon;
    m_samplenum = samplenum;
    m_sample_lowerbound = sample_lowerbound;
    m_sample_upperbound = sample_upperbound;
    m_dist_goal = dist_goal;
    m_solution_vertices = 0;
    srand(seed_rrt_connect);
}
// Member function to add a vertex to the tree
void SBP_RRT_Connect::add_vertex(vector<double> sample_vertex, int tree_val)
{
    // Sample vertex is an array/vector of sample joint configurations. 
    // Every joint configuration has an integer associated to it.
    // Update dict
    if (tree_val == 1)
    {
        vertex_dict_T1[m_total_vertices_T1].joint_config = sample_vertex;
        // Increment total vertices
        m_total_vertices_T1++;
    }
    if (tree_val == 2)
    {
        vertex_dict_T2[m_total_vertices_T2].joint_config = sample_vertex;
        // Increment total vertices
        m_total_vertices_T2++;
    }
    
}
// Member function to sample random joint config
vector <double> SBP_RRT_Connect::sample_node()
{
    
    vector <double> random_config;
    // Get dof number of configs
    for(int i=0; i<m_numDOF; i++)
    {
        double a_random_double = (double)(rand()%100)/100*2*PI;
        random_config.push_back(a_random_double);
    }
    return random_config;
}
// Member function to determine if node is in goal region
int SBP_RRT_Connect::inside_goal(vector<double> random_node)
{
    double eu_dist_goal(0);
    double angle_diff_goal(0);
    for(int i=0;i<m_numDOF;i++)
    {
        eu_dist_goal = eu_dist_goal + (m_goal_position[i] - random_node[i])*(m_goal_position[i] - random_node[i]);
    }
    eu_dist_goal = sqrt(eu_dist_goal);
    if (eu_dist_goal <= m_dist_goal)
    {
            return 1;
    }
    else
    {
            return 0;
    }
}
// Member function to execute path
void SBP_RRT_Connect::execute_path(double*** plan, int* planlength)
{
    // printf("Here");
    // printf("num vetices=%d",m_vertices.size());
    vector<int> planned_path;
    // Feed planned path from Tree 1
    int goal_T1 = m_total_vertices_T1-1;
    planned_path.push_back(goal_T1);
    while (goal_T1!=0)
    {
        goal_T1 = vertex_dict_T1[goal_T1].m_parent;
        planned_path.push_back(goal_T1);
    }
    int tree_1_plan = planned_path.size();
    reverse(planned_path.begin(), planned_path.end());
    // Feed planned path from Tree 2
    int goal_T2 = m_total_vertices_T2 -1;
    while (goal_T2!=0)
    {
        goal_T2 = vertex_dict_T2[goal_T2].m_parent;
        planned_path.push_back(goal_T2);
    }

    m_solution_vertices = planned_path.size(); // Total nodes in solution

    // Feed path in plan pointer
    *planlength = planned_path.size();
    *plan = NULL; 
    *plan = (double**) malloc(planned_path.size()*sizeof(double*));
    // Iterate over planned path
    for (int j=0;j<planned_path.size();j++)
    {
        (*plan)[j] = (double*) malloc(m_numDOF*sizeof(double)); 
        for(int k=0;k<m_numDOF;k++)
        {
            if (j<tree_1_plan)
                (*plan)[j][k] = vertex_dict_T1[planned_path[j]].joint_config[k];
            else
                (*plan)[j][k] = vertex_dict_T2[planned_path[j]].joint_config[k];;
            
        }
    }
}
// Member function to build RRT Connect Trees
void SBP_RRT_Connect::build_RRT(double*** plan, int* planlength)
{
    // Add init to tree 1
    assert(m_total_vertices_T1 == 0);
    add_vertex(m_start_position,1);
    vertex_dict_T1[m_total_vertices_T1-1].relative_cost = 0;
    // Add goal to tree 2
    assert(m_total_vertices_T2 == 0);
    add_vertex(m_goal_position,2);
    vertex_dict_T2[m_total_vertices_T2-1].relative_cost = 0;

    // Start adding samples
    for(int K=0; K<m_samplenum; K++)
    {
        printf("K = %d\n",K);
        // Generate a random config
        vector<double> random_sample = sample_node();
        // Extend the tree
        if (K%2 == 0)
        {
            if (extend_T1(random_sample))
            {
                printf("K = %d \n",K);
                // Goal has been reached
                execute_path(plan,planlength);
                return;
            }
        }
        else
        {
            if (extend_T2(random_sample))
            {
                printf("K = %d \n",K);
                // Goal has been reached
                execute_path(plan,planlength);
                return;
            }
        }
    }
    printf("No solution found");
    

}
// Member function for extend RRT Connect for Tree 1
int SBP_RRT_Connect::extend_T1(vector <double> random_node) 
{
    // Find nearest neighbor of random_node in tree
    double min_dist = DBL_MAX;
    int parent_neighbor;
    vector<double> vec_dir;
    vector<double> vec_dir_tmp;
    for(int i=0; i<m_total_vertices_T1; i++)
    {   
        double eu_dist (0);
        vec_dir_tmp = {};
        for(int j=0; j<m_numDOF; j++)
        {
            vec_dir_tmp.push_back(random_node[j]-vertex_dict_T1[i].joint_config[j]);
            eu_dist = eu_dist + (random_node[j]-vertex_dict_T1[i].joint_config[j])*(random_node[j]-vertex_dict_T1[i].joint_config[j]);
        }

        eu_dist = sqrt(eu_dist);
        // Check if least dist
        if (eu_dist<min_dist)
        {
            parent_neighbor = i;
            min_dist = eu_dist;   
            vec_dir = vec_dir_tmp;
        }
    }
    // Nearest neighbor found. Interpolate to find vertex to add
    // Interpolate in the direction of the vector connecting parent and sample node till epsilon
    double interp_step = 0.05;
    double dj = interp_step;
    
    vector<double> q_new;
    while (dj<=m_epsilon && dj<=min_dist)
    {
        vector<double> new_vertex;    
        for(int k=0; k<m_numDOF;k++)
        {
            new_vertex.push_back(vertex_dict_T1[parent_neighbor].joint_config[k] + dj*vec_dir[k]/min_dist);
        }
        if (IsValidArmConfiguration_RRT(new_vertex, m_numDOF, m_map, m_x_size, m_y_size))
        {
            q_new = new_vertex;
        }
        else
        {
            break;
        }           
        dj = dj + interp_step;
    }

    if (q_new.size()!=0)
    {
        add_vertex(q_new,1); // add vertex to tree 1
        vertex_dict_T1[m_total_vertices_T1-1].m_parent = parent_neighbor; // add edge
        vertex_dict_T1[m_total_vertices_T1-1].relative_cost = find_distance(vertex_dict_T1[m_total_vertices_T1-1].joint_config,vertex_dict_T1[parent_neighbor].joint_config);
        // Run connect function for tree 2
        vector<double> new_node = connect_T2(q_new);
        if (new_node.size()!=0)
        {
            // Check if new_node is q_new --> RRT connect found a path
            if (find_distance(q_new, new_node) < 0.001)
            {
                return 1;
            }
            else
            {
                return 0;
            }
            
        }
        else
        {
            return 0; // Trapped
        }           
    }
    else
    {
        return 0 ; //Trapped
    }
}
// Member function for extend RRT Connect for Tree 2
int SBP_RRT_Connect::extend_T2(vector <double> random_node)
{
    // Find nearest neighbor of random_node in tree
    double min_dist = DBL_MAX;
    int parent_neighbor;
    vector<double> vec_dir;
    vector<double> vec_dir_tmp;
    for(int i=0; i<m_total_vertices_T2; i++)
    {   
        double eu_dist (0);
        vec_dir_tmp = {};
        for(int j=0; j<m_numDOF; j++)
        {
            vec_dir_tmp.push_back(random_node[j]-vertex_dict_T2[i].joint_config[j]);
            eu_dist = eu_dist + (random_node[j]-vertex_dict_T2[i].joint_config[j])*(random_node[j]-vertex_dict_T2[i].joint_config[j]);
        }

        eu_dist = sqrt(eu_dist);
        // Check if least dist
        if (eu_dist<min_dist)
        {
            parent_neighbor = i;
            min_dist = eu_dist;   
            vec_dir = vec_dir_tmp;
        }
    }
    // Nearest neighbor found. Interpolate to find vertex to add
    // Interpolate in the direction of the vector connecting parent and sample node till epsilon
    double interp_step = 0.05;
    double dj = interp_step;
    
    vector<double> q_new;
    while (dj<=m_epsilon && dj<=min_dist)
    {
        vector<double> new_vertex;    
        for(int k=0; k<m_numDOF;k++)
        {
            new_vertex.push_back(vertex_dict_T2[parent_neighbor].joint_config[k] + dj*vec_dir[k]/min_dist);
        }
        if (IsValidArmConfiguration_RRT(new_vertex, m_numDOF, m_map, m_x_size, m_y_size))
        {
            q_new = new_vertex;
        }
        else
        {
            break;
        }           
        dj = dj + interp_step;
    }

    if (q_new.size()!=0)
    {
        add_vertex(q_new,2); // add vertex to tree 2
        vertex_dict_T2[m_total_vertices_T2-1].m_parent = parent_neighbor; // add edge
        vertex_dict_T2[m_total_vertices_T2-1].relative_cost = find_distance(vertex_dict_T2[m_total_vertices_T2-1].joint_config,vertex_dict_T2[parent_neighbor].joint_config);
        // Run connect function for tree 1
        vector<double> new_node = connect_T1(q_new);
        if (new_node.size()!=0)
        {
            // Check if new_node is q_new --> RRT connect found a path
            if (find_distance(q_new, new_node) < 0.001)
            {
                return 1;
            }
            else
            {
                return 0;
            }
            
        }
        else
        {
            return 0; // Trapped
        }           
    }
    else
    {
        return 0 ; //Trapped
    }
}

// Member function for connect for RRT-Connect for Tree 2
vector<double> SBP_RRT_Connect::connect_T2(vector<double> x_new)
{
    // x_new is added to Tree 1. Then find nearest neighbors of x_new in other tree. Connect as far as possible
    double min_dist = DBL_MAX;
    int nearest_neighbor;
    for (int i=0; i<m_total_vertices_T2; i++)
    {
        double dist_val = find_distance(x_new, vertex_dict_T2[i].joint_config);
        if (dist_val < min_dist)
        {
            min_dist = dist_val;
            nearest_neighbor = i;
        }
    }
    // Try to connect to nearest neighbor as much as possible
    double dj = 0.05;
    vector<double> new_connect_node = {};
    while (dj <= min_dist)
    {
        vector<double> new_config;
        for (int j=0;j<m_numDOF;j++)
        {
            new_config.push_back(vertex_dict_T2[nearest_neighbor].joint_config[j] + dj*(x_new[j]-vertex_dict_T2[nearest_neighbor].joint_config[j])/min_dist);
        }
        if (IsValidArmConfiguration_RRT(new_config,m_numDOF,m_map,m_x_size,m_y_size))
        {
            new_connect_node = new_config;
        }
        else
        {
            break;
        }
        
        dj+=dj;
    }
    if (new_connect_node.size()!=0)
    {
        add_vertex(new_connect_node,2);
        vertex_dict_T2[m_total_vertices_T2-1].m_parent = nearest_neighbor;
        vertex_dict_T2[m_total_vertices_T2-1].relative_cost = dj;
    }

    return new_connect_node;
}
// Member function for connect for RRT-Connect for Tree 1
vector<double> SBP_RRT_Connect::connect_T1(vector<double> x_new)
{
    // x_new is added to Tree 2. Then find nearest neighbors of x_new in tree 1. Connect as far as possible
    double min_dist = DBL_MAX;
    int nearest_neighbor;
    for (int i=0; i<m_total_vertices_T1; i++)
    {
        double dist_val = find_distance(x_new, vertex_dict_T1[i].joint_config);
        if (dist_val < min_dist)
        {
            min_dist = dist_val;
            nearest_neighbor = i;
        }
    }
    // Try to connect to nearest neighbor as much as possible
    double dj = 0.05;
    vector<double> new_connect_node = {};
    while (dj <= min_dist)
    {
        vector<double> new_config;
        for (int j=0;j<m_numDOF;j++)
        {
            new_config.push_back(vertex_dict_T1[nearest_neighbor].joint_config[j] + dj*(x_new[j]-vertex_dict_T1[nearest_neighbor].joint_config[j])/min_dist);
        }
        if (IsValidArmConfiguration_RRT(new_config,m_numDOF,m_map,m_x_size,m_y_size))
        {
            new_connect_node = new_config;
        }
        else
        {
            break;
        }
        
        dj+=dj;
    }
    if (new_connect_node.size()!=0)
    {
        add_vertex(new_connect_node,1);
        vertex_dict_T1[m_total_vertices_T1-1].m_parent = nearest_neighbor;
        vertex_dict_T1[m_total_vertices_T1-1].relative_cost = dj;
    }

    return new_connect_node;
}

// Member function to judge quality of path
double SBP_RRT_Connect::cost_from_start(int sample_node, unordered_map<int, m_tree_node> vertex_dict)
{
    double cost(0);
    int vertex = sample_node;
    while (vertex!=0)
    {
        cost += vertex_dict[vertex].relative_cost;
        vertex = vertex_dict[vertex].m_parent;
    }
    return cost;
}
// Member helper function to find Euclidean distance between two joint configurations
double SBP_RRT_Connect::find_distance(vector<double> joint_config_1,vector<double> joint_config_2)
{
    double distance(0);
    for (int i=0; i<joint_config_1.size();i++)
    {
        distance += (joint_config_1[i] - joint_config_2[i])*(joint_config_1[i] - joint_config_2[i]);
    }
    distance = sqrt(distance);
    return distance;
}   
// Member function to check quality of path
double SBP_RRT_Connect::path_quality()
{
    double total_cost(0);
    if (m_solution_vertices > 0)
    {
        total_cost = cost_from_start(m_total_vertices_T1-1,vertex_dict_T1); 
        total_cost = total_cost + cost_from_start(m_total_vertices_T2-1,vertex_dict_T2);// cost of path
    }
    return total_cost;
}     