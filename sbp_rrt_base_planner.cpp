/*=================================================================
 *
 * RRT Planner.cpp
 *
 *=================================================================*/
#include "sbp_rrt_base_planner.h"

#include <iostream>
#include <math.h>
#include <vector>
#include <unordered_map>
#include <random>
#include <chrono>
#include <cassert>
#include <float.h>
#include <cmath>
#include<ctime>

using namespace std;
using namespace std::chrono; 
#define PI 3.141592654

unsigned seed_rrt_base = time(0);
int IsValidArmConfiguration_RRT(vector<double> angles, int numofDOFs, double*	map,
		   int x_size, int y_size);

// Make constructor 
SBP_RRT_base::SBP_RRT_base(double* map, int x_size, int y_size, vector<double> start_position, 
vector<double> goal_position, int numDOF, double epsilon, double dist_goal, int samplenum,
double sample_lowerbound, double sample_upperbound)
{
        m_map = map;
        m_x_size = x_size;
        m_y_size = y_size;
        m_start_position = start_position;
        m_goal_position = goal_position;
        m_numDOF = numDOF;
        m_total_vertices = 0;
        m_epsilon = epsilon;
        m_samplenum = samplenum;
        m_sample_lowerbound = sample_lowerbound;
        m_sample_upperbound = sample_upperbound;
        m_dist_goal = dist_goal;
        m_solution_vertices = 0;
        srand(seed_rrt_base);
    }

// Member function to add a vertex to the tree
void SBP_RRT_base::add_vertex(vector<double> sample_vertex)
{
    // Sample vertex is an array/vector of sample joint configurations. 
    // Every joint configuration has an integer associated to it.
    // Update dict
    vertex_dict[m_total_vertices].joint_config = sample_vertex;
    // Increment total vertices
    m_total_vertices++;
}
// Member function to sample random node
vector <double> SBP_RRT_base::sample_node()
{
    
    vector <double> random_config;
    
    double sample_rate = (double)(rand()%1000)/1000;
    if (sample_rate >0.95)
    {
        return m_goal_position;
    }
    // Get dof number of configs   
    random_config = {};
    for(int i=0; i<m_numDOF; i++)
    {
        double a_random_double = (double)(rand()%100)/100*2*PI;
        random_config.push_back(a_random_double);
    }
    return random_config;
}

// Member function to check if random node is in goal
int SBP_RRT_base::inside_goal(vector<double> random_node)
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
void SBP_RRT_base::execute_path(double*** plan, int* planlength)
{
    vector<int> planned_path;
    int goal = m_total_vertices-1;
    planned_path.push_back(goal);
    while (goal!=0)
    {
        goal = vertex_dict[goal].m_parent;
        planned_path.push_back(goal);
    }
    // Feed path in plan pointer
    m_solution_vertices = planned_path.size();
    *planlength = planned_path.size();
    *plan = NULL; 
    *plan = (double**) malloc(planned_path.size()*sizeof(double*));
    // Iterate over planned path
    for (int j=0;j<planned_path.size();j++)
    {
        (*plan)[j] = (double*) malloc(m_numDOF*sizeof(double)); 
        for(int k=0;k<m_numDOF;k++)
        {
            (*plan)[j][k] = vertex_dict[planned_path[planned_path.size()-1-j]].joint_config[k];
        }
    }
}

// Member function to extend path
int SBP_RRT_base::extend(vector <double> random_node) 
{
    // Find nearest neighbor of random_node in tree
    double min_dist = DBL_MAX;
    int parent_neighbor;
    vector<double> vec_dir;
    vector<double> vec_dir_tmp;
    for(int i=0; i<m_total_vertices; i++)
    {   
        double eu_dist (0);
        vec_dir_tmp = {};
        double angle_diff(0);
        double angle_diff_val(0);
        for(int j=0; j<m_numDOF; j++)
        {                    
            vec_dir_tmp.push_back(random_node[j]-vertex_dict[i].joint_config[j]);
            eu_dist = eu_dist + (random_node[j]-vertex_dict[i].joint_config[j])*(random_node[j]-vertex_dict[i].joint_config[j]);
            
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
            double new_angle = vertex_dict[parent_neighbor].joint_config[k] + dj*vec_dir[k]/min_dist;
            new_vertex.push_back(new_angle);
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
        add_vertex(q_new); // add vertex
        vertex_dict[m_total_vertices-1].m_parent = parent_neighbor; // add edge
        vertex_dict[m_total_vertices-1].relative_cost = find_distance(vertex_dict[m_total_vertices-1].joint_config, vertex_dict[parent_neighbor].joint_config);
        if (inside_goal(q_new))// Check if q_new is in goal region
        {
            return 1; // Reached
        }
        else
        {
            return 0; // Not yet reached
        }              
    }
    else
    {
        return 0 ; //Trapped
    }
}

// Member function to build RRT tree
void SBP_RRT_base::build_RRT(double*** plan, int* planlength)
{
    // Add init to tree
    assert(m_total_vertices == 0);
    add_vertex(m_start_position);
    // Start adding samples
    for(int K=0; K<m_samplenum; K++)
    {
        printf("K = %d\n",K);
        // Generate a random config
        vector<double> random_sample = sample_node();
        // Extend the tree
        if (extend(random_sample))
        {
            printf("K = %d \n",K);
            // Goal has been reached
            execute_path(plan,planlength);
            return;
        }
    }
    printf("No solution found");
}

// Member function to find distance between two configurations
double SBP_RRT_base::find_distance(vector<double> joint_config_1,vector<double> joint_config_2)
{
    double distance(0);
    for (int i=0; i<joint_config_1.size();i++)
    {
        distance += (joint_config_1[i] - joint_config_2[i])*(joint_config_1[i] - joint_config_2[i]);
    }
    distance = sqrt(distance);
    return distance;
}
// Member function to calculate quality of path
double SBP_RRT_base::cost_from_start(int sample_node)
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

// Member function to check quality of path
double SBP_RRT_base::path_quality()
{
    double total_cost(0);
    if (m_solution_vertices > 0)
    {
        total_cost = cost_from_start(m_total_vertices-1); // cost of path
    }
    return total_cost;
}