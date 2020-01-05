#include "sbp_rrt_star_planner.h"

#include <iostream>
#include <math.h>
#include <vector>
/*=================================================================
 *
 * RRT Star.cpp
 *
 *=================================================================*/
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

unsigned seed_rrt_star = time(0);


// Make constructor 
// plan was first argument --added rrt build input, execute path, exec path def   
SBP_RRT_Star::SBP_RRT_Star(double* map, int x_size, int y_size, vector<double> start_position, 
vector<double> goal_position, int numDOF, double epsilon, double dist_goal, int samplenum,
double sample_lowerbound, double sample_upperbound)
{
    m_map = map;
    m_x_size = x_size;
    m_y_size = y_size;
    m_start_position = start_position;
    m_goal_position = goal_position;
    m_numDOF = numDOF;
    // m_plan = plan;
    // m_planlength = planlength;
    m_total_vertices = 0;
    m_epsilon = epsilon;
    m_samplenum = samplenum;
    m_sample_lowerbound = sample_lowerbound;
    m_sample_upperbound = sample_upperbound;
    m_dist_goal = dist_goal;
    m_solution_vertices = 0;
    srand(seed_rrt_star);
}
// Member function to add a vertex to the tree
void SBP_RRT_Star::add_vertex(vector<double> sample_vertex)
{
    // Sample vertex is an array/vector of sample joint configurations. 
    // Every joint configuration has an integer associated to it.
    // Update dict
    vertex_dict[m_total_vertices].joint_config = sample_vertex;
    // Increment total vertices
    m_total_vertices++;
}
// Member function to sample random joint config
vector <double> SBP_RRT_Star::sample_node()
{
    
    vector <double> random_config;
    
    double sample_rate = (double)(rand()%1000)/1000;
    if (sample_rate >0.95)
    {
        return m_goal_position;
    }
    // Get dof number of configs
    for(int i=0; i<m_numDOF; i++)
    {
        double a_random_double = (double)(rand()%100)/100*2*PI;
        random_config.push_back(a_random_double);
    }
    return random_config;
}
// Member function to check if node inside goal region
int SBP_RRT_Star::inside_goal(vector<double> random_node)
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

// Member function to execute planned path
void SBP_RRT_Star::execute_path(double*** plan, int* planlength)
{
    vector<int> planned_path;
    int goal = m_total_vertices-1;
    planned_path.push_back(goal);
    while (goal!=0)
    {
        goal = vertex_dict[goal].m_parent;
        planned_path.push_back(goal);
    }
    m_solution_vertices = planned_path.size();
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
            (*plan)[j][k] = vertex_dict[planned_path[planned_path.size()-1-j]].joint_config[k];
        }
    }
}
// Member function for RRT Star extend
int SBP_RRT_Star::extend(vector <double> random_node) 
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
            new_vertex.push_back(vertex_dict[parent_neighbor].joint_config[k] + dj*vec_dir[k]/min_dist);
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
        vertex_dict[m_total_vertices-1].relative_cost = find_distance(vertex_dict[m_total_vertices-1].joint_config,vertex_dict[parent_neighbor].joint_config);
        double rrt_star_radius = pow((double)1/2*log(m_total_vertices)/m_total_vertices, 1.0/(1.0+m_numDOF));
        if (rrt_star_radius>m_epsilon)
            rrt_star_radius = m_epsilon; 
        // Do RRT Star Rewiring
        rewire(m_total_vertices-1, rrt_star_radius);
        
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
// Member function to build RRT Star 
void SBP_RRT_Star::build_RRT(double*** plan, int* planlength)
{
    // Add init to tree
    assert(m_total_vertices == 0);
    add_vertex(m_start_position);
    vertex_dict[m_total_vertices-1].relative_cost = 0;
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

// Member function to calculate quality of path
double SBP_RRT_Star::cost_from_start(int sample_node)
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

// Member function to find distance between two configurations
double SBP_RRT_Star::find_distance(vector<double> joint_config_1,vector<double> joint_config_2)
{
    double distance(0);
    for (int i=0; i<joint_config_1.size();i++)
    {
        distance += (joint_config_1[i] - joint_config_2[i])*(joint_config_1[i] - joint_config_2[i]);
    }
    distance = sqrt(distance);
    return distance;
}

// Member function to find nearest neighbors
vector<int> SBP_RRT_Star::nearest_neighbors(int sample_node, double radius)
{
    vector<int> neighbors;
    // Iterate over all vertices to check if they are inside rrt_star_radius
    for (int i=0;i<m_total_vertices;i++)
    {
        if (find_distance(vertex_dict[sample_node].joint_config, vertex_dict[i].joint_config) < radius)
        neighbors.push_back(i);
    }
    return neighbors;
}

// Member function to check if direct connection can be made
int SBP_RRT_Star::obstacle_free(int sample_1, int sample_2)
{
    double dj = 0.05;
    double total_dist = find_distance(vertex_dict[sample_1].joint_config, vertex_dict[sample_2].joint_config);
    vector<double> config_1 = vertex_dict[sample_1].joint_config;
    vector<double> config_2 = vertex_dict[sample_2].joint_config;
    while(dj<=total_dist)
    {
        vector<double> new_config;
        for(int i=0; i<m_numDOF;i++)
        {
            new_config.push_back(config_1[i] + dj*(config_2[i]-config_2[i])/total_dist);
        }
        if (!IsValidArmConfiguration_RRT(new_config,m_numDOF,m_map,m_x_size,m_y_size))
            return 0;
        dj +=dj;
    }
    return 1;
}

// Member function to do rewiring for RRT Star
void SBP_RRT_Star::rewire(int Xnew, double radius)
{        
    vector<int> neighbors = nearest_neighbors(Xnew,radius); // Find nearest neighbors according to euclidean distance
    vector<double> neighbor_costs; // Vector to store costs
    vector<int> good_neighbors; // Vector to store neighbors with low costs
    // Find least cost path from different vertices in a neighborhood to Xnew
    for (int i = 0; i<neighbors.size();i++)
    {
        if(cost_from_start(Xnew) > (cost_from_start(neighbors[i]) + find_distance(vertex_dict[Xnew].joint_config, vertex_dict[neighbors[i]].joint_config)))
        {
            neighbor_costs.push_back(cost_from_start(neighbors[i]) + find_distance(vertex_dict[Xnew].joint_config, vertex_dict[neighbors[i]].joint_config));
            good_neighbors.push_back(neighbors[i]);
        }
    }
    // Check if all good neighbors are obstacle free
    for (int j=0;j<good_neighbors.size();j++)
    {
        if (!obstacle_free(good_neighbors[j], Xnew))
        {
            good_neighbors[j] = -1; // set neighbors that are not free as -1
        }
    }
    // Find neighbor with minimum cost
    int X_best(-1);
    double min_cost = DBL_MAX;
    for (int k=0;k<good_neighbors.size();k++)
    {
        if(good_neighbors[k]!=-1)
        {
            // Compare cost with min_cost
            if (neighbor_costs[k]<min_cost)
            {
                min_cost = neighbor_costs[k];
                X_best = good_neighbors[k];
            }
        }
    }
    if (X_best!=-1)
    {
        vertex_dict[Xnew].m_parent = X_best; // Set parent of Xnew as X_best
        vertex_dict[Xnew].relative_cost = find_distance(vertex_dict[Xnew].joint_config, vertex_dict[X_best].joint_config); // set relative cost
    }
    // First rewiring done. Proceed for the second rewiring
    // Iterate over all the good neighbors (apart from -1s) to see if its easier to reach them
    // through Xnew
    int debug1 = good_neighbors.size();
    for (int debug2=0;debug2<good_neighbors.size();debug2++)
        int debug3 = good_neighbors[debug2];

    for (int l=0;l<good_neighbors.size();l++)
    {
        if (good_neighbors[l]!=-1)
        {
            double relative_cost = find_distance(vertex_dict[Xnew].joint_config, vertex_dict[good_neighbors[l]].joint_config);
            if (cost_from_start(Xnew) + relative_cost < cost_from_start(good_neighbors[l]))
            {
                // Change parent and relative cost
                vertex_dict[good_neighbors[l]].m_parent = Xnew;
                vertex_dict[good_neighbors[l]].relative_cost = relative_cost;
            }
        }
    }
}
// Member function to check quality of path
double SBP_RRT_Star::path_quality()
{
    double total_cost(0);
    if (m_solution_vertices > 0)
    {
        total_cost = cost_from_start(m_total_vertices-1); // cost of path
    }
    return total_cost;
}  

        
        