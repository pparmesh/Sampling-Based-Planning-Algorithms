/*=================================================================
 *
 * RRT Planer.h
 *
 *=================================================================*/
#ifndef SBP_RRT_BASE_PLANNER_H
#define SBP_RRT_BASE_PLANNER_H

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
// Make a RRT base class
class SBP_RRT_base
{
    public:
        double* m_map; // Planner will need map for collision checking
        int m_x_size; int m_y_size; // Size of map
        vector<double> m_start_position; // array of start angle positions
        vector<double> m_goal_position; //array of goal angle positions
        int m_numDOF; // number of dimensions of the planning problem
        double m_epsilon; // RRT epsilon
        double m_dist_goal; // form a goal region
        int m_samplenum; // Number of samples
        double m_sample_lowerbound; // lower bound for random sampling
        double m_sample_upperbound; // upper bound for random sampling
        // data structures for the tree
        struct m_tree_node
        {
            vector <double> joint_config;
            vector <int> edges;
            double relative_cost;
            int m_parent;
        };
        unordered_map <int, m_tree_node> vertex_dict; 
        int m_total_vertices;
        int m_solution_vertices; // Number of vertices in solution
        

    public:
        // Make constructor 
        SBP_RRT_base(double* map, int x_size, int y_size, vector<double> start_position, 
        vector<double> goal_position, int numDOF, double epsilon, double dist_goal, int samplenum,
        double sample_lowerbound, double sample_upperbound);

        // Member function to add a vertex to the tree
        void add_vertex(vector<double> sample_vertex);
        // Member function to sample random node
        vector <double> sample_node();
        // Member function to check if random node is in goal
        int inside_goal(vector<double> random_node);

        // Member function to execute path
        void execute_path(double*** plan, int* planlength);

        // Member function to extend path
        int extend(vector <double> random_node);

        // Member function to build RRT tree
        void build_RRT(double*** plan, int* planlength);  

        // Member function to find distance between two configurations
        double find_distance(vector<double> joint_config_1,vector<double> joint_config_2); 

        // Member function to calculate quality of path
        double cost_from_start(int sample_node);

        // Member function to check quality of path
        double path_quality();
};
#endif