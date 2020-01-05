/*=================================================================
 *
 * RRT Star planner.h
 *
 *=================================================================*/
#ifndef SBP_RRT_STAR_PLANNER_H
#define SBP_RRT_STAR_PLANNER_H

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

int IsValidArmConfiguration_RRT(vector<double> angles, int numofDOFs, double*	map,
		   int x_size, int y_size);

class SBP_RRT_Star
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
            int m_parent;
            double relative_cost;
        };
        unordered_map <int, m_tree_node> vertex_dict; 
        int m_solution_vertices; // For path quality
        int m_total_vertices;
        

    public:
        // Make constructor 
        SBP_RRT_Star(double* map, int x_size, int y_size, vector<double> start_position, 
        vector<double> goal_position, int numDOF, double epsilon, double dist_goal, int samplenum,
        double sample_lowerbound, double sample_upperbound);

        // Member function to add a vertex to the tree
        void add_vertex(vector<double> sample_vertex);

        // Member function to sample random joint config
        vector <double> sample_node();

        // Member function to check if node inside goal region
        int inside_goal(vector<double> random_node);

        // Member function to execute planned path
        void execute_path(double*** plan, int* planlength);
        
        // Member function for RRT Star extend
        int extend(vector <double> random_node); 
        
        // Member function to build RRT Star 
        void build_RRT(double*** plan, int* planlength);
        
        // Member function to calculate quality of path
        double cost_from_start(int sample_node);
        
        // Member function to find distance between two configurations
        double find_distance(vector<double> joint_config_1,vector<double> joint_config_2);

        // Member function to find nearest neighbors
        vector<int> nearest_neighbors(int sample_node, double radius);

        // Member function to check if direct connection can be made
        int obstacle_free(int sample_1, int sample_2);

        // Member function to do rewiring for RRT Star
        void rewire(int Xnew, double radius); 
        
        // Member function to check quality of path
        double path_quality();
};

#endif