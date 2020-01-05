/*=================================================================
 *
 * RRT Connect.h
 *
 *=================================================================*/
#ifndef SBP_RRT_CONNECT_PLANNER_H
#define SBP_RRT_CONNECT_PLANNER_H

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

int IsValidArmConfiguration_RRT(vector<double> angles, int numofDOFs, double*	map,
		   int x_size, int y_size);
// Make a RRT Connect class
class SBP_RRT_Connect
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
        unordered_map <int, m_tree_node> vertex_dict_T1; // Tree 1 
        int m_total_vertices_T1 // Total vertices in Tree 1
        ;
        unordered_map <int, m_tree_node> vertex_dict_T2; // Tree 2
        int m_total_vertices_T2; // Total vertices in Tree 2
        int m_solution_vertices;

    public:
        // Make constructor 
        // plan was first argument --added rrt build input, execute path, exec path def   
        SBP_RRT_Connect(double* map, int x_size, int y_size, vector<double> start_position, 
        vector<double> goal_position, int numDOF, double epsilon, double dist_goal, int samplenum,
        double sample_lowerbound, double sample_upperbound);

        // Member function to add a vertex to the tree
        void add_vertex(vector<double> sample_vertex, int tree_val);

        // Member function to sample random joint config
        vector <double> sample_node();

        // Member function to determine if node is in goal region
        int inside_goal(vector<double> random_node);
 
        // Member function to execute path
        void execute_path(double*** plan, int* planlength);

        // Member function to build RRT Connect Trees
        void build_RRT(double*** plan, int* planlength);
        
        // Member function for extend RRT Connect for Tree 1
        int extend_T1(vector <double> random_node) ;

        // Member function for extend RRT Connect for Tree 2
        int extend_T2(vector <double> random_node);
        
        // Member function for connect for RRT-Connect for Tree 2
        vector<double> connect_T2(vector<double> x_new);
        
        // Member function for connect for RRT-Connect for Tree 1
        vector<double> connect_T1(vector<double> x_new);
        
        // Member function to judge quality of path
        double cost_from_start(int sample_node, unordered_map<int, m_tree_node> vertex_dict);

        // Member helper function to find Euclidean distance between two joint configurations
        double find_distance(vector<double> joint_config_1,vector<double> joint_config_2);   

        // Member function to check quality of path
        double path_quality();     
};
#endif