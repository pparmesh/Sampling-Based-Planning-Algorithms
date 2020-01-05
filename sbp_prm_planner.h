/*=================================================================
 *
 * PRM.h
 *
 *=================================================================*/
#ifndef SBP_PRM_PLANNER_H
#define SBP_PRM_PLANNER_H

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

int IsValidArmConfiguration_RRT(vector<double> angles, int numofDOFs, double*	map,
		   int x_size, int y_size);

class SBP_PRM
{
    public:
        double* m_map; // Planner will need map for collision checking
        vector<double> m_start_position; // start position
        vector<double> m_goal_position; // goal position
        int m_x_size; // size of map in x
        int m_y_size; // size of map in y
        int m_numDOFs; // total number of DOFs
        int m_total_nodes; // total number of vertices the graph should have
        int m_k_nearest; // number of k nearest neighbors
        double m_neighbor_radius; // radius in which to look for neighbors
        struct graph_node
        {
            vector<double> joint_config; // joint configuration of the node
            vector<int> edges; // edges of the node
            vector<double> edge_costs;
            int parent = -1; // Cell in the solution will have a parent
            double f = DBL_MAX, g = DBL_MAX, h = DBL_MAX; // Cells will have a total cost, heuristic cost and cost to go
        };
        unordered_map <int, graph_node> prm_graph; // map stores prm graph. int--> index of node, graph_node--> properties
        int m_solution_vertices;
        // Data members for graph search
        // Type define a pair that includes a cell's 'f' value and its coordinates index
        typedef pair<double, int> f_COORDINATE;

    public:
        // Make constructor
        SBP_PRM(double* map, vector<double> start_position, vector<double> goal_position,
         int x_size, int y_size, int numDOFS, int total_nodes, int k_nearest,
             double neighbor_radius);
        // Make member functions
        double find_distance(vector<double> joint_config_1,vector<double> joint_config_2);

        // Helper function to check if edge connection can be made
        int obstacle_free(int sample_1, int sample_2);
        
        // Function to get random configuration
        vector <double> get_random_config();

        // Function to construct roadmap
        void construct_roadmap();

        // Member function to calculate quality of path
        double cost_from_start(int sample_node);
        
        // Functions for graph search
        void solutionPath(int goal, double*** plan, int* planlength);
        
        // Dijkstra search to find the path
        void dijkstra_search(int start_node, int goal_node, double*** plan, int* planlength);
        // Member function to check quality of path
        double path_quality();   

};

#endif