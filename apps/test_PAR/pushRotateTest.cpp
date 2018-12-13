/*  +---------------------------------------------------------------------------+
*  |                                                                           |
*  | Authors: Mauro Bellone - http://www.maurobellone.com                      |
*  | Released under GNU GPL License.                                           |
*  +---------------------------------------------------------------------------+ */

#include "par.h"
#include <iostream>
#include <ctime>
#include <ostream>

// PAR test



// --------------
// -----Help-----
// --------------
void printUsage(const char* progName)
{
	std::cout << " Reproduce the algorithm in \n "
		<< " Luna, Ryan J., and Kostas E. Bekris. 'Push and swap : Fast cooperative path - finding with completeness guarantees.' "
		<< " Twenty-Second International Joint Conference on Artificial Intelligence. 2011 \n"
		<< " \n\nUsage: " << progName << " [options]\n\n"
		<< "Options:\n"
		<< "-------------------------------------------\n"
		<< "-h           this help\n"
		<< "\n\n";
}



int	main(int argc, char** argv)
{
	using namespace multivehicle_library;
	using namespace boost;
	printUsage(argv[0]);

	
	
		// WRITE YOUR CODE !!

		// Creates the set of the vertices
	enum { A, B, C, D, E, F, G, H, N }; // Make convenient labels for the vertices
	const int num_nodes = N;
	//const char* name_vertices = "ABCDEFGH"; // declared as global 


	// writing out the edges in the graph
	
	std::cout << " Graph structure : \n\n"
		<< "    A \n"
		<< "    |  \                G           \n"
		<< "    | 3 \   D        2 / |           \n"
		<< "    |   5\  | 1  5    /  |             \n"
		<< "    B-----C---E--- F     |  6           \n"
		<< "              3       \  |           \n"
		<< "                      3\ |           \n"
		<< "                        \|             \n"
		<< "                         H               \n" << std::endl;

	Edge edge_array[] = {
		Edge(A, B), Edge(B, A),
		Edge(B, C), Edge(C, B),
		Edge(C, D), Edge(D, C),
		Edge(C, E), Edge(E, C),
		Edge(E, F), Edge(F, E),
		Edge(F, G), Edge(G, F),
		Edge(F, H), Edge(H, F),
		Edge(G, H), Edge(H, G)
	};
	int num_arcs = sizeof(edge_array) / sizeof(Edge);

	// declare the weights
	int weights[] = { 3, 3,
		5, 5,
		1, 1,
		3, 3,
		5, 5,
		2, 2,
		3, 3,
	    6, 6};

	// declare the graph object
	
	graph_t g(edge_array, edge_array + num_arcs, weights, num_nodes, num_nodes);
	
	//boost::property_map<graph_t, boost::edge_weight_t>::type weightmap = get(boost::edge_weight, g);
	//std::vector<vertex_descriptor> p(boost::num_vertices(g));
	//std::vector<int> d(boost::num_vertices(g));
	//vertex_descriptor s = vertex(A, g);

	// Create the set of the agents
	enum { R1, R2, R3, M }; // Make convenient labels for the agents
	agentType agent_array[] = { R1, R2, R3 };
	const int num_agents = M;
	const char* name_agents = "Agents";
	// the number of agents must be less than the number of vertex N-2 



	// the starting assignment place the agents in some specific vertices
	Assignment start_assignment;
	Assignment target_assignment;


	start_assignment.push_back(agent_pose(H, R1));
	start_assignment.push_back(agent_pose(G, R2));
	start_assignment.push_back(agent_pose(A, R3));

	target_assignment.push_back(agent_pose(A, R1));
	target_assignment.push_back(agent_pose(D, R2));
	target_assignment.push_back(agent_pose(G, R3)); 





}

