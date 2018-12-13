/*  +---------------------------------------------------------------------------+
*  |                                                                           |
*  |                                                                           |
*  | Authors: Mauro Bellone - http://www.maurobellone.com                      |
*  | Released under GNU GPL License.                                           |
*  +---------------------------------------------------------------------------+ */

#include "pas.h"
#include <iostream>
#include <ctime>
#include <ostream>

// PAS test



// --------------
// -----Help-----
// --------------
void printUsage (const char* progName)
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


int	main (int argc, char** argv)
{
	using namespace multivehicle_library;
	printUsage(argv[0]);

	/**try {
		// analyse parsed arguments 
		if (argc < 2) {
			printUsage(argv[0]);
			return 0;
		}**/
	
	
	// WRITE YOUR CODE !!
	
	// Creates the set of the vertices
		enum { A, B, C, D, E, F, G, H, N }; // Make convenient labels for the vertices
		const int num_nodes = N;
		//const char* name_vertices = "ABCDEFGH"; // declared as global 


		// writing out the edges in the graph
		/**
		*  My graph structure is:
		*
		*   A
		*   |                    G
		*   |3      D           /2
		*   |   7   |1     5   /
		*   B-------C---E-----F
		*             3        \
		*                       \3
		*                        \
		*                         H
		*
		**/
		std::cout << " Graph structure : \n\n"
			<< "    A \n"
			<< "    |                   G \n"
			<< "    | 3     D          / 2 \n"
			<< "    |   7   | 1     5 / \n"
			<< "    B-------C---E--- F \n"
			<< "              3       \\ \n"
			<< "                       \\ 3 \n"
			<< "                        \\ \n"
			<< "                         H \n" << std::endl;

		Edge edge_array[] = {
			Edge(A, B), Edge(B, A),
			Edge(B, C), Edge(C, B),
			Edge(C, D), Edge(D, C),
			Edge(C, E), Edge(E, C),
			Edge(E, F), Edge(F, E),
			Edge(F, G), Edge(G, F),
			Edge(F, H), Edge(H, F),
		};
		int num_arcs = sizeof(edge_array) / sizeof(Edge);

		// declare the weights
		int weights[] = { 3, 3,
			7, 7,
			1, 1,
			3, 3,
			5, 5,
			2, 2,
			3, 3 };

		// declare the graph object
		graph_t g(edge_array, edge_array + num_arcs, weights, num_nodes);

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
			target_assignment.push_back(agent_pose(G, R3)); /**/

		

		//HERE THE PUSH AND SWAP ALGORITHM 
		std::vector<agentType> set_of_agents_in_the_target = {}; // the set of agents in the target is now empty 
		PathAssignments optimal_path;
		PushAndSwap q;
		 q.PUSH_AND_SWAP(g,
			agent_array,
			start_assignment,
			target_assignment,
			set_of_agents_in_the_target,
			optimal_path);


	 //just print the solution
		std::cout << " printing solution optimal_path size " << optimal_path.size() << std::endl;
		for (int i = 0; i < optimal_path.size(); i++) { // for all the assignments
			for (int j = 0; j < optimal_path.at(i).size(); j++) { // for all the agent poses
				std::cout << " optimal path : assignment " << i
					//<< " agent pose " << j 
					<< " agent = R" << optimal_path.at(i).at(j).second
					<< " --> vertex = " << name_vertices[optimal_path.at(i).at(j).first] << std::endl;
			}
			std::cout << "\n" << std::endl;
		}

		std::cout << " FINISHED, press enter to exit ....  CIAO !!! " << std::endl;
		std::cin.get();
		return 0;
	
/**
		std::cout << "\n >>> TEST finished, press ok to exit  <<< \n\n" << std::endl;
		cin.get();	
	}
	catch ( std::exception &e)
	{
		cerr << "Exception : " << e.what() << endl;
		cin.get();
	}
	catch ( ... ) {
		cerr << "Unhandled Exception: " <<  endl;
		cin.get();
	}

	std::cout << "\n >>> ciao  <<< \n\n" <<  std::endl;

	return 0;**/
}

