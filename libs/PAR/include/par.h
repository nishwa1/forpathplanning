/* +---------------------------------------------------------------------------+
*  |                              			                                   |
*  |                                                                           |
*  |                                                                           |
*  | Copyright (c) 2019, - All rights reserved.                                |
*  | Authors:                                                                  |
*  | Released under ___ License.                                               |
*  +---------------------------------------------------------------------------+ */

#pragma once

#include "export.h"

// OS Specific 
#if defined (_WIN64) || defined (_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN   // Exclude rarely-used stuff from Windows headers
#endif

// thread and mutex generate warning C4251 - dll interface needed
// however the objects are private so I will never export this data, warning can be ignored
// see http://www.unknownroad.com/rtfm/VisualStudio/warningC4251.html
#pragma warning( disable: 4251 )
#ifndef _EXPORT_H
#define _EXPORT_H
#endif _EXPORT_H
//#include <windows.h>  // OS specific Sleep
#else
#include <unistd.h>
#endif

// standard libraries 
#define _USE_MATH_DEFINES
#include <boost/config.hpp>
#include <vector>
#include <list>
#include <cmath>
#include <iostream>
#include <string>
#include <cstdio>
#include <ctime>
#include <iterator>

// third party libraries
#include <boost/graph/biconnected_components.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>


//_______________INCLUDE HERE WHAT YOU NEED 

namespace boost
{
	struct edge_component_t
	{
		enum
		{
			num = 555
		};
		typedef edge_property_tag kind;
	}
	edge_component;
}

namespace multivehicle_library
{
	using namespace boost;

	typedef boost::adjacency_list <
		boost::vecS, boost::vecS, 
		boost::undirectedS,	boost::no_property, 
		boost::property < edge_component_t, std::size_t > >graph_t;
	   //property_map <  >::type
	//typedef boost::property_map < graph_t, edge_component_t >::type;
	typedef boost::graph_traits < graph_t >::vertex_descriptor vertex_t;
	typedef boost::graph_traits < graph_t >::edge_iterator ei, ei_end;
	

	typedef int vertexType;

	typedef std::pair<vertexType, vertexType> Edge;
	typedef std::vector<Edge> biconnect;
	typedef std::pair<biconnect, biconnect> sub_graph ;
	typedef std::vector<sub_graph> all_graph;


	typedef int agentType;
    typedef std::vector<graph_t::edge_descriptor> PathType;

	typedef std::pair<vertexType, agentType> agent_pose;

	//writing assignments, i.e. every agent is places in a vertex
	typedef std::vector<agent_pose> Assignment;
	typedef std::pair<Assignment, Assignment> Action;
	typedef std::vector<Assignment> PathAssignments;
	const char* name_vertices = "ABCDEFGH";

	class PAS_EXPORT PushAndRotate
	{



	private:

		// create a typedef for the Graph type
		//typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS> Graph;




	//private: 
	// use this for your data members
	public:
		/** \brief Constructor, initialize objects and parameters using default values
		*
		*/
		PushAndRotate() {};

		/** \brief Destructor implementation to make sure everything is properly closed
		  *
		  *   Make sure everything is properly closed, then free memory
		  */
		~PushAndRotate() {};

		

		//	void solvePushAndSwap();
		vertexType extractNextVertex(PathType _path, graph_t _g, 
			Assignment _start_assignment, agentType _agent, 
			const char* _name_vertices, agent_pose &_busy_vertex);


		/**vertexType closestEmptyVertex(vertexType _v,  // the occupied vertex 
			graph_t _g,   // the graph
			Assignment _current_assignment);

		
		PathType shortestPath(graph_t _g,
			vertex_t _start, //Assignment _current_assignment,
			vertex_t _target, //Assignment _target_assignment,
			agentType _agent);
		agentType getAgentInVertex(Assignment _current_assignment, vertexType _v);

		
		std::vector<vertexType> getSwapVertices(graph_t _g,
			vertex_t _start,
			vertex_t _target,
			agentType _agent);

		
		bool PUSH(PathAssignments &_optimal_path,
			graph_t _g,
			Assignment &_current_assignment,
			Assignment _target_assignment,
			agentType _agent,
			std::vector<agentType> _set_of_agents_in_the_target);

		bool MULTIPUSH(PathAssignments &_assignments_path,
			graph_t _g,
			Assignment &_current_assignment,
			Assignment &_target_assignment,
			agentType _agent,
			agentType _agent_to_swap,
			PathType _path);

		bool CLEAR(PathAssignments &_assignments_path,
			graph_t _g,
			Assignment &_current_assignment,
			Assignment &_target_assignment,
			agentType _agent,
			agentType _agent_to_swap,
			PathType _path);

		bool SWAP(PathAssignments &_optimal_path,
			graph_t _g,
			Assignment &_current_assignment,
			Assignment _target_assignment,
			agentType _agent,
			std::vector<agentType> _set_of_agents_in_the_target);**/

		vertexType biconnectedcomponenets(graph_t _g, edge_component_t edge_component, ei _n1,
			ei_end _n2,biconnect _firstedge, biconnect _secondedge, vertexType vertex);


	};


}





