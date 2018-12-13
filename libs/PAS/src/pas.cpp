#include "include/pas.h"

using namespace multivehicle_library;
//do it! multivehicle_library::PushAndSwap::
/** Extract the next vertex along the path
  * \return -1 if fail
  */

vertexType PushAndSwap::extractNextVertex(PathType _path, graph_t _g, Assignment _start_assignment, agentType _agent, const char* _name_vertices, agent_pose &_busy_vertex)
{
	vertexType current_vertex = -1;

	for (PathType::reverse_iterator pathIterator = _path.rbegin(); pathIterator != _path.rend(); ++pathIterator) {
		if (_name_vertices[boost::source(*pathIterator, _g)] == _name_vertices[_start_assignment.at(_agent).first]) {
			current_vertex = (int)boost::target(*pathIterator, _g);
			//std::cout << " extractNextVertex:: found :: current vertex is " << _name_vertices[current_vertex] << std::endl;

			//check if the current vertex is busy
			for (size_t i = 0; i < _start_assignment.size(); i++) {
				if (current_vertex == _start_assignment.at(i).first) {
					std::cout << " vertex busy " << std::endl;
					_busy_vertex.first = current_vertex;  // mark the current vertex as busy
					_busy_vertex.second = _start_assignment.at(i).second;
					return -1; // and return no vertex
				}
			}

			_busy_vertex = { -1, -1 }; // no vertex busy
			break; // vertex found
		} // end if
	}
	return current_vertex;
};


vertexType PushAndSwap::closestEmptyVertex(vertexType _v,  // the occupied vertex 
	graph_t _g,   // the graph
	Assignment _current_assignment)
{
	vertexType closest_vertex = _v; // the closest is initialized in itself

	std::vector<vertex_descriptor> p(boost::num_vertices(_g));
	std::vector<int> d(boost::num_vertices(_g));
	auto pred_map = predecessor_map(boost::make_iterator_property_map(p.begin(), get(boost::vertex_index, _g)));
	auto prop_map = boost::make_iterator_property_map(d.begin(), get(boost::vertex_index, _g));

	dijkstra_shortest_paths(_g, _v,
		pred_map.distance_map(prop_map));


	// get the closest vertex
	int d_min = 10000; // any big number
	graph_t::out_edge_iterator out_begin, out_end;
	for (boost::tie(out_begin, out_end) = out_edges(_v, _g); out_begin != out_end; ++out_begin)
	{
		//std::cout << " target " << target(*out_begin, _g) << " with cost " << d[target(*out_begin, _g)] << std::endl;
		if (d_min > d[target(*out_begin, _g)])
		{
			// if a new minimum exists, it is necessary to check if it is oppupied by another agent
			bool isBusy = false;
			for (size_t i = 0; i < _current_assignment.size(); i++)
			{
				if (target(*out_begin, _g) == _current_assignment.at(i).first)
				{
					//std::cout << " the target " << name_vertices[target(*out_begin, _g)] << " is busy " << std::endl;
					isBusy = true;
				}
			}
			if (!isBusy) {
				closest_vertex = (int)target(*out_begin, _g);
				d_min = d[target(*out_begin, _g)];
			}
		}

	}
	std::cout << std::endl;

	//std::cout << " the closest vertex to " << name_vertices[_v] << " is the vertex " << name_vertices[closest_vertex] << " with cost " << d_min << std::endl;
	return closest_vertex;

};



PathType PushAndSwap::shortestPath(graph_t _g,
	vertex_descriptor _start, //Assignment _current_assignment,
	vertex_descriptor _target, //Assignment _target_assignment,
	agentType _agent)
{
	PathType path;


	std::vector<vertex_descriptor> p(boost::num_vertices(_g));
	std::vector<int> d(boost::num_vertices(_g));
	auto pred_map = predecessor_map(boost::make_iterator_property_map(p.begin(), get(boost::vertex_index, _g)));
	auto prop_map = boost::make_iterator_property_map(d.begin(), get(boost::vertex_index, _g));

	dijkstra_shortest_paths(_g, _start,
		pred_map.distance_map(prop_map));



	// Extract the shortest path

	std::cout << std::endl;
	vertex_descriptor v = _target;
	for (vertex_descriptor u = p[(int)v];
		u != v; // Keep tracking the path until we get to the source
		v = u, u = p[(int)v]) // Set the current vertex to the current predecessor,     and the predecessor to one level up
	{
		std::pair<graph_t::edge_descriptor, bool> edgePair = boost::edge(u, v, _g);
		graph_t::edge_descriptor edge = edgePair.first;
		path.push_back(edge);
	}


	// Print the shortest path

	std::cout << "Shortest path from " << _start << " to " << _target << " : " << std::endl;
	float totalDistance = 0;
	for (PathType::reverse_iterator pathIterator = path.rbegin(); pathIterator != path.rend(); ++pathIterator)
	{
		std::cout << name_vertices[boost::source(*pathIterator, _g)] << " -> " << name_vertices[boost::target(*pathIterator, _g)]
			<< " = " << boost::get(boost::edge_weight, _g, *pathIterator) << std::endl;

	}

	return path;
};




agentType PushAndSwap::getAgentInVertex(Assignment _current_assignment, vertexType _v)
{

	for (size_t i = 0; i < _current_assignment.size(); i++) {
		if (_current_assignment.at(i).first == _v) return _current_assignment.at(i).second;
	}

	return -1; // vertex not found
};



std::vector<vertexType> PushAndSwap::getSwapVertices(graph_t _g,
	vertex_descriptor _start,
	vertex_descriptor _target,
	agentType _agent)
{
	std::vector<vertexType> swap_vertices;

	std::vector<vertex_descriptor> p(boost::num_vertices(_g));
	std::vector<int> d(boost::num_vertices(_g));
	auto pred_map = predecessor_map(boost::make_iterator_property_map(p.begin(), get(boost::vertex_index, _g)));
	auto prop_map = boost::make_iterator_property_map(d.begin(), get(boost::vertex_index, _g));

	dijkstra_shortest_paths(_g, _start,
		pred_map.distance_map(prop_map));

	std::cout << "distances and parents:" << std::endl;
	boost::graph_traits < graph_t >::vertex_iterator vi, vend;
	//boost::tie(vi, vend) = vertices(_g);
	//std::cout << " size of vi is " << sizeof(vi) << std::endl;
	//std::cout << " size of vend is " << sizeof(vend) << std::endl;

	swap_vertices.push_back((int)p[0]);  // this suppose that we actually have parents !!

	for (boost::tie(vi, vend) = vertices(_g); vi != vend; ++vi) {
		std::cout << "distance(" << name_vertices[*vi] << ") = " << d[*vi] << ", ";
		std::cout << "parent(" << name_vertices[*vi] << ") = " << name_vertices[p[*vi]] << std::endl;

		if (swap_vertices.back() != p[*vi])
			swap_vertices.push_back((int)p[*vi]);   // in this way we have all the vertices with parents

	}
	//std::cout << std::endl;


	return swap_vertices;
}

/* PUSH - Algorithm 2 in the paper
*
*/
bool PushAndSwap::PUSH(PathAssignments &_optimal_path,
	graph_t _g,
	Assignment &_current_assignment,
	Assignment _target_assignment,
	agentType _agent,
	std::vector<agentType> _set_of_agents_in_the_target)
{
	// some variable initialization
	PathType path;
	//vertex_descriptor s = _current_assignment[_agent].first;

	//LINE 1:
	// find the shortest path
	path = shortestPath(_g, _current_assignment[_agent].first, _target_assignment[_agent].first, _agent);

	//LINE 2: 
	// extract the first vertex in the optimal path after the assignment
	std::cout << " start assignment of robot  " << _current_assignment.at(_agent).second << " is the vertex " << name_vertices[_current_assignment.at(_agent).first] << std::endl;
	vertexType assignment_verterx = _current_assignment.at(0).first;
	std::cout << " target assignment of robot " << _target_assignment.at(_agent).second << " is the vertex " << name_vertices[_target_assignment.at(_agent).first] << std::endl;
	vertexType current_vertex;
	agent_pose busy_vertex = { -1, -1 };


	//LINE 3:
	while (_current_assignment[_agent].first != _target_assignment[_agent].first)
	{
		// get the current vertex
		current_vertex = extractNextVertex(path, _g, _current_assignment, _agent, name_vertices, busy_vertex);
		//LINE 4:
		// while exists v and v is empty on _g do 
		while (current_vertex != -1)  // vertex cannot be negative, it will be negative when empty
		{

			//LINE 5:
			// move the _agent one position forward in the plan in the current vertex 
			_current_assignment[_agent].first = current_vertex;

			//LINE 6:
			// add the assignment to the optimal path
			_optimal_path.push_back(_current_assignment);

			//LINE 7:
			// extract the next vertex in the plan
			current_vertex = extractNextVertex(path, _g, _current_assignment, _agent, name_vertices, busy_vertex); // return -1 if the vertex is occupied by anther agent

		}// end while line 4

		//std::cout << " start assignment of robot  " << _current_assignment.at(_agent).second << " is the vertex " << name_vertices[_current_assignment.at(_agent).first] << std::endl;
		//std::cout << " target assignment of robot " << _target_assignment.at(_agent).second << " is the vertex " << name_vertices[_target_assignment.at(_agent).first] << std::endl;

		//LINE 8:
		// if the assignment is different than the target then push
		if (_current_assignment[_agent].first != _target_assignment[_agent].first)
		{
			//LINE 9:
			// mark the assignment and the set of targets in place as blocked on G
			std::cout << " we have found a busy vertex and the agent is not in the solution, busy vertex = " << name_vertices[busy_vertex.first] << std::endl;

			//LINE 10: 
			//  store the closest free vertex in _g
			vertexType v_empty = closestEmptyVertex(busy_vertex.first, _g, _current_assignment);
			// at this point busy_vertex is the blocked vertex whereas v_empty is the closest empty vertex in the path

			//LINE 11: 
			//get the closest path from the busy vertex to the empty vertex 
			PathType path_push = shortestPath(_g, busy_vertex.first, v_empty, _agent);


			//LINE 12:
			// if the path is empty the algorithm fail because we cannot move anymore
			if (path_push.size() == 0)
			{

				//LINE 13:
				std::cout << " No valid path to push --- swap will be necessary  " << std::endl;
				return false;
			}

			//LINE 14:
			// if the path is not empty then we can move
			// mark the assignment as free 
			Assignment next_assignment = _current_assignment;
			//next_assignment.at(_agent).first = busy_vertex;

			//LINE 15:
			// 
			vertexType vertex_st = (int)path_push.back().m_source;
			path_push.pop_back();

			//LINE 16:
			//
			vertexType vertex_nd = v_empty;
			std::cout << " v_empty " << name_vertices[v_empty] << std::endl;

			//LINE 17:
			// while v_nd is not v do
			while (vertex_nd != busy_vertex.first)//_current_assignment[_agent].first)
			{

				//LINE 18:
				// r' = agent for which A(r) = v'
				// retrieve the agent that occupy the busy vertex
				agentType agent_st = busy_vertex.second;

				//LINE 19:
				// A(r_st) = v_nd
				_current_assignment.at(agent_st).first = vertex_nd;
				//agentType r_st;

				//LINE 20:
				// add the assignment to the path
				_optimal_path.push_back(_current_assignment);


				//LINE 21:
				// v_nd = v_st
				vertex_nd = vertex_st;

				//LINE 22: 
				// get teh previous vertex along the path_push
				if (path_push.size() > 0){ // aviod the empty path to give error
					vertex_st = (int)path_push.at(path_push.size() - 1).m_source;
					std::cout << " new vertex before " << name_vertices[vertex_st] << " number " << vertex_st << std::endl;
				}
			}//end if line 17
		}// end if line 8
		else
		{
			std::cout << " the agent " << _agent << " reached its target " << name_vertices[_target_assignment.at(_agent).first] << std::endl;
			//_set_of_agents_in_the_target.push_back(_agent);
			return true;
		}
	} // end while line 3

	//LINE 23:
	std::cin.get();
	return false;
}


bool PushAndSwap::MULTIPUSH(PathAssignments &_assignments_path,
	graph_t _g,
	Assignment &_current_assignment,
	Assignment &_target_assignment,
	agentType _agent,
	agentType _agent_to_swap,
	PathType _path)
{

	//push the _agent one step back on its path toward the swapping vertex
	std::vector<agentType> _set_of_agents_in_the_target = {};
	//vertexType v_to_go = _path.back().m_source;
	Assignment tmp_target_assignment = _target_assignment;

	if (_path.size() == 0) {
		// we do not have a target to go ! 
		return false;
	}

	tmp_target_assignment.at(_agent).first = (int)_path.front().m_target;
	if (!PUSH(_assignments_path, _g, _current_assignment, tmp_target_assignment, _agent, _set_of_agents_in_the_target)) {
		// if we cannot move the multipush fails
		return false;
	}

	return true;
}


bool PushAndSwap::CLEAR(PathAssignments &_assignments_path,
	graph_t _g,
	Assignment &_current_assignment,
	Assignment &_target_assignment,
	agentType _agent,
	agentType _agent_to_swap,
	PathType _path
	)
{

	// get vertex to go considering only the set of agent to move
	// the vertex to go is the closest vertex that is not busy and not in _path
	Assignment tmp_current_assignment;
	tmp_current_assignment.push_back(_current_assignment.at(_agent));
	tmp_current_assignment.push_back(_current_assignment.at(_agent_to_swap));
	vertexType v_empty = closestEmptyVertex((int)_path.back().m_source, _g, tmp_current_assignment);

	Assignment tmp_target_assignment = _target_assignment;
	tmp_target_assignment.at(_agent).first = (int)_path.front().m_target;

	// Looking for the proper swap candidate vertex
	vertexType swap_candidate;

	std::vector<vertex_descriptor> p(boost::num_vertices(_g));
	std::vector<int> d(boost::num_vertices(_g));
	auto pred_map = predecessor_map(boost::make_iterator_property_map(p.begin(), get(boost::vertex_index, _g)));
	auto prop_map = boost::make_iterator_property_map(d.begin(), get(boost::vertex_index, _g));

	dijkstra_shortest_paths(_g, _current_assignment.at(_agent_to_swap).first,
		pred_map.distance_map(prop_map));

	//std::cout << "distances and parents:" << std::endl;
	boost::graph_traits < graph_t >::vertex_iterator vi, vend;
	//boost::tie(vi, vend) = vertices(_g);
	//std::cout << " size of vi is " << sizeof(vi) << std::endl;
	//std::cout << " size of vend is " << sizeof(vend) << std::endl;

	for (boost::tie(vi, vend) = vertices(_g); vi != vend; ++vi) {
		//std::cout << "distance(" << name_vertices[*vi] << ") = " << d[*vi] << ", ";
		//std::cout << "parent(" << name_vertices[*vi] << ") = " << name_vertices[p[*vi]] << std::endl;

		// check if another agent in the the vertex 
		bool no_busy = false;
		for (int i = 0; i < _current_assignment.size(); i++){
			if (*vi == _current_assignment.at(i).first)  {
				//std::cout << " the vertex is busy " << std::endl;
				no_busy = false;
				break;
			}
			else{
				no_busy = true;
			}
		}


		// check if the vertex is in the path
		bool is_not_in_the_path = false;
		for (int i = 0; i < _path.size(); i++)
		{
			if (*vi == _path.at(i).m_source || *vi == _path.at(i).m_target) {
				is_not_in_the_path = false;
				break;
			}
			else {
				is_not_in_the_path = true;
			}
		}

		if (no_busy == true && is_not_in_the_path == true){
			swap_candidate = (int)*vi;

			//Push the _agent_to_swap one step back on the path of the _agent
			std::vector<agentType> _set_of_agents_in_the_target = {};
			tmp_target_assignment.at(_agent_to_swap).first = swap_candidate;// v_empty;//_path.back().m_source;
			if (!PUSH(_assignments_path, _g, _current_assignment, tmp_target_assignment, _agent_to_swap, _set_of_agents_in_the_target)) {
				// if we cannot move the multipush fails
				no_busy = false;
				is_not_in_the_path = false;
				//we cannot move there, retry with another vector
			}
			else {
				// _aget_to_swap moved
				break;
			}
		}
	}
	//std::cout << std::endl;

	return true;
}


/* SWAP - Algorithm 3 in the paper
*
*/
bool PushAndSwap::SWAP(PathAssignments &_optimal_path,
	graph_t _g,
	Assignment &_current_assignment,
	Assignment _target_assignment,
	agentType _agent,
	std::vector<agentType> _set_of_agents_in_the_target)
{

	// some variable initialization
	PathType path_star;
	PathAssignments new_assignments_path;

	//LINE 1: 
	// get the shortest path
	path_star = shortestPath(_g, _current_assignment[_agent].first, _target_assignment[_agent].first, _agent);

	//LINE 2:
	// agent on the first vertex in path after A(r), that is the agent to swap
	agentType agent_to_swap = getAgentInVertex(_current_assignment, (int)path_star.back().m_target);

	//LINE 3:
	bool success = false;

	//LINE 4:
	// swap vertices, i.e. gets all the vertices with deg >2 (where it is possible to move in more directions)
	std::vector<vertexType> swap_vertices = getSwapVertices(_g, _current_assignment[_agent].first, _target_assignment[_agent].first, _agent);

	//LINE 5:
	// while there are swap vertices and there is no success in the algorithm do
	while (swap_vertices.size() > 0)
	{

		//LINE 6:
		// extract a candidate vertex to swap
		vertexType v = swap_vertices.front();
		swap_vertices.erase(swap_vertices.begin()); // and erase it from the list 

		//LINE 7:
		// shortest path to swap
		PathType path = shortestPath(_g, _current_assignment[_agent].first, v, _agent);

		//LINE 8:
		// Make sure the new set of assignments is empty 
		new_assignments_path = {};

		//LINE 9:
		// attempt to multipush the candidate
		if (MULTIPUSH(new_assignments_path, _g, _current_assignment, _target_assignment, _agent, agent_to_swap, path))
		{

			//LINE 10:
			// the function clear evacuate an agent from the busy vertex 
			if (CLEAR(new_assignments_path, _g, _current_assignment, _target_assignment, _agent, agent_to_swap, path))
			{

				//LINE 11:
				// evacuation success
				success = true;
				break;
			}//end clear
		} //end multipush

	}//end while swap vertices

	//LINE 12:
	// if is not success
	if (!success) {

		//LINE 13: 
		// the algorithm fails
		std::cout << " the algorithm is failed ... no swap operations allowed " << std::endl;
		return false;
	}

	//LINE 14:
	// if the algorithm still not fails
	// merge the assignment paths to get the entire assignment
	for (int i = 0; i < new_assignments_path.size(); i++){
		_optimal_path.push_back(new_assignments_path.at(i));
	}

	//LINE 15:
	// execute swap 
	// this has been integrated in the evacuation funtion

	//LINE 16:
	// reverse path --- the agent get back to the target
	if (!PUSH(_optimal_path, _g, _current_assignment, _target_assignment, _agent, _set_of_agents_in_the_target)){
		// if _agent cannot reach the target the multipush fail
		return false;
	}
	//reverse_assignments_path = REVERSE();  // exchange paths between two agents

	//LINE 17:
	// merge the reverse assignment paths to get the entire assignment
	// not necessary merge as in the reverse path we are already including the new path to _optimal_path variable


	//LINE 18:
	if (_current_assignment == _target_assignment)
	{

		//LINE 19:
		// the algorithm can be risolved RESOLVE 
		return true;
	}

	//LINE 20:
	// the algorithm is risolved
	return true;
}


/* PUSH AND SWAP - Algorithm 1 in the paper
*
*/
bool PushAndSwap::PUSH_AND_SWAP(graph_t _g,
	agentType agent_array[],
	Assignment _start_assignment,
	Assignment _target_assignment,
	std::vector<agentType> &_set_of_agents_in_the_target,
	PathAssignments &_optimal_path)
{
	int number_of_agents = sizeof(agent_array) / sizeof(agentType);

	//LINE 1:
	// initialize the starting assignment
	Assignment current_assignment = _start_assignment;

	//LINE 2:	
	// initialize the optimal path as a succession of assignments for each agent
	//_optimal_path.clear();
	_optimal_path.push_back(_start_assignment);

	//LINE 3:
	// initialize a set of agents that already reached the target
	//_set_of_agents_in_the_target[number_of_agents] = {}; // not necessary, it has been passed as parameter

	//LINE 4:
	// for all r \in R do
	for (int i = 0; i <= number_of_agents; i++)  // for all agents
	{
		// some debug prints
		//std::cout << " agent array " << agent_array[i] << std::endl;
		//std::cout << " _set_of_agents_in_the_target " << _set_of_agents_in_the_target.size() << std::endl;

		//LINE 5: 
		// while - the assignment path of the robot i is not in its target - do
		while (_optimal_path.back().at(i).first != _target_assignment.at(i).first)
		{

			//LINE 6:
			if (PUSH(_optimal_path,
				_g,
				current_assignment,
				_target_assignment,
				agent_array[i],
				_set_of_agents_in_the_target) == false)
			{

				//LINE 7:
				if (SWAP(_optimal_path,
					_g,
					current_assignment,
					_target_assignment,
					agent_array[i],
					_set_of_agents_in_the_target) == false)
				{
					//LINE 8:
					std::cout << " Algorithm failed " << std::endl;
					return false;
				} // end SWAP
				std::cin.get(); // just to stop the algorithm during the iterations
			} // end PUSH
		} // end WHILE

		//LINE 9:
		// add the agent to the list of agents in the target
		_set_of_agents_in_the_target.push_back(agent_array[i]);
		current_assignment.at(i).first = _target_assignment.at(i).first;
	}
	//LINE 10: 
	std::cout << " Algorithm success !!! " << std::endl;
	return true;


}