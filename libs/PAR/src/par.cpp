#include "include/par.h"

using namespace multivehicle_library;
using namespace boost;
//do it! multivehicle_library::PushAndSwap::
/** Extract the next vertex along the path
  * \return -1 if fail
  */

vertexType PushAndRotate::extractNextVertex(PathType _path, graph_t _g, Assignment _start_assignment, agentType _agent, const char* _name_vertices, agent_pose &_busy_vertex)
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

vertexType PushAndRotate::biconnectedcomponenets(graph_t _g, edge_component_t edge_component, ei _n1,
	ei_end _n2,biconnect _firstedge,biconnect _secondedge,vertexType vertex)
{
	std::vector<vertexType> vertex;
	biconnect	 bicon;
 boost::property_map < graph_t, edge_component_t >::type

 component=get(edge_component, _g);
 std::size_t num_comps = biconnected_components(_g, component);
 std::cerr << "Found " << num_comps << " biconnected components.\n";

 std::vector<vertex_t> art_points;
 articulation_points(_g, std::back_inserter(art_points));
 std::cerr << "Found " << art_points.size() << " articulation points.\n";

 for (std::size_t i = 0; i < art_points.size(); ++i) {
	 char p = (char)(art_points[i] + 'A');
	 std::cout << p << " [ style=\"filled\", fillcolor=\"red\" ];"
		 << std::endl;
	 //return p; }

	 for (boost::tie(_n1, _n2) = edges(_g); _n1 != _n2; ++_n1)
		
	 {
		 _firstedge[vertex].first = (source(*_n1, _g) );
		 _secondedge[vertex].first =(target(*_n1, _g) );
        //_firstedge[vertex].first = c;
	    //_secondedge[vertex].first = d;
	bicon.push_back(_firstedge.at(vertex));
	bicon.push_back(_secondedge.at(vertex));

		return bicon;
		

 }
 

 std::cout << "}\n";

		//component = get(edge_component, _g);
//std::size_t num_comps = biconnected_components(_g, components );
//std::cerr << "Found " << num_comps << " biconnected components.\n";

};