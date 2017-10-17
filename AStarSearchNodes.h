#ifndef A_STAR_H
#define A_STAR_H

#include <boost/graph/astar_search.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/numeric/conversion/cast.hpp> 
#include <vector>
#include <list>
#include <iostream>
#include <fstream>
#include <math.h>
 
using namespace boost;
 
class AStarSearchNodes {
    private:
        typedef std::pair<int, int> ID_pair;
        std::vector<Node> nodes;
        std::vector<double> weights;
        std::vector<ID_pair> nearestNeighbor;
         
        /*template <class Graph, class CostType>
        class distance_heuristic : astar_heuristic<Graph, CostType>
        {
            public:
                typedef typename graph_traits<Graph>::vertex_descriptor Vertex;
                distance_heuristic() {}
                CostType operator()(Vertex u) { return static_cast<CostType>(0); }
            private:
                Vertex mGoal;
        };*/
 
        /*
         * Borrowed from 
         * http://kukuruku.co/hub/cpp/masking-a-class-in-boost-graph-part-3-finding-the-path
         */
        struct found_goal { };
         
        template <class Vertex>
        struct astar_goal_visitor : public default_astar_visitor
        {
            private:
                //typedef typename graph_traits<Graph>::vertex_descriptor Vertex;
                Vertex mGoal;           
            public:         
                astar_goal_visitor(Vertex goal) : mGoal (goal) { }
                 
                template <class Graph>
                void examine_vertex(Vertex point, Graph& g) {
                    if(point == mGoal) {
                        throw found_goal();
                    }
                }
        };
         
    public:
        AStarSearchNodes(std::vector<Node> astarNodes, std::vector<std::vector<double> > cost_matrix) :
            nodes(astarNodes)   {
            for(int i = 0; i < astarNodes.size(); i++)
            {
                for(int j = 0; j < astarNodes.size(); j++)
                {
                    if(cost_matrix[i][j] < 100 && cost_matrix[i][j] > 0) {             
                        ID_pair tempPair(astarNodes[i].getNodeID(), astarNodes[j].getNodeID());
                        nearestNeighbor.push_back(tempPair);
                        weights.push_back(cost_matrix[i][j]);
                    }
                }
            }
         }
 
        std::vector<int> findPath(int startId, int goalId) {
                //define "node_graph_t" as the type of a graph described by an adjacency list with the following features:
            //It is a vector of vertices (vecS), each one with a list of vertices to which it is connected to (listS),
            //the edges are undirected (undirectedS), the vertices have no properties, and 
            //the edges have a property defined (property<edge_weight_t, cost> - a double -).
            //typedef property<vertex_name_t, Node> vertexProperty;
            typedef property<edge_weight_t, double> edgeProperty;
            typedef adjacency_list<vecS, vecS, undirectedS, no_property, edgeProperty> node_graph_t;
     
            //define "WeightMap" as the type "property_map<node_graph_t, edge_weight_t>::type" 
            typedef property_map<node_graph_t, edge_weight_t>::type WeightMap;
 
            //define vertex as the vertex_descriptor of the nodes of node_graph_t
            typedef node_graph_t::vertex_descriptor vertex;
 
            //define edge_descriptor as the edge_descriptor of the edges of node_graph_t
            typedef node_graph_t::edge_descriptor edge_descriptor;
 
            //define vertex_iterator as the vertex_iterator of the nodes of node_graph_t
            typedef node_graph_t::vertex_iterator vertex_iterator;          
                         
            //create the graph
            node_graph_t g(nodes.size());
 
            //Put the weights of the arcs in weightmap, which is defined as a WeightMap
            //The type WeightMap has been defined as the type "property_map<node_graph_t, edge_weight_t>::type"
            //The function get returns the edge_weight of graph g.
            //Graph g is a graph node_graph_t that is defined as an adjacency_list graph with the edge 
            //property defined as property<edge_weight_t, cost> 
            WeightMap weightmap = get(edge_weight, g);
 
            // load the edges to the graph
            int num_edges = sizeof(nearestNeighbor) / 
                                 sizeof(nearestNeighbor[0]);
            for(int k = 0; k < nearestNeighbor.size(); ++k) {
                edge_descriptor e; 
                bool inserted;
                tuples::tie(e, inserted) = add_edge(nearestNeighbor[k].first, nearestNeighbor[k].second, g);
                weightmap[e] = weights[k];
            }
            // select start/goal
            vertex start = startId; //integer values
            vertex goal = goalId;
 
            //vector p to store the path
            std::vector<node_graph_t::vertex_descriptor> predecessors(num_vertices(g));
 						std::vector<int> predecessorId;
 						
            //vector d with cost edges
            std::vector<double> distances(num_vertices(g));
 						std::list<vertex> shortest_path;
            try {
             // call astar named parameter interface
                astar_search
                    (g, start,
                    astar_heuristic<node_graph_t, double>(),
                 predecessor_map(&predecessors[0]).distance_map(&distances[0]).visitor(astar_goal_visitor<vertex>(goal)));
            } 
            catch(found_goal fg)
            { // found a path to the goal
                 for(vertex v = goal;; v = predecessors[v]) 
                 {
                    shortest_path.push_front(v);
                    predecessorId.insert(predecessorId.begin(), v);
                    if(predecessors[v] == v)
                      break;
                 }
		          /*shortest_path.push_front(start);
		          std::cout << "Shortest path from " << nodes[start].getNodeID() << " to "
		                 << nodes[goal].getNodeID() << ": " ;// << "\t ";
		          std::list<vertex>::iterator spi = shortest_path.begin();
		          std::cout << nodes[start].getNodeID();
		          for(++spi++; spi != shortest_path.end(); ++spi)
		             std::cout << " -> " << nodes[*spi].getNodeID();
		          std::cout << std::endl << "Total travel time: " << distances[goal] << std::endl;
            	*/
            }
            return predecessorId;
        }
};
#endif /* A_STAR_H */
