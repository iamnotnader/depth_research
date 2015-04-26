#ifndef MIN_ST_CUT_HPP
#define MIN_ST_CUT_HPP

#include "min_st_cut.h"
#include <limits>
#include <algorithm>
#include "logging_utils.h"

namespace min_st_cut {

namespace internal {

template<typename ValueType, typename EdgeType>
vector<graph_utils::Node<ValueType,max_flow::EdgeCapacity<EdgeType>>*>
    find_reachable_nodes(
        graph_utils::Node<ValueType,max_flow::EdgeCapacity<EdgeType>>* source) {
  typedef graph_utils::Node<ValueType,max_flow::EdgeCapacity<EdgeType>> NODE;
  vector<NODE*> reachable_nodes;
  vector<NODE*> frontier_nodes = {source};
  while (!frontier_nodes.empty()) {
    NODE* current_node = frontier_nodes.back();
    frontier_nodes.pop_back();
    reachable_nodes.push_back(current_node);

    LOG2("Found reachable node: " << current_node->id);

    for (int i = 0; i < current_node->neighbors.size(); i++) {
      NODE* neighbor = current_node->neighbors[i];
      LOG3("Neighbor: " << neighbor->id << " capacity: "
           << current_node->weights[i].capacity << " flow: "
           << current_node->weights[i].flow);
      if(std::find(reachable_nodes.begin(), reachable_nodes.end(), neighbor) ==
            reachable_nodes.end() &&
         current_node->weights[i].residual() > 0) {
        frontier_nodes.push_back(neighbor);
      }
    }
  }
  assert(frontier_nodes.empty());
  return std::move(reachable_nodes);
}

template<typename ValueType, typename EdgeType>
vector<MinCutEdge<ValueType, EdgeType>> find_outgoing_edges(
    const vector<graph_utils::Node<ValueType,max_flow::EdgeCapacity<EdgeType>>*>&
    reachable_nodes) {
  typedef graph_utils::Node<ValueType,max_flow::EdgeCapacity<EdgeType>> NODE;
  vector<MinCutEdge<ValueType, EdgeType>> min_st_cut_edges;
  for (NODE* reachable_node : reachable_nodes) {
    for (int i = 0; i < reachable_node->neighbors.size(); i++) {
      NODE* neighbor_node = reachable_node->neighbors[i];
      if (std::find(reachable_nodes.begin(), reachable_nodes.end(),
                    neighbor_node) ==
          reachable_nodes.end()) {
        MinCutEdge<ValueType, EdgeType> node_to_add(reachable_node, neighbor_node,
                                          reachable_node->weights[i]);
        min_st_cut_edges.push_back(node_to_add);
      }
    }
  }
  return std::move(min_st_cut_edges);
}

} // namespace internal

template<typename ValueType, typename EdgeType>
vector<MinCutEdge<ValueType, EdgeType>> compute_min_st_cut(
    graph_utils::Graph<ValueType,max_flow::EdgeCapacity<EdgeType>,false>* g) {
  typedef graph_utils::Node<ValueType,max_flow::EdgeCapacity<EdgeType>> NODE;
  // First we compute the max flow for the graph. This sets the flow on all of
  // the edges properly.
  //
  //TODO(daddy): Migrate this over to MaxFlowComputerPushRelabel
  max_flow::MaxFlowComputerFordFulkerson<ValueType, EdgeType> mfc;
  mfc.compute_max_flow(g);

  auto& nodes = g->nodes;
  NODE* source = &nodes[0];

  // Find nodes reachable from source in the residual graph.
  vector<NODE*> reachable_nodes = internal::find_reachable_nodes(source);

  // Find edges connecting reachable vertices to non-reachable vertices.
  vector<MinCutEdge<ValueType, EdgeType>> min_st_cut_edges = internal::find_outgoing_edges(
      reachable_nodes);
  
  return std::move(min_st_cut_edges);
}

} // namespace min_st_cut

#endif
