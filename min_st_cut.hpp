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
        graph_utils::Node<ValueType,max_flow::EdgeCapacity<EdgeType>>* source,
        int64_t total_nodes) {
  typedef graph_utils::Node<ValueType,max_flow::EdgeCapacity<EdgeType>> NODE;
  vector<NODE*> reachable_nodes;
  reachable_nodes.reserve(total_nodes);
  vector<NODE*> frontier_nodes = {source};
  frontier_nodes.reserve(total_nodes);
  vector<bool> visited(total_nodes, false);
  visited[source->id] = true;
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
      if(!visited[neighbor->id] &&
         current_node->weights[i].residual() > 0) {
        visited[neighbor->id] = true;
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
    reachable_nodes, int64_t total_nodes) {
  typedef graph_utils::Node<ValueType,max_flow::EdgeCapacity<EdgeType>> NODE;
  vector<MinCutEdge<ValueType, EdgeType>> min_st_cut_edges;
  vector<bool> node_is_reachable(total_nodes, false);
  for (NODE* reachable_node : reachable_nodes) {
    node_is_reachable[reachable_node->id] = true;
  }
  for (NODE* reachable_node : reachable_nodes) {
    for (int i = 0; i < reachable_node->neighbors.size(); i++) {
      NODE* neighbor_node = reachable_node->neighbors[i];
      if (!node_is_reachable[neighbor_node->id]) {
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
  max_flow::MaxFlowComputerPushRelabel<ValueType, EdgeType> mfc;
  mfc.compute_max_flow(g);

  auto& nodes = g->nodes;
  NODE* source = &nodes[0];

  // Find nodes reachable from source in the residual graph.
  vector<NODE*> reachable_nodes = internal::find_reachable_nodes(
      source, nodes.size());

  // Find edges connecting reachable vertices to non-reachable vertices.
  vector<MinCutEdge<ValueType, EdgeType>> min_st_cut_edges =
      internal::find_outgoing_edges(reachable_nodes, nodes.size());
  
  return std::move(min_st_cut_edges);
}

} // namespace min_st_cut

#endif
