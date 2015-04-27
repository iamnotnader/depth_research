#ifndef BARRIER_MIN_ST_CUT
#define BARRIER_MIN_ST_CUT

// TODO(daddy): migrate this over to max_flow_push_relabel.hpp
#include "max_flow_push_relabel.hpp"
#include "graph_utils.hpp"

namespace min_st_cut {

template<typename ValueType, typename EdgeType>
class MinCutEdge {
 public:
  MinCutEdge(
      graph_utils::Node<ValueType, max_flow::EdgeCapacity<EdgeType>>* from,
      graph_utils::Node<ValueType, max_flow::EdgeCapacity<EdgeType>>* to,
      max_flow::EdgeCapacity<EdgeType> edge)
    : from(from), to(to), edge(edge) { }

  graph_utils::Node<ValueType, max_flow::EdgeCapacity<EdgeType>>* from;
  graph_utils::Node<ValueType, max_flow::EdgeCapacity<EdgeType>>* to;
  max_flow::EdgeCapacity<EdgeType> edge;
};

// Computes the minimum st cut of an undirected graph.
// Returns edges on the minimum st cut.
template<typename ValueType, typename EdgeType>
vector<MinCutEdge<ValueType, EdgeType>> compute_min_st_cut(
    graph_utils::Graph<ValueType,max_flow::EdgeCapacity<EdgeType>,false>* g); 

} // namespace min_st_cut

#endif
