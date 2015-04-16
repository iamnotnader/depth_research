#ifndef BARRIER_MIN_CUT
#define BARRIER_MIN_CUT

#include "max_flow.hpp"
#include "graph_utils.hpp"

namespace min_cut {

template<typename ValueType>
class MinCutEdge {
 public:
  MinCutEdge(graph_utils::Node<ValueType, max_flow::EdgeCapacity>* from,
      graph_utils::Node<ValueType, max_flow::EdgeCapacity>* to,
      max_flow::EdgeCapacity edge)
    : from(from), to(to), edge(edge) { }

  graph_utils::Node<ValueType, max_flow::EdgeCapacity>* from;
  graph_utils::Node<ValueType, max_flow::EdgeCapacity>* to;
  max_flow::EdgeCapacity edge;
};

// Computes the minimum st cut of an undirected graph.
// Returns edges on the minimum st cut.
template<typename ValueType>
vector<MinCutEdge<ValueType>> compute_min_cut(
    graph_utils::Graph<ValueType,max_flow::EdgeCapacity,false>* g); 

} // namespace min_cut

#endif
