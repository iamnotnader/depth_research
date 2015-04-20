#ifndef MAX_FLOW
#define MAX_FLOW

#include "graph_utils.hpp"
#include "status_defs.h"

namespace max_flow {

template<typename EdgeType>
class EdgeCapacity {
public:
  EdgeCapacity(double capacity, double flow, EdgeType edge_data)
      : capacity(capacity), flow(flow), edge_data(edge_data) { }

  EdgeCapacity(double capacity, EdgeType edge_data)
      : capacity(capacity), flow(0), edge_data(edge_data) { }

  EdgeCapacity(double capacity)
      : capacity(capacity), flow(0) { }

  double residual() const {
    return capacity - flow;
  }

  double capacity;
  double flow;
  EdgeType edge_data;
};

// Computes the maximum flow of an undirected graph.
// Returns a graph with maximum flow for each edge by reference.
template<typename ValueType, typename EdgeType>
double compute_max_flow(
    graph_utils::Graph<ValueType,
    EdgeCapacity<EdgeType>,false>* g); 

} // namespace max_flow

#endif

