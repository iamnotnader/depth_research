#ifndef MAX_FLOW
#define MAX_FLOW

#include "graph_utils.hpp"
#include "status_defs.h"

namespace max_flow {

class EdgeCapacity {
public:
  EdgeCapacity(double capacity, double flow)
      : capacity(capacity), flow(flow) { }

  EdgeCapacity(double capacity)
      : capacity(capacity), flow(0) { }

  double residual() const {
    return capacity - flow;
  }

  double capacity;
  double flow;
};

// Computes the maximum flow of an undirected graph.
// Returns a graph with maximum flow for each edge by reference.
template<typename ValueType>
double compute_max_flow(graph_utils::Graph<ValueType,EdgeCapacity,false>* g); 

} // namespace max_flow

#endif

