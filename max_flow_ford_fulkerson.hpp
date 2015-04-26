#ifndef MAX_FLOW_HPP
#define MAX_FLOW_HPP

#include "max_flow.h"
#include <limits>
#include <algorithm>
#include "logging_utils.h"

namespace max_flow {

using std::cout;
using status_defs::Status;
using std::vector;
using graph_utils::Graph;
using std::pair;
using graph_utils::Node;

namespace internal { 

static const int NEIGHBOR_CUTOFF = 10;

template<typename ValueType, typename EdgeType>
EdgeCapacity<EdgeType>* get_edge_between(
    Node<ValueType,EdgeCapacity<EdgeType>>* node_from,
    Node<ValueType,EdgeCapacity<EdgeType>>* node_to) {
  for (int i = 0; i < node_from->neighbors.size(); i++) {
    if (node_from->neighbors[i] == node_to) {
      return &(node_from->weights[i]);
    }
  }
  return nullptr;
}

static unsigned long x=123456789, y=362436069, z=521288629;
auto fast_random = [](int i) { 
  unsigned long t;
  x ^= x << 16;
  x ^= x >> 5;
  x ^= x << 1;

  t = x;
  x = y;
  y = z;
  z = t ^ x ^ y;

  return z % i;
};

static vector<int>* random_indices(int N) {
  vector<int>* indices = new vector<int>(N);
  for (int i = 0; i < N; i++) {
    (*indices)[i] = i;
  }
  std::random_shuffle((*indices).begin(), (*indices).end(), fast_random);
  return indices;
}

// TODO(daddy): This is a really crappy way of getting around having to call
// malloc and free constantly...
class RandomVectorFactory {
 public:
  RandomVectorFactory() {}
  
  vector<int>* get_random_vector(int N) {
    for (size_t i = 0; i < free_vector_lists.size(); i++) {
      pair<int, vector<vector<int>*>>& lists = free_vector_lists[i];
      if (lists.first == N) {
        if (lists.second.size() > 0) {
          vector<int>* ret = std::move(lists.second.back());
          lists.second.pop_back();
          return std::move(ret);
        }
        break;
      }
    }
    return random_indices(N);
  }

  // Takes ownership of vec_to_free.
  void free_vector(vector<int>* vec_to_free) {
    std::random_shuffle(vec_to_free->begin(), vec_to_free->end(), fast_random);
    for (size_t i = 0; i < free_vector_lists.size(); i++) {
      pair<int, vector<vector<int>*>>& lists = free_vector_lists[i];
      if (lists.first == (int)(vec_to_free->size())) {
        free_vector_lists[i].second.push_back(std::move(vec_to_free));
        return;
      }
    }
    vector<vector<int>*> my_vecs({vec_to_free});
    free_vector_lists.push_back({vec_to_free->size(), std::move(my_vecs)});
  }

  private:
   vector<pair<int, vector<vector<int>*>>> free_vector_lists;
};

struct dfs_stack_context {
  vector<int>* indices;
  int current_index;
  static void free_dfs_stack(vector<dfs_stack_context>* dfs_stack,
                      RandomVectorFactory* vec_factory) {
    for (auto& ctx : *dfs_stack) {
      vec_factory->free_vector(std::move(ctx.indices));
    }
  }
};

template<typename ValueType, typename EdgeType>
bool find_augmenting_path_random_dfs(
    Node<ValueType,EdgeCapacity<EdgeType>>* source,
    Node<ValueType,EdgeCapacity<EdgeType>>* sink,
    vector<Node<ValueType,EdgeCapacity<EdgeType>>*>* path,
    vector<bool>& visited) {
  typedef Node<ValueType,EdgeCapacity<EdgeType>> NODE;

  static RandomVectorFactory vec_factory;
  if (source == nullptr || sink == nullptr) {
    return false;
  }

  vector<dfs_stack_context> dfs_stack;
  visited[source->id] = true;
  dfs_stack.push_back({
      vec_factory.get_random_vector(source->neighbors.size()), 0});
  path->push_back(source);
  while (!dfs_stack.empty()) {
    assert(dfs_stack.size() == path->size());
    NODE* current_node = path->back();
    if (current_node == sink) {
      dfs_stack_context::free_dfs_stack(&dfs_stack, &vec_factory);
      return true;
    }

    dfs_stack_context ctx = dfs_stack.back();
    const vector<NODE*>& neighbors = current_node->neighbors;
    const vector<EdgeCapacity<EdgeType>>& weights = current_node->weights;

    // Always trying to visit the sink first provides a small speed-up, but
    // only if we do it for nodes with a small number of neighbors. We
    // determined NEIGHBOR_CUTOFF empirically.
    if (neighbors.size() < NEIGHBOR_CUTOFF) {
      for (size_t ind = ctx.current_index; ind < ctx.indices->size(); ind++) {
          int i = (*ctx.indices)[ind];
          if (neighbors[i]->id != sink->id) {
            continue;
          }
          if (weights[i].residual() > 0) {
            dfs_stack_context::free_dfs_stack(&dfs_stack, &vec_factory);
            path->push_back(neighbors[i]);
            return true;
          }
      }
    }

    for (; ctx.current_index < (int)(ctx.indices->size()); (ctx.current_index)++) {
      int i = (*ctx.indices)[ctx.current_index];
      NODE* next_node = neighbors[i];
      const EdgeCapacity<EdgeType>& weight_edge = weights[i];
      if (visited[next_node->id]) {
        continue;
      }
      if (weight_edge.residual() <= 0) {
        continue;
      }
      // At this point, we have a useful neighbor that isn't the sink.
      // Mark as visited and push onto the stack.
      visited[next_node->id] = true;
      dfs_stack.push_back({
          vec_factory.get_random_vector(next_node->neighbors.size()), 0});
      path->push_back(next_node);
      break;
    }
    if (ctx.current_index >= (int)ctx.indices->size()) {
      vec_factory.free_vector(std::move(dfs_stack.back().indices));
      dfs_stack.pop_back();
      path->pop_back();
      continue;
    }
  }
  if (dfs_stack.empty()) {
    return false;
  }
  dfs_stack_context::free_dfs_stack(&dfs_stack, &vec_factory);
  return true;
}

// Tries to find an augmenting path. Puts the path in "path" and returns true
// if an augmenting path exists. Otherwise, leaves "path" empty and returns
// false.
template<typename ValueType, typename EdgeType>
bool find_augmenting_path(
    Node<ValueType,EdgeCapacity<EdgeType>>* source,
    Node<ValueType,EdgeCapacity<EdgeType>>* sink,
    vector<Node<ValueType,EdgeCapacity<EdgeType>>*>* path,
    vector<bool>& visited) {
  std::fill(visited.begin(), visited.end(), false);
  typedef Node<ValueType,EdgeCapacity<EdgeType>> NODE;
  return find_augmenting_path_random_dfs<ValueType, EdgeType>(
      source, sink, path, visited);
}

} // namespace internal

template<typename ValueType, typename EdgeType>
class MaxFlowComputerFordFulkerson :
    public MaxFlowComputer<ValueType, EdgeType> {
 public:
  double compute_max_flow(
      Graph<ValueType,EdgeCapacity<EdgeType>,false>* g) override;
};

// Computes the maximum flow of an undirected graph.
template<typename ValueType, typename EdgeType>
double MaxFlowComputerFordFulkerson<ValueType, EdgeType>::compute_max_flow(
    Graph<ValueType,EdgeCapacity<EdgeType>,false>* g) {
  typedef Node<ValueType,EdgeCapacity<EdgeType>> NODE;
  auto& nodes = g->nodes;
  NODE* source = &nodes[0];
  NODE* sink = &nodes[1];

  vector<NODE*> path;
  vector<bool> visited(nodes.size());
  while (true) {
    path.clear();

    bool exists_augmenting_path =
      internal::find_augmenting_path<ValueType>(source, sink, &path, visited);

    if (!exists_augmenting_path) {
      LOG3("Computing maximum flow by summing outflows from source.");
      // If no augmenting path exists, compute the maximum flow and return.
      double max_flow = 0.0;
      for (int i = 0; i < source->neighbors.size(); i++) {
        LOG3("flow: " << source->weights[i].flow << " capacity: "
             << source->weights[i].capacity << " from: " << source->id
             << " to: " << source->neighbors[i]->id);
        assert(source->weights[i].flow >= 0);
        max_flow += source->weights[i].flow;
      }
      return max_flow;
    }

    // Otherwise, push as much flow as we can through the augmenting path.
    double flow_to_add = std::numeric_limits<double>::max();

    // Find the edge with the smallest extra capacity (aka residual).
    for (int i = 0; i < path.size()-1; i++) {
      NODE* node_from = path[i];
      NODE* node_to = path[i+1];
      EdgeCapacity<EdgeType>* edge = internal::get_edge_between(node_from, node_to);

      assert(edge != nullptr);
      flow_to_add = std::min(flow_to_add, edge->residual());
    }

    // Add the flow we found to all the edges in the augmenting path.
    // Note that we need to modify BOTH the outgoing edge from each node AND
    // the incoming edge. In particular, for the outgoing edge we have:
    //    new_flow = old_flow + flow_to_add
    // For the incoming edge we have:
    //    new_flow = old_flow - flow_to_add
    for (int i = 0; i < path.size()-1; i++) {
      NODE* node_from = path[i];
      NODE* node_to = path[i+1];
      EdgeCapacity<EdgeType>* edge_forward =
          internal::get_edge_between(node_from, node_to);
      EdgeCapacity<EdgeType>* edge_reverse =
          internal::get_edge_between(node_to, node_from);

      assert(edge_forward != nullptr);
      edge_forward->flow += flow_to_add;
      
      assert(edge_reverse != nullptr);
      edge_reverse->flow -= flow_to_add;
    }
  }
}

} // namespace max_flow
#endif
