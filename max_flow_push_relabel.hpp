#ifndef MAX_FLOW_PUSH_RELABEL
#define MAX_FLOW_PUSH_RELABEL

#include "max_flow.h"
#include <limits>
#include <algorithm>
#include "logging_utils.h"
#include <sstream>
#include <limits>
#include <queue>
#include <deque>
#include <unordered_map>
#include <algorithm>

namespace max_flow {

using std::cout;
using status_defs::Status;
using std::vector;
using graph_utils::Graph;
using std::pair;
using graph_utils::Node;

const int SOURCE_ID = 0;
const int SINK_ID = 1;

namespace internal_push_relabel { 

// A convenience method for adjusting the flow on an undirected edge, which
// consists of two directed edges with inverse flow. The function is suffixed
// with in_out to remind the caller that the edges must be given in order
// with the incoming edge first.
template<typename EdgeType>
void add_flow_in_out(
    EdgeCapacity<EdgeType>* incoming_edge,
    EdgeCapacity<EdgeType>* outgoing_edge,
    double flow_to_add) {
  assert(incoming_edge->capacity == outgoing_edge->capacity);
  assert(incoming_edge->flow == -outgoing_edge->flow);
  assert(incoming_edge != outgoing_edge);
  outgoing_edge->flow = outgoing_edge->flow + flow_to_add;
  incoming_edge->flow = incoming_edge->flow - flow_to_add;
}

// A convenience method for computing the net flow on an undirected edge, which
// consists of two directed edges with inverse flow. The function is suffixed
// with in_out to remind the caller that the edges must be given in order
// with the incoming edge first.
template<typename EdgeType>
double compute_net_flow_in_out(
    const EdgeCapacity<EdgeType>& incoming_edge,
    const EdgeCapacity<EdgeType>& outgoing_edge) {
  assert(incoming_edge.capacity == outgoing_edge.capacity);
  assert(incoming_edge.flow == -outgoing_edge.flow);
  return (incoming_edge.flow - outgoing_edge.flow) / 2;
}

struct NodeInfo {
  NodeInfo(double net_flow, int64_t label) : net_flow(net_flow), label(label) {}
  double net_flow = 0.0;
  int64_t label = 0;

  string DebugString() {
    std::ostringstream convert;
    convert << "net_flow: " << net_flow << " label: " << label;
    return convert.str();
  }
};

} // namespace internal_push_relabel

template<typename ValueType, typename EdgeType>
class MaxFlowComputerPushRelabel :
    public MaxFlowComputer<ValueType, EdgeType> {
 public:
  typedef Node<ValueType,EdgeCapacity<EdgeType>> NODE;
  double compute_max_flow(
      Graph<ValueType,EdgeCapacity<EdgeType>,false>* g) override;

 private:
  // Basically maps node id's to <net_incoming_flow, label>
  vector<internal_push_relabel::NodeInfo> node_info_;
  // The number of TOTAL nodes (including inactive) for each label. Used for
  // quickly determining whether or not we want to do a gap relabel.
  vector<NODE*> binned_all_nodes;
  // The number of active nodes for each label.
  vector<vector<NODE*>> binned_active_nodes_;
  // The index into binned_active_nodes_ where the highest active node can
  // be found.
  int highest_active_node_label_;
  int num_active_nodes_;
  // Used to do BFS in global_relabel. The cost of reallocating it every time
  // was too large.
  vector<bool> visited_;

  // Save a convenience pointer to the graph. Yes, this is dirty.
  Graph<ValueType,EdgeCapacity<EdgeType>,false>* g_;
  // Initialize all of our data structures.
  // TODO(daddy): This is basically a constructor. We need to clean up this
  // interface. Maybe make the constructor take a unique_ptr to the grah or
  // somthing...
  void initialize_data_structures(
      Graph<ValueType,EdgeCapacity<EdgeType>,false>* g);
  // Performs the global relabel heuristic from a paper entitled
  // "on implementing push-relabel method for the maximum flow problem" by
  // Goldberg + Cherkassky '94.
  void global_relabel();
  // Implements the gap relabel heuristic from the same paper.
  void gap_relabel(int64_t gap_label);
  // The critical methods of the push-relabel algorithm by Tarjan+Goldberg.
  void discharge_highest_label_node();
  void relabel_node(NODE* node);
  void push_flow(NODE*from, NODE* to, EdgeCapacity<EdgeType>* edge_from_to);
  // Convenience methods.
  bool is_node_active(NODE* node);
  void add_active_node(NODE* node);
  NODE* pop_highest_active_node();
  int64_t get_label_for_node(NODE* node);
  double get_net_incoming_flow(NODE* node);
  void set_net_incoming_flow(NODE* node, double flow);
  void add_node_to_bin(NODE* node, int64_t bin);
  void remove_node_from_bin(NODE* node, int64_t bin);
};

template<typename ValueType, typename EdgeType>
void MaxFlowComputerPushRelabel<ValueType, EdgeType>
    ::add_node_to_bin(NODE* node, int64_t bin) {
  int64_t node_label = get_label_for_node(node);
  assert(node_label == bin);
  NODE* first_node = binned_all_nodes[bin];
  if (first_node == nullptr) {
    binned_all_nodes[bin] = node;
    node->prev = nullptr;
    node->next = nullptr;
    return;
  }
  // Insert the node into the linked-list corresponding to nodes with label
  // zero.
  binned_all_nodes[bin] = node;
  first_node->prev = node;
  node->next = first_node;
  node->prev = nullptr;
}

template<typename ValueType, typename EdgeType>
void MaxFlowComputerPushRelabel<ValueType, EdgeType>
    ::remove_node_from_bin(NODE* node, int64_t bin) {
  assert (get_label_for_node(node) == bin);
  NODE* node_prev = node->prev;
  NODE* node_next = node->next;
  if (node_next == nullptr && node_prev == nullptr) {
    binned_all_nodes[bin] = nullptr;
    return;
  }
  if (node_prev != nullptr) {
    node_prev->next = node_next;
  } else {
    binned_all_nodes[bin] = node_next;
  }
  if (node_next != nullptr) {
    node_next->prev = node_prev;
  }
  node->next = nullptr;
  node->prev = nullptr;
}

template<typename ValueType, typename EdgeType>
void MaxFlowComputerPushRelabel<ValueType, EdgeType>
    ::global_relabel() {
  // Remove all the active nodes so we can muddle all their labels.
  while (num_active_nodes_ > 0) {
    pop_highest_active_node();
  }
  for (NODE& n : g_->nodes) {
    remove_node_from_bin(&n, get_label_for_node(&n));
  }

  // Reset the visited vector.
  for (size_t i = 0; i < visited_.size(); i++) {
    visited_[i] = false;
  }

  // Relabel all the nodes.
  vector<NODE>& nodes = g_->nodes;
  NODE* sink = &(nodes[SINK_ID]);
  visited_[SINK_ID] = true;
  std::deque<NODE*> q = {sink};
  int current_label = 0;
  int num_nodes_in_last_level = 1;
  int num_nodes_in_new_level = 0;
  while (!q.empty()) {
    NODE* cur = q.front();
    q.pop_front();
    node_info_[cur->id].label = current_label;
    num_nodes_in_last_level--;
    for (size_t i = 0; i < cur->neighbors.size(); i++) {
      NODE* nei = cur->neighbors[i];
      if (!visited_[nei->id] &&
          cur->weights[i].corresponding_edge->residual() > 0) {
        q.push_back(nei);
        visited_[nei->id] = true; 
        num_nodes_in_new_level++;
      }
    }

    if (num_nodes_in_last_level == 0) {
      num_nodes_in_last_level = num_nodes_in_new_level;
      num_nodes_in_new_level = 0;
      current_label++;
    }
  }

  // Add all the active nodes back.
  for (NODE& n : g_->nodes) {
    add_node_to_bin(&n, get_label_for_node(&n));
    if (get_net_incoming_flow(&n) > 0 &&
        n.id != SINK_ID && n.id != SOURCE_ID) {
      add_active_node(&n);
    }
  }
}

static int num_relabels = 0;
template<typename ValueType, typename EdgeType>
void MaxFlowComputerPushRelabel<ValueType, EdgeType>
    ::relabel_node(NODE* node) {
  using internal_push_relabel::compute_net_flow_in_out;
  num_relabels++;

  assert(is_node_active(node));
  const vector<NODE*>& neighbors = node->neighbors;
  const vector<EdgeCapacity<EdgeType>>& weights = node->weights;
  int64_t min_label = std::numeric_limits<int64_t>::max();
  for (size_t i = 0; i < neighbors.size(); i++) {
    NODE* node_to = neighbors[i];
    double flow_can_send = weights[i].residual();
    if (flow_can_send > 0) {
      // Labels for nodes we are relabelling should ALWAYS be <= labels for
      // neighboring nodes we can send flow to. Otherwise, we would do a push
      // operation to one of those nodes instead.
      assert(get_label_for_node(node) <= get_label_for_node(node_to));
      if (get_label_for_node(node_to) < min_label) {
        min_label = get_label_for_node(node_to);
      }
    }
  }
  assert(min_label != std::numeric_limits<int64_t>::max());  
  remove_node_from_bin(node, get_label_for_node(node));
  int64_t new_label = min_label + 1;
  node_info_[node->id].label = new_label;
  add_node_to_bin(node, new_label);
}

template<typename ValueType, typename EdgeType>
double MaxFlowComputerPushRelabel<ValueType, EdgeType>
    ::get_net_incoming_flow(NODE* node) {
  return node_info_[node->id].net_flow;
}

template<typename ValueType, typename EdgeType>
void MaxFlowComputerPushRelabel<ValueType, EdgeType>
    ::set_net_incoming_flow(NODE* node,
        double flow) {
  node_info_[node->id].net_flow = flow;
}

template<typename ValueType, typename EdgeType>
int64_t MaxFlowComputerPushRelabel<ValueType, EdgeType>
    ::get_label_for_node(NODE* node) {
  return node_info_[node->id].label;
}

template<typename ValueType, typename EdgeType>
bool MaxFlowComputerPushRelabel<ValueType, EdgeType>
    ::is_node_active(NODE* node) {
  return get_net_incoming_flow(node) > 0;
}

template<typename ValueType, typename EdgeType>
void MaxFlowComputerPushRelabel<ValueType, EdgeType>
    ::add_active_node(NODE* node) {
  assert(is_node_active(node));
  int64_t node_label = get_label_for_node(node);
  binned_active_nodes_[node_label].push_back(node);
  if (node_label > highest_active_node_label_) {
    highest_active_node_label_ = node_label;
  }
  num_active_nodes_++;
}

template<typename ValueType, typename EdgeType>
Node<ValueType,EdgeCapacity<EdgeType>>*
    MaxFlowComputerPushRelabel<ValueType, EdgeType>
        ::pop_highest_active_node() {
  assert(num_active_nodes_ > 0);
  NODE* top = binned_active_nodes_[highest_active_node_label_].back();
  binned_active_nodes_[highest_active_node_label_].pop_back();
  while (highest_active_node_label_ > 0 &&
         binned_active_nodes_[highest_active_node_label_].size() == 0) {
    highest_active_node_label_--;
  }
  num_active_nodes_--;
  return top;
}

template<typename ValueType, typename EdgeType>
void MaxFlowComputerPushRelabel<ValueType, EdgeType>
    ::push_flow(NODE* node_from, NODE* node_to,
                EdgeCapacity<EdgeType>* edge_from_to) {
  using internal_push_relabel::compute_net_flow_in_out;
  using internal_push_relabel::add_flow_in_out;
  if (get_label_for_node(node_from) <= get_label_for_node(node_to)) {
    return;
  }
  double flow_at_node = get_net_incoming_flow(node_from);
  EdgeCapacity<EdgeType>* edge_to_from = edge_from_to->corresponding_edge;

  // Net flow is usually the flow INTO a node, so we have to negate it
  // to get the amount that's flowing FROM the node.
  double flow_from_highest = -compute_net_flow_in_out<EdgeType>(
      *edge_to_from, *edge_from_to);
  double flow_can_send = edge_from_to->capacity - flow_from_highest;
  if (flow_can_send > 0) {
    assert(node_to->id == SOURCE_ID || get_net_incoming_flow(node_to) >= 0);
    double flow_to_send = std::min(flow_at_node, flow_can_send);
    double old_flow = get_net_incoming_flow(node_to);
    add_flow_in_out(edge_to_from, edge_from_to, flow_to_send);
    flow_at_node -= flow_to_send;
    set_net_incoming_flow(node_from, flow_at_node);
    set_net_incoming_flow(node_to, old_flow + flow_to_send);
    if (old_flow == 0 && node_to->id != SOURCE_ID &&
        node_to->id != SINK_ID) {
      add_active_node(node_to);
    }
  }
}

template<typename ValueType, typename EdgeType>
void MaxFlowComputerPushRelabel<ValueType, EdgeType>
    ::gap_relabel(int64_t gap_label) {
  // TODO(daddy): I think we can actually relabel gapped nodes to infinity
  // but need to see...
  int64_t num_nodes = g_->nodes.size(); 
  for (int i = gap_label; i < num_nodes; i++) {
    // Upgrade all the nodes, including the inactive ones. This messes up
    // binned_active_nodes_ but it's ok because we fix it right after.
    while (binned_all_nodes[i] != nullptr) {
      NODE* cur = binned_all_nodes[i];
      remove_node_from_bin(cur, i);
      node_info_[cur->id].label = num_nodes;
      add_node_to_bin(cur, num_nodes);
    }
    // Upgrade all of the active nodes (fixing what we broke above).
    for (NODE* n : binned_active_nodes_[i]) {
      node_info_[n->id].label = num_nodes;
      binned_active_nodes_[num_nodes].push_back(n);
    }
    binned_active_nodes_[i].clear();
  }
  if (highest_active_node_label_ < num_nodes &&
      binned_active_nodes_[num_nodes].size() > 0) {
    highest_active_node_label_ = num_nodes;
  }
}

template<typename ValueType, typename EdgeType>
void MaxFlowComputerPushRelabel<ValueType, EdgeType>
    ::discharge_highest_label_node() {
  NODE* highest_node = pop_highest_active_node();
  const vector<NODE*>& neighbors = highest_node->neighbors;
  vector<EdgeCapacity<EdgeType>>& weights = highest_node->weights;
  while (get_net_incoming_flow(highest_node) > 0) {
    for (size_t i = 0; i < neighbors.size(); i++) {
      push_flow(highest_node, neighbors[i], &(weights[i]));
      if (get_net_incoming_flow(highest_node) == 0) { break; }
    }
    if (get_net_incoming_flow(highest_node) > 0) {
      int64_t highest_node_label = get_label_for_node(highest_node);
      assert (binned_all_nodes[highest_node_label] != nullptr);
      if (binned_all_nodes[highest_node_label]->next == nullptr) {
        // If this is the last node with its label, we'll have a gap. So we
        // should also relabel all the other nodes.
        gap_relabel(highest_node_label);
      } else {
        relabel_node(highest_node);
      }
    }
  }
}

template<typename ValueType, typename EdgeType>
void MaxFlowComputerPushRelabel<ValueType, EdgeType>
    ::initialize_data_structures(
        Graph<ValueType,EdgeCapacity<EdgeType>,false>* g) {
  num_relabels = 0;
  using internal_push_relabel::NodeInfo;
  using internal_push_relabel::compute_net_flow_in_out;
  using internal_push_relabel::add_flow_in_out;
  g_ = g;
  num_active_nodes_ = 0;
  auto& nodes = g->nodes;
  NODE* source = &nodes[SOURCE_ID];
  NODE* sink = &nodes[SINK_ID];
  binned_all_nodes = vector<NODE*>(2*nodes.size(), nullptr);

  // Used by bfs in global_relabel()
  visited_ = vector<bool>(nodes.size());

  auto node_compare = [this](NODE* a, NODE* b) {
    return get_label_for_node(a) < get_label_for_node(b);
  };
  binned_active_nodes_ = vector<vector<NODE*>>(2*nodes.size(), {});
  highest_active_node_label_ = 0;
  int num_nodes = nodes.size();
  auto node_pair_hash = [num_nodes](const pair<NODE*,NODE*>& p) {
    // TODO(daddy): Make this more rando. Seems to work fine right now and
    // it's fast...
    return p.first->id + p.second->id * num_nodes;
  };
  std::unordered_map<
      pair<NODE*, NODE*>, EdgeCapacity<EdgeType>*,
      std::function<size_t(const pair<NODE*, NODE*>&)>>
          nodes_to_edges(nodes.size()*10, node_pair_hash);
  // Build the hashmap of node pairs to edges.
  for (NODE& n : nodes) {
    for (int i = 0; i < n.neighbors.size(); i++) {
      NODE* neighb = n.neighbors[i];
      n.weights[i].flow = 0;
      n.next = nullptr;
      n.prev = nullptr;
      nodes_to_edges[pair<NODE*,NODE*>(&n, neighb)] = &(n.weights[i]);
    }
  }
  // Use the hashmap we built above to correspond all the edges with each other.
  for (NODE& n : nodes) {
    for (int i = 0; i < n.neighbors.size(); i++) {
      auto edge_from_n = &(n.weights[i]);
      auto edge_to_n = nodes_to_edges[pair<NODE*,NODE*>(n.neighbors[i], &n)];
      edge_from_n->corresponding_edge = edge_to_n;
      edge_to_n->corresponding_edge = edge_from_n;
    }
  }

  LOG3("Setting initial edge weights from source.");
  node_info_ = vector<NodeInfo>(nodes.size(), {0.0, 0});
  node_info_[source->id] = {0.0, (int64_t)(nodes.size())};
  LOG3("source: " << node_info_[source->id].DebugString());
  LOG3("sink: " << node_info_[sink->id].DebugString());

  const vector<NODE*>& neighbors = source->neighbors;
  vector<EdgeCapacity<EdgeType>>& weights = source->weights;
  // Saturate the edges leaving the source.
  for (size_t i = 0; i < neighbors.size(); i++) {
    NODE* node_to = neighbors[i];
    auto& edge_from_source = weights[i];
    auto& edge_to_source = *(weights[i].corresponding_edge);

    assert(edge_from_source.capacity == edge_to_source.capacity);

    add_flow_in_out<EdgeType>(
        &edge_to_source, &edge_from_source, edge_from_source.capacity);

    node_info_[source->id].net_flow += compute_net_flow_in_out<EdgeType>(
        edge_to_source, edge_from_source);
    node_info_[node_to->id].net_flow = compute_net_flow_in_out<EdgeType>(
        edge_from_source, edge_to_source);
    if (get_net_incoming_flow(node_to) > 0) {
      add_active_node(node_to);
    }
    LOG3("neighbor: " << node_info_[node_to->id].DebugString());
  }
  LOG3("source: " << node_info_[source->id].DebugString());
  LOG3("Number of active nodes: " << num_active_nodes_);
  for (NODE& n : nodes) {
    int64_t node_label = get_label_for_node(&n);
    add_node_to_bin(&n, node_label);
  }
}

// Computes the maximum flow of an undirected graph.
// Assumes node id's correspond directly to position in graph's "nodes" list.
template<typename ValueType, typename EdgeType>
double MaxFlowComputerPushRelabel<ValueType, EdgeType>::compute_max_flow(
    Graph<ValueType,EdgeCapacity<EdgeType>,false>* g) {
  LOG3("Initializing...");
  initialize_data_structures(g);
  LOG3("Finished initialization.");

  int64_t num_discharges = 0;
  while (num_active_nodes_ > 0) {
    if (num_discharges % (g->nodes.size()) == 0) {
      global_relabel();
    }
    if (num_discharges % 1000000 == 0) {
      LOG0("Percent complete: "
           << (double)num_relabels / (g->nodes.size() * g->nodes.size()) * 100);
    }
    // Discharge the node with the highest label.
    discharge_highest_label_node();
    num_discharges++;
  }

  // Compute the max flow value for the graph and return.
  auto& nodes = g->nodes;
  NODE* sink = &nodes[SINK_ID];
  double max_flow = 0.0;
  for (int i = 0; i < sink->neighbors.size(); i++) {
    LOG3("flow: " << sink->weights[i].flow << " capacity: "
          << sink->weights[i].capacity << " from: " << sink->id
          << " to: " << sink->neighbors[i]->id);
    assert(sink->weights[i].flow <= 0);
    max_flow -= sink->weights[i].flow;
  }
  return max_flow;
}

} // namespace max_flow
#endif
