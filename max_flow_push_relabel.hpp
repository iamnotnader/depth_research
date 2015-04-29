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
  vector<internal_push_relabel::NodeInfo> node_info_;
  std::priority_queue<
      NODE*, vector<NODE*>, std::function<bool(NODE*, NODE*)>>
          highest_node_queue_;
  std::unordered_map<
      pair<NODE*, NODE*>, EdgeCapacity<EdgeType>*,
      std::function<size_t(const pair<NODE*, NODE*>&)>>
          nodes_to_edges_;

  void initialize_data_structures(
      Graph<ValueType,EdgeCapacity<EdgeType>,false>* g);
  void global_relabel(Graph<ValueType,EdgeCapacity<EdgeType>,false>* g);
  bool is_node_active(NODE* node);
  void add_active_node(NODE* node);
  NODE* pop_highest_active_node();
  EdgeCapacity<EdgeType>* get_edge_between(NODE* from, NODE* to);
  int64_t get_label_for_node(NODE* node);
  double get_net_incoming_flow(NODE* node);
  void set_net_incoming_flow(NODE* node, double flow);
  void relabel_node(NODE* node);
  void discharge_highest_label_node();
  void push_flow(NODE*from, NODE* to);
};

template<typename ValueType, typename EdgeType>
void MaxFlowComputerPushRelabel<ValueType, EdgeType>
    ::global_relabel(Graph<ValueType,EdgeCapacity<EdgeType>,false>* g) {
  // Remove everything from our priority queue.
  auto node_compare = [this](NODE* a, NODE* b) {
    return get_label_for_node(a) < get_label_for_node(b);
  };
  highest_node_queue_ =
      std::priority_queue<NODE*, vector<NODE*>,
          std::function<bool(NODE*, NODE*)>>(node_compare);

  // Relabel all the nodes.
  vector<NODE>& nodes = g->nodes;
  vector<bool> visited(nodes.size(), false);
  NODE* sink = &(nodes[SINK_ID]);
  visited[SINK_ID] = true;
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
      if (!visited[nei->id] &&
          get_edge_between(nei, cur)->residual() > 0) {
        q.push_back(nei);
        visited[nei->id] = true; 
        num_nodes_in_new_level++;
      }
    }

    if (num_nodes_in_last_level == 0) {
      num_nodes_in_last_level = num_nodes_in_new_level;
      num_nodes_in_new_level = 0;
      current_label++;
    }
  }

  // Add all the active nodes back onto the priority queue.
  for (NODE& n : g->nodes) {
    if (get_net_incoming_flow(&n) > 0 &&
        n.id != SINK_ID && n.id != SOURCE_ID) {
      highest_node_queue_.push(&n);
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
    // Net flow is usually the flow INTO a node, so we have to negate it
    // to get the amount that's flowing FROM the node.
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
  node_info_[node->id].label = min_label + 1;
}

template<typename ValueType, typename EdgeType>
EdgeCapacity<EdgeType>* MaxFlowComputerPushRelabel<ValueType, EdgeType>
    ::get_edge_between(NODE* from, NODE* to) {
  return nodes_to_edges_[pair<NODE*, NODE*>(from, to)];
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
  LOG3("Adding active node: " << node->id);
  highest_node_queue_.push(node);
}

template<typename ValueType, typename EdgeType>
Node<ValueType,EdgeCapacity<EdgeType>>*
    MaxFlowComputerPushRelabel<ValueType, EdgeType>
        ::pop_highest_active_node() {
  assert(highest_node_queue_.size() > 0);
  NODE* top = highest_node_queue_.top();
  highest_node_queue_.pop();
  return top;
}

template<typename ValueType, typename EdgeType>
void MaxFlowComputerPushRelabel<ValueType, EdgeType>
    ::push_flow(NODE* node_from, NODE* node_to) {
  using internal_push_relabel::compute_net_flow_in_out;
  using internal_push_relabel::add_flow_in_out;
  if (get_label_for_node(node_from) <= get_label_for_node(node_to)) {
    return;
  }
  double flow_at_node = get_net_incoming_flow(node_from);
  EdgeCapacity<EdgeType>* edge_from_highest =
      get_edge_between(node_from, node_to);
  EdgeCapacity<EdgeType>* edge_to_highest =
      get_edge_between(node_to, node_from);
  // Net flow is usually the flow INTO a node, so we have to negate it
  // to get the amount that's flowing FROM the node.
  double flow_from_highest = -compute_net_flow_in_out<EdgeType>(
      *edge_to_highest, *edge_from_highest);
  double flow_can_send = edge_from_highest->capacity - flow_from_highest;
  if (flow_can_send > 0) {
    assert(node_to->id == SOURCE_ID || get_net_incoming_flow(node_to) >= 0);
    double flow_to_send = std::min(flow_at_node, flow_can_send);
    double old_flow = get_net_incoming_flow(node_to);
    add_flow_in_out(edge_to_highest, edge_from_highest, flow_to_send);
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
    ::discharge_highest_label_node() {
  NODE* highest_node = pop_highest_active_node();
  const vector<NODE*>& neighbors = highest_node->neighbors;
  const vector<EdgeCapacity<EdgeType>>& weights = highest_node->weights;
  while (get_net_incoming_flow(highest_node) > 0) {
    for (size_t i = 0; i < neighbors.size(); i++) {
      push_flow(highest_node, neighbors[i]);
      if (get_net_incoming_flow(highest_node) == 0) { break; }
    }
    if (get_net_incoming_flow(highest_node) > 0) {
      relabel_node(highest_node);
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
  auto& nodes = g->nodes;
  NODE* source = &nodes[SOURCE_ID];
  NODE* sink = &nodes[SINK_ID];

  auto node_compare = [this](NODE* a, NODE* b) {
    return get_label_for_node(a) < get_label_for_node(b);
  };
  highest_node_queue_ = 
      std::priority_queue<NODE*, vector<NODE*>,
          std::function<bool(NODE*, NODE*)>>(node_compare);
  int NUM_NODES = nodes.size();
  auto node_pair_hash = [NUM_NODES](const pair<NODE*,NODE*>& p) {
    // Perfect hash function since we know the number of nodes.
    return p.first->id + p.second->id * NUM_NODES;
  };
  nodes_to_edges_ = std::unordered_map<
      pair<NODE*, NODE*>, EdgeCapacity<EdgeType>*,
      std::function<size_t(const pair<NODE*, NODE*>&)>>(10, node_pair_hash);
  for (NODE& n : nodes) {
    for (int i = 0; i < n.neighbors.size(); i++) {
      NODE* neighb = n.neighbors[i];
      n.weights[i].flow = 0;
      nodes_to_edges_[pair<NODE*,NODE*>(&n, neighb)] = &(n.weights[i]);
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
    auto& edge_to_source = *(get_edge_between(neighbors[i], source));

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
  LOG3("Number of active nodes: " << highest_node_queue_.size());
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
  while (!highest_node_queue_.empty()) {
    if (num_discharges % (g->nodes.size()) == 0) {
      global_relabel(g);
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
