#ifndef MAX_FLOW_PUSH_RELABEL
#define MAX_FLOW_PUSH_RELABEL

#include "max_flow.h"
#include <limits>
#include <algorithm>
#include "logging_utils.h"
#include <sstream>
#include <limits>

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

template<typename ValueType, typename EdgeType>
EdgeCapacity<EdgeType>* get_edge_between(
    Node<ValueType,EdgeCapacity<EdgeType>>* node_from,
    Node<ValueType,EdgeCapacity<EdgeType>>* node_to) {
  for (size_t i = 0; i < node_from->neighbors.size(); i++) {
    if (node_from->neighbors[i] == node_to) {
      return &(node_from->weights[i]);
    }
  }
  return nullptr;
}

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
  double compute_max_flow(
      Graph<ValueType,EdgeCapacity<EdgeType>,false>* g) override;
 private:
  vector<vector<Node<ValueType,EdgeCapacity<EdgeType>>*>> binned_active_nodes_;
  int64_t highest_active_node_bin_;
  vector<internal_push_relabel::NodeInfo> node_info_;
  int64_t num_active_nodes_;

  void initialize_data_structures(
      Graph<ValueType,EdgeCapacity<EdgeType>,false>* g);
  void add_active_node(Node<ValueType,EdgeCapacity<EdgeType>>* node);
  Node<ValueType,EdgeCapacity<EdgeType>>* pop_highest_active_node();
  bool is_node_active(Node<ValueType,EdgeCapacity<EdgeType>>* node);
  void discharge_highest_label_node();
  void relabel_node(Node<ValueType,EdgeCapacity<EdgeType>>* node);
  int64_t get_label_for_node(Node<ValueType,EdgeCapacity<EdgeType>>* node);
  double get_net_incoming_flow(Node<ValueType,EdgeCapacity<EdgeType>>* node);
  void set_net_incoming_flow(Node<ValueType,EdgeCapacity<EdgeType>>* node,
      double flow);

};

template<typename ValueType, typename EdgeType>
void MaxFlowComputerPushRelabel<ValueType, EdgeType>
    ::relabel_node(Node<ValueType,EdgeCapacity<EdgeType>>* node) {
  typedef Node<ValueType,EdgeCapacity<EdgeType>> NODE;
  using internal_push_relabel::get_edge_between;
  using internal_push_relabel::compute_net_flow_in_out;

  assert(is_node_active(node));
  const vector<NODE*>& neighbors = node->neighbors;
  const vector<EdgeCapacity<EdgeType>>& weights = node->weights;
  int64_t min_label = std::numeric_limits<int64_t>::max();
  for (size_t i = 0; i < neighbors.size(); i++) {
    NODE* node_to = neighbors[i];
    const EdgeCapacity<EdgeType>& edge_from_node = weights[i];
    const EdgeCapacity<EdgeType>& edge_to_node =
        *(get_edge_between(node_to, node));
    // Net flow is usually the flow INTO a node, so we have to negate it
    // to get the amount that's flowing FROM the node.
    double flow_from_node = -compute_net_flow_in_out<EdgeType>(
        edge_to_node, edge_from_node);
    double flow_can_send = edge_from_node.capacity - flow_from_node;
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
double MaxFlowComputerPushRelabel<ValueType, EdgeType>
    ::get_net_incoming_flow(Node<ValueType,EdgeCapacity<EdgeType>>* node) {
  return node_info_[node->id].net_flow;
}

template<typename ValueType, typename EdgeType>
void MaxFlowComputerPushRelabel<ValueType, EdgeType>
    ::set_net_incoming_flow(Node<ValueType,EdgeCapacity<EdgeType>>* node,
        double flow) {
  node_info_[node->id].net_flow = flow;
}

template<typename ValueType, typename EdgeType>
int64_t MaxFlowComputerPushRelabel<ValueType, EdgeType>
    ::get_label_for_node(Node<ValueType,EdgeCapacity<EdgeType>>* node) {
  return node_info_[node->id].label;
}

template<typename ValueType, typename EdgeType>
bool MaxFlowComputerPushRelabel<ValueType, EdgeType>
    ::is_node_active(Node<ValueType,EdgeCapacity<EdgeType>>* node) {
  return get_net_incoming_flow(node) > 0;
}

template<typename ValueType, typename EdgeType>
void MaxFlowComputerPushRelabel<ValueType, EdgeType>
    ::add_active_node(Node<ValueType,EdgeCapacity<EdgeType>>* node) {
  assert(is_node_active(node));
  LOG3("Adding active node: " << node->id);
  int64_t node_label = node_info_[node->id].label;
  if (node_label > highest_active_node_bin_) {
    LOG3("Adjusting active node index to: " << node_label);
    highest_active_node_bin_ = node_label;
  }
  num_active_nodes_++;
  binned_active_nodes_[node_label].push_back(node);
}

template<typename ValueType, typename EdgeType>
Node<ValueType,EdgeCapacity<EdgeType>>*
    MaxFlowComputerPushRelabel<ValueType, EdgeType>
        ::pop_highest_active_node() {
  assert(num_active_nodes_ > 0);
  typedef Node<ValueType,EdgeCapacity<EdgeType>> NODE;
  NODE* highest_node = binned_active_nodes_[highest_active_node_bin_].back();
  binned_active_nodes_[highest_active_node_bin_].pop_back();
  if (binned_active_nodes_[highest_active_node_bin_].size() == 0) {
    // If we popped the only node with the highest label, we need to move the
    // index down to the next-highest node.
    while (highest_active_node_bin_ > 0) {
      if (binned_active_nodes_[highest_active_node_bin_].size() > 0) {
        break;
      }
      highest_active_node_bin_--;
    }
  }
  num_active_nodes_--;
  assert((num_active_nodes_ == 0 && highest_active_node_bin_ == 0) ||
         binned_active_nodes_[highest_active_node_bin_].size() > 0);
  return highest_node;
}

template<typename ValueType, typename EdgeType>
void MaxFlowComputerPushRelabel<ValueType, EdgeType>
    ::discharge_highest_label_node() {
  typedef Node<ValueType,EdgeCapacity<EdgeType>> NODE;
  using internal_push_relabel::get_edge_between;
  using internal_push_relabel::compute_net_flow_in_out;
  using internal_push_relabel::add_flow_in_out;

  NODE* highest_node = pop_highest_active_node();
  const vector<NODE*>& neighbors = highest_node->neighbors;
  const vector<EdgeCapacity<EdgeType>>& weights = highest_node->weights;
  double flow_at_highest;
  while ((flow_at_highest = get_net_incoming_flow(highest_node)) > 0) {
    for (size_t i = 0; i < neighbors.size(); i++) {
      NODE* node_to = neighbors[i];
      assert(weights[i].residual() == 0 ||
          get_label_for_node(highest_node) <=
          (get_label_for_node(node_to)+1));
      if (get_label_for_node(highest_node) !=
          (get_label_for_node(node_to) + 1)) {
        continue;
      }
      EdgeCapacity<EdgeType>* edge_from_highest =
          get_edge_between(highest_node, node_to);
      EdgeCapacity<EdgeType>* edge_to_highest =
          get_edge_between(node_to, highest_node);
      // Net flow is usually the flow INTO a node, so we have to negate it
      // to get the amount that's flowing FROM the node.
      double flow_from_highest = -compute_net_flow_in_out<EdgeType>(
          *edge_to_highest, *edge_from_highest);
      double flow_can_send = edge_from_highest->capacity - flow_from_highest;
      if (flow_can_send > 0) {
        assert(node_to->id == SOURCE_ID || get_net_incoming_flow(node_to) >= 0);
        double flow_to_send = std::min(flow_at_highest, flow_can_send);
        double old_flow = get_net_incoming_flow(node_to);
        add_flow_in_out(edge_to_highest, edge_from_highest, flow_to_send);
        flow_at_highest -= flow_to_send;
        set_net_incoming_flow(highest_node, flow_at_highest);
        set_net_incoming_flow(node_to, old_flow + flow_to_send);
        if (old_flow == 0 && node_to->id != SOURCE_ID &&
            node_to->id != SINK_ID) {
          add_active_node(node_to);
        }
        if (flow_at_highest == 0) { break; }
      }
    }
    if (flow_at_highest > 0) {
      relabel_node(highest_node);
    }
  }
}

template<typename ValueType, typename EdgeType>
void MaxFlowComputerPushRelabel<ValueType, EdgeType>
    ::initialize_data_structures(
        Graph<ValueType,EdgeCapacity<EdgeType>,false>* g) {
  typedef Node<ValueType,EdgeCapacity<EdgeType>> NODE;
  using internal_push_relabel::NodeInfo;
  using internal_push_relabel::get_edge_between;
  using internal_push_relabel::compute_net_flow_in_out;
  using internal_push_relabel::add_flow_in_out;
  auto& nodes = g->nodes;
  NODE* source = &nodes[SOURCE_ID];
  NODE* sink = &nodes[SINK_ID];

  binned_active_nodes_ = vector<vector<NODE*>>(2*nodes.size(), {});
  highest_active_node_bin_ = 0;
  num_active_nodes_ = 0;

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
      std::cout << node_to->id << std::endl;
      add_active_node(node_to);
    }
    LOG3("neighbor: " << node_info_[node_to->id].DebugString());
  }
  LOG3("source: " << node_info_[source->id].DebugString());
  LOG3("Number of active nodes: " << num_active_nodes_);
}

// Computes the maximum flow of an undirected graph.
// Assumes node id's correspond directly to position in graph's "nodes" list.
template<typename ValueType, typename EdgeType>
double MaxFlowComputerPushRelabel<ValueType, EdgeType>::compute_max_flow(
    Graph<ValueType,EdgeCapacity<EdgeType>,false>* g) {
  typedef Node<ValueType,EdgeCapacity<EdgeType>> NODE;
  LOG3("Initializing...");
  initialize_data_structures(g);
  LOG3("Finished initialization.");
  while (num_active_nodes_ != 0) {
    // Discharge the node with the highest label.
    discharge_highest_label_node();
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
