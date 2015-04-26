#include "max_flow_ford_fulkerson.hpp"
#include "max_flow_push_relabel.hpp"
#include <iostream>
#include "logging_utils.h"

// Being very lazy here...
using namespace graph_utils;
using namespace max_flow;
using std::vector;
using std::pair;

void TestTinyGraphFordFulkerson() {
  // Node indexes, edges, and capacities
  Graph<int,EdgeCapacity<int>,false> my_g({{0,1,2},{{1,2},{2,0}},{1.0,2.0}});
  MaxFlowComputerFordFulkerson<int,int> mfc;
  double max_flow = mfc.compute_max_flow(&my_g);
  if (max_flow == 1) {
    cgreen("PASSED: src <1> 2 <2> sink = 1") << endl;
  } else {
    cred("FAILED: src <1> 2 <2> sink = 1") << endl;
    cout << "Expected 1; Got: " << max_flow << endl;
  }
}
void TestMediumGraphFordFulkerson() {
  // Node indexes, edges, and capacities
  Graph<int,EdgeCapacity<int>,false> my_g = Graph<int,EdgeCapacity<int>,false>(
      {{0,1,2,3},
      {{0,2},{0,3},{2,3},{2,1},{3,1}},
      {1.0,3.0,2.0,3.0,2.0}});
  MaxFlowComputerFordFulkerson<int,int> mfc;
  double max_flow = mfc.compute_max_flow(&my_g);
  if (max_flow == 4) {
    cgreen("PASSED: four nodes") << endl;
  } else {
    cred("FAILED: four nodes") << endl;
    cout << "Expected 4; Got: " << max_flow << endl;
  }
}

void TestLargeGraphFordFulkerson() {
  Graph<int,EdgeCapacity<int>,false> my_g = Graph<int,EdgeCapacity<int>,false>(
      {{0,1,2,3,4,5,6,7,8},
      {{0,2},{0,3},{0,4},{2,3},{3,4},{2,8},{2,6},{3,6},{3,5},{4,5},{5,6},
       {5,7},{4,7},{8,6},{6,7},{8,1},{6,1},{7,1}},
      {12,15,20,5,11,5,2,6,3,4,6,1,8,9,7,18,13,10}});
  MaxFlowComputerFordFulkerson<int,int> mfc;
  double max_flow = mfc.compute_max_flow(&my_g);
  if (max_flow == 28) {
    cgreen("PASSED: 9 nodes") << endl;
  } else {
    cred("FAILED: 9 nodes") << endl;
    cout << "Expected 28; Got: " << max_flow << endl;
  }
}

void TestTinyGraphPushRelabel() {
  // Node indexes, edges, and capacities
  Graph<int,EdgeCapacity<int>,false> my_g({{0,1,2},{{1,2},{2,0}},{1.0,2.0}});
  MaxFlowComputerPushRelabel<int,int> mfc;
  double max_flow = mfc.compute_max_flow(&my_g);
  if (max_flow == 1) {
    cgreen("PASSED: src <1> 2 <2> sink = 1") << endl;
  } else {
    cred("FAILED: src <1> 2 <2> sink = 1") << endl;
    cout << "Expected 1; Got: " << max_flow << endl;
  }
}
void TestMediumGraphPushRelabel() {
  // Node indexes, edges, and capacities
  Graph<int,EdgeCapacity<int>,false> my_g = Graph<int,EdgeCapacity<int>,false>(
      {{0,1,2,3},
      {{0,2},{0,3},{2,3},{2,1},{3,1}},
      {1.0,3.0,2.0,3.0,2.0}});
  MaxFlowComputerPushRelabel<int,int> mfc;
  double max_flow = mfc.compute_max_flow(&my_g);
  if (max_flow == 4) {
    cgreen("PASSED: four nodes") << endl;
  } else {
    cred("FAILED: four nodes") << endl;
    cout << "Expected 4; Got: " << max_flow << endl;
  }
}

void TestLargeGraphPushRelabel() {
  Graph<int,EdgeCapacity<int>,false> my_g = Graph<int,EdgeCapacity<int>,false>(
      {{0,1,2,3,4,5,6,7,8},
      {{0,2},{0,3},{0,4},{2,3},{3,4},{2,8},{2,6},{3,6},{3,5},{4,5},{5,6},
       {5,7},{4,7},{8,6},{6,7},{8,1},{6,1},{7,1}},
      {12,15,20,5,11,5,2,6,3,4,6,1,8,9,7,18,13,10}});
  MaxFlowComputerPushRelabel<int,int> mfc;
  double max_flow = mfc.compute_max_flow(&my_g);
  if (max_flow == 28) {
    cgreen("PASSED: 9 nodes") << endl;
  } else {
    cred("FAILED: 9 nodes") << endl;
    cout << "Expected 28; Got: " << max_flow << endl;
  }
}

int main() {

  logging_utils::print_stack_traces();

  cout << "Running max flow tests. Nodes are indexed. 0=source, 1=sink."
       << endl;

  TestTinyGraphFordFulkerson();
  TestMediumGraphFordFulkerson();
  TestLargeGraphFordFulkerson();

  TestTinyGraphPushRelabel();
  TestMediumGraphPushRelabel();
  TestLargeGraphPushRelabel();


  return 0;
}
