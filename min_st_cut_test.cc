#include "min_st_cut.hpp"
#include "graph_utils.hpp"
#include "logging_utils.h"

// We are really lazy here...
using namespace max_flow;
using namespace min_st_cut;
using namespace graph_utils;

void EXPECT_EQ(int a, int b, string message) {
  if (a != b) {
    cred("FAILED: ") << message << " expected: " << a << " but got: " << b
        << std::endl;
  } else {
    cgreen("PASSED: ") << message << std::endl;
  }
}

void TestSmallGraph() {
  // Node indexes, edges, and capacities
  Graph<int,EdgeCapacity,false> my_g({{0,1,2},{{1,2},{2,0}},{1.0,2.0}});
  vector<MinCutEdge<int>> min_st_cut_edges = compute_min_st_cut<int>(&my_g);
  EXPECT_EQ(1, min_st_cut_edges.size(),
      "Small min cut should contain single edge.");
  EXPECT_EQ(1, min_st_cut_edges[0].edge.capacity,
      "Small min cut edge weight should be 1.0.");
}


void TestMediumGraph() {
  // Node indexes, edges, and capacities
  Graph<int,EdgeCapacity,false> my_g = Graph<int,EdgeCapacity,false>(
      {{0,1,2,3},
      {{0,2},{0,3},{2,3},{2,1},{3,1}},
      {1.0,3.0,2.0,3.0,2.0}});
  vector<MinCutEdge<int>> min_st_cut_edges = compute_min_st_cut<int>(&my_g);
  EXPECT_EQ(2, min_st_cut_edges.size(),
      "Medium min cut should contain two edges.");
  double min_st_cut_cost = 0.0;
  for (size_t i = 0; i < min_st_cut_edges.size(); i++) {
    min_st_cut_cost += min_st_cut_edges[i].edge.capacity; 
  }
  EXPECT_EQ(4.0, min_st_cut_cost,
      "Medium min cut cost should be 4.0.");
}

void TestLargeGraph() {
  // Node indexes, edges, and capacities
  Graph<int,EdgeCapacity,false> my_g = Graph<int,EdgeCapacity,false>(
      {{0,1,2,3,4,5,6,7,8,9,10,11},
      {{0,4},{0,3},{0,2},{4,3},{3,2},{2,5},{3,6},{4,6},{5,6},{4,8},{5,7},{6,7},
       {6,8},{7,8},{7,9},{7,10},{8,10},{8,11},{9,1},{10,1},{11,1}},
      {9.0,6.0,3.0,4.0,3.0,7.0,6.0,2.0,5.0,8.0,1.0,1.0,7.0,1.0,3.0,5.0,2.0,1.0,
       5.0,4.0,3}});
  vector<MinCutEdge<int>> min_st_cut_edges = compute_min_st_cut<int>(&my_g);
  EXPECT_EQ(5, min_st_cut_edges.size(),
      "Large min cut should contain five edges.");
  double min_st_cut_cost = 0.0;
  for (size_t i = 0; i < min_st_cut_edges.size(); i++) {
    min_st_cut_cost += min_st_cut_edges[i].edge.capacity; 
  }
  EXPECT_EQ(6.0, min_st_cut_cost,
      "Large min cut cost should be 4.0.");
}

int main() {
  LOG0("Running min cut tests.");

  TestSmallGraph();
  TestMediumGraph();
  TestLargeGraph();

  return 0;
}
