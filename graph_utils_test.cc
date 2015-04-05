#include "graph_utils.hpp"

int main() {
  using namespace graph_utils;
  using namespace std;
  Graph<int,double,true> directed({0,1,2,3,4,5}, {{1,2},{4,5},{3,2},{3,1},},
      {1.0, 2.0, 3.0, 4.0});
  string expected = "0: \n1: 2 \n2: \n3: 2 1 \n4: 5 \n5: \n";
  if (expected == directed.to_string()) {
    cout << "PASSED: directed graph" << endl;
  } else {
    cout << "FAILED: directed graph" << endl;
    cout << "Expected: " << endl << expected << endl << "But got: " << endl
         << directed.to_string() << endl; 
  }

  directed.remove_edge(1, 2);
  expected = "0: \n1: \n2: \n3: 2 1 \n4: 5 \n5: \n";
  if (expected == directed.to_string()) {
    cout << "PASSED: remove directed graph" << endl;
  } else {
    cout << "FAILED: remove directed graph" << endl;
    cout << "Expected: " << endl << expected << endl << "But got: " << endl
         << directed.to_string() << endl; 
  }

  Graph<int,double,false> undirected({0,1,2,3,4,5}, {{1,2},{4,5},{3,2},{3,1},},
      {1.0, 2.0, 3.0, 4.0});
  expected = "0: \n1: 2 3 \n2: 1 3 \n3: 2 1 \n4: 5 \n5: 4 \n";
  if (expected == undirected.to_string()) {
    cout << "PASSED: undirected graph." << endl;
  } else {
    cout << "FAILED: undirected graph." << endl;
    cout << "Expected: " << endl << expected << endl << "But got: " << endl
         << undirected.to_string() << endl; 
  }

  undirected.remove_edge(4, 5);
  expected = "0: \n1: 2 3 \n2: 1 3 \n3: 2 1 \n4: \n5: \n";
  if (expected == undirected.to_string()) {
    cout << "PASSED: remove undirected graph." << endl;
  } else {
    cout << "FAILED: remove undirected graph." << endl;
    cout << "Expected: " << endl << expected << endl << "But got: " << endl
         << undirected.to_string() << endl; 
  }

  return 0;
}
