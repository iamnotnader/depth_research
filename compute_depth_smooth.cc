#include "compute_depth_smooth.h"
#include "compute_depth_fast.h"
#include "max_flow.hpp"
#include "graph_utils.hpp"
#include "min_st_cut.hpp"

using std::unique_ptr;
using cv::Mat;
using max_flow::EdgeCapacity;

namespace compute_depth_smooth {

const int SOURCE_ID = 0;
const int SINK_ID = 1;

namespace {

int row_col_to_node_id(int row, int col, int num_cols) {
  // We add 2 for the source and sink.
  return row*num_cols + col + 2;
};

}

void GraphCutEnergyMinimizer::connect_pixel_to_neighbor(
     int from_row, int from_col, int to_row, int to_col,
     int alpha_expansion_depth,
     const std::unique_ptr<Mat>& depth_map,
     vector<PixelMetadata>* pixels,
     vector<pair<int,int>>* pixel_connections,
     vector<EdgeCapacity<EdgeMetadata>>* pixel_weights) {
  if (from_row < 0 || from_row >= depth_map->cols ||
      to_row < 0 || to_row >= depth_map->cols ||
      from_col < 0 || from_col >= depth_map->rows ||
      to_col < 0 || to_col >= depth_map->rows) {
    // Don't do anything if the pixel is out of bounds.
    return;
  }

  int from_pixel_depth = depth_map->at<uchar>(from_row, from_col, 0);
  int to_pixel_depth = depth_map->at<uchar>(to_row, to_col, 0);

  int from_id = row_col_to_node_id(from_row, from_col, depth_map->cols);
  int to_id = row_col_to_node_id(to_row, to_col, depth_map->cols);

  if (from_pixel_depth == to_pixel_depth) {
    // smooth(a, alpha)
    double to_from_cost =
        smoothing_error_(from_pixel_depth, alpha_expansion_depth);
    pixel_connections->push_back(pair<int,int>(from_id, to_id));
    pixel_weights->push_back(
        EdgeCapacity<EdgeMetadata>(
            to_from_cost, EdgeMetadata(EdgeType::PixelToPixel)));
  } else {
    // Create a new node, mid, whose value doesn't matter
    int mid_id = pixels->size();
    pixels->push_back(PixelMetadata(-1,-1,-1));

    // THe edge from <-> mid = smooth(from, alpha)
    double from_mid_cost =
        smoothing_error_(from_pixel_depth, alpha_expansion_depth);
    pixel_connections->push_back(pair<int,int>(mid_id, from_id));
    pixel_weights->push_back(
        EdgeCapacity<EdgeMetadata>(
            from_mid_cost, EdgeMetadata(EdgeType::PixelToMid)));

    // The edge to <-> mid = smooth(to, alpha)
    double to_mid_cost =
        smoothing_error_(to_pixel_depth, alpha_expansion_depth);
    pixel_connections->push_back(pair<int,int>(mid_id, to_id));
    pixel_weights->push_back(
        EdgeCapacity<EdgeMetadata>(
            to_mid_cost, EdgeMetadata(EdgeType::PixelToMid)));

    // The edge mid <-> sink = smooth(from, to)
    double mid_sink_cost = smoothing_error_(from_pixel_depth, to_pixel_depth);
    pixel_connections->push_back(pair<int,int>(mid_id, SINK_ID));
    pixel_weights->push_back(
        EdgeCapacity<EdgeMetadata>(
            mid_sink_cost, EdgeMetadata(EdgeType::MidToSink)));
  }
}

void GraphCutEnergyMinimizer::normalize_depth_map(cv::Mat* depth_map) {
  assert(depth_map->type() == CV_8UC1);
  for (int row_index = 0; row_index < depth_map->rows; row_index++) {
    for (int col_index = 0; col_index < depth_map->cols; col_index++) {
      uchar disparity = depth_map->at<uchar>(row_index, col_index, 0);
      if (disparity < 0) {
        disparity = 0;
      }
      assert(disparity <= max_disparity_);
      depth_map->at<uchar>(row_index, col_index, 0) =
          disparity * 255 / max_disparity_;
    }
  }
}


graph_utils::Graph<GraphCutEnergyMinimizer::PixelMetadata,
                   EdgeCapacity<GraphCutEnergyMinimizer::EdgeMetadata>,
                   false>
    GraphCutEnergyMinimizer::construct_graph_for_alpha(
        const std::unique_ptr<Mat>& depth_map,
        int alpha_expansion_depth) {
  // First we create an object for each pixel.
  // The first two points are the source and the sink, whose values don't
  // matter.
  vector<PixelMetadata> pixels;
  pixels.push_back(PixelMetadata(-1, -1, -1));
  pixels.push_back(PixelMetadata(-1, -1, -1));
  int num_alpha_pixels = 0;
  for (int row_index = 0; row_index < depth_map->rows; row_index++) {
    for (int col_index = 0; col_index < depth_map->cols; col_index++) {
      pixels.push_back(PixelMetadata(row_index, col_index,
                       depth_map->at<uchar>(row_index, col_index, 0)));      
      if (depth_map->at<uchar>(row_index, col_index, 0) == alpha_expansion_depth) {
        num_alpha_pixels++;
      }
    }
  }
  LOG0("Number of alpha pixels: " << num_alpha_pixels);
  LOG0("Number of rows x cols: " << depth_map->rows << " " << depth_map->cols);

  // Now we create connections between the pixels.
  vector<pair<int,int>> pixel_connections;
  vector<EdgeCapacity<EdgeMetadata>> pixel_weights;
  for (int row_index = 0; row_index < depth_map->rows; row_index++) {
    for (int col_index = 0; col_index < depth_map->cols; col_index++) {
      int pixel_depth = depth_map->at<uchar>(row_index, col_index, 0);
      int pixel_id = row_col_to_node_id(row_index, col_index, depth_map->cols);

      if (col_index + pixel_depth >= image_right_.cols ||
          col_index + alpha_expansion_depth >= image_right_.cols) {
        continue;
      }

      // Connect to right neighbor.
      connect_pixel_to_neighbor(
          row_index, col_index, row_index, col_index+1,
          alpha_expansion_depth, depth_map,
          &pixels, &pixel_connections, &pixel_weights);
      
      // Connect to bottom neighbor.
      connect_pixel_to_neighbor(
          row_index, col_index, row_index+1, col_index,
          alpha_expansion_depth, depth_map,
          &pixels, &pixel_connections, &pixel_weights);
 
      if (col_index + alpha_expansion_depth < image_right_.cols) {
        // Connect pixel to source. The source node is our "alpha" node and
        // cutting the edge between it and the pixel corresponds to assigning
        // the pixel a depth of alpha.
        double pixel_source_cost = pointwise_error_(image_left_, image_right_,
          row_index, col_index, row_index, col_index + alpha_expansion_depth);
        pixel_connections.push_back(pair<int,int>(pixel_id, SOURCE_ID));
        pixel_weights.push_back(
            EdgeCapacity<EdgeMetadata>(
                pixel_source_cost, EdgeMetadata(EdgeType::PixelToSource)));
      }
      
      if (pixel_depth == alpha_expansion_depth) {
        // If the pixel's depth is alpha, we create an edge with infinite cost
        // between the pixel and the sink (the sink is our non-alpha node).
        // TODO(daddy): I'm pretty sure this isn't necessary.
        pixel_connections.push_back(pair<int,int>(pixel_id, SINK_ID));
        pixel_weights.push_back(
            EdgeCapacity<EdgeMetadata>(
                std::numeric_limits<double>::max(),
                EdgeMetadata(EdgeType::InfiniteAlphaLink)));
      } else {
        // Otherwise, the edge cost is the pointwise cost of giving pixel the
        // depth it previously had. Cutting the edge between the sink and the
        // pixel corresponds to leaving the depth of the pixel unchanged.
        double pixel_sink_cost = pointwise_error_(image_left_, image_right_,
            row_index, col_index, row_index, col_index + pixel_depth);
        pixel_connections.push_back(pair<int,int>(pixel_id, SINK_ID));
        pixel_weights.push_back(
            EdgeCapacity<EdgeMetadata>(
                pixel_sink_cost, EdgeMetadata(EdgeType::PixelToSink)));
      }
    }
  }
  LOG0("Number of nodes: " << pixels.size());

  return graph_utils::Graph<PixelMetadata, EdgeCapacity<EdgeMetadata>, false>
      (pixels, pixel_connections, pixel_weights);
}

double GraphCutEnergyMinimizer::compute_optimal_alpha_expansion(
    int alpha_expansion_depth, const unique_ptr<Mat>& depth_map) {
  
  // Construct the graph that. This will be a graph whose min-cut corresponds
  // to an alpha-expansion that minimzes the energy function.
  graph_utils::Graph<
      PixelMetadata, EdgeCapacity<EdgeMetadata>, false> expansion_graph =
          construct_graph_for_alpha(depth_map, alpha_expansion_depth);

  // Compute the min-cut of the constructed graph. The edges in this graph
  // will have their EdgeMetadata set to true iff they connect to pixels whose
  // depth estimates should change to alpha_expansion_depth.
  vector<min_st_cut::MinCutEdge<PixelMetadata, EdgeMetadata>> min_st_cut =
      min_st_cut::compute_min_st_cut<PixelMetadata, EdgeMetadata>(
          &expansion_graph); 

  int counts[EdgeType::NUM_EDGE_TYPES];
  for (auto i = 0U; i < min_st_cut.size(); i++) {
    counts[min_st_cut[i].edge.edge_data.edge_type]++;
  }

  LOG0("PixelToSource: " << counts[EdgeType::PixelToSource]);
  LOG0("PixelToSink: " << counts[EdgeType::PixelToSink]);
  LOG0("PixelToMid: " << counts[EdgeType::PixelToMid]);
  LOG0("PixelToPixel: " << counts[EdgeType::PixelToPixel]);
  LOG0("MidToSink: " << counts[EdgeType::MidToSink]);
  LOG0("InfiniteAlphaLink: " << counts[EdgeType::InfiniteAlphaLink]);

  return 0.0;
}

unique_ptr<Mat> GraphCutEnergyMinimizer::compute_depth_for_images() {
  // Use a faster algorithm to compute a rough estimate of the depth for
  // each pixel in order to seed our graph-cutting approach.
  compute_depth_fast::DepthComputerFast comp(image_left_, image_right_,
      pointwise_error_, occlusion_penalty_, max_disparity_);
  unique_ptr<Mat> depth_map(comp.compute_depth_for_images());

  compute_optimal_alpha_expansion(10, depth_map);

  return depth_map;
}

}
