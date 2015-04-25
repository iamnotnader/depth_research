#ifndef BARRIER_COMPUTE_DEPTH_SMOOTH
#define BARRIER_COMPUTE_DEPTH_SMOOTH

#include <utility>
#include "max_flow.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

namespace compute_depth_smooth {

typedef std::function<double(
    const cv::Mat& left_image,
    const cv::Mat& right_image,
    int left_row, int left_col, int right_row, int right_col)>
  PointwiseError;

typedef std::function<double(int depth_1, int depth_2)> SmoothingError;

class GraphCutEnergyMinimizer {
 public:
  GraphCutEnergyMinimizer(
      const cv::Mat& image_left,
      const cv::Mat& image_right,
      const PointwiseError& pointwise_error,
      const SmoothingError& smoothing_error,
      const double occlusion_penalty,
      const int max_disparity)
      : image_left_(image_left), image_right_(image_right),
        pointwise_error_(pointwise_error),
        smoothing_error_(smoothing_error),
        occlusion_penalty_(occlusion_penalty),
        max_disparity_(max_disparity) {
    LOG0(image_left_.cols);
    LOG0(max_disparity);
    assert(image_left_.cols > max_disparity_);
    assert(image_right_.cols > max_disparity_);
    assert(image_left_.rows == image_right_.rows);
    assert(image_left_.type() == CV_8UC3);
    assert(image_right_.type() == CV_8UC3);
  }

  std::unique_ptr<cv::Mat> compute_depth_for_images();

 private:
  struct PixelMetadata {
    PixelMetadata(int row, int col, int depth)
        : row(row), col(col), depth(depth) { }

    int row;
    int col;
    int depth;
  };

  enum EdgeType { PixelToSource, PixelToSink, PixelToMid, PixelToPixel,
                  MidToSink, InfiniteAlphaLink, NUM_EDGE_TYPES };
  struct EdgeMetadata {
    EdgeMetadata(EdgeType edge_type)
        : edge_type(edge_type) { }

    EdgeType edge_type;
  };

  void normalize_depth_map(cv::Mat* depth_map);

  double compute_optimal_alpha_expansion(
      int alpha_depth, const std::unique_ptr<cv::Mat>& depth_map);

  graph_utils::Graph<PixelMetadata, max_flow::EdgeCapacity<EdgeMetadata>, false>
      construct_graph_for_alpha(
          const std::unique_ptr<cv::Mat>& depth_map,
          int alpha_expansion_depth);

  void connect_pixel_to_neighbor(
      int from_row, int from_col, int to_row, int to_col,
      int alpha_expansion_depth,
      const std::unique_ptr<cv::Mat>& depth_map,
      vector<PixelMetadata>* pixels,
      vector<pair<int,int>>* pixel_connections,
      vector<max_flow::EdgeCapacity<EdgeMetadata>>* pixel_weights);

  const cv::Mat& image_left_;
  const cv::Mat& image_right_; 
  const PointwiseError pointwise_error_;
  const SmoothingError smoothing_error_;
  const double occlusion_penalty_;
  const int max_disparity_;
};

static double piecewise_smooth(int depth_1, int depth_2) {
  // TODO(daddy): Make it so the cost is 0 if the pixels are the same OR have
  // very different pointwise errors.
  if (depth_1 == depth_2) {
    return 0.0;
  }
  return 1000;
}

}

#endif
