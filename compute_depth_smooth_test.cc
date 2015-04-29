#include "compute_depth_smooth.h"
#include "compute_depth_fast.h"
#include "logging_utils.h"
#include "image_utils.h"

int main() {
  LOG0("Running compute_depth_smooth tests...");
  logging_utils::print_stack_traces();
  
  cv::Mat image_1 = cv::imread("/Users/daddy/Development/tmp/play/sample_left.png", 1);
  cv::Mat image_2 = cv::imread("/Users/daddy/Development/tmp/play/sample_right.png", 1);
  int MAX_DISPARITY = 50;
  compute_depth_smooth::GraphCutEnergyMinimizer comp(image_1, image_2,
      compute_depth_fast::pointwise_error, compute_depth_smooth::piecewise_smooth,
      1000, MAX_DISPARITY);
  std::unique_ptr<cv::Mat> depth_map(comp.compute_depth_for_images());
  cv::Mat img_depth_map_graph = image_utils::convert_to_uchar_image(*depth_map);
  image_utils::normalize_depth_map(&img_depth_map_graph, MAX_DISPARITY);

  // Display the computed depth map.
  namedWindow("Depth Map", cv::WINDOW_AUTOSIZE);
  imshow("Depth Map", img_depth_map_graph);

  cv::waitKey();

  return 0;
}
