#include "compute_depth_smooth.h"
#include "compute_depth_fast.h"
#include "logging_utils.h"

int main() {
  LOG0("Running compute_depth_smooth tests...");
  logging_utils::print_stack_traces();
  
  cv::Mat image_1 = cv::imread("/Users/daddy/Development/tmp/play/sample_left.png", 1);
  cv::Mat image_2 = cv::imread("/Users/daddy/Development/tmp/play/sample_right.png", 1);
  compute_depth_smooth::GraphCutEnergyMinimizer comp(image_1, image_2,
      compute_depth_fast::pointwise_error, compute_depth_smooth::piecewise_smooth,
      1000, 50);
  std::unique_ptr<cv::Mat> depth_map(comp.compute_depth_for_images());

  // Display the computed depth map.
  namedWindow("Depth Map", cv::WINDOW_AUTOSIZE);
  imshow("Depth Map", *depth_map);

  cv::waitKey();

  return 0;
}
