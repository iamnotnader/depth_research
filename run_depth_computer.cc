#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include "compute_depth_fast.h"
#include "logging_utils.h"
#include "image_utils.h"

using cv::imshow;
using cv::Mat;
using std::unique_ptr;

int main(int argc, char** argv) {
  LOG0("Running...");

  if (argc < 3) {
    std::cout << "usage: register_images <image_1.jpg> <image_2.jpg>"
              << std::endl;
    return -1;
  }

  Mat image_1 = cv::imread(argv[1], 1);
  Mat image_2 = cv::imread(argv[2], 1);
  compute_depth_fast::DepthComputerFast comp2(image_1, image_2,
      compute_depth_fast::pointwise_error,
      1000, 100);
  unique_ptr<Mat> depth_map(comp2.compute_depth_for_images());
  cv::Mat img_depth_map = image_utils::convert_to_uchar_image(*depth_map);
  image_utils::normalize_depth_map(&img_depth_map, 100);

  // Display the two input images.
  namedWindow("Left Image", cv::WINDOW_AUTOSIZE);
  imshow("Left Image", image_1);
  namedWindow("Right Image", cv::WINDOW_AUTOSIZE);
  imshow("Right Image", image_2);

  std::cout << "If your depth map is all black, try reversing the "
            << "order of the input images." << std::endl;

  // Display the computed depth map.
  namedWindow("Depth Map", cv::WINDOW_AUTOSIZE);
  imshow("Depth Map", img_depth_map);

  cv::waitKey();
  return 0;
}
