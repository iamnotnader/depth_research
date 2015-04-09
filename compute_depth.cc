#include "compute_depth.h"
#include "logging_utils.h"
#include <algorithm>
#include <math.h>

using cv::Mat;
using std::function;
using std::make_pair;
using std::map;
using std::pair;
using std::unique_ptr;
using std::vector;
using std::min;
using std::max;
using std::reverse;

namespace compute_depth {

void DepthComputer::initialize_cost(int row) {
  for (auto i = 0; i < right_cols; i++) {
    set_cost(i, 0, i*occlusion_penalty);
    set_previous_move(i, 0, PreviousMove::OccludeRight);
  }
  for (auto i = 0; i < left_cols; i++) {
    set_cost(0, i, i*occlusion_penalty);
    set_previous_move(0, i, PreviousMove::OccludeLeft);
  }
  for (int i = 1; i < right_cols; i++) {
    for (int j = 1; j < left_cols; j++) {
      double occlude_left = get_cost(i, j-1) + occlusion_penalty;
      double occlude_right = get_cost(i-1, j) + occlusion_penalty;
      double correspond_pixels = get_cost(i-1, j-1) +
          error_func_(image_left_, image_right_, row, j-1, row, i-1);
      double min_path_cost =
          min(occlude_left, min(occlude_right, correspond_pixels));
      set_cost(i, j, min_path_cost);
      if (min_path_cost == occlude_left) {
        set_previous_move(i, j, PreviousMove::OccludeLeft);
      } else if (min_path_cost == occlude_right) {
        set_previous_move(i, j, PreviousMove::OccludeRight);
      } else {
        set_previous_move(i, j, PreviousMove::Match);
      }
    }
  }
}

vector<int> DepthComputer::compute_correspondence_for_line(int row) {
  initialize_cost(row);
  int right_pixel = image_right_.cols;
  int left_pixel = image_left_.cols;
  vector<int> ret;
  while (left_pixel != 0) {
    switch(get_previous_move(right_pixel, left_pixel)) {
      case PreviousMove::Match: {
        ret.push_back(right_pixel-1);
        right_pixel--;
        left_pixel--;
        break;
      } case PreviousMove::OccludeLeft: {
        ret.push_back(-1);
        left_pixel--;
        break;
      } case PreviousMove::OccludeRight: {
        right_pixel--;
        break;
      } default: {
        assert(false);
      }
    }
  }
  assert(ret.size() == image_left_.cols);
  reverse(ret.begin(), ret.end());
  return std::move(ret);
};

unique_ptr<Mat> DepthComputer::compute_depth_for_images() {
  LOG2("compute_depth_for_images: called.");
  unique_ptr<Mat> depth_map(new Mat(image_left_.rows, image_left_.cols,
      CV_8UC1));
  for (int current_row = 0; current_row < image_left_.rows; current_row++) {
    LOG2("compute_depth_for_images: row: " << current_row);
    vector<int> correspondences =
        compute_correspondence_for_line(current_row);
    for (int current_col = 0; current_col < image_left_.cols; current_col++) {
      LOG2("compute_depth_for_images: col: " << current_col);
      if (correspondences[current_col] == -1) {
        LOG3("Disparity: " << -1);
        depth_map->at<unsigned int>(current_row, current_col, 0) = 0;
      } else {
        LOG3("Current col: " << current_col << " Correspondence: " <<
             correspondences[current_col]);
        int disparity =
            (correspondences[current_col] - current_col)*10*256/left_cols;
        depth_map->at<uchar>(current_row, current_col, 0) = disparity;
        LOG3("Disparity: " << disparity << " UCHAR: "  <<
             (int)depth_map->at<uchar>(current_row, current_col, 0));
      }
    }
  }
  return std::move(depth_map);
}

} // namespace compute_depth

#include "opencv2/highgui/highgui.hpp"
using cv::imshow;
int main(int argc, char** argv) {
  LOG0("In main.");
  if (argc < 3) {
    std::cout << "usage: register_images <image_1.jpg> <image_2.jpg>"
              << std::endl;
    return -1;
  }
  compute_depth::ErrorFunc error_func = [](
      const cv::Mat& img1, const cv::Mat& img2, int row1, int col1, int row2,
      int col2) {
    const cv::Vec3b& left_pixel = img1.at<cv::Vec3b>(row1, col1);
    const cv::Vec3b& img_start = img2.at<cv::Vec3b>(row2, col2);
    double c1 = left_pixel.val[0] - img_start.val[0];
    double c2 = left_pixel.val[1] - img_start.val[1];
    double c3 = left_pixel.val[2] - img_start.val[2];
    return c1*c1 + c2*c2 + c3*c3;
  };
  Mat image_1 = cv::imread(argv[1], 1);
  Mat image_2 = cv::imread(argv[2], 1);
  LOG0("Error func same: " << error_func(image_1, image_2, 0, 0, 0, 0));
  LOG0("Error func diff: " << error_func(image_1, image_2, 75, 70, 0, 0));
  compute_depth::DepthComputer comp(image_1, image_2, error_func, 500);
  unique_ptr<Mat> depth_map(comp.compute_depth_for_images());

  // Display the two input images.
  namedWindow("Left Image", cv::WINDOW_AUTOSIZE);
  imshow("Left Image", image_1);
  namedWindow("Right Image", cv::WINDOW_AUTOSIZE);
  imshow("Right Image", image_2);

  // Display the computed depth map.
  namedWindow("Depth Map", cv::WINDOW_AUTOSIZE);
  imshow("Depth Map", *depth_map);

  cv::waitKey();
  return 0;
}
