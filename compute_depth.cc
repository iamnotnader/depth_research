#include "compute_depth.h"
#include "logging_utils.h"
#include <math.h>

using cv::Mat;
using std::function;
using std::make_pair;
using std::map;
using std::pair;
using std::unique_ptr;
using std::vector;
using std::min;

namespace compute_depth {

const Answer& dummy_answer = Answer(0, {});

const Answer& DepthComputer::compute_correspondence_for_line(
    int left_pixel, int img_start, int img_end, int row) {
  LOG3("compute_correspondence_for_line: left_pixel-> " << left_pixel
      << " img_start-> " << img_start << " img_end-> " << img_end << " row-> "
      << row);
  // Simple base case.
  if (left_pixel == img_end) {
    LOG3("compute_correspondence_for_line: Reached end of image.");
    return dummy_answer;
  }

  // Try to return the answer for this (left_pixel, img_start) pair from the
  // cache.
  auto&& params = std::make_pair(left_pixel, img_start);
  map<pair<int,int>,Answer>::iterator it =
      map_ptr->find(params);
  if (it != map_ptr->end()) {
    LOG3("compute_correspondence_for_line: Found cached answer");
    return it->second;
  }

  // Try to correspond left_pixel with all the pixels in the right image and
  // compute the cost of each placement recursively, keeping track of the best
  // placement we've seen.
  vector<int> rest_of_shit;
  int max_disparity = min(img_end, img_start+MAX_DISPARITY);
  int best_pos_to_try = -1;
  double best_cost = std::numeric_limits<double>::infinity();
  for (int pos_to_try = img_start; pos_to_try < max_disparity; pos_to_try++) {
    double pixel_cost = error_func_(image_left_, image_right_, row, left_pixel,
        row, pos_to_try);

    const Answer& rest_assuming_pos =
        compute_correspondence_for_line(
            left_pixel+1, pos_to_try, img_end, row);
    double total_pos_cost = pixel_cost + rest_assuming_pos.cost;
    if (total_pos_cost < best_cost) {
      best_cost = total_pos_cost;
      best_pos_to_try = pos_to_try;
    }
  }
  const Answer& rest_assuming_pos =
      compute_correspondence_for_line(
          left_pixel+1, best_pos_to_try, img_end, row);
  Answer best_answer(best_cost, {best_pos_to_try});
  for (int corr : rest_assuming_pos.correspondences) {
    assert(corr >= best_pos_to_try);
    best_answer.correspondences.push_back(corr);
  }
  LOG3("compute_correspondence_for_line: Cost-> " << best_answer.cost);

  // Once we've found the best placement, add it to the map and return.
  map_ptr->insert(make_pair(params, std::move(best_answer)));
  return map_ptr->find(params)->second;
};

unique_ptr<Mat> DepthComputer::compute_depth_for_images() {
  LOG2("compute_depth_for_images: called.");
  unique_ptr<Mat> depth_map(new Mat(image_left_.rows, image_left_.cols,
      CV_8UC1));
  for (int current_row = 0; current_row < image_left_.rows; current_row++) {
    LOG2("compute_depth_for_images: row: " << current_row);
    map_ptr->clear();
    const Answer& ans = compute_correspondence_for_line(0, 0, depth_map->cols,
        current_row);
    for (int current_col = 0; current_col < image_left_.cols; current_col++) {
      LOG2("compute_depth_for_images: col: " << current_col);
      int disparity = ans.correspondences[current_col] - current_col;
      LOG2("compute_depth_for_images: disparity: " << disparity);
      assert(disparity < MAX_DISPARITY);
      uchar val = (uchar)disparity;
      depth_map->at<uchar>(current_row, current_col, 0) = val;
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
    const cv::Vec3b& right_pixel = img2.at<cv::Vec3b>(row2, col2);
    double c1 = left_pixel.val[0] - right_pixel.val[0];
    double c2 = left_pixel.val[1] - right_pixel.val[1];
    double c3 = left_pixel.val[2] - right_pixel.val[2];
    return c1*c1 + c2*c2 + c3*c3;
  };
  Mat image_1 = cv::imread(argv[1], 1);
  Mat image_2 = cv::imread(argv[2], 1);
  compute_depth::DepthComputer comp(image_1, image_2, error_func);
  unique_ptr<Mat> depth_map = comp.compute_depth_for_images();

  // Display the two input images.
  namedWindow("Left Image", cv::WINDOW_AUTOSIZE);
  imshow("Left Image", image_1);
  namedWindow("Right Image", cv::WINDOW_AUTOSIZE);
  imshow("Left Image", image_2);

  // Display the computed depth map.
  namedWindow("Depth Map", cv::WINDOW_AUTOSIZE);
  imshow("Depth Map", *depth_map);

  return 0;
}
