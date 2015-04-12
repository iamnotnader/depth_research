#include "compute_depth_fast.h"
#include "logging_utils.h"
#include <algorithm>
#include <math.h>
#include "image_utils.h"

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
using image_utils::MatsAreEqual;

namespace compute_depth_fast {

void DepthComputerFast::initialize_cost(int row) {
  // The base costs need to be set up diagonally until the Nth row where N
  // is the maximum disparity allowed.
  //    
  // Example of initial setup for max_disparity = 2:
  //
  //           right pixels  
  //                |
  //                V
  //                  0    1    2    3    4    5
  // left pixels ->   -    -    -    -    -    -
  //             0  |                0   100  200
  //             1  |           100
  //             2  |      200
  //             3  | 300 
  //             4  | 400
  //             5  | 500
  //
  for (auto i = 0; i < right_cols; i++) {
    auto col_index = 0;
    if (i <= max_disparity_+1) {
      col_index = max_disparity_+1-i;
    }
    set_cost(i, col_index, i*occlusion_penalty);
    set_previous_move(i, col_index, PreviousMove::OccludeRight);
  }
  for (auto i = max_disparity_-1; i < left_cols; i++) {
    set_cost(0, i, (i-max_disparity_-1)*occlusion_penalty);
    set_previous_move(0, i, PreviousMove::OccludeLeft);
  }
  #ifdef LOG_LEVEL_3
  std::cout << "Initial cost setup: " << std::endl;
  for (int i = 0; i < right_cols; i++) {
    std::cout << i << ": ";
    for (int j = 0; j < left_cols; j++) {
      std::cout << get_cost(i, j) << "," << get_previous_move(i,j) << "\t";
    }
    std::cout << std::endl;
  }
  #endif

  for (auto i = 1; i < right_cols; i++) {
    int start_index = max(max_disparity_-i+1, 0) + 1;
    for (int j = start_index; j < left_cols; j++) {
      double occlude_left = std::numeric_limits<double>::max();
      if (j-1 >= 0) {
        occlude_left = get_cost(i, j-1) + occlusion_penalty;
      }
      double occlude_right = std::numeric_limits<double>::max();
      if (j+1 < left_cols && i-1 >= 0) {
        occlude_right = get_cost(i-1, j+1) + occlusion_penalty;
      }
      int left_pixel_index = j - (max_disparity_+2-i);
      int right_pixel_index = i-1;
      double correspond_pixels = std::numeric_limits<double>::max();
      if (left_pixel_index >= 0 && right_pixel_index >= 0 &&
          left_pixel_index < image_left_.cols && right_pixel_index < image_right_.cols) {
        correspond_pixels = get_cost(i-1, j) +
            error_func_(image_left_, image_right_, row, left_pixel_index, row,
                        right_pixel_index);
      }

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

  #ifdef LOG_LEVEL_3
  std::cout << "Final cost matrix: " << std::endl;
  for (int i = 0; i < right_cols; i++) {
    std::cout << i << ": ";
    for (int j = 0; j < left_cols; j++) {
      std::cout << get_cost(i, j) << "," << get_previous_move(i,j) << "\t";
    }
    std::cout << std::endl;
  }
  #endif
}

vector<int> DepthComputerFast::compute_correspondence_for_line(int row) {
  initialize_cost(row);
  int right_pixel = right_cols-1;
  int left_pixel = max_disparity_+1;
  vector<int> ret;
  while (ret.size() < (unsigned long)image_left_.cols) {
    switch(get_previous_move(right_pixel, left_pixel)) {
      case PreviousMove::Match: {
        ret.push_back(right_pixel-1);
        right_pixel--;
        break;
      } case PreviousMove::OccludeLeft: {
        ret.push_back(-1);
        left_pixel--;
        break;
      } case PreviousMove::OccludeRight: {
        right_pixel--;
        left_pixel++;
        break;
      } default: {
        assert(false);
      }
    }
  }
  assert(ret.size() == image_left_.cols);
  reverse(ret.begin(), ret.end());
  #ifdef LOG_LEVEL_3
  for (auto x : ret) {
    std::cout << x << " ";
  }
  #endif
  return std::move(ret);
};

unique_ptr<Mat> DepthComputerFast::compute_depth_for_images() {
  LOG2("Called.");
  unique_ptr<Mat> depth_map(new Mat(image_left_.rows, image_left_.cols,
      CV_8UC1));
  for (int current_row = 0; current_row < image_left_.rows; current_row++) {
    LOG2(current_row*100/image_left_.rows << "%");
    vector<int> correspondences =
        compute_correspondence_for_line(current_row);
    for (int current_col = 0; current_col < image_left_.cols; current_col++) {
      if (correspondences[current_col] == -1) {
        depth_map->at<unsigned int>(current_row, current_col, 0) = 0;
      } else {
        // TODO(daddy): Should make sure disparity maps to [0, 256). Luckily
        // it's rare to find things more than 256 pixels away..
        int disparity =
            min(correspondences[current_col] - current_col, 255);
        depth_map->at<uchar>(current_row, current_col, 0) = disparity;
      }
    }
  }
  return std::move(depth_map);
}

} // namespace compute_depth_fast

#include "opencv2/highgui/highgui.hpp"
using cv::imshow;
int main(int argc, char** argv) {
  LOG0("Running...");

  compute_depth_fast::ErrorFunc error_func = [](
      const cv::Mat& img1, const cv::Mat& img2, int row1, int col1, int row2,
      int col2) {
    assert(col1 >= 0 && row1 >= 0);
    assert(col1 < img1.cols && row1 < img1.rows);
    assert(col2 >= 0 && row2 >= 0);
    assert(col2 < img2.cols && row2 < img2.rows);
    const cv::Vec3b& left_pixel = img1.at<cv::Vec3b>(row1, col1);
    const cv::Vec3b& img_start = img2.at<cv::Vec3b>(row2, col2);
    double c1 = left_pixel.val[0] - img_start.val[0];
    double c2 = left_pixel.val[1] - img_start.val[1];
    double c3 = left_pixel.val[2] - img_start.val[2];
    return c1*c1 + c2*c2 + c3*c3;
  };

  if (argc == 1) {
    LOG0("No arguments found-- running tests.");
    LOG0("To actually use run: ./a.out <image_1.jpg> <image_2.jpg>");

    // TODO(daddy): Move this test out into a separate test file.
    uchar left[12] = {200,0,0,100,0,0,0,0,0,0,0,0};
    uchar right[12] = {0,0,0,0,0,0,200,0,0,100,0,0};
    cv::Mat m1(1, 4, CV_8UC3, &left);
    cv::Mat m2(1, 4, CV_8UC3, &right);
    compute_depth_fast::DepthComputerFast comp(m1,m2, error_func, 100, 2);
    unique_ptr<Mat> depth_map(comp.compute_depth_for_images());
    uchar expected_arr[4] = {2, 2, 0, 0};
    cv::Mat expected_mat({1, 4, CV_8UC1, &expected_arr});
    if (!MatsAreEqual(expected_mat, *depth_map)) {
      cred("Failed: {2, 2, 0, 0}");
      LOG0("Expected: " << expected_mat << std::endl << "But got: " <<
          *depth_map);
    } else {
      cgreen("PASSED: {2, 2, 0, 0}") << std::endl;
    }
    // End test.
    return 0;
  } else if (argc < 3) {
    std::cout << "usage: register_images <image_1.jpg> <image_2.jpg>"
              << std::endl;
    return -1;
  }

  Mat image_1 = cv::imread(argv[1], 1);
  Mat image_2 = cv::imread(argv[2], 1);
  compute_depth_fast::DepthComputerFast comp2(image_1, image_2, error_func,
      1000, 50);
  unique_ptr<Mat> depth_map2(comp2.compute_depth_for_images());

  // Display the two input images.
  namedWindow("Left Image", cv::WINDOW_AUTOSIZE);
  imshow("Left Image", image_1);
  namedWindow("Right Image", cv::WINDOW_AUTOSIZE);
  imshow("Right Image", image_2);

  // Display the computed depth map.
  namedWindow("Depth Map", cv::WINDOW_AUTOSIZE);
  imshow("Depth Map", *depth_map2);

  cv::waitKey();
  return 0;
}
