#include "compute_depth_fast.h"
#include "logging_utils.h"
#include <algorithm>
#include <math.h>
#include "image_utils.h"
#include "opencv2/highgui/highgui.hpp"

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
  // TODO(daddy): Maybe factor this out into smaller functions. Too lazy
  // right now.
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
          left_pixel_index < image_left_.cols && right_pixel_index <
          image_right_.cols) {
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
  vector<int32_t> ret;
  while (ret.size() < (unsigned long)image_left_.cols) {
    switch(get_previous_move(right_pixel, left_pixel)) {
      case PreviousMove::Match: {
        ret.push_back(right_pixel-1);
        right_pixel--;
        break;
      } case PreviousMove::OccludeLeft: {
        ret.push_back(OCCLUSION_DEPTH);
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
  #ifdef LOG_LEVEL_3
  std::cout << "RET: " << std::endl;
  for (auto x : ret) {
    std::cout << x << " ";
  }
  #endif
  reverse(ret.begin(), ret.end());
  return std::move(ret);
};

unique_ptr<Mat> DepthComputerFast::compute_depth_for_images() {
  LOG2("Called.");
  unique_ptr<Mat> depth_map(new Mat(image_left_.rows, image_left_.cols,
      CV_32SC1));
  for (int current_row = 0; current_row < image_left_.rows; current_row++) {
    LOG2(current_row*100/image_left_.rows << "%");
    vector<int> correspondences =
        compute_correspondence_for_line(current_row);
    for (int current_col = 0; current_col < image_left_.cols; current_col++) {
      // Try reversing the pictures to prevent this from happening.
      int disparity = OCCLUSION_DEPTH;
      if (correspondences[current_col] != OCCLUSION_DEPTH) {
        disparity = correspondences[current_col] - current_col; 
      }
      assert(disparity <= max_disparity_);
      depth_map->at<int32_t>(current_row, current_col, 0) = disparity;
    }
  }
  return std::move(depth_map);
}

double get_intensity(const cv::Vec3b& pixel) {
  double c1 = pixel.val[0];
  double c2 = pixel.val[1];
  double c3 = pixel.val[2];
  return c1+c2+c3;
}

double pointwise_error(
    const cv::Mat& img1, const cv::Mat& img2, int row1, int col1, int row2,
    int col2) {
  assert(col1 >= 0 && row1 >= 0);
  assert(col1 < img1.cols && row1 < img1.rows);
  assert(col2 >= 0 && row2 >= 0);
  assert(col2 < img2.cols && row2 < img2.rows);
  const cv::Vec3b& left_pixel = img1.at<cv::Vec3b>(row1, col1);
  const cv::Vec3b& right_pixel = img2.at<cv::Vec3b>(row2, col2);
  double c1 = left_pixel.val[0] - right_pixel.val[0];
  double c2 = left_pixel.val[1] - right_pixel.val[1];
  double c3 = left_pixel.val[2] - right_pixel.val[2];
  return c1*c1 + c2*c2 + c3*c3;
}

double census_error(
    const cv::Mat& img1, const cv::Mat& img2, int row1, int col1, int row2,
    int col2) {
  assert(col1 >= 0 && row1 >= 0);
  assert(col1 < img1.cols && row1 < img1.rows);
  assert(col2 >= 0 && row2 >= 0);
  assert(col2 < img2.cols && row2 < img2.rows);

  LOG3("row1: " << row1 << " col1: " << col1 << " row2: " << row2 << " col2: "
       << col2 << std::endl);
  double census_sum = 0.0;
  int num_pixels_used = 0;
  const cv::Vec3b& left_pixel_mid = img1.at<cv::Vec3b>(row1, col1);
  const cv::Vec3b& right_pixel_mid = img2.at<cv::Vec3b>(row2, col2);
  for (int i = 0; i < 5; i++) {
    for (int j = 0; j < 5; j++) {
      int left_image_col = col1 - 2 + i;
      int left_image_row = row1 - 2 + j;
      int right_image_col = col2 - 2 + i;
      int right_image_row = row2 - 2 + j;
      if (left_image_col < 0 || left_image_col >= img1.cols ||
          right_image_col < 0 || right_image_col >= img2.cols ||
          left_image_row < 0 || left_image_row >= img1.rows ||
          right_image_row < 0 || right_image_row >= img2.rows) {
        continue;
      }

      const cv::Vec3b& left_pixel =
          img1.at<cv::Vec3b>(left_image_row, left_image_col);
      const cv::Vec3b& right_pixel =
          img2.at<cv::Vec3b>(right_image_row, right_image_col);

      double left_mid_intensity = get_intensity(left_pixel_mid);
      double left_intensity = get_intensity(left_pixel);
      double right_intensity = get_intensity(right_pixel);
      double right_mid_intensity = get_intensity(right_pixel_mid);

      bool left_pixel_larger = left_intensity > left_mid_intensity;
      bool right_pixel_larger = right_intensity > right_mid_intensity;
      if (left_pixel_larger != right_pixel_larger) {
        census_sum += 1;
      }

      LOG3("lr: " << left_image_row << " lc: " << left_image_col
          << " rr: " << right_image_row << " rc: " << right_image_col
          << " left_bool: " << left_pixel_larger << " right_bool: "
          << right_pixel_larger << " agree?: "
          << (left_pixel_larger == right_pixel_larger) <<  std::endl);
      num_pixels_used++;
    }
  }
  return census_sum / num_pixels_used;
}

} // namespace compute_depth_fast

