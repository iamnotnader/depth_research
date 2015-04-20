#ifndef BARRIER_COMPUTE_DEPTH_FAST
#define BARRIER_COMPUTE_DEPTH_FAST

#include "status_defs.h"
#include "opencv2/imgproc/imgproc.hpp"
#include <map>

namespace compute_depth_fast {

typedef std::function<double(const cv::Mat&,const cv::Mat&, int,int,int,int)>
    ErrorFunc;

enum PreviousMove { Match, OccludeLeft, OccludeRight };

// See DepthComputer. Assumes the disparity can be no larger than specified
// value max_disparity. This allows us to perform a similar dynamic programming
// as DepthComputer, but using only a max_disparity x right_cols matrix
// rather than one of size left_cols x right_cols.
class DepthComputerFast {
 public:
  DepthComputerFast(const cv::Mat& image_left, const cv::Mat& image_right,
      const ErrorFunc& error_func, const double occlusion_penalty,
      const int max_disparity) 
      : image_left_(image_left), image_right_(image_right),
        error_func_(error_func),
        occlusion_penalty(occlusion_penalty),
        max_disparity_(max_disparity),
        left_cols(2*max_disparity_+2),
        right_cols(image_right_.cols+1),
        path_costs(new double[left_cols*right_cols]),
        previous_moves(new PreviousMove[left_cols*right_cols]) {
    assert(image_left_.cols > max_disparity_);
    assert(image_right_.cols > max_disparity_);
    assert(image_left_.rows == image_right_.rows);
    assert(image_left_.type() == CV_8UC3);
    assert(image_right_.type() == CV_8UC3);
  }

  // For each pixel in the left image, gives the pixel in the right image that
  // it corresponds to for a given row.
  std::vector<int> compute_correspondence_for_line(int row);

  std::unique_ptr<cv::Mat> compute_depth_for_images();

  const cv::Mat& image_left_;
  const cv::Mat& image_right_; 
  const ErrorFunc error_func_;
  const double occlusion_penalty;

 private:
  const int max_disparity_;
  void initialize_cost(int row);

  // Compiler should inline these everywhere..
  double get_cost(int row, int col) {
    assert(col >= 0 && row >= 0);
    assert(col < left_cols && row < right_cols);
    return path_costs.get()[left_cols*row + col];
  }
  double set_cost(int row, int col, double val) {
    assert(col >= 0 && row >= 0);
    assert(col < left_cols && row < right_cols);
    return path_costs.get()[left_cols*row + col] = val;
  }
  PreviousMove get_previous_move(int row, int col) {
    assert(col >= 0 && row >= 0);
    assert(col < left_cols && row < right_cols);
    return previous_moves.get()[left_cols*row + col];
  }
  PreviousMove set_previous_move(int row, int col, PreviousMove val) {
    assert(col >= 0 && row >= 0);
    assert(col < left_cols && row < right_cols);
    return previous_moves.get()[left_cols*row + col] = val;
  }

  // We make left_cols and right_cols const ints in order to make sure the
  // compiler inlines our calls to get_cost.
  const int left_cols;
  const int right_cols;

  // We make this cost a single allocation for efficiency. We don't want
  // each row coming from a random part of the heap. It will be a
  // right_cols x left_cols sized matrix.
  std::unique_ptr<double> path_costs;
  std::unique_ptr<PreviousMove> previous_moves;
};

double pointwise_error(
    const cv::Mat& img1, const cv::Mat& img2, int row1, int col1, int row2,
    int col2);

// TODO(daddy): The images generated with this error function suck-- but I feel
// like they shouldn't. Figure out if it's a bug in the implementation or if
// it's OK/expected for the images to suck.
double census_error(
    const cv::Mat& img1, const cv::Mat& img2, int row1, int col1, int row2,
    int col2);

} // namespace compute_depth_fast

double get_intensity(const cv::Vec3b& pixel);

#endif
