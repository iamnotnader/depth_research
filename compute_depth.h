#ifndef BARRIER_COMPUTE_DEPTH
#define BARRIER_COMPUTE_DEPTH

#include "status_defs.h"
#include "opencv2/imgproc/imgproc.hpp"
#include <map>

namespace compute_depth {

typedef std::function<double(const cv::Mat&,const cv::Mat&, int,int,int,int)>
    ErrorFunc;

enum PreviousMove { Match, OccludeLeft, OccludeRight };

// Given two images, computes a disparity value for each pixel (porportional
// to depth, but need focal length and baseline to convert to meters).
// Assumes the two images have been registered so that each of their
// epipolar lines corresponds to the same row in both images. Practically,
// this is equivalent to saying both images are taken by one camera that has
// been shifted to the right, without changing any of the other coordinates.
//
// Computes depth by creating matrix of size left_cols x right_cols, where
// left_cols and right_cols are the number of columns in each of the respective
// images. This can be made more efficient if we assume the disparity can
// be no larger than some value D.
class DepthComputer {
 public:
  DepthComputer(const cv::Mat& image_left, const cv::Mat& image_right,
      const ErrorFunc& error_func, const double occlusion_penalty) 
      : image_left_(image_left), image_right_(image_right),
        error_func_(error_func),
        occlusion_penalty(occlusion_penalty),
        left_cols(image_left_.cols+1),
        right_cols(image_right_.cols+1),
        path_costs(new double[left_cols*right_cols]),
        previous_moves(new PreviousMove[left_cols*right_cols]) {
    assert(image_left_.rows == image_right_.rows);
    assert(image_left_.cols == image_right_.cols);
    assert(image_left_.type() == CV_8UC3);
    assert(image_right_.type() == CV_8UC3);
  }

  // For each pixel in the left image, gives the pixel in the right image that
  // it corresponds to for a given row.
  std::vector<int> compute_correspondence_for_line(int row);

  std::unique_ptr<cv::Mat> compute_depth_for_images();

  const cv::Mat image_left_;
  const cv::Mat image_right_; 
  const ErrorFunc error_func_;
  const double occlusion_penalty;

 private:
  void initialize_cost(int row);

  // Compiler should inline these everywhere..
  double get_cost(int row, int col) {
    assert(col < left_cols && row < right_cols);
    return path_costs.get()[left_cols*row + col];
  }
  double set_cost(int row, int col, double val) {
    assert(col < left_cols && row < right_cols);
    return path_costs.get()[left_cols*row + col] = val;
  }
  PreviousMove get_previous_move(int row, int col) {
    assert(col < left_cols && row < right_cols);
    return previous_moves.get()[left_cols*row + col];
  }
  PreviousMove set_previous_move(int row, int col, PreviousMove val) {
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

} // namespace compute_depth

#endif
