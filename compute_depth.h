#ifndef BARRIER_COMPUTE_DEPTH
#define BARRIER_COMPUTE_DEPTH

#include "status_defs.h"
#include "opencv2/imgproc/imgproc.hpp"
#include <map>

#define MAX_DISPARITY 256

namespace compute_depth {

typedef std::function<double(const cv::Mat&,const cv::Mat&, int,int,int,int)>
    ErrorFunc;

class Answer {
 public:
  double cost;
  std::vector<int> correspondences;

  Answer(double cost, std::vector<int>&& correspondences)
      : cost(cost), correspondences(std::move(correspondences)) { };
};

// Given two images, computes a disparity value for each pixel (porportional
// to depth, but need focal length and baseline to convert to meters).
// Assumes the two images have been registered so that each of their
// epipolar lines corresponds to the same row in both images. Practically,
// this is equivalent to saying both images are taken by one camera that has
// been shifted to the right, without changing any of the other coordinates.
// Also assumes both images are the same width.
class DepthComputer {
 public:
  DepthComputer(const cv::Mat& image_left, const cv::Mat& image_right,
      const ErrorFunc& error_func) 
      : image_left_(image_left), image_right_(image_right),
        error_func_(error_func),
        map_ptr(new std::map<std::pair<int,int>,Answer>()) {
    assert(image_left_.rows == image_right_.rows);
    assert(image_left_.cols == image_right_.cols);
    assert(image_left_.type() == CV_8UC3);
    assert(image_right_.type() == CV_8UC3);
  }

  // Uses dynamic programming to compute depth for a line. Keeps track of a
  // mapping from pairs of (pixel, start) -> depth so we can short-circuit
  // once we've assigned a pixel to a depth before.
  const Answer& compute_correspondence_for_line(
      int left_pixel, int img_start, int img_end, int row);

  std::unique_ptr<cv::Mat> compute_depth_for_images();

  const cv::Mat image_left_;
  const cv::Mat image_right_; 
  const ErrorFunc error_func_;
  const std::unique_ptr<std::map<std::pair<int,int>,Answer>> map_ptr;
};

} // namespace compute_depth

#endif
