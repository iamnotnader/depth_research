#ifndef IMAGE_UTILS
#define IMAGE_UTILS

#include "opencv2/imgproc/imgproc.hpp"
#include "status_defs.h"

namespace image_utils {

// This is a namespace used for unit-testing functions that should not be
// visible as part of the public interface.
namespace internal {

status_defs::Status GetImageSlice(const cv::Mat& image_in, cv::Mat* slice,
    int row_index, int col_index, int window_width, int window_height);

} // namespace internal

// Computes the output of a convolution kernel for a single pixel.
typedef std::function<double(int row_index, int col_index, int filter_width,
    int filter_height)> CONVOLUTION_FUNC;

// Convolutes image_in with the given filter and stores the result in
// image_out.
status_defs::Status ConvolveImageWithFilter(const cv::Mat& image_in,
    cv::Mat* image_out, CONVOLUTION_FUNC filter, int filter_width,
    int filter_height);

bool MatsAreEqual(const cv::Mat& mat1, const cv::Mat& mat2);

// Computes a function over a window of pixel values. Outputs one value for
// each channel of the input matrix.
typedef std::function<std::vector<double>(const cv::Mat& window)> WINDOW_FUNC;

// Slides a window function over an image and puts the resulting output into
// an output matrix. This is effectively a more general form of convolution
// that allows you to do anything you want with the pixels in a given window.
status_defs::Status ApplyWindowFunctionToImage(const cv::Mat& image_in,
    cv::Mat* image_out, WINDOW_FUNC func, int window_width, int window_height);

} // namespace image_utils

#endif
