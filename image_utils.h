#ifndef IMAGE_UTILS
#define IMAGE_UTILS

#include "opencv2/imgproc/imgproc.hpp"
#include "status_defs.h"

namespace image_utils {

typedef std::function<double(int row_index, int col_index, int filter_width,
    int filter_height)> CONVOLUTION_FUNC;

status_defs::Status ConvolveImageWithFilter(const cv::Mat& image_in,
    cv::Mat* image_out, CONVOLUTION_FUNC filter, int filter_width,
    int filter_height);

bool MatsAreEqual(const cv::Mat& mat1, const cv::Mat& mat2);

} // namespace image_utils

#endif
