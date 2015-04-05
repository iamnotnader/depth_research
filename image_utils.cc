#include "image_utils.h"
#include "logging_utils.h"
#include <iostream>

using cv::Mat;
using status_defs::Status;
using std::function;
using std::endl;
using std::cout;
using std::abs;

namespace image_utils {

namespace internal {

template<typename T>
Status ConvolveOnePixel(const Mat& image_in, Mat* image_out,
    image_utils::CONVOLUTION_FUNC filter, int filter_width,
    int filter_height, int image_row, int image_col) {
  LOG3("Convolving single pixel.");
  LOG3("row: " << image_row << " col: " << image_col);

  if (!image_in.isContinuous()) {
    LOG_ERROR("ConvolveOnePixel can only process continuous images.");
    return Status::ERROR;
  }

  LOG3("Interpreted image as CV_32SC1");
  int conv_index = image_row*image_in.cols+image_col;
  const T*data = (const T*)image_in.data;
  LOG3("input value found: " << (int)data[conv_index]);

  T* output_data = (T*)image_out->data;

  // Loop over channels and do each one separately, then each dimension of
  // the 2d filter.
  for (int c = 0; c < image_in.channels(); c++) {
    double sum = 0;
    for (int i = -filter_height / 2; i <= filter_height / 2; i++) {
      for (int j = -filter_width / 2; j <= filter_width / 2; j++) {
        // Recall that for proper convolution, the filter must be reversed
        // on both axes relative to the image. For this reason, we start
        // at the bottom-right of the image and the top-left of the filter.
        // As we iterate over this loop, we get the desired reversal.
        int current_conv_row = abs(image_row - i);
        int current_conv_col = abs(image_col - j);

        // Sorry the math is a little opaque here...
        // If we're outside the image bounds, we reflect the index.
        // For example, if we're convolving on a point at row n-1, and we're
        // currently on a point at row n+1 (two points to the RIGHT of our
        // convolution point), then we will actually use the point
        // at n-3 (two points to the LEFT of our convolution point). 
        if (current_conv_row >= image_in.rows) {
          current_conv_row = 2*image_in.rows - current_conv_row - 2;
        }
        if (current_conv_col >= image_in.cols) {
          current_conv_col = 2*image_in.cols - current_conv_col - 2;
        }

        int other_point_index = current_conv_row * image_in.cols *
          image_in.channels() + current_conv_col*image_in.channels() + c;

        // Again, some weird math here.
        // We're iterating over the image starting at
        // -filter_[height|width]/2 and ending at +filter_[height|width]/2.
        // Iterating in this way makes the math fo compute the conv_[row|col]
        // simpler BUT it means we have to do this quick conversion before we
        // get our filter value.
        //
        // Note that we start at the top-left of the filter and move to the
        // bottom-right as we iterate, as the comment states above.
        int filter_row = i + filter_height/2;
        int filter_col = j + filter_width/2;
        double filter_value = filter(filter_row, filter_col, filter_width,
            filter_height);

        sum += data[other_point_index] * filter_value;
      
        LOG3("Multiplying image index: " << other_point_index);
        LOG3("FILTER: " << filter_row << " " << filter_col
            << " IMAGE: " << current_conv_row << " " << current_conv_col
            << " FILTER_VAL: " << filter_value << " IMAGE_VAL: "
            << (int)data[other_point_index]);
      }
    }
    int current_conv_index = image_row*image_in.cols*image_in.channels() +
        image_col*image_in.channels() + c;
    LOG2("Current convolution index: " << current_conv_index);
    output_data[current_conv_index] = sum;
  }

  LOG3("Done convolving single pixel.");
  return Status::SUCCESS;
}

status_defs::Status GetImageSlice(const Mat& image_in, Mat* slice,
    int row_index, int col_index, int window_width, int window_height) {
  *slice = Mat(window_height, window_width, image_in.type());
  for (int i = 0; i < window_height; i++) {
    for (int j = 0; j < window_width; j++) {
      LOG2("" << i << " " << j);
      int input_row = i + row_index;
      int input_col = j + col_index;

      // If we are out of bounds of the image, use its reflection.
      if (input_row >= image_in.rows) {
        input_row = 2*image_in.rows  - input_row - 2;
      }
      if (input_col >= image_in.cols) {
        input_col = 2*image_in.cols - input_col - 2;
      }

      int es = image_in.elemSize();
      int image_data_index = input_row*image_in.cols*es +
                             input_col*es;
      uchar* image_in_bytes = image_in.data + image_data_index;;
      uchar* slice_bytes = slice->data;
      for (size_t c = 0; c < image_in.elemSize(); c++) {
        LOG3("image index: " << image_data_index + c);
        slice_bytes[i*window_width*es + j*es+c] = image_in_bytes[c];
      }
      LOG2("image indices: " << input_row << " " << input_col << " " << image_data_index);
    }
  }
  return status_defs::Status::SUCCESS;
}

} // namespace internal

status_defs::Status ConvolveImageWithFilter(const cv::Mat& image_in,
    cv::Mat* image_out, CONVOLUTION_FUNC filter, int filter_width,
    int filter_height) {
  using namespace internal;
  assert(image_in.rows == image_out->rows);
  assert(image_in.cols == image_out->cols);
  assert(image_in.depth() == image_out->depth());

  for (int i = 0; i < image_in.rows; i++) {
    for (int j = 0; j < image_in.cols; j++) {
      LOG2("Row: " << i << " Col: " << j);
      if (image_in.type() == CV_32SC1 || image_in.type() == CV_32SC2) {
        LOG3("Convolving int!");
        ConvolveOnePixel<int>(image_in, image_out, filter, filter_width,
            filter_height, i, j);
      } else if (image_in.type() == CV_8UC1 || image_in.type() == CV_8UC2 ||
          image_in.type() == CV_8UC3) {
        LOG3("Convolving uchar!");
        ConvolveOnePixel<uchar>(image_in, image_out, filter, filter_width,
            filter_height, i, j);
      } else {
        LOG0("Not convolving; unknown depth.");
      }
    }
  }
  return Status::SUCCESS;
}

bool MatsAreEqual(const Mat& mat1, const Mat& mat2) {
  if (mat1.depth() != mat2.depth() ||
      mat1.channels() != mat2.channels() ||
      mat1.rows != mat2.rows ||
      mat1.cols != mat2.cols) {
    LOG2("Mats have different dimensions.");

    LOG3("mat1.rows: " << mat1.rows << endl
        << "mat2.rows: " << mat2.rows << endl
        << "mat1.cols: " << mat1.cols << endl
        << "mat2.cols: " << mat2.cols << endl
        << "mat1.elemSize(): " << mat1.elemSize()
        << "mat2.elemSize(): " << mat2.elemSize() << endl
        << "mat1.channels(): " << mat1.channels() << endl
        << "mat2.channels(): " << mat2.channels());
    return false;
  }
  int char_length = mat1.rows * mat1.cols * mat1.elemSize();
  const uchar* mat1_data = (const uchar*)mat1.data;
  const uchar* mat2_data = (const uchar*)mat2.data;
  for (int i = 0; i < char_length; i++) {
    if (mat1_data[i] != mat2_data[i]) {
      LOG2("Mats differ at index " << i);
      return false;
    }
  }
  LOG2("Mats are equal.");
  return true;
}

status_defs::Status ApplyWindowFunctionToImage(const cv::Mat& image_in,
    cv::Mat* image_out, WINDOW_FUNC func, int window_width, int window_height) {
  using namespace internal;
  for (int i = 0; i < image_in.rows; i++) {
    for (int j = 0; j < image_in.cols; j++) {
      Mat slice;
      GetImageSlice(image_in, &slice, i, j, window_width, window_height);
      // get the slice
      // pass it to your func
      // get the output of your func and put it in your output
    }
  }
  return status_defs::Status::SUCCESS;
}

} // namespace image_utils

