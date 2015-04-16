// Crappy program that displays two images.

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "iostream"

// Because we're lazy..
using namespace std;
using namespace cv;

char window_name1[] = "image 1";
char window_name2[] = "image 2";
char window_name3[] = "image 2 -> image 1";

void convolve_one_pixel(Mat* img, const Mat& filter, int img_row, int img_col) {
  Vec3b pixel = img->at<Vec3b>(img_row, img_col);
  
  int filter_width = filter.cols;
  int filter_height = filter.rows;

  int base_img_row = img_row - filter_height/2;
  int base_img_col = img_col - filter_width/2;

  Vec3f sum = 0;
  for (int i = 0; i < filter.rows; i++) {
    for (int j = 0; j < filter.cols; j++) {
      int row_index = base_img_row + i;
      int col_index = base_img_col + j;
      if (row_index < 0) {
        row_index *= -1;
      }
      if (col_index < 0) {
        col_index *= -1;
      }
      Vec3b pixel = img->at<Vec3b>(row_index, col_index);
      double filter_val = ((double*)filter.ptr(0))[i*filter.rows+j];
      sum[0] += pixel[0] * filter_val;
      sum[1] += pixel[1] * filter_val;
      sum[2] += pixel[2] * filter_val;
    }
  }

  img->at<Vec3b>(img_row, img_col) = sum;
}

void convolve(Mat* img, const Mat& filter) {
  for (int i = 0; i < img->rows; i++) {
    for (int j = 0; j < img->cols; j++) {
      convolve_one_pixel(img, filter, i, j);
    }
  }
}

int main(int argc, char** argv)
{
  if (argc < 3) {
    cout << "usage: register_images <image_1.jpg> <image_2.jpg>" << endl;
    return -1;
  }

  /// Load the source image
  Mat image_1 = imread(argv[1], 1);
  Mat image_2 = imread(argv[2], 1);
  assert(image_1.depth() == CV_8U);

  cout << image_1.rows << " " << image_1.cols << endl;

  // Convolve the image
  Mat filter(5, 5, CV_64F);
  int filter_size = filter.rows*filter.cols;
  for (int i = 0; i < filter_size; i++) {
    double* thing = (double*)filter.ptr(0);
    thing[i] = 1.0/filter_size;
  }
  convolve(&image_1, filter);

  // Display the two input images.
  namedWindow(window_name1, WINDOW_AUTOSIZE);
  imshow(window_name1, image_1);

  // Display the two input images.
  namedWindow(window_name2, WINDOW_AUTOSIZE);
  imshow(window_name2, image_2);

  waitKey();
  return 0;
}
