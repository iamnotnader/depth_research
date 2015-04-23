#include "logging_utils.h"
#include "opencv2/highgui/highgui.hpp"
#include "compute_depth_fast.h"
#include "image_utils.h"

using std::unique_ptr;
using cv::Mat;
using image_utils::MatsAreEqual;

bool TestSingleRowPointwise() {
  uchar left[12] = {200,0,0,100,0,0,0,0,0,0,0,0};
  uchar right[12] = {0,0,0,0,0,0,200,0,0,100,0,0};
  cv::Mat m1(1, 4, CV_8UC3, &left);
  cv::Mat m2(1, 4, CV_8UC3, &right);
  compute_depth_fast::DepthComputerFast comp(m1,m2,
      compute_depth_fast::pointwise_error, 100, 2);
  unique_ptr<Mat> depth_map(comp.compute_depth_for_images());
  uchar expected_arr[4] = {2, 2, 0, 0};
  cv::Mat expected_mat({1, 4, CV_8UC1, &expected_arr});
  if (!MatsAreEqual(expected_mat, *depth_map)) {
    cred("Failed: {2, 2, 0, 0} pointwise");
    LOG0("Expected: " << expected_mat << std::endl << "But got: " <<
        *depth_map);
    return false;
  } else {
    cgreen("PASSED: {2, 2, 0, 0} pointwise") << std::endl;
  }
  return true;
}

bool TestSingleRowCensus() {
  uchar left[12] = {200,0,0,100,0,0,0,0,0,0,0,0};
  uchar right[12] = {0,0,0,0,0,0,200,0,0,100,0,0};
  cv::Mat m1(1, 4, CV_8UC3, &left);
  cv::Mat m2(1, 4, CV_8UC3, &right);
  compute_depth_fast::DepthComputerFast comp(m1,m2,
      compute_depth_fast::census_error, .1, 2);
  unique_ptr<Mat> depth_map(comp.compute_depth_for_images());
  uchar expected_arr[4] = {2, 2, 0, 0};
  cv::Mat expected_mat({1, 4, CV_8UC1, &expected_arr});
  if (!MatsAreEqual(expected_mat, *depth_map)) {
    cred("Failed: {2, 2, 0, 0} census");
    LOG0("Expected: " << expected_mat << std::endl << "But got: " <<
        *depth_map);
    return false;
  } else {
    cgreen("PASSED: {2, 2, 0, 0} census") << std::endl;
  }
  return true;
}

bool Test3x3Pointwise() {
  uchar left_img[27] = {0,0,0, 200,0,0, 200,0,0,
                        0,0,0, 100,0,0, 100,0,0,
                        100,0,0, 100,0,0, 0,0,0};
  uchar right_img[27] = {0,0,0, 0,0,0, 200,0,0,
                         0,0,0, 0,0,0, 100,0,0,
                         0,0,0, 100,0,0, 100,0,0};
  cv::Mat m1(3, 3, CV_8UC3, &left_img);
  cv::Mat m2(3, 3, CV_8UC3, &right_img);
  compute_depth_fast::DepthComputerFast comp(m1,m2,
      compute_depth_fast::pointwise_error, 100, 2);
  unique_ptr<Mat> depth_map_census(comp.compute_depth_for_images());
  uchar expected_arr[9] = {0,1,0,
                            0,1,0,
                            1,1,0};
  cv::Mat expected_mat({3, 3, CV_8UC1, &expected_arr});
  if (!MatsAreEqual(expected_mat, *depth_map_census)) {
    cred("Failed: 3x3 pointwise");
    LOG0("Expected: " << expected_mat << std::endl << "But got: " <<
        *depth_map_census);
  } else {
    cgreen("PASSED: 3x3 pointwise") << std::endl;
  }
  return false;
}

bool Test3x3CensusSingleCell() {
  uchar left_img[27] = {0,0,0, 200,0,0, 200,0,0,
                        0,0,0, 100,0,0, 100,0,0,
                        100,0,0, 100,0,0, 0,0,0};
  uchar right_img[27] = {0,0,0, 0,0,0, 200,0,0,
                         0,0,0, 0,0,0, 100,0,0,
                         0,0,0, 100,0,0, 100,0,0};
  cv::Mat m1(3, 3, CV_8UC3, &left_img);
  cv::Mat m2(3, 3, CV_8UC3, &right_img);
  double ret = compute_depth_fast::census_error(
    m1, m2, 1, 1, 1, 1);
  if (ret == 4.0/9.0) {
    cgreen("PASSED: census single cell") << std::endl;
    return true;
  } else {
    cred("Failed: census single cell");
    LOG0("Expected: " << (4.0/9.0) << std::endl << "But got: " <<
        ret);
  }
  return false;
}

bool Test3x3CensusSingleCellAllSame() {
  uchar left_img[27] = {0,0,0, 200,0,0, 200,0,0,
                        0,0,0, 100,0,0, 100,0,0,
                        100,0,0, 100,0,0, 0,0,0};
  uchar right_img[27] = {0,0,0, 200,0,0, 200,0,0,
                        0,0,0, 100,0,0, 100,0,0,
                        100,0,0, 100,0,0, 0,0,0};
  cv::Mat m1(3, 3, CV_8UC3, &left_img);
  cv::Mat m2(3, 3, CV_8UC3, &right_img);
  double ret = compute_depth_fast::census_error(
    m1, m2, 1, 1, 1, 1);
  if (ret == 0.0) {
    cgreen("PASSED: census single cell") << std::endl;
    return true;
  } else {
    cred("Failed: census single cell");
    LOG0("Expected: " << (0.0) << std::endl << "But got: " <<
        ret);
  }
  return false;
}

bool Test3x3CensusBig() {
  uchar left_img[5*5*3];
  uchar right_img[5*5*3];

  for (int i = 0; i < 5*5*3; i++) {
    left_img[i] = i;
    right_img[i] = 5*5*3-i;
  }

  cv::Mat m1(5, 5, CV_8UC3, &left_img);
  cv::Mat m2(5, 5, CV_8UC3, &right_img);
  double ret = compute_depth_fast::census_error(
    m1, m2, 2, 2, 2, 2);
  if (ret == 24.0/25.0) {
    cgreen("PASSED: census big cell") << std::endl;
    return true;
  } else {
    cred("Failed: census big cell");
    LOG0("Expected: " << (24.0/25.0) << std::endl << "But got: " <<
        ret);
  }
  return false;
}

int main() {
  LOG0("Running tests...");

  TestSingleRowPointwise();
  TestSingleRowCensus();
  Test3x3Pointwise();
  Test3x3CensusSingleCell();
  Test3x3CensusSingleCellAllSame();
  Test3x3CensusBig();

  return 0;
}
