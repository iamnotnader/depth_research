// Compile with -DLOG_LEVEL_0 and run the main to see test results.

#include "image_utils.h"
#include "logging_utils.h"
#include <iostream>

using cv::Mat;
using status_defs::Status;
using std::function;
using std::endl;
using std::cout;
using std::abs;
using image_utils::ConvolveImageWithFilter;
using image_utils::MatsAreEqual;

namespace {
double IdentityFilter(int row, int col, int width, int height) {
  LOG3("Filter row: " << row << " filter col: " << col);
  if (row == height/2 && col == width/2) {
    return 1.0;
  }
  return 0.0;
}

// This is a dumb function to test the convolution stuff.
double PrimeTestFilter(int row, int col, int width, int height) {
  assert(width == 3 && height == 3);
  static int filt[9] = {2, 3, 5, 7, 11, 113, 17, 19, 23};
  return filt[row*width + col];
}
} // namespace

int main() {
  LOG0("Running tests."); 

  // Tests for MatsAreEqual
  int arr1[18] = {1,2,3,4,5,6,7,8,9,10,11,9999999,13,14,15,16,17,18};
  int arr2[18] = {1,2,3,4,5,6,7,8,9,10,11,9999999,13,14,15,16,17,-1};
  Mat test_mat_1(3, 3, CV_32SC2, &arr1);
  Mat test_mat_2(3, 3, CV_32SC2, &arr2);
  EXPECT_TRUE(!MatsAreEqual(test_mat_1, test_mat_2),
      "PASSED: DifferentMatsAreNOTEqualTwoChannelInts",
      "FAILED: DifferentMatsAreNOTEqualTwoChannelInts");

  Mat test_mat_3(9, 2, CV_32SC1, &arr1);
  Mat test_mat_4(9, 2, CV_32SC1, &arr2);
  EXPECT_TRUE(!MatsAreEqual(test_mat_3, test_mat_4),
      "PASSED: DifferentMatsAreNOTEqualSingleChannelInts",
      "FAILED: DifferentMatsAreNOTEqualSingleChannelInts");

  Mat test_mat_5(18, 4, CV_8UC1, &arr1);
  Mat test_mat_6(18, 4, CV_8UC1, &arr2);
  EXPECT_TRUE(!MatsAreEqual(test_mat_5, test_mat_6),
      "PASSED: DifferentMatsAreNOTEqualSingleChannelUchars",
      "FAILED: DifferentMatsAreNOTEqualSingleChannelUchars");

  Mat test_mat_7(9, 4, CV_8UC2, &arr1);
  Mat test_mat_8(9, 4, CV_8UC2, &arr2);
  EXPECT_TRUE(!MatsAreEqual(test_mat_7, test_mat_8),
      "PASSED: DifferentMatsAreNOTEqualTwoChannelUchars",
      "FAILED: DifferentMatsAreNOTEqualTwoChannelUchars");
  
  arr2[17] = 18;
  EXPECT_TRUE(MatsAreEqual(test_mat_1, test_mat_2),
      "PASSED: SameMatsAreEqualTwoChannelInts",
      "FAILES: SameMatsAreEqualTwoChannelInts");

  EXPECT_TRUE(MatsAreEqual(test_mat_3, test_mat_4),
      "PASSED: SameMatsAreEqualSingleChannelInts",
      "FAILED: SameMatsAreEqualSingleChannelInts");
  
  EXPECT_TRUE(MatsAreEqual(test_mat_5, test_mat_6),
      "PASSED: SameMatsAreEqualSingleChannelUchars",
      "FAILES: SameMatsAreEqualSingleChannelUchars");

  EXPECT_TRUE(MatsAreEqual(test_mat_7, test_mat_8),
      "PASSED: SameMatsAreEqualSingleChannelUChars",
      "FAILED: SameMatsAreEqualSingleChannelUChars");

  // Tests for ConvolveImageWithFilter 
  int conv_arr_1[9] = {1,2,3,4,5,9999999,7,8,9};
  Mat conv_mat_1(3, 3, CV_32SC1, &conv_arr_1);
  Mat output_mat_1(3, 3, CV_32SC1);
  Mat expected_mat_1(3, 3, CV_32SC1, &conv_arr_1);
 
  ConvolveImageWithFilter(conv_mat_1, &output_mat_1, IdentityFilter, 3, 3);
  EXPECT_TRUE(
      MatsAreEqual(output_mat_1, expected_mat_1),
      "PASSED: IdentityFilterSingleChannelInts",
      "FAILED: IdentityFilterSingleChannelInts"
  );
  LOG2("Created the following input_matrix: " << endl << conv_mat_1);
  LOG2("Got the following output_matrix: " << endl << output_mat_1);
  LOG2("Expected the following output_matrix: " << endl << expected_mat_1);

  Mat expected_mat_2 = Mat::ones(3, 3, CV_32SC1) * 200;
  ConvolveImageWithFilter(Mat::ones(3, 3, CV_32SC1), &output_mat_1,
      PrimeTestFilter, 3, 3);
  EXPECT_TRUE(
      MatsAreEqual(output_mat_1, expected_mat_2),
      "PASSED: PrimeTestFilteSingleChannelInts1",
      "FAILED: PrimeTestFilteSingleChannelInts1"
  );
  LOG2("Got the following output_matrix: " << endl << output_mat_1);
  LOG2("Expected the following output_matrix: " << endl << expected_mat_2);

  int arr_prime_output[9] = {58,135,47,139,36,123,47,29,58};
  Mat expected_mat_3(3, 3, CV_32SC1, &arr_prime_output);
  ConvolveImageWithFilter(Mat::eye(3, 3, CV_32SC1), &output_mat_1,
      PrimeTestFilter, 3, 3);
  EXPECT_TRUE(
      MatsAreEqual(output_mat_1, expected_mat_3),
      "PASSED: PrimeTestFilteSingleChannelInts2",
      "FAILED: PrimeTestFilteSingleChannelInts2"
  );
  LOG2("Created the following input_matrix: " << endl <<
       Mat::eye(3, 3, CV_32SC1));
  LOG2("Got the following output_matrix: " << endl << output_mat_1);
  LOG2("Expected the following output_matrix: " << endl << expected_mat_3);

  int arr_prime_output_2[18] = {58,0,135,0,47,0,139,0,36,0,123,0,47,0,29,0,58,0};
  Mat expected_mat_4(3, 3, CV_32SC2, &arr_prime_output_2);
  Mat output_mat_2(3, 3, CV_32SC2);

  ConvolveImageWithFilter(Mat::eye(3, 3, CV_32SC2), &output_mat_2,
      PrimeTestFilter, 3, 3);
  EXPECT_TRUE(
      MatsAreEqual(output_mat_2, expected_mat_4),
      "PASSED: PrimeTestFilteSingleChannelIntsTwoChannel",
      "FAILED: PrimeTestFilteSingleChannelIntsTwoChannel"
  );
  LOG2("Created the following input_matrix: " << endl <<
       Mat::eye(3, 3, CV_32SC2));
  LOG2("Got the following output_matrix: " << endl << output_mat_2);
  LOG2("Expected the following output_matrix: " << endl << expected_mat_4);

  uchar arr_prime_output_3[18] = {58,0,135,0,47,0,139,0,36,0,123,0,47,0,29,0,58,0};
  Mat expected_mat_5(3, 3, CV_8UC2, &arr_prime_output_3);
  Mat output_mat_3(3, 3, CV_8UC2);

  ConvolveImageWithFilter(Mat::eye(3, 3, CV_8UC2), &output_mat_3,
      PrimeTestFilter, 3, 3);
  EXPECT_TRUE(
      MatsAreEqual(output_mat_3, expected_mat_5),
      "PASSED: PrimeTestFilteTwoChannelUChar",
      "FAILED: PrimeTestFilteTwoChannelUChar"
  );
  LOG2("Created the following input_matrix: " << endl <<
       Mat::eye(3, 3, CV_8UC2));
  LOG2("Got the following output_matrix: " << endl << output_mat_3);
  LOG2("Expected the following output_matrix: " << endl << expected_mat_5);

  int slice_arr_1[16] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16};
  Mat slice_mat_1(4,4, CV_32SC1, &slice_arr_1);
  Mat output_slice_mat_1;
  int expected_slice_arr_1[4] = {1, 2, 5, 6};
  Mat expected_slice_mat_1(2, 2, CV_32SC1, &expected_slice_arr_1);
  image_utils::internal::GetImageSlice(slice_mat_1, &output_slice_mat_1,
      0, 0, 2, 2);
  EXPECT_TRUE(
      MatsAreEqual(output_slice_mat_1, expected_slice_mat_1),
      "PASSED: GetImageSlice2x2At0,0",
      "FAILED: GetImageSlice2x2At0,0"
  );
  LOG2("Created the following input_matrix: " << endl << slice_mat_1);
  LOG2("Slicing: 0 0 2 2");
  LOG2("Expected the following output_matrix: " << endl
      << expected_slice_mat_1);
  LOG2("Got the following output_matrix: " << endl << output_slice_mat_1);

  int slice_arr_2[16] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16};
  Mat slice_mat_2(4,4, CV_32SC1, &slice_arr_2);
  Mat output_slice_mat_2;
  int expected_slice_arr_2[2] = {6, 7};
  Mat expected_slice_mat_2(1, 2, CV_32SC1, &expected_slice_arr_2);
  image_utils::internal::GetImageSlice(slice_mat_2, &output_slice_mat_2,
      1, 1, 2, 1);
  EXPECT_TRUE(
      MatsAreEqual(output_slice_mat_2, expected_slice_mat_2),
      "PASSED: GetImageSlice2x1At1,1",
      "FAILED: GetImageSlice2x1At1,1"
  );
  LOG2("Created the following input_matrix: " << endl << slice_mat_2);
  LOG2("Slicing: 1 1 2 1");
  LOG2("Expected the following output_matrix: " << endl
      << expected_slice_mat_2);
  LOG2("Got the following output_matrix: " << endl << output_slice_mat_2);

  int slice_arr_3[16] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16};
  Mat slice_mat_3(4,4, CV_32SC1, &slice_arr_3);
  Mat output_slice_mat_3;
  int expected_slice_arr_3[6] = {16, 15, 14, 12, 11, 10};
  Mat expected_slice_mat_3(2, 3, CV_32SC1, &expected_slice_arr_3);
  image_utils::internal::GetImageSlice(slice_mat_3, &output_slice_mat_3,
      3, 3, 3, 2);
  EXPECT_TRUE(
      MatsAreEqual(output_slice_mat_3, expected_slice_mat_3),
      "PASSED: GetImageSlice2x3At3,3",
      "FAILED: GetImageSlice2x3At3,3"
  );
  LOG2("Created the following input_matrix: " << endl << slice_mat_3);
  LOG2("Slicing: 3 3 3 2");
  LOG2("Expected the following output_matrix: " << endl
      << expected_slice_mat_3);
  LOG2("Got the following output_matrix: " << endl << output_slice_mat_3);

  int slice_arr_4[16] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16};
  Mat slice_mat_4(4, 2, CV_32SC2, &slice_arr_4);
  Mat output_slice_mat_4;
  int expected_slice_arr_4[6] = {7, 8, 5, 6};
  Mat expected_slice_mat_4(1, 2, CV_32SC2, &expected_slice_arr_4);
  image_utils::internal::GetImageSlice(slice_mat_4, &output_slice_mat_4,
      1, 1, 2, 1);
  EXPECT_TRUE(
      MatsAreEqual(output_slice_mat_4, expected_slice_mat_4),
      "PASSED: GetImageSlice1x2At1,1TwoChannel",
      "FAILED: GetImageSlice1x2At1,1TwoChannel"
  );
  LOG2("Created the following input_matrix: " << endl << slice_mat_4);
  LOG2("Slicing: 1 1 2 1");
  LOG2("Expected the following output_matrix: " << endl
      << expected_slice_mat_4);
  LOG2("Got the following output_matrix: " << endl << output_slice_mat_4);

  return 0;
}
