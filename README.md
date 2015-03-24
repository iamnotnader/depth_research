Compile the tests with: 
clang++ image_utils.cc image_utils_test.cc -lopencv_core -lopencv_highgui \
    -lopencv_imgproc -std=c++11 -DLOG_LEVEL_0

Run the tests with:
./a.out
