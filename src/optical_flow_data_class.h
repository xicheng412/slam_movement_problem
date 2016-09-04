#ifndef OPTICAL_FLOW_DATA_CLASS_H
#define OPTICAL_FLOW_DATA_CLASS_H

#include <opencv2/core/core.hpp>

using namespace std;
using namespace cv;

class frame_data{
public:
	frame_data(int record_frame_number);
	~frame_data();
	void record(const Mat& frame);
	Mat get(int n);
private:
	int record_frame_number;
	Mat *frame_pointer;
	int this_pointer;
	int this_pointer_change(int i);
};
#endif
