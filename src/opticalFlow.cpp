#include <opencv2/video/video.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

#include <iostream>
#include <cstdio>

using namespace std;
using namespace cv;

void recorde_feature_points();
void get_accept_points();
void draw_lines(Mat &output);
void tracking(Mat &frame, Mat &output);
void getrandompoint(Mat &img,int iter);
bool addbackgroundPoints();
bool addNewPoints();
bool is_accept_TrackedPoint(int i);
void halfsize(Mat &img,Mat &output);

string window_name = "optical flow tracking";
Mat gray;
Mat gray_prev;
vector<Point2f> track_point_start;
vector<Point2f> frame_point_before;
vector<Point2f> frame_point_after;

vector <Point2f> features;
vector<float > norm_of_features;

int maxCount = 200;
double qLevel = 0.01;
double minDist = 10.0;
vector<uchar> status;
vector<float> err;

int main()
{
	Mat frame;
	Mat result;
    cout << "--start--" <<endl;

	// CvCapture* capture = cvCaptureFromCAM( -1 );
	// VideoCapture capture("/home/kim/slam/src/bike.avi");
	VideoCapture capture(0);
    cout << "video capture 0" << endl;

	if(capture.isOpened())
	{
        cout << "video is opened" << endl;
		while(true){
// 			frame = cvQueryFrame( capture );
			capture >> frame;
			
			halfsize(frame,frame);
			if(!frame.empty()){
				tracking(frame, result);
			}
			else{
				printf(" --(!) No captured frame -- Break!");
				break;
			}

			int c = waitKey(30);
			if( (char)c == 27 ){
				break;
			}
		}
	}
	else{
		cout << "cam not found" << endl;
	}
	return 0;
}

//scale image to half size
void halfsize(Mat &img,Mat &output){
	int proportion2=1;
	resize(img,output,Size(img.cols>>proportion2,img.rows>>proportion2),0.5,0.5,INTER_CUBIC);
}

void recorde_feature_points(){
		//feature points
		goodFeaturesToTrack(gray, features, maxCount, qLevel, minDist);
		for (vector<Point2f>::iterator iter= features.begin(); iter != features.end(); ++iter){
			frame_point_before.push_back(*iter);
			track_point_start.push_back(*iter);
		}
}

void get_accept_points(){
	
	int k = 0;
	int length=frame_point_after.size();
	for (int i=0; i<length; i++)
	{
		// criterion for judgement
		if (is_accept_TrackedPoint(i))
		{
			track_point_start[k] = track_point_start[i];
			frame_point_after[k] = frame_point_after[i];
			k++;
		}
	}

	frame_point_after.resize(k);
	track_point_start.resize(k);
}
void draw_points(Mat &output){
	int length=frame_point_before.size();
	for (int i=0; i<length; i++){
		circle(output, frame_point_before[i], 2, Scalar(50,200,0), -1);
	}
}

void draw_lines(Mat &output){
	int length=frame_point_after.size();
	for (int i=0; i<length; i++)
	{
		line(output, track_point_start[i], frame_point_after[i], Scalar(0, 0, 255));
		norm_of_features.push_back(norm(Mat(track_point_start[i]), Mat(frame_point_after[i]), NORM_L2)) ;
		circle(output, frame_point_after[i], 2, Scalar(255, 0, 0), -1);
	}
}

void tracking(Mat &frame, Mat &output)
{
	cvtColor(frame, gray, CV_BGR2GRAY);
	frame.copyTo(output);

	//ensure when init ,frame is not empty
	if (gray_prev.empty()){
		gray.copyTo(gray_prev);
	}
	
	if(addNewPoints()){
		recorde_feature_points();
	}

	calcOpticalFlowPyrLK(gray_prev, gray, frame_point_before, frame_point_after, status, err);

	get_accept_points();
	
	draw_points(output);
	draw_lines(output);
	
	swap(frame_point_before, frame_point_after);
	swap(gray_prev, gray);

	imshow(window_name, output);
}

bool addNewPoints()
{
	return frame_point_before.size() <= 10;
}

bool is_accept_TrackedPoint(int i)
{
	//status accept
	if(status[i]){
		//move distance > 2
		float dx=abs(frame_point_before[i].x - frame_point_after[i].x);
		float dy=abs(frame_point_before[i].y - frame_point_after[i].y);
		if((dx+dy) > 2){
			return true;
		}
		
	}
	return false;
}

