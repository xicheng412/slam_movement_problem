#include <opencv2/video/video.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

#include <iostream>
#include <cstdio>

using namespace std;
using namespace cv;

void tracking(Mat &frame, Mat &output);
void getrandompoint(Mat &img,int iter);
bool addbackgroundPoints();
bool addNewPoints();
bool acceptTrackedPoint(int i);

string window_name = "optical flow tracking";
Mat gray;	
Mat gray_prev;	
vector<Point2f> points[2];	
vector<Point2f> initial;	
vector <Point2f> features;
vector<Point2f> random_points;
vector<Point2f> background_points;
vector<float > norm_of_features;
int maxCount = 500;
double qLevel = 0.01;	
double minDist = 10.0;	
vector<uchar> status;	
vector<float> err;
int main()
{
	Mat frame;
	Mat result;

// 	CvCapture* capture = cvCaptureFromCAM( -1 );	
	// VideoCapture capture("/home/kim/slam/src/bike.avi");
	VideoCapture capture(0);

	if(capture.isOpened())	
	{
		while(true)
		{
// 			frame = cvQueryFrame( capture );	
			capture >> frame;

			if(!frame.empty())
			{ 
				tracking(frame, result);
			}
			else
			{ 
				printf(" --(!) No captured frame -- Break!");
				break;
			}

			int c = waitKey(100);
			if( (char)c == 27 )
			{
				break; 
			} 
		}
	}
	return 0;
}

void tracking(Mat &frame, Mat &output)
{
	cvtColor(frame, gray, CV_BGR2GRAY);
	frame.copyTo(output);

	// squre of background points
	// int intr = 50;
	// if(random_points.size() < intr*intr)
	// {
	// 	// random_points.push_back(Point2f(output.size().width,output.size().height));
	// 	for(size_t i = 0 ; i < intr ; i++)
	// 	{
	// 		for(size_t j = 0; j < intr ; j++)
	// 		{
	// 			random_points.push_back(Point2f((intr-i)*(output.size().width/intr),(intr-j)*(output.size().height/intr)));
	// 			// cout<<"random_points["<<i*intr+j<<"]:"<<random_points[i*intr+j]<<endl;
	// 		}
	// 	}
	// }

	// calcOpticalFlowPyrLK(gray_prev,gray,random_points,background_points,status, err);


	if (addNewPoints())
	{
		//feature points
		goodFeaturesToTrack(gray, features, maxCount, qLevel, minDist);
		// points[0].insert(points[0].end(), random_points.begin(), random_points.end());
		points[0].insert(points[0].end(), features.begin(), features.end());
		// cout<<"points[0]"<<points[0]<<endl;
		// initial.insert(initial.end(), random_points.begin(), random_points.end());
		initial.insert(initial.end(), features.begin(), features.end());
	}

	if (gray_prev.empty())
	{
		gray.copyTo(gray_prev);
	}

	calcOpticalFlowPyrLK(gray_prev, gray, points[0], points[1], status, err);
// delete the wrong points
	// cout<<"test_1"<<endl;
	//random points
	int k = 0;
	for (size_t i=0; i<points[1].size(); i++)
	{
		if (acceptTrackedPoint(i))  // criterion for judgement
		{
			initial[k] = initial[i];
			points[1][k++] = points[1][i];
		}

	}

	points[1].resize(k);
	initial.resize(k);
	for (size_t i=0; i<points[1].size(); i++)
	{
		line(output, initial[i], points[1][i], Scalar(0, 0, 255));
		// cout<<"initial[~]"<<initial[i]<<endl;
		// cout<<"points[1][~]"<<points[1][i]<<endl;
		// cout<<"length of the line:"<< norm(Mat(initial[i]), Mat(points[1][i]), NORM_L2)<<endl;
		norm_of_features.push_back(norm(Mat(initial[i]), Mat(points[1][i]), NORM_L2)) ;
		// cout<<"norm_of_features:"<<norm_of_features<<endl;
		circle(output, points[1][i], 3, Scalar(255, 0, 0), -1);
	}

	// Mat gray_prev_resize
	// Mat gray_resize
	// Mat flow
	// resize(gray_prev,gray_prev_resize,size(64,48),0,0,CV_INTER_LINEAR);
	// resize(gray,gray_resize,size(64,48),0,0,CV_INTER_LINEAR);


	// calcOpticalFlowFarneback(gray_prev_resize, gray_resize, flow, 0.5, 3, 15, 3, 5, 1.2, 0); 
	// for(size_t i = 0; i<flow.size();i++)
	// {

	// 	line(output, random_points[i], background_points[i], Scalar(0, 0, 255));
	// 	circle(output, flow[i], 3, Scalar(255, 255, 0), -1);
	// }

	swap(points[1], points[0]);
	swap(gray_prev, gray);

	imshow(window_name, output);
}

bool addNewPoints()
{
	return points[0].size() <= 10;
}
// bool addbackgroundPoints()
// {
// 	return ;
// }

bool acceptTrackedPoint(int i)
{
	return status[i] && ((abs(points[0][i].x - points[1][i].x) + abs(points[0][i].y - points[1][i].y)) > 2);
}

