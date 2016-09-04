#include <opencv2/video/video.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

#include <iostream>
#include <cstdio>

#include "array_2d_template.h"

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
void clear_used_frame_data();
void calc_movement();

string window_name = "optical flow tracking";

Mat gray;
Mat gray_prev;

vector<Point2f> track_point_start;
vector<Point2f> track_point_end;
vector<Point2f> track_point_direction;

vector<Point2f> frame_point_before;
vector<Point2f> frame_point_after;

vector <Point2f> features;
vector<float> norm_of_features;
Point2f frame_direction;

int maxCount = 50;
double qLevel = 0.01;
double minDist = 10.0;
vector<uchar> status;
vector<float> err;
float **two_points_distance;

void init_main(){
	//init 2 points distance 2d array
	two_points_distance=new_Array2D<float>(maxCount,maxCount);
}

void exit_main(){
	delete_Array2D(two_points_distance,maxCount,maxCount);
}

int main()
{
	Mat frame;
	Mat result;
	init_main();
	
    cout << "--start--" <<endl;

	// CvCapture* capture = cvCaptureFromCAM( -1 );
	// VideoCapture capture("/home/kim/slam/src/bike.avi");
	VideoCapture capture(0);
    cout << "video capture 0" << endl;
	
	if(capture.isOpened())
	{
        cout << "video is opened" << endl;
		while(capture.isOpened()){
			//frame = cvQueryFrame( capture );
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
	
	exit_main();
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
		frame_point_before.clear();
		for (vector<Point2f>::iterator iter= features.begin(); iter != features.end(); ++iter){
			frame_point_before.push_back(*iter);
		}
}

void get_accept_points(){
	int k = 0;
	int length=frame_point_after.size();
	for (int i=0; i<length; i++)
	{
		if (is_accept_TrackedPoint(i))
		{
			track_point_start.push_back(frame_point_before[i]);
			track_point_end.push_back(frame_point_after[i]);
		}
	}
}

//draw info
void draw_points(Mat &output){
	int length=track_point_end.size();
	for (int i=0; i<length; i++){
		circle(output, frame_point_before[i], 2, Scalar(50,200,0), -1);
	}
}

void draw_lines(Mat &output){
	int length=track_point_end.size();
	for (int i=0; i<length; i++)
	{
		line(output, track_point_start[i], track_point_end[i], Scalar(0, 0, 255));
		circle(output, track_point_end[i], 2, Scalar(255, 0, 0), -1);
	}
}

void draw_infomation(Mat &output){
	char str_i[50];
	sprintf(str_i,"all:%d move:%d",frame_point_before.size(),track_point_start.size());
	putText(output, str_i, cvPoint( 20, 20),CV_FONT_HERSHEY_COMPLEX,0.5,Scalar(50,0,20));
	sprintf(str_i,"direction x:%4.3f  y:%4.3f",frame_direction.x,frame_direction.y);
	putText(output, str_i, cvPoint( 20, 60),CV_FONT_HERSHEY_COMPLEX,0.5,Scalar(20,0,40));
}

//calc 2 point distance
float points_distance(Point2f point1,Point2f point2){
	float dx=point2.x-point1.x;
	float dy=point2.y-point1.y;
	return sqrt(dx*dx+dy*dy);
}

//calculate all points distance with other points
void calc_points_distance(){
	int points_count=track_point_end.size();
	for(int m =0; m<points_count;m++){
		for(int n=0;n<points_count;n++){
				two_points_distance[m][n]=points_distance(track_point_end[m],track_point_start[n]);
		}
	}
}

//weight is the distance from the trackpoint
//sum the other vector with weight,to get the direction
Point2f sum_with_weight(int m){
	float tempx,tempy;
	int points_count=track_point_end.size();
	Point2f res=Point2f(0,0);
	for(int n=0;n<points_count;n++){
		tempx=track_point_direction[m].x/two_points_distance[m][n];
		tempy=track_point_direction[m].y/two_points_distance[m][n];
		res+=Point2f(tempx,tempy);
	}
	return res;
} 

void sum_all_with_weight(){
	
}

//get all points direction
void get_point_direction(){
	Point2f tempp2f,avgp2f;
	int length=track_point_end.size();
	for(int i=0; i<length; i++){
		tempp2f=track_point_end[i]-track_point_start[i];
		track_point_direction.push_back(tempp2f);
	}
}

void calc_movement(){
	get_point_direction();
	
	Point2f avgp2f;
	int length=track_point_end.size();
	for(int i=0; i<length; i++){
		avgp2f+=track_point_direction[i];
	}
	frame_direction=Point2f(avgp2f.x/length,avgp2f.y/length);
}

void tracking(Mat &frame, Mat &output){
	cvtColor(frame, gray, CV_BGR2GRAY);
	frame.copyTo(output);

	//ensure when init ,frame is not empty
	if (gray_prev.empty()){
		gray.copyTo(gray_prev);
	}
	
	//optical flow
	recorde_feature_points();
	calcOpticalFlowPyrLK(gray_prev, gray, frame_point_before, frame_point_after, status, err);
	
	//calc movement
	get_accept_points();
	calc_movement();
	
	//draw text and lines
	draw_points(output);
	draw_lines(output);
	draw_infomation(output);

	//show pic
	imshow(window_name, output);
	
	//clear used data
	clear_used_frame_data();
}


void clear_used_frame_data(){
	frame_point_before.clear();
	//gray_prev.clear();
	
	swap(frame_point_before, frame_point_after);
	swap(gray_prev, gray);

	track_point_end.clear();
	track_point_start.clear();
}

bool is_accept_TrackedPoint(int i)
{
	//status accept
	if(status[i]){
		//move distance > 3
		float dx=frame_point_before[i].x - frame_point_after[i].x;
		float dy=frame_point_before[i].y - frame_point_after[i].y;
		if((dx*dx+dy*dy) > 3*3){
			return true;
		}
	}
	return false;
}

