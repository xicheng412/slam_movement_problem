#include "optical_flow_data_class.h"

frame_data::frame_data(int record_frame_number){
	this->record_frame_number=record_frame_number;
	this->frame_pointer=new Mat[record_frame_number];
	this->this_pointer=0;
}

frame_data::~frame_data(){
	delete this->frame_pointer;
}

//this pointer add i(i may positive or negative)
int frame_data::this_pointer_change(int i){
	if(i<0){
		i=i+((-i)/this->record_frame_number+1)*this->record_frame_number;
	}
	return (this->this_pointer+i)%this->record_frame_number;
}

void frame_data::record(const Mat& frame){
	frame.copyTo(this->frame_pointer[this->this_pointer]);
	this->this_pointer=this_pointer_change(1);
}
//when n==0 is this frame,-1 is last frame
Mat frame_data::get(int n){
	return this->frame_pointer[this_pointer_change(n)];
}
