#include <FL/Fl_Double_Window.H>
#include <opencv2/opencv.hpp>
#include "Fl_ViewerCV.h"
Fl_ViewerCV* flcv;

struct flocvs{
	cv::VideoCapture cap;
	cv::Mat* dst;
	flocvs(){ 
		if(!cap.open(0))
			return ; 
		dst=new cv::Mat;
	}
	void draw(){ 
		  cv::Mat frame;
		  cap >> frame;
			cv::flip(frame, *dst, 1);
		  if( frame.empty() ) return;  
			flcv->SetImage(dst); 
 
	}  
};
flocvs* flocv;
void idle_cb(){
	flocv->draw();
}
int main(){

	int w=800;
	int h=480;
	Fl_Double_Window* win=new Fl_Double_Window(0,0,w,h);   
	flcv=new Fl_ViewerCV(0,0,w,h);
	win->show();
	flcv->show();
	flocv=new flocvs;
	Fl::set_idle(idle_cb);
	Fl::run();
	return 0;
	
}