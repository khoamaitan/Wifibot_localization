#pragma once
#include <opencv2/opencv.hpp>
#include <iostream>
#include <io.h>
#include <stdlib.h>
#include <math.h>
#include <random>

using namespace std;
using namespace cv;
class KLT_Tracking {
public:
	Mat img1, img2;
	int w_size;
	Point* arr_ps=NULL;
	vector<Point2f> vec_input;
	vector<uchar> stats;
	
	int num_p;
	KLT_Tracking(Mat _img1);
	void track(Mat img2);
	void change_points(Point* _arr_ps, int _num_p) ;

};
