#pragma once
#include <opencv2/opencv.hpp>
#include <iostream>
#include <io.h>
#include <stdlib.h>
#include <math.h>
#include "real_map.h"

#define PI 3.1415f

//using namespace cv;
using namespace std;

class robot
{
private:
	const float angle_IR_front = 0.416f;
	const float angle_IR_rear = 0.416f;
public:
	float x;
	float y;
	float dir;
	float sensor[4];

	real_map* pcarte;
	real_map* pcarte2;
	robot();
	robot(float _x, float _y, float _dir);
	~robot();
	void move(float deltax, float deltay, float delta_a);
	void move2(long pulseL, long pulseR, float sigmax, float sigmay, float sigmaa);
	void set(float _x, float _y, float _dir);
	float cal_angle_ir(char ir);
	void sense();
	void line(cv::Point pt1, cv::Point pt2);
	float line_measure(cv::Point pt1, cv::Point pt2);
	void draw_on_map(cv::Mat* img,int _size,cv::Scalar color);
	void adc_m(int irLF,int irRF,int irLR,int irRR);
	robot& operator=(const robot _robotin);
};
