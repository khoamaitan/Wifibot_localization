#pragma once
#include <opencv2/opencv.hpp>
#include <iostream>
#include <io.h>
#include <stdlib.h>
#include <math.h>

//using namespace cv;
using namespace std;

typedef struct _obstacle
{
public:
	_obstacle();
	float x;
	float y;
	float width;
	float height;
}  obstacle;
class real_map
{
public:
	obstacle* p_obs=NULL;
	int num_obs;
	cv::Mat* img_map;
	float m_per_pixel; //meter in realife per pixel
	real_map();
	real_map(int width, int height, float _m_per_pixel);
	real_map(float r_width, float r_height, float _m_per_pixel);
	~real_map();
	int add_p_obstacle(obstacle* _p_obs, int num_obs);
	int add_a_obstacle(obstacle arr_obs[], int num_obs);
	int draw_on_map();
	int width();
	int height();
	
};
