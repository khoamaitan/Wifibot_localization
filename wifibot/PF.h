#pragma once
#include <opencv2/opencv.hpp>
#include <iostream>
#include <io.h>
#include <stdlib.h>
#include <cmath>
#include <random>
#include <vector>
#include "real_map.h"
#include "robot.h"

using namespace std;

#define rnp ( ( (rand() % 1000 +1) ) / 1000.0f)
#define Np 500

class PF
{
public:
	//Random generator
	uniform_real_distribution<float> distribution_s;
	uniform_real_distribution<float> distribution_a;
	std::default_random_engine generator;
	//Particle parameter
	int num_particle = Np;
	float weight[Np];
	float N_thresh = 0.7*num_particle;
	float xmin = 2, xmax = 18, ymin = 2, ymax = 18;
	//Particle Variables
	float* sense; // Data captured
	bool reset = 0;
	real_map *m; //Init Map with size and scale
	real_map *m2;
	robot *p_robot = NULL; // Pointer to robot particle
	robot e_robot; // Estimated robot
	cv::Mat test; //Display particle

	PF();
	void init_particle();
	robot* create_particle(int num, real_map* rm, float* weight, float _xmin, float _xmax, float _ymin, float _ymax);
	float likelihood(float x, float mu, float sigma);
	void update_state(long pulseL, long pulseR);
	void measure(float* sense);
	void estimate(bool reset);
	void renormalize(bool reset);
	void draw_particle();
	void reset_particle();
};






