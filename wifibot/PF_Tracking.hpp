#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <stdio.h>
#include <math.h>

#define rn ( ( (rand() % 2000) - 1000) / 1000.0f)
#define rnp ( ( (rand() % 1000 +1) ) / 1000.0f)

using namespace cv;
using namespace std;


	class Particle_Tracking
	{
	protected:
		// variance of states at the begining
		float isd_x, isd_vx, isd_y, isd_vy, isd_w, isd_h;
		// variance for updating state
		float sd_transition, sd_w, sd_h;
		int Nh, Ns, Nv;
		float c1, c2;
		vector<float> edge1;
		vector<float> edge2;
		vector<float> edge3;
		vector<float> pdf_tg;
		int N; // Number of particles
		float dt; // frames' sample time
		int N_thresh; // Threshold to do resampling
		Mat A; // Transition matrix 6x6
		Mat sd_init; // sd of noise at the begining  6x6
		Mat sd_evolution; // sd of noise in transition 6x6
		Mat pn_state; // states of N particles Nx6
		Mat weight; // weight for every praticle Nx1
	public:
		Mat p_state; // initialized states and moyen states 6x1
		Particle_Tracking();
		void Init(Mat img, int x, int y, int w, int h);
		void target_distribution(Mat image, int x, int y, int w, int h);
		int val2bin(float zi, vector<float> edge, int Nbin);
		void pdf_radm(Mat hsv, vector<float> &pdf, int x, int y, int w, int h, vector<float> edge1, vector<float> edge2, vector<float> edge3);
		void track(Mat img);
		void Cholesky(const Mat& A, Mat& S);
		void draw(Mat &img, bool center_mean = true, bool center_particles = true, bool window_mean = true);
	};

