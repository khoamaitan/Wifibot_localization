#include "PF_Tracking.hpp"



Particle_Tracking::Particle_Tracking()
{
	srand(time(NULL));
	isd_x = 60;
	isd_vx = 1;
	isd_y = 60;
	isd_vy = 1;
	isd_w = 5;
	isd_h = 5;
	sd_transition = 0.7;
	sd_w = 5;
	sd_h = 5;
	N = 100;
	N_thresh = 0.7*N;
	Nh = 16;
	Ns = 8;
	Nv = 8;
	float sigma_color = 0.3;
	c2 = (1.0f / (sqrt(2 * 3.1416)*sigma_color));
	c1 = 1.0f / (2 * sigma_color*sigma_color);
	dt = 0.7;
	float var_transition = sd_transition*sd_transition;
	float var_w = sd_w*sd_w;
	float var_h = sd_h*sd_h;
	A = (Mat_<float>(6, 6) << 1, dt, 0, 0, 0, 0,
		0, 1, 0, 0, 0, 0,
		0, 0, 1, dt, 0, 0,
		0, 0, 0, 1, 0, 0,
		0, 0, 0, 0, 1, 0,
		0, 0, 0, 0, 0, 1);
	sd_init = (Mat_<float>(6, 6) << isd_x, 0, 0, 0, 0, 0,
		0, isd_vx, 0, 0, 0, 0,
		0, 0, isd_y, 0, 0, 0,
		0, 0, 0, isd_vy, 0, 0,
		0, 0, 0, 0, isd_w, 0,
		0, 0, 0, 0, 0, isd_h);
	//TODO: solve sd for evolution
	sd_evolution = (Mat_<float>(6, 6) << var_transition*pow(dt, 3) / 3, var_transition*pow(dt, 2) / 2, 0, 0, 0, 0,
		var_transition*pow(dt, 2) / 2, dt, 0, 0, 0, 0,
		0, 0, var_transition*pow(dt, 3) / 3, var_transition*pow(dt, 2) / 2, 0, 0,
		0, 0, var_transition*pow(dt, 2) / 2, dt, 0, 0,
		0, 0, 0, 0, var_w, 0,
		0, 0, 0, 0, 0, var_h);
	Cholesky(sd_evolution, sd_evolution);
	pn_state = Mat::zeros(N, 6, CV_32FC1);
	p_state = Mat::zeros(6, 1, CV_32FC1);
	weight = Mat::ones(N, 1, CV_32FC1);
	edge1.resize(Nh + 1);
	edge2.resize(Ns + 1);
	edge3.resize(Nv + 1);
}

void Particle_Tracking::Init(Mat img, int x, int y, int w, int h)
{
	int i;
	// Create target distribution
	target_distribution(img, x, y, w, h);
	pn_state = Mat::zeros(N, 6, CV_32FC1);
	p_state = Mat::zeros(6, 1, CV_32FC1);
	weight = Mat::ones(N, 1, CV_32FC1);
	//Create first state
	float* ps = (float*)p_state.data;
	*(ps + 0) = x;
	*(ps + 1) = 0;
	*(ps + 2) = y;
	*(ps + 3) = 0;
	*(ps + 4) = w;
	*(ps + 5) = h;
	// Create first state for N particle with inputs and sd_init
	float* pns = (float*)pn_state.data;
	for (i = 0;i < N;i++)
	{
		*(pns + 0 + i * 6) = *(ps + 0) + isd_x*(float)(((rand() % 2000) - 1000) / 1000.0f);
		*(pns + 1 + i * 6) = *(ps + 1) + isd_vx*(float)(((rand() % 2000) - 1000) / 1000.0f);
		*(pns + 2 + i * 6) = *(ps + 2) + isd_y*(float)(((rand() % 2000) - 1000) / 1000.0f);
		*(pns + 3 + i * 6) = *(ps + 3) + isd_vy*(float)(((rand() % 2000) - 1000) / 1000.0f);
		*(pns + 4 + i * 6) = *(ps + 4) + isd_w*(float)(((rand() % 2000) - 1000) / 1000.0f);
		*(pns + 5 + i * 6) = *(ps + 5) + isd_h*(float)(((rand() % 2000) - 1000) / 1000.0f);
	}
}

void Particle_Tracking::track(Mat img)
{
	int i, j, k;
	//Evolution of system:
	Mat hsv;
	cvtColor(img, hsv, CV_BGR2HSV);
	for (i = 0;i < N;i++)
	{
		Mat mat_rd = (Mat_<float>(6, 1) << rn, rn, rn, rn, rn, rn);
		Rect rect_temp = Rect(0, i, 6, 1);
		Mat temp = pn_state(rect_temp);
		temp = temp.reshape(0, 6);
		temp = A*temp + sd_evolution*mat_rd;
		vector<float> pdf_n;
		pdf_radm(hsv, pdf_n, temp.at<float>(0, 0), temp.at<float>(2, 0), (int)temp.at<float>(4, 0), (int)temp.at<float>(5, 0), edge1, edge2, edge3);
		vector<float> multi(Nh*Ns*Nv);
		multiply(pdf_n, pdf_tg, multi);
		sqrt(multi, multi);
		Scalar t = sum(multi);
		float likelihood = c2*exp((t[0] - 1)*c1);
		//cout << likelihood << endl;
		weight.at<float>(i, 0) = weight.at<float>(i, 0)*likelihood;

	}
	//Normalize weight
	normalize(weight, weight, 1.0, 0, NORM_L1); //Condition all value must > 0
												//cout << weight << endl;
	// Estimate mean
	for (i = 0;i < 6;i++)
	{
		Rect rec_d = Rect(i, 0, 1, N);
		Mat temp = pn_state(rec_d);
		Mat mult;
		multiply(temp, weight, mult);
		Scalar somme = sum(mult);
		p_state.at<float>(i, 0) = somme[0];

	}
	//Resample
	Mat mult;
	multiply(weight, weight, mult);
	Scalar somme = sum(mult);
	float N_eff = 1.0f / somme[0];
	//cout << N_eff << "/" << N_thresh << endl;

	if (N_eff < N_thresh)
	{
		//cout << "Resampling..." << endl;
		Mat ntemp_state;
		pn_state.copyTo(ntemp_state);
		vector<float> cumw(N);
		vector<float> uu(N);
		cumw[0] = weight.at<float>(0, 0);
		uu[0] = -log(rnp);
		float sumcdf = cumw[0];
		float sumuu = uu[0];

		for (i = 1; i < N; i++)
		{
			float tmp = weight.at<float>(i, 0) + cumw[i - 1];
			cumw[i] = tmp;
			tmp = uu[i - 1] - log(rnp);
			uu[i] = tmp;
			sumcdf += cumw[i];
			sumuu += uu[i];
		}
		sumcdf = 1.0f / sumcdf;
		sumuu = 1.0f / sumuu;

		/* Normalize */

		for (i = 0; i < N; i++)
		{
			cumw[i] *= sumcdf;
			uu[i] *= sumuu;
		}

		/*  On recopie les N premiers éléments de u_ord dans le vecteur uu. Le N + 1 élément est égal à 2. */

		i = 0;
		j = 0;
		while (i < N)
		{
			if ((uu[i] < cumw[j]) | (j == N - 1))
			{
				int idx = j ;

				//cout << idx << endl;
				Rect rect_temp = Rect(0, idx, 6, 1);
				Rect rect_state = Rect(0, i, 6, 1);
				Mat mat_temp = ntemp_state(rect_temp);
				Mat mat_state = pn_state(rect_state);
				mat_state = (mat_temp * 1);
				weight.at<float>(i, 0) = 1;
				i++;
			}
			else
			{
				if (j < N-1)
				{
					j++;
				}
			}
		}
	}



}

void Particle_Tracking::pdf_radm(Mat hsv, vector<float> &pdf, int x, int y, int w, int h, vector<float> edge1, vector<float> edge2, vector<float> edge3)
{
	int Np, i;
	int row = hsv.rows;
	int col = hsv.cols;
	pdf.resize(Nh * Ns * Nv);
	int counter = 0;
	uchar* p = hsv.data;
	Np = w*h/2;
	for (i = 0;i < Np;i++)
	{
		float rdx = (int)((rand() % w) - w / 2);
		float rdy = (int)((rand() % h) - h / 2);
		float wx = exp(-abs(rdx));
		float wy = exp(-abs(rdy));
		float weight = min(wx,wy);
		float newx = x + rdx;
		float newy = y + rdy;
		int inewx = floor(newx);
		int inewy = floor(newy);
		float ax = newx-inewx;
		float ay = newy-inewy;
		float w00 = (1-ax)*(1-ay);
		float w01 = (ax)*(1-ay);
		float w10 = (1-ax)*(ay);
		float w11 = (1-w00-w01-w10);
		if (inewx < 0 || (inewx+1) >= col || inewy < 0 || (inewy+1) >= row)
		{

		}
		else
		{

			float valh = (*(p + inewy*col * 3 + inewx * 3 + 0))*w00+(*(p + inewy*col * 3 + (inewx+1) * 3 + 0))*w01+(*(p + (inewy+1)*col * 3 + inewx * 3 + 0))*w10+(*(p + (inewy+1)*col * 3 + (inewx+1) * 3 + 0))*w11;
			float vals = (*(p + inewy*col * 3 + inewx * 3 + 1))*w00+(*(p + inewy*col * 3 + (inewx+1) * 3 + 1))*w01+(*(p + (inewy+1)*col * 3 + inewx * 3 + 1))*w10+(*(p + (inewy+1)*col * 3 + (inewx+1) * 3 + 1))*w11;
			float valv = (*(p + inewy*col * 3 + inewx * 3 + 2))*w00+(*(p + inewy*col * 3 + (inewx+1) * 3 + 2))*w01+(*(p + (inewy+1)*col * 3 + inewx * 3 + 2))*w10+(*(p + (inewy+1)*col * 3 + (inewx+1) * 3 + 2))*w11;
			int binh = val2bin(valh, edge1, Nh);
			int bins = val2bin(vals, edge2, Ns);
			int binv = val2bin(valv, edge3, Nv);
			pdf[binh + bins*Nh + Nh*Ns*binv]+=1;
			counter++;

		}
	}
	if (counter != 0)
	{
		for (i = 0;i < pdf.size();i++)
			pdf[i] /= counter;
	}

}

void Particle_Tracking::target_distribution(Mat image, int x, int y, int w, int h)
{

	int i;
	int row = image.rows;
	int col = image.cols;
	Mat hsv;
	cvtColor(image, hsv, CV_BGR2HSV);
	Mat splited[3];
	split(hsv, splited);

	int histSize = 256;
	// hue varies from 0 to 179, see cvtColor
	float hranges[] = { 0, 180 };
	float sranges[] = { 0, 256 };
	const float* ranges = { hranges };
	Mat histh, hists, histv;
	calcHist(&splited[0], 1, 0, Mat(), histh, 1, &histSize, &ranges, true, false);
	const float* rangev = { sranges };
	calcHist(&splited[1], 1, 0, Mat(), hists, 1, &histSize, &rangev, true, false);
	calcHist(&splited[2], 1, 0, Mat(), histv, 1, &histSize, &rangev, true, false);


	vector<float> csumh(histSize);
	vector<float> csums(histSize);
	vector<float> csumv(histSize);
	int idxh = 0;
	vector<float> stepmath(Nh);
	int idxs = 0;
	int idxv = 0;
	vector<float> stepmatsv(Ns); //Ns=Nv


	edge1[0] = 0;edge1[Nh] = 256;
	edge2[0] = 0;edge2[Ns] = 256;
	edge3[0] = 0;edge3[Nv] = 256;
	//Calculate step

	for (i = 0;i < Nh;i++)
	{
		stepmath[i] = (i + 1)*(1.0f / Nh);
	}
	for (i = 0;i < Ns;i++)
	{
		stepmatsv[i] = (i + 1)*(1.0f / Ns);
	}

	float* phisth = (float*)histh.data;
	float* phists = (float*)hists.data;
	float* phistv = (float*)histv.data;
	float* pcumh = csumh.data();
	float* pcums = csums.data();
	float* pcumv = csumv.data();
	int area = row*col;
	for (i = 0;i < histSize;i++)
	{
		if (i == 0)
		{
			*pcumh = *phisth / area;
			*pcums = *phists / area;
			*pcumv = *phistv / area;
		}
		else
		{
			*pcumh = *(pcumh - 1) + *phisth / area;
			*pcums = *(pcums - 1) + *phists / area;
			*pcumv = *(pcumv - 1) + *phistv / area;
		}
		if ( (idxh < Nh) && (*pcumh > stepmath[idxh]) )
		{
			edge1[idxh + 1] = i;
			//cout << edgeh[idxh + 1] << endl;
			idxh++;
		}
		if ( (idxs < Ns) && (*pcums > stepmatsv[idxs])  )
		{
			edge2[idxs + 1] = i;
			idxs++;
		}
		if ( (idxs < Nv) && (*pcumv > stepmatsv[idxv]) )
		{
			edge3[idxv + 1] = i;
			idxv++;
		}
		//cout << *pcumh << endl;
		if(i!=(histSize-1))
		{ 
		
		pcumh++;pcums++;pcumv++;
		phisth++;phists++;phistv++;
		}
	}

	pdf_radm(hsv, pdf_tg, x, y, w, h, edge1, edge2, edge3);

}

int Particle_Tracking::val2bin(float zi, vector<float> edge, int Nbin)

{
	int k;
	int k0 = 0, N1 = Nbin, k1 = N1;

	if ((zi >= edge[0]) && (zi <= edge[N1]))
	{
		k = (k0 + k1) / 2;
		while (k0 < k1 - 1)
		{
			if (zi >= edge[k])
			{
				k0 = k;
			}
			else
			{
				k1 = k;
			}
			k = (k0 + k1) / 2;
		}
		k = k0;
	}
	return k;
}

void Particle_Tracking::Cholesky(const Mat &A, Mat &S)
{
	CV_Assert(A.type() == CV_32F);

	int dim = A.rows;
	S.create(dim, dim, CV_32F);

	int i, j, k;

	for (i = 0; i < dim; i++)
	{
		for (j = 0; j < i; j++)
			S.at<float>(i, j) = 0.f;

		float sum = 0.f;
		for (k = 0; k < i; k++)
		{
			float val = S.at<float>(k, i);
			sum += val*val;
		}

		S.at<float>(i, i) = std::sqrt(std::max(A.at<float>(i, i) - sum, 0.f));
		float ival = 1.f / S.at<float>(i, i);

		for (j = i + 1; j < dim; j++)
		{
			sum = 0;
			for (k = 0; k < i; k++)
				sum += S.at<float>(k, i) * S.at<float>(k, j);

			S.at<float>(i, j) = (A.at<float>(i, j) - sum)*ival;
		}
	}
}

void Particle_Tracking::draw(Mat& img, bool center_mean, bool center_particles, bool window_mean)
{
	int i;
	Scalar red(0, 0, 255);
	Scalar blue(255, 0, 0);
	Scalar green(0, 255, 0);
	Mat affiche;
	img.copyTo(affiche);
	int xm = (int)p_state.at<float>(0, 0);
	int ym = (int)p_state.at<float>(2, 0);
	int wm = (int)p_state.at<float>(4, 0);
	int hm = (int)p_state.at<float>(5, 0);
	if (center_particles)
	for (i = 0;i<N;i++) {
		circle(affiche, Point((int)pn_state.at<float>(i, 0), (int)pn_state.at<float>(i, 2)), 3, blue, -1);

	}
	if (center_mean)
	circle(affiche, Point(xm, ym), 3, red, -1);
	if (window_mean)	
	rectangle(affiche, Rect(xm - wm / 2, ym - hm / 2, wm, hm), green, 2);
	imshow("PF_Tracker", affiche);
	//vector<float> temp = pdf_tg;
}
