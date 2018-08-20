#include "PF.h"

PF::PF()
{
	init_particle();
}

void PF::init_particle()
{
	// Create MAP

	m = new real_map(13.14f, 18.53f, 0.01f);
	m2 = new real_map(13.14f, 18.53f, 0.05f);
	obstacle t1[15];
	obstacle* pt1 = t1;
	t1[0].x = 0;t1[0].y=0, t1[0].width = 0.1; t1[0].height = 0.48;
	t1[1].x = 0; t1[1].y = 1.29, t1[1].width = 0.1; t1[1].height = 2;
	t1[2].x = 0; t1[2].y = 4.62, t1[2].width = 0.1; t1[2].height = 12.33;
	t1[3].x = 1.6; t1[3].y = 0, t1[3].width = 0.1; t1[3].height = 0.48;
	t1[4].x = 1.6; t1[4].y = 1.29, t1[4].width = 0.1; t1[4].height = 2;
	t1[5].x = 1.6; t1[5].y = 4.62, t1[5].width = 0.1; t1[5].height = 12.33;
	t1[6].x = 0; t1[6].y=18.43, t1[6].width = 4.61; t1[6].height = 0.1;
	t1[7].x = 6.06; t1[7].y = 18.43, t1[7].width = 5.01; t1[7].height = 0.1;
	t1[8].x = 1.6; t1[8].y = 16.85, t1[8].width = 3.01; t1[8].height = 0.1;
	t1[9].x = 4.61; t1[9].y = 16.75, t1[9].width = 1.45; t1[9].height = 0.1;
	t1[10].x = 6.06; t1[10].y = 16.85, t1[10].width = 5.01; t1[10].height = 0.1;
	//t1[11].x = 6.3; t1[11].y = 16.95, t1[11].width = 4.5; t1[11].height = 0.43;
	t1[11].x = 12.49; t1[11].y = 18.43, t1[11].width = 0.7; t1[11].height = 0.1;
	t1[12].x = 11.07; t1[12].y = 16.75, t1[12].width = 1.42; t1[12].height = 0.1;
	t1[13].x = 12.49; t1[13].y = 16.85, t1[13].width = 0.7; t1[13].height = 0.1;
	t1[14].x = 1.6; t1[14].y = 0, t1[14].width = 11.5; t1[14].height = 16.85;

	m->add_p_obstacle(pt1, 16);
	m->draw_on_map();
	m2->add_p_obstacle(pt1, 16);
	m2->draw_on_map();
	distribution_s = uniform_real_distribution<float>(-0.1f, 0.1f);
	distribution_a = uniform_real_distribution<float>(-PI/10, PI/10);
	/************************************************************/
	//Init Particle
	p_robot = create_particle(num_particle, m, weight, xmin, xmax, ymin, ymax); //Pointer to particle of robot
	e_robot.pcarte = m;
}

robot * PF::create_particle(int const num, real_map* rm, float* weight, float _xmin, float _xmax, float _ymin, float _ymax)
{
	robot* p_robot = new robot[num];
	float newx, newy, newa;
	int ix, iy;
	float _weight = 1.0f / num;
	std::uniform_real_distribution<double> distribution_x(0, 13.14);
	std::uniform_real_distribution<double> distribution_y(0, 18.53);
	//std::uniform_real_distribution<double> distribution_x(1.0,1.4);
	//std::uniform_real_distribution<double> distribution_y(1.0, 1.4);
	std::uniform_real_distribution<double> distribution_a(10*PI/8, 14*PI/8);
	for (int i = 0; i < num; i++)
	{
		do
		{
			newx = distribution_x(generator); newy = distribution_y(generator); 
			newa = distribution_a(generator);
			//newa = 0;
			ix = (int)(newx / rm->m_per_pixel);
			iy = (int)(newy / rm->m_per_pixel);
		} while (rm->img_map->at<cv::Vec3b>(iy, ix).val[1] == 255);
		p_robot[i].set(newx, newy, newa);
		p_robot[i].pcarte = rm;
		weight[i] = _weight;
	}

	return p_robot;
}

float PF::likelihood(float x, float mu, float sigma)
{
	return exp(-((x - mu)*(x - mu)) / (2 * sigma*sigma));
}

void PF::update_state(long pulseL,long pulseR)
{
	for (int i = 0; i < num_particle; i++)
	{
		float sigmax = distribution_s(generator), sigmay = distribution_s(generator), sigmaa = distribution_a(generator);
		p_robot[i].move2(pulseL,pulseR,sigmax, sigmay, sigmaa);
		int ix = (int)(p_robot[i].x / m->m_per_pixel);
		int iy = (int)(p_robot[i].y / m->m_per_pixel);
		if ((iy >= m->img_map->rows) || (iy < 0) || (ix < 0) || (ix >= m->img_map->cols) || m->img_map->at<cv::Vec3b>(iy, ix).val[1] == 255) weight[i] = 0;

	}
}

void PF::measure(float* sense)
{
	float sum_w = 0;
	for (int i = 0; i < num_particle; i++)// likelihood score
	{
		if (weight[i] != 0) {
			p_robot[i].sense();
			float wlf = likelihood(sense[0], p_robot[i].sensor[0], 0.02f);
			float wrf = likelihood(sense[1], p_robot[i].sensor[1], 0.02f);
			float wlr = likelihood(sense[2], p_robot[i].sensor[2], 0.02f);
			float wrr = likelihood(sense[3], p_robot[i].sensor[3], 0.02f);
			weight[i] *= (wlf+wrf+wlr+wrr);
			sum_w += weight[i];
		}
	}
	if (sum_w != 0) // Normalize weight
	{
		for (int i = 0; i < num_particle; i++)
		{
			weight[i] = weight[i] / sum_w;
		}
		reset = 0;
	}
	else
	{
		delete[] p_robot;
		p_robot = create_particle(num_particle, m, weight, xmin, xmax, ymin, ymax);
		for (int i = 0; i < num_particle; i++)
		{
			weight[i] = weight[i] / num_particle;
		}
		reset = 1;
		//cout << "Reshuffle particle" << endl;
	}
}

void PF::estimate(bool reset)
{
	if (!reset)
	{
		float e_x = 0, e_y = 0, e_a = 0; // Update estimated position

		for (int i = 0; i < num_particle; i++)
		{
			e_x += weight[i] * p_robot[i].x;
			e_y += weight[i] * p_robot[i].y;
			e_a += weight[i] * p_robot[i].dir;
		}
		e_robot.set(e_x, e_y, e_a);
		e_robot.draw_on_map(&test, 5, cv::Scalar(0, 0, 255));
		e_robot.sense();
		//std::cout << "Estimated..........." << std::endl;
		//printf("IRLF= %0.2f, IRRF= %0.2f \n IRLR= %0.2f, IRRR=%0.2f \n",
		//	e_robot.sensor[0], e_robot.sensor[1], e_robot.sensor[2], e_robot.sensor[3]);
		//printf("Position: %0.2f, %0.2f", e_robot.x, e_robot.y);
	}
	else
	{
		//std::cout << "Reshuffle Particle..." << std::endl;
	}
}

void PF::renormalize(bool reset)
{
	if (!reset)
	{
		float sum_w = 0;
		for (int i = 0; i < num_particle; i++)
		{
			sum_w += (weight[i] * weight[i]);
		}
		float N_eff = 1.0f / sum_w;
		cout << N_eff << "/" << N_thresh << endl;
		if (N_eff < N_thresh)
		{
			//cout << "Resampling..." << endl;
			robot* p_newrobot = new robot[num_particle];
			vector<float> cumw(num_particle);
			vector<float> uu(num_particle);
			cumw[0] = weight[0];
			uu[0] = -log(rnp);
			float sumcdf = cumw[0];
			float sumuu = uu[0];

			for (int i = 1; i < num_particle; i++)
			{
				float tmp = weight[i] + cumw[i - 1];
				cumw[i] = tmp;
				tmp = uu[i - 1] - log(rnp);
				uu[i] = tmp;
				sumcdf += cumw[i];
				sumuu += uu[i];
			}
			sumcdf = 1.0f / sumcdf;
			sumuu = 1.0f / sumuu;

			/* Normalize */
			for (int i = 0; i < num_particle; i++)
			{
				cumw[i] *= sumcdf;
				uu[i] *= sumuu;
			}
			/*  On recopie les N premiers éléments de u_ord dans le vecteur uu. Le N + 1 élément est égal à 2. */
			int i = 0;
			int j = 0;
			while (i < num_particle)
			{
				if ((uu[i] < cumw[j]) | (j == num_particle - 1))
				{
					p_newrobot[i].x = p_robot[j].x;
					p_newrobot[i].y = p_robot[j].y;
					p_newrobot[i].dir = p_robot[j].dir;
					p_newrobot[i].pcarte = p_robot[j].pcarte;
					weight[i] = 1;
					i++;
				}
				else
				{
					if (j < num_particle - 1)
					{
						j++;
					}
				}
			}


			vector<float>().swap(cumw);
			vector<float>().swap(uu);
			delete[] p_robot;
			p_robot = p_newrobot;
		}
	}
}

void PF::draw_particle()
{
	m2->img_map->copyTo(test);
	for (int i = 0; i < num_particle; i++)
	{
		if (weight[i] != 0)
			p_robot[i].draw_on_map(&test, 5, cv::Scalar(255, 0, 255));
	}
}

void PF::reset_particle()
{
	delete[] p_robot;
	p_robot = create_particle(num_particle, m, weight, xmin, xmax, ymin, ymax);
}

