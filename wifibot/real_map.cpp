#include "real_map.h"

_obstacle::_obstacle()
{
	x = 0;
	y = 0;
	width = 1;
	height = 1;
}
real_map::real_map()
{
	m_per_pixel = 0.1f;
	img_map = new cv::Mat(60, 60, CV_8UC3);
	*img_map = cv::Scalar(0, 0, 0);
	p_obs = NULL;
}

real_map::real_map(int width, int height, float _m_per_pixel)
{
	m_per_pixel = _m_per_pixel;
	img_map = new cv::Mat(height, width, CV_8UC3);
	*img_map = cv::Scalar(0, 0, 0);
	p_obs = NULL;
}

real_map::real_map(float r_width, float r_height, float _m_per_pixel)
{
	m_per_pixel = _m_per_pixel;
	int width = (int)(r_width / m_per_pixel);
	int height = (int)(r_height / m_per_pixel);
	img_map = new cv::Mat(height, width, CV_8UC3);
	*img_map = cv::Scalar(0, 0, 0);
}

real_map::~real_map()
{
	if (p_obs != NULL) delete[] p_obs;
	img_map->release();
	cout << "Deconstructor of real_map" << endl;
}

int real_map::add_p_obstacle(obstacle * _p_obs, int _num_obs)
{
	if (p_obs != NULL) delete[] p_obs;
	num_obs = _num_obs;
	p_obs = new obstacle[num_obs];

	memcpy(p_obs, _p_obs, sizeof(obstacle)*num_obs);
	return 0;
}

int real_map::add_a_obstacle(obstacle arr_obs[], int _num_obs)
{
	if (p_obs != NULL) delete[] p_obs;
	num_obs = _num_obs;
	p_obs = new obstacle[num_obs];
	memcpy(p_obs, arr_obs, num_obs * sizeof(obstacle));
	return 0;
}

int real_map::draw_on_map()
{
	*img_map = cv::Scalar(0, 0, 0);
	if (p_obs == NULL) return 0;
	int x, y, width, height;
	obstacle* p = p_obs;
	for (int i = 0; i < num_obs; i++)
	{
		x = (int)(p->x / m_per_pixel);
		y = (int)(p->y / m_per_pixel);
		width = (int)(p->width / m_per_pixel);
		width = max(width, 1);
		height = (int)(p->height / m_per_pixel);
		height = max(height, 1);
		rectangle(*img_map, cv::Point(x, y), cv::Point(x + width - 1, y + height - 1), cv::Scalar(0, 255, 0), -1);
		p++;
		//cout << i << endl;
		//imshow("Debug ", *img_map);
		//cvWaitKey(0);
	}
}

int real_map::width()
{
	return img_map->cols;
}

int real_map::height()
{
	return img_map->rows;
}