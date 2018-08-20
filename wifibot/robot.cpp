#include "robot.h"

const float lim_m = 1.5f;

robot::robot()
{
	x = 0;
	y = 0;
	dir = 0;
}

robot::robot(float _x, float _y, float _dir)
{
	x = _x;
	y = _y;
	dir = _dir;
}

robot::~robot()
{
	pcarte=NULL;
	//cout << "deconstructor robot" << endl;
}

void robot::move(float deltax, float deltay, float delta_a)
{
	x = x + deltax;
	y = y + deltay;
	dir = (dir + delta_a);
	while (dir >= 2 * PI)
	{
		dir -= 2 * PI;
	}
	while (dir < 0)
	{
		dir += 2 * PI;
	}

}

void robot::move2(long pulseL, long pulseR,float sigmax,float sigmay,float sigmaa)
{
	
	if ((pulseL > 0) && (pulseR > 0))
	{
		float r = abs(pulseL)*47.1f / (340*100.0f);
		x = x + r*cos(dir)+sigmax;
		y = y - r*sin(dir)+sigmay;
	}
	else if ((pulseL < 0) && (pulseR > 0))
	{
		float rad = pulseR*PI / (2 * 260);
		dir = dir + rad+sigmaa;
	}
	else if ((pulseL > 0) && (pulseR < 0))
	{
		float rad = pulseL*PI / (2 * 260);
		dir = dir - rad+sigmaa;
	}
	else if ((pulseL < 0) && (pulseR < 0))
	{
		float r = pulseL*47.1f / (340*100.0f);
		x = x - r*cos(dir)+sigmax;
		y = y + r*sin(dir)+sigmay;

	}
	while (dir >= 2 * PI)
	{
		dir -= 2 * PI;
	}
	while (dir < 0)
	{
		dir += 2 * PI;
	}
}

void robot::set(float _x, float _y, float _dir)
{
	x = _x;
	y = _y;
	dir = _dir;
}

float robot::cal_angle_ir(char ir)
{
	float abs_angle;
	switch (ir)
	{
	case 0: // IR left front
		abs_angle = dir + angle_IR_front;
		break;
	case 1: // IR right front
		abs_angle = dir - angle_IR_front;
		break;
	case 2: // IR left rear
		abs_angle = dir + (PI - angle_IR_rear);
		break;
	case 3: // IR right rear
		abs_angle = dir - (PI - angle_IR_rear);
		break;
	default:
		abs_angle = -100.0f;

	}
	//if (abs_angle != -100.0f)
	while (abs_angle >= 2 * PI)
	{
		abs_angle -= 2 * PI;
	}
	while (abs_angle < 0)
	{
		abs_angle += 2 * PI;
	}
	return abs_angle;
}

void robot::sense()
{
	// Rescale x,y to integer value on map
	int ix = (int)(x / pcarte->m_per_pixel);
	int iy = (int)(y / pcarte->m_per_pixel);
	if (pcarte->img_map->at<cv::Vec3b>(iy, ix).val[1] == 255)
	{
		sensor[0] = 0; //left front
		sensor[1] = 0; // right front
		sensor[2] = 0; // left rear
		sensor[3] = 0; // right rear
		return;
	}
	else
	{
		// Calculate max radius
		unsigned int xx = ix*ix;
		unsigned int xdx = (pcarte->img_map->cols - ix)*(pcarte->img_map->cols - ix);
		unsigned int yy = iy*iy;
		unsigned int ydy = (pcarte->img_map->rows - y)*(pcarte->img_map->rows - y);
		unsigned int r1 = xx + yy;
		unsigned int r2 = xdx + yy;
		unsigned int r3 = xx + ydy;
		unsigned int r4 = xdx + ydy;
		unsigned int r = max(r1, max(r2, max(r3, r4)));
		float rad = sqrt(r);
		// Compute angle for sensor
		float angle_lf = cal_angle_ir(0);
		float angle_rf = cal_angle_ir(1);
		float angle_lr = cal_angle_ir(2);
		float angle_rr = cal_angle_ir(3);
		// Compute second point for drawing distance
		cv::Point lf(ix + rad*cos(angle_lf), iy - rad*sin(angle_lf));
		cv::Point rf(ix + rad*cos(angle_rf), iy - rad*sin(angle_rf));
		cv::Point lr(ix + rad*cos(angle_lr), iy - rad*sin(angle_lr));
		cv::Point rr(ix + rad*cos(angle_rr), iy - rad*sin(angle_rr));
		// Compute distance (pixel) from robot to obstacle
		float dlf = line_measure(cv::Point(ix, iy), lf);
		float drf = line_measure(cv::Point(ix, iy), rf);
		float dlr = line_measure(cv::Point(ix, iy), lr);
		float drr = line_measure(cv::Point(ix, iy), rr);
		// Return real value of distance(m) after sensing
		sensor[0] = (dlf*pcarte->m_per_pixel) > lim_m ? 9999 : (dlf*pcarte->m_per_pixel); //left front
		sensor[1] = (drf*pcarte->m_per_pixel) > lim_m ? 9999 : (drf*pcarte->m_per_pixel); // right front
		sensor[2] = (dlr*pcarte->m_per_pixel) > lim_m ? 9999 : (dlr*pcarte->m_per_pixel); // left rear
		sensor[3] = (drr*pcarte->m_per_pixel) > lim_m ? 9999 : (drr*pcarte->m_per_pixel); // right rear

		//cout << "angle lf " << angle_lf * 180 / PI << endl;
		//cout << "angle rf " << angle_rf * 180 / PI << endl;
		//cout << "angle lr " << angle_lr * 180 / PI << endl;
		//cout << "angle rr " << angle_rr * 180 / PI << endl;
		//cout << "biggest rad: " << rad << endl;
		//cout << "dlf pixel " << dlf << " meter: " << dlf*pcarte->m_per_pixel << endl;
		//cout << "drf pixel " << drf << " meter: " << drf*pcarte->m_per_pixel << endl;
		//cout << "dlr pixel " << dlr << " meter: " << dlr*pcarte->m_per_pixel << endl;
		//cout << "drr pixel " << drr << " meter: " << drr*pcarte->m_per_pixel << endl;
	}
}

void robot::line(cv::Point pt1, cv::Point pt2)
{

	return;
}
float robot::line_measure(cv::Point pt1, cv::Point pt2)
{
	int dPx, dPy, dPxy, P;
	int dx = abs(pt1.y - pt2.y);
	int dy = abs(pt1.x - pt2.x);
	int currentx = pt1.y;
	int currenty = pt1.x;
	int incr_x = pt2.y > pt1.y ? 1 : -1;
	int incr_y = pt2.x > pt1.x ? 1 : -1;
	cv::Vec3b intensity;
	intensity.val[0] = 255; intensity.val[1] = 0; intensity.val[2] = 0;
	//Vec3b intensity; intensity.val[0] = 0; intensity.val[1] = 255; intensity.val[2] = 0;
	// x is row ,y is column 
	if (dy >= dx)
	{
		dPy = dx * 2;
		dPxy = dPy - dy * 2;
		P = dPy - dy;
		for (; dy >= 0; dy--)
		{
			// Uncomment for drawing line on map
			//pcarte->img_map->at<Vec3b>(currentx, currenty) = intensity;
			if (P>0)
			{
				currentx += incr_x;
				currenty += incr_y;
				P += dPxy;
			}
			else
			{
				currenty += incr_y;
				P += dPy;
			}
			if ((currentx >= pcarte->img_map->rows) || (currentx < 0) || (currenty < 0) || (currenty >= pcarte->img_map->cols)) break;
			if (pcarte->img_map->at<cv::Vec3b>(currentx, currenty).val[1] == 255) return sqrt((pt1.x - currenty)*(pt1.x - currenty) + (pt1.y - currentx)*(pt1.y - currentx));


		}
	}
	else
	{
		dPx = dy * 2;
		dPxy = dPx - dx * 2;
		P = dPx - dx;

		for (; dx >= 0; dx--)
		{
			// Uncomment for drawing line on map
			//pcarte->img_map->at<Vec3b>(currentx, currenty) = intensity;

			if (P>0)
			{
				currenty += incr_y;
				currentx += incr_x;
				P += dPxy;
			}
			else
			{
				currentx += incr_x;
				P += dPx;
			}
			if ((currentx >= pcarte->img_map->rows) || (currentx < 0) || (currenty < 0) || (currenty >= pcarte->img_map->cols)) break;
			if (pcarte->img_map->at<cv::Vec3b>(currentx, currenty).val[1] == 255) return sqrt((pt1.x - currenty)*(pt1.x - currenty) + (pt1.y - currentx)*(pt1.y - currentx));

		}

	}
	return sqrt((pt1.x - currenty)*(pt1.x - currenty) + (pt1.y - currentx)*(pt1.y - currentx));
}

void robot::draw_on_map(cv::Mat* img,int _size,cv::Scalar color)
{
	//Mat temp;  
	int ix = (int)(x / 0.05f);// pcarte->m_per_pixel);
	int iy = (int)(y / 0.05f);// pcarte->m_per_pixel);
	cv::Point front(ix + (2*_size)*cos(dir), iy - (2 * _size)*sin(dir));
	line_measure(cv::Point(ix, iy), front);
	rectangle(*img, cv::Point(ix - _size, iy - _size), cv::Point(ix + _size, iy + _size), color, -1);
	arrowedLine(*img, cv::Point(ix, iy), front,cv::Scalar(255,200,0),1);
	//imshow("Robot", temp);

}

//Convert adc from IR sensor to meters
void robot::adc_m(int irLF, int irRF, int irLR, int irRR)
{
	float dLF, dRF, dLR, dRR;
	float in[4] = { irLF,irRF,irLR,irRR };
	float t[14] = { 2.55,2,1.5,1.22,0.97,0.85,0.72,0.67,0.63,0.58,0.52,0.5,0.47,0.4 };
	float d[14] = { 0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1,1.1,1.2,1.3,1.4,1.5 };
	//dLF = (69.3f - 0.25f*irLF) / (0.966f*100); //LF
	//dRF = (71.39f - 0.28f*irRF) / (0.9587f*100);//RF
	//dLR = (69.3f - 0.25f*irLR) / (0.966f*100);//LR
	//dRR = (71.39f - 0.28f*irRR) / (0.9587f*100);//RR
	for (int j = 0; j < 4; j++)
	{
		float vol_in = 0.01289f*in[j];
		bool found = false;
		for (int i = 1; i < 14; i++)
		{
			if (vol_in > t[i])
			{
				float wl = 1 / max(abs(vol_in - t[i - 1]), 0.0001f);
				float wr = 1 / max(abs(vol_in - t[i]), 0.0001f);
				sensor[j] = (wl*d[i - 1] + wr*d[i]) / (wl + wr);
				found = true;
				break;
			}
		}
		if (!found)
		{
			sensor[j] = 9999;
		}
	}
	//dLF = 0.01289*irLF; //LF
	//dRF = irRF*0.01289;//RF
	//dLR = 0.01289*irLR;//LR
	//dRR = 0.01289*irRR;//RR
	//sensor[0] = dLF;// > (lim_m) ? lim_m : dLF;
	//sensor[1] = dRF;// > lim_m ? lim_m : dRF;
	//sensor[2] = dLR;// > lim_m ? lim_m : dLR;
	//sensor[3] = dRR;// > lim_m ? lim_m : dRR;
}



robot & robot::operator=(const robot _robotin)
{
	robot new_robot;
	new_robot.set(_robotin.x, _robotin.y, _robotin.dir);
	new_robot.pcarte = _robotin.pcarte;
	return new_robot;
}

