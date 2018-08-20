#include "KLT_Tracking.h"

KLT_Tracking::KLT_Tracking(Mat _img1)
{
	_img1.copyTo(img1);
}

void KLT_Tracking::track(Mat img2)
{
//	calcOpticalFlowPyrLK(img1, img2, vec_input, vec_input,stats,NULL,Size(9,9),3);
}

void KLT_Tracking::change_points(Point * _arr_ps, int _num_p)
{
	if (arr_ps != NULL) 
		delete [] arr_ps;
		arr_ps = new Point[_num_p];
		arr_ps = _arr_ps;
}
