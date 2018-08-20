
#include <thread>
#include <mutex>

#include "Wifibot.h"
#include "PF.h"
#include "PF_Tracking.hpp"
#include <opencv2\objdetect.hpp>
//using namespace cv;
using namespace std;

void Update_Particle(robot* p_robot,PF* p_PF, long pulseL,long pulseR);
// Thread function
void UpdateData(Wifibot* p_wb, wifibot_data* data, bool* handle);
void UpdateRobot(robot* p_robot,wifibot_data* data, PF* p_PF, bool* handle);
void getkey(bool* handle);
void Move_Pulse(int _left, int _right, long* pulseL, long*pulseR);
void captureImage(cv::VideoCapture* cap,cv::Mat*img, char  tick, bool* handle);
void Keyboard();
// Image processing function
bool traitement_image(cv::Mat image, int* posx, int* posy);
void track_obj(cv::Mat* image,bool* handle);
void control_obj_rouge(bool zone_rouge_detectee, int* posx, int* posy);
void human_detection(cv::Mat image, cv::HOGDescriptor* hog);
// Handle to stop function
bool handleCapture = false;
bool handleControl = false;
bool handleMeasure = false;
bool handleUp = false;
bool handleTracking = false;
bool handleGetkey = false;
//Switch application var
bool img_proc = false;
bool b_local = false;
bool manual_ctr = false;
//Mutex 
wifibot_data shared_data;
std::mutex m_wifibot;
std::mutex m_img;
//Variance of global variables
float move_x, move_y;
cv::Ptr<cv::BackgroundSubtractor> pMOG2;
robot real_robot(1.2, 1.2, PI / 2);
PF* p_pf=NULL;
cv::Rect* obj=NULL;
Particle_Tracking* p_pt=NULL;
Wifibot* p_wifibot;
Point pos2track;
cv::Mat img;
//Callback function
void CallBackFunc(int event, int x, int y, int flags, void* userdata);
int main()

{
	CascadeClassifier cc;
	cc.load("haarcascade_fullbody.xml");
	PF pf_robot;
	p_pf = &pf_robot;
	p_pf->draw_particle();
	Particle_Tracking pt;
	p_pt = &pt;
	//cv::imshow("MAP", *pf_robot.m2->img_map);
	//cv::imshow("PF", pf_robot.test);
	//cvWaitKey(0);


	//real_robot.pcarte = pf_robot.m;
	//while (1)
	//{
	//	pf_robot.update_state(0,0);
	//	pf_robot.draw_particle();
	//	real_robot.draw_on_map(&pf_robot.test, 3, cv::Scalar(255, 0, 0));
	//	real_robot.sense();
	//	float* sens = real_robot.sensor;
	//	printf("IRLF= %0.2f, IRRF= %0.2f \n IRLR= %0.2f, IRRR=%0.2f \n",
	//		real_robot.sensor[0], real_robot.sensor[1], real_robot.sensor[2], real_robot.sensor[3]);
	//	pf_robot.measure(sens);
	//	pf_robot.estimate(pf_robot.reset);
	//	pf_robot.renormalize(pf_robot.reset);
	//	cv::imshow("PF", pf_robot.test);
	//	char d = cvWaitKey(0);
	//	if (d == 'q') break;

	//}
	//return 0;
	p_wifibot = new Wifibot();
	//cv::Mat img;

	
	
	bool b_connect = p_wifibot->Connect(15020, "192.168.1.31");
	//robot real_robot(0, 0, PI);
	real_robot.pcarte = pf_robot.m;
	if (b_connect)
	{ 

		/************************************************************/
		cout << "Connect successfully" << endl;
		cv::VideoCapture cap;// = VideoCapture(0);
		std::thread thread_Capture;
		bool cam_connect = false;
		cam_connect=cap.open("http://192.168.1.31:8080/cameras/1");
		if (cam_connect)
		{
			cout << "Init Thread Capture Image..." << endl;
			thread_Capture = std::thread(captureImage,&cap, &img, 20, &handleCapture);
			
		}
		std::thread thread_getkey(getkey,&handleGetkey); //control robot manuelly
		std::thread thread_update(UpdateData, p_wifibot, &shared_data, &handleControl); //update state of wifibot
		std::thread thread_tracking(track_obj,&img,&handleTracking); // tracking red object
		std::thread thread_ReceiveData(UpdateRobot,&real_robot,&shared_data,&pf_robot,&handleMeasure);
		std::thread thread_controlKeyboard(Keyboard);		
		while (!handleGetkey);
		thread_update.join();
		thread_tracking.join();
		thread_ReceiveData.join();
		thread_getkey.join();
		if (cam_connect) thread_Capture.join();
		thread_controlKeyboard.join();
		//thread_move.join();		
		//thread_DisplayData.join();
		p_wifibot->Disconnect();
	}
	else
	{
		return -1;
	}
	return 0;
}

void Update_Particle(robot* p_robot,PF * p_PF, long pulseL,long pulseR)
{
	p_PF->update_state(pulseL,pulseR);
	p_PF->draw_particle();
	p_PF->measure(p_robot->sensor);
	p_PF->estimate(p_PF->reset);
	p_PF->renormalize(p_PF->reset);
	cv::imshow("PF", p_PF->test);
	char d = cvWaitKey(1);
}

void UpdateData(Wifibot * p_wb, wifibot_data * data, bool * handle)
{
	while (!(*handle))
	{
		m_wifibot.lock();
		p_wb->Update(data);
		m_wifibot.unlock();
		Sleep(5);
		
	}
}

void UpdateRobot(robot* p_robot,wifibot_data * data, PF* p_PF,bool * handle)
{
	char last_move = 0;
	int speed = 30;
	long old_L, old_R, new_L, new_R;
	old_L = data->odometryLeft;
	old_R = data->odometryRight;
	while (!(*handle))
	{

		float* sens;
		new_L = data->odometryLeft; new_R = data->odometryRight;
		p_robot->adc_m(data->irLeft, data->irRight, data->irRight2, data->irLeft2 );
		sens = p_robot->sensor;
		//printf("IRLF= %0.2f, IRRF= %0.2f \n IRLR= %0.2f, IRRR=%0.2f \n",
			//p_robot->sensor[0], p_robot->sensor[1], p_robot->sensor[2], p_robot->sensor[3]);
		
		if(b_local) Update_Particle(p_robot, p_PF, (new_L - old_L), (new_R - old_R));
		old_L = new_L;
		old_R = new_R;
		if (b_local)
		{
			if (sens[0] >= 0.6 && sens[1] >= 0.6)
			{
				shared_data.leftSpeed = speed;
				shared_data.rightSpeed = speed;
				last_move = 0;
			}
			else if (sens[0] < 0.6 && sens[1] >= 0.6)
			{
				if (last_move == 0)
				{
					shared_data.leftSpeed = speed;
					shared_data.rightSpeed = -speed;
					last_move = 1;
				}
				else if (last_move == 2)
				{
					shared_data.leftSpeed = -speed;
					shared_data.rightSpeed = speed;
					last_move = 2;
				}
			}
			else if (sens[0] >= 0.6 && sens[1] < 0.6)
			{
				if (last_move == 0)
				{
					shared_data.leftSpeed = -speed;
					shared_data.rightSpeed = speed;
					last_move = 2;
				}
				else if (last_move == 1)
				{
					shared_data.leftSpeed = speed;
					shared_data.rightSpeed = -speed;
				}
			}
			else if (sens[0] < 0.6 && sens[1] < 0.6)
			{
				if (last_move == 1)
				{
					shared_data.leftSpeed = speed;
					shared_data.rightSpeed = -speed;
				}
				else if (last_move == 2)
				{
					shared_data.leftSpeed = -speed;
					shared_data.rightSpeed = speed;
				}
				else
				{
					shared_data.leftSpeed = speed;
					shared_data.rightSpeed = -speed;
				}
			}
		}

		Sleep(100);
	}
}

void getkey(bool * handle)
{
	long old_L, old_R, new_L, new_R;
	old_L = shared_data.odometryLeft;
	old_R = shared_data.odometryRight;
	static char last_state = 5;
	static int speed = 50;
	while (!(*handle))
	{
		//float* sens;
		new_L = shared_data.odometryLeft; new_R = shared_data.odometryRight;
		//real_robot.adc_m(shared_data.irLeft, shared_data.irRight, shared_data.irRight2, shared_data.irLeft2);
		//sens = real_robot.sensor;
		if (manual_ctr)
		{
			if ((GetAsyncKeyState(VK_UP) & 0x8000))
			{
				m_wifibot.lock();
				shared_data.leftSpeed = speed;
				shared_data.rightSpeed = speed;
				m_wifibot.unlock();
				last_state = 0;
			}
			else if ((GetAsyncKeyState(VK_DOWN) & 0x8000))
			{
				m_wifibot.lock();
				shared_data.leftSpeed = -speed;
				shared_data.rightSpeed = -speed;
				m_wifibot.unlock();
				last_state = 1;
			}
			else if ((GetAsyncKeyState(VK_LEFT) & 0x8000))
			{
				m_wifibot.lock();
				shared_data.leftSpeed = -(speed-20);
				shared_data.rightSpeed = speed-20;
				m_wifibot.unlock();
				last_state = 2;
			}
			else if ((GetAsyncKeyState(VK_RIGHT) & 0x8000))
			{
				m_wifibot.lock();
				shared_data.leftSpeed = (speed-20);
				shared_data.rightSpeed = -(speed-20);
				m_wifibot.unlock();
				last_state = 3;
			}
			else if ((GetAsyncKeyState(VK_RIGHT & VK_UP)) & 0x8000)
			{
				m_wifibot.lock();
				shared_data.leftSpeed = (speed);
				shared_data.rightSpeed = (speed - 20);
				m_wifibot.unlock();
			}
			else
			{
				m_wifibot.lock();
				shared_data.leftSpeed = 0;
				shared_data.rightSpeed = 0;
				m_wifibot.unlock();
				last_state = 4;
			}
		}
		old_L = new_L;
		old_R = new_R;
		Sleep(200);
	}
}

void Move_Pulse(int _left, int _right,long* pulseL,long*pulseR)
{
	long oldL = shared_data.odometryLeft;
	long oldR = shared_data.odometryRight;
	//printf("LeftOdo old : %d \n", oldL);
	//printf("RightOdo old : %d \n", oldR);
	m_wifibot.lock();
	shared_data.leftSpeed = _left>= 0 ? 20:-20; 
	shared_data.rightSpeed =_right>= 0 ? 20:-20;
	m_wifibot.unlock();
	long newL = oldL;
	long newR = oldR;
	do
	{
		newL = shared_data.odometryLeft;
		newR = shared_data.odometryRight;
		//printf("LeftOdo old : %d \n", newL);
		//printf("RightOdo old : %d \n", newR);
	} while ( (abs(newL-oldL) <= abs(_left)) && (abs(newR - oldR) <= abs(_right)));
	m_wifibot.lock();
	shared_data.leftSpeed = 0; shared_data.rightSpeed = 0;
	m_wifibot.unlock();
	*pulseL = newL - oldL;
	*pulseR = newR - oldR;
	//printf("Delta R : %d \n", abs(newR - oldR));
	//printf("Delta L : %d \n", abs(newL - oldL));
}

void captureImage(cv::VideoCapture * cap,cv::Mat* img, char tick, bool * handle)
{
	//cv::HOGDescriptor hog;
	//hog.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());
	//pMOG2 = cv::createBackgroundSubtractorMOG2();
	while (!(*handle))
	{
		m_img.lock();
		*cap >> *img;
		m_img.unlock();
		Mat overlayed_img;
		img->copyTo(overlayed_img);
		float* sens;
		real_robot.adc_m(shared_data.irLeft, shared_data.irRight, shared_data.irRight2, shared_data.irLeft2);
		sens = real_robot.sensor;
		putText(overlayed_img, "TL: " + to_string((int)(sens[0]*100)), Point(10, 10), FONT_HERSHEY_PLAIN, 0.8, Scalar(0, 0, 255), 1);
		putText(overlayed_img, "TR: " + to_string((int)(sens[1] * 100)), Point(250, 10), FONT_HERSHEY_PLAIN, 0.8, Scalar(0, 0, 255), 1);
		putText(overlayed_img, "BL: " + to_string((int)(sens[2] * 100)), Point(10, 230), FONT_HERSHEY_PLAIN, 0.8, Scalar(0, 0, 255), 1);
		putText(overlayed_img, "BR: " + to_string((int)(sens[3] * 100)), Point(250, 230), FONT_HERSHEY_PLAIN, 0.8, Scalar(0, 0, 255), 1);
		cv::resize(overlayed_img, overlayed_img, Size(), 2, 2);
		cv::imshow("Captured", overlayed_img);
		setMouseCallback("Captured", CallBackFunc, (void*)img);
		// Traitement d'image
		//if (img_proc)
		//{

			//human_detection(*img, &hog);
			//track_obj(img, NULL);
			

		//}
		cvWaitKey(tick);
		//Sleep(tick);
	}
	cap->release();
}
//Detect zone rouge
bool traitement_image(cv::Mat image, int* posx, int* posy)
{
	//Mat imgOriginal;
	//Mat rot_mat;
	//Point center = Point( image.cols/2, image.rows/2 );
	//rot_mat = getRotationMatrix2D( center, 180,1 );

	// warpAffine(image, imgOriginal, rot_mat, Size(image.cols, image.rows));

	
	cv::Mat imgHSV;

	cv::cvtColor(image, imgHSV, cv::COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV(Couleur Saturation Intensité)

	cv::Mat imgFiltre, imgFiltre2;
	//éléments de masque fixés pour ces éléments de tableau qui sont dans la zone de délimitation spécifique à l'élément
	inRange(imgHSV, /*Scalar(0, 100, 100)*/cv::Scalar(0, 70, 50), cv::Scalar(10, 255, 255), imgFiltre); //Threshold the image
	inRange(imgHSV, /*Scalar(160, 100, 100)*/cv::Scalar(170, 70, 50), cv::Scalar(180, 255, 255), imgFiltre2);

	cv::Mat red_hue_image;
	addWeighted(imgFiltre, 1.0, imgFiltre2, 1.0, 0.0, red_hue_image);
	//morphological opening (remove small objects from the foreground)
	erode(red_hue_image, red_hue_image, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
	dilate(red_hue_image, red_hue_image, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

	//morphological closing (fill small holes in the foreground)
	dilate(red_hue_image, red_hue_image, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
	erode(red_hue_image, red_hue_image, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

	//Calculate the moments of the thresholded image(calcule moments de la forme pixellisée ou d'un vecteur de points)
	cv::Moments M = moments(red_hue_image);

	double dM01 = M.m01;
	double dM10 = M.m10;
	double dArea = M.m00;

	//calculate the position where and are the components of the centroid
	int posX = dM10 / dArea;
	int posY = dM01 / dArea;
	// get the image data

	//100 pixel=2.646 cm
	// if the area <= 10000, I consider that the there are no object in the image and it's because of the noise, the area is not zero 
	bool result = false;
	if (dArea > 10000)
	{
		printf("Position x=% d y=%d \n", posX, posY);
		*posx = posX;
		*posy = posY;
		cv::circle(image, cv::Point(posX, posY), 5, cv::Scalar(0, 0, 255), -1);
		result = true;
	}
	else
	{
		printf("\nZone rouge NON détectée\n");
		result = false;
	}
	//Mat dst1,dst2;
	cv::imshow("Detect Screen",image );
	cv::imshow("Filtered Image", red_hue_image);
	cvWaitKey(1);
		//   resize(imgOriginal, dst1, Size(640, 480), 0, 0, INTER_CUBIC);
		//  resize(red_hue_image, dst2, Size(640, 480), 0, 0, INTER_CUBIC);
		// cv::imshow( "Image transformée", dst2 ); //show the thresholded image
		//imshow("Image Réel", dst1); //show the original image
	return result;
}
void human_detection(cv::Mat image,cv::HOGDescriptor* hog)
{


	vector<cv::Rect> found, found_filtered;
	hog->detectMultiScale(image, found, 0, cv::Size(8, 8), cv::Size(32, 32), 1.05, 2);
	size_t i, j;
	for (i = 0; i<found.size(); i++)
	{
		cv::Rect r = found[i];
		for (j = 0; j<found.size(); j++)
			if (j != i && (r & found[j]) == r)
				break;
		if (j == found.size())
			found_filtered.push_back(r);
	}

	for (i = 0; i<found_filtered.size(); i++)
	{
		cv::Rect r = found_filtered[i];
		//r.x += cvRound(r.width*0.1);
		//r.width = cvRound(r.width*0.8);
		//r.y += cvRound(r.height*0.07);
		//r.height = cvRound(r.height*0.8);
		rectangle(image, r.tl(), r.br(), cv::Scalar(0, 255, 0), 3);
	}
	cv::imshow("Ppl", image);
}
void track_obj(cv::Mat* image,bool*handle)
{
	//cout << "Start thread tracking" << endl;
	cv::HOGDescriptor hog;
	hog.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());
	Mat t;
	while (!(*handle)) {
		if (img_proc) {
			int posx = 0;
			int posy = 0;
			bool detected = traitement_image(*image, &posx, &posy);
			control_obj_rouge(detected, &posx, &posy);
			//img.copyTo(t);
			//human_detection(t, &hog);
			cvWaitKey(1);
		}
		

	}
}
// Loi de contrôle pour poursuivre l'objet rouge
void control_obj_rouge(bool zone_rouge_detectee, int* posx, int* posy)
{
	int leftIR_forward;
	int rightIR_forward;
	int leftIR_back;
	int rightIR_back;
	int speed = 20;
	static int last_move = 0;
	//m_wifibot.lock();
	//shared_data.leftSpeed = 0;
	//shared_data.rightSpeed = 0;
	//m_wifibot.unlock();
	// Boucle while(1) ou while (objet pas trop proche)
	// 1 Récupération de l'image courante -> *image_courante

	// 2 Traitement de l'image courante
	// Post: zone_rouge_detectee mis à jour
	//       position_croix_x et position_croix_y mis à jour le cas échéant (si zone_rouge_detectee vaut true)
	// 3 Récupération de la valeur des capteurs infra-rouge
	leftIR_forward = shared_data.irLeft;
	//leftIR_back = get_wifibot_irLeft2();
	rightIR_forward = shared_data.irRight;
	//rightIR_back = get_wifibot_irRight2();
	// 4 Calcul de la vitesse des roue droite et gauche et mise à jours de leftSpeed et rightSpeed
	//m_wifibot.lock();
	if (zone_rouge_detectee == false)
	{
		// Le robot tourne sur lui-même
		if (last_move == 0)
		{
			shared_data.leftSpeed = speed;
			shared_data.rightSpeed = -speed;
		}
	}
	else
	{
		if ((*posx - 160) > 20)
		{
			if (leftIR_forward < 100 && rightIR_forward < 100)
			{
				// La cible est sur la droite
				// le robot tourne sur lui-même sur la droite
				printf("Cible sur la droite.\n");

				last_move = 1;
				shared_data.leftSpeed = speed;
				shared_data.rightSpeed = -speed;
			}
			else
			{
				printf("Obstacle détecté!\n");
				last_move = 0;
				shared_data.leftSpeed = 0;
				shared_data.rightSpeed = 0;
			}
		}
		else if ((*posx - 160) < -20)
		{
			if (leftIR_forward < 100 && rightIR_forward < 100)
			{
				// La cible est sur la gauche
				// le robot tourne sur lui-même sur la gauche
				printf("Cible sur la gauche.\n");
				last_move = 2;
				shared_data.leftSpeed = -speed;
				shared_data.rightSpeed = speed;
			}
			else
			{
				printf("Obstacle détecté!\n");
				last_move = 0;
				shared_data.leftSpeed = 0;
				shared_data.rightSpeed = 0;
			}
		}
		else
		{
			if (leftIR_forward < 100 && rightIR_forward < 100)
			{
				printf("Cible au centre, pas d'obstacles devant.\n");
				// Le robot avance tout droit
				shared_data.leftSpeed = speed;
				shared_data.rightSpeed = speed;
			}
			else
			{
				// On est arrivé à l'objectif!
				printf("Cible au centre, présence d'obstacles devant.\n");
				shared_data.leftSpeed = 0;
				shared_data.rightSpeed = 0;
			}
		}
	}
	//m_wifibot.unlock();

}

void CallBackFunc(int event, int x, int y, int flags, void * userdata)
{
	//Mat t = *(Mat*)userdata;
	//
	//static int xx,yy;
	//static char state = 0;
	//if (event == EVENT_LBUTTONUP)
	//{
	//	state = 0;
	//	if (obj != NULL) delete[] obj;
	//	if ((abs(xx - x) > 1) && (abs(yy - y) > 1))
	//	{
	//		obj = new Rect(xx, yy, abs(xx - x), abs(yy - y));
	//		Mat img2 = (t(*obj));
	//		imshow("obj", img2);
	//		int posx = (xx + x) / 2;
	//		int posy = (yy + y) / 2;
	//		p_pt->Init(t, posx, posy, abs(xx - x), abs(yy - y));
	//	}
	//}
	//else if (event == EVENT_LBUTTONDOWN)
	//{
	//	if (state == 0)
	//	{
	//		xx = x;
	//		yy = y;
	//		state = 1;
	//	}

	//}
	//else if (event == EVENT_MOUSEMOVE)
	//{
	//	if (state == 1)
	//	{
	//		rectangle(t, cv::Point(xx, yy), cv::Point(x, y), cv::Scalar(255,0,0), 2);
	//	}

	//}
	//imshow("Event_Image", t);
}


void Keyboard()
{
	printf("Wait for command... \n Press q to quit\n Press m to manual control \n Press s to start tracking red object \n Press l to start localisation \n");
	long pulseL = 0;
	long pulseR = 0;
	while (1)
	{
		char d = getchar();
		if (d == 'q') {
			printf("Terminate ALL THREAD... \n");
			handleControl = true;	
			handleMeasure = true;
			handleUp = true;
			handleCapture = true;
			handleTracking = true;
			handleGetkey = true;
			break;
		}
		else if (d == 'u') // Move forward 10 cm
		{

			Move_Pulse(356, 356,&pulseL,&pulseR);
			Update_Particle(&real_robot, p_pf, pulseL,pulseR);
		}
		else if (d == 'j')
		{
			Move_Pulse(-356, -356, &pulseL, &pulseR);
			Update_Particle(&real_robot, p_pf, pulseL, pulseR);
		}
		else if (d == 'h')
		{
			Move_Pulse(-260, 500, &pulseL, &pulseR);
			cout << pulseL << pulseR << endl;
			Update_Particle(&real_robot, p_pf, pulseL, pulseR);
		}
		else if (d == 'k')
		{
			Move_Pulse(260, -260, &pulseL, &pulseR);
			Update_Particle(&real_robot, p_pf, pulseL, pulseR);
		}
		else if (d == 'p')
		{
			Update_Particle(&real_robot, p_pf, 0, 0);
		}
		else if (d == 's')
		{
			if (img_proc)
			{
				printf("Switch off tracking object... \n ");
			
					img_proc = false;
					Sleep(100);
					shared_data.leftSpeed = 0;
					shared_data.rightSpeed = 0;
			}
			else
			{
				printf("Switch to tracking red object... \n ");
				
				b_local = false;
				//manual_ctr = false;
				Sleep(100);
				shared_data.leftSpeed = 0;
				shared_data.rightSpeed = 0;
				img_proc = true;
			}
		}
		else if (d == 'l')
		{
			if (b_local)
			{
				printf("Switch off localization... \n ");

				b_local = false;

				Sleep(100);
				shared_data.leftSpeed = 0;
				shared_data.rightSpeed = 0;
			}
			else
			{
				printf("Switch on localization.. \n ");
				p_pf->reset_particle();
				b_local = true;
				img_proc = false;
				manual_ctr = false;
			}
		}
		else if (d == 'm')
		{
			if (manual_ctr)
			{
				printf("Switch off manual control... \n ");
				manual_ctr = false;
			}
			else
			{
				printf("Switch on manual control... \n ");
				manual_ctr = true;
				b_local = false;
				//img_proc = false;
			}
		}
	}
	printf("Release thread keyboard.. \n");
}

