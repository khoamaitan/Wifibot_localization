// Wifibot.h

#ifndef _WIFIBOT_H
#define _WIFIBOT_H

//#define WIFIBOT_TRACE_ON

#define WIFIBOT_ERR_NONE true
#define WIFIBOT_NO_VALUE -9999

#include <stdio.h>
#include <winsock.h>
#pragma comment(lib,"ws2_32.lib")

typedef struct wifibot_data_t
{
	int batVoltage;
	int speedFrontLeft;
	int speedRearLeft;
	int speedFrontRight;
	int speedRearRight;
	int irLeft;
	int	irRight;
	int irLeft2;
	int	irRight2;
	long odometryLeft;
	long odometryRight;
	int leftSpeed;
	int rightSpeed;
} wifibot_data;

class Wifibot
{
private:
	wifibot_data data;
	void ResetWifibot_data();
public:
	Wifibot();
	~Wifibot();
	bool Connect(int portNo, char* ipAddress);
	void Disconnect( );
	void SetWifibotSpeed(int leftSpeed, int rightSpeed);
	void GetWifibotData( wifibot_data *new_data);
	bool Update(wifibot_data *new_data );
	bool SendData(wifibot_data *new_data);
	bool ReceiveData(wifibot_data* recv_data);
};

#endif