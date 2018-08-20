// Il s'agit du fichier DLL principal.

#include "Wifibot.h"

//#define USE_PTHREAD_MUTEX
#ifdef USE_PTHREAD_MUTEX
#include <pthread.h>
#endif

#define BUF_LENGTH 50

static SOCKET wifibotSocket; //Socket handle
static BYTE sbuf[BUF_LENGTH];
static BYTE rcvbuf[BUF_LENGTH];
static int current_left_speed;
static int current_right_speed;

#ifdef USE_PTHREAD_MUTEX
static pthread_mutex_t mutexWifibotData;
#endif

short Crc16(unsigned char *Adresse_tab , unsigned char Taille_max)
{
	unsigned int Crc = 0xFFFF;
	unsigned int Polynome = 0xA001;
	unsigned int CptOctet = 0;
	unsigned int CptBit = 0;
	unsigned int Parity= 0;

	Crc = 0xFFFF;
	Polynome = 0xA001;
	for ( CptOctet= 0 ; CptOctet < Taille_max ; CptOctet++)
	{
		Crc ^= *( Adresse_tab + CptOctet);

		for ( CptBit = 0; CptBit <= 7 ; CptBit++)
		{
			Parity= Crc;
			Crc >>= 1;
			if (Parity%2 == true) Crc ^= Polynome;
		}
	}
	return(Crc);
}

//--------------------------------------------------

void Wifibot::ResetWifibot_data()
{
	this->data.batVoltage = WIFIBOT_NO_VALUE;
	this->data.irLeft = WIFIBOT_NO_VALUE;
	this->data.irLeft2 = WIFIBOT_NO_VALUE;
	this->data.irRight = WIFIBOT_NO_VALUE;
	this->data.irRight2 = WIFIBOT_NO_VALUE;
	this->data.odometryLeft = WIFIBOT_NO_VALUE;
	this->data.odometryRight = WIFIBOT_NO_VALUE;
	this->data.speedFrontLeft = WIFIBOT_NO_VALUE;
	this->data.speedFrontRight = WIFIBOT_NO_VALUE;
	this->data.speedRearLeft = WIFIBOT_NO_VALUE;
	this->data.speedRearRight = WIFIBOT_NO_VALUE;
	this->data.leftSpeed = 0;
	this->data.rightSpeed = 0;
	current_left_speed = 0;
	current_right_speed = 0;
}

//--------------------------------------------------

Wifibot::Wifibot( )
{
	ResetWifibot_data();
#ifdef USE_PTHREAD_MUTEX
	printf("Initializing mutex\n");
	pthread_mutex_init (&mutexWifibotData, NULL);
	printf("Done\n");
#endif
}

//--------------------------------------------------

Wifibot::~Wifibot( )
{

}

//--------------------------------------------------

bool Wifibot::Connect(int portNo, char* ipAddress)
{
	    //Start up Winsock…
    WSADATA wsadata;

    int error = WSAStartup(0x0202, &wsadata);

    //Did something happen?
    if (error)
	{
#ifdef WIFIBOT_TRACE_ON
		printf("!!! Cannot startup socket Wifibot::Connect()\n");
#endif
        return false;
	}

    //Did we get the right Winsock version?
    if (wsadata.wVersion != 0x0202)
    {
#ifdef WIFIBOT_TRACE_ON
		printf("!!! Wring version Wifibot::Connect(), verson = %d <> 0x0202\n", wsadata.wVersion);
#endif
        WSACleanup(); //Clean up Winsock
        return false;
    }

    //Fill out the information needed to initialize a socket…
    SOCKADDR_IN target; //Socket address information

    target.sin_family = AF_INET; // address family Internet
    target.sin_port = htons (portNo); //Port to connect on
    target.sin_addr.s_addr = inet_addr (ipAddress); //Target IP

    wifibotSocket = socket (AF_INET, SOCK_STREAM, IPPROTO_TCP); //Create socket
    if (wifibotSocket == INVALID_SOCKET)
    {
#ifdef WIFIBOT_TRACE_ON
		printf("!!! Cannot create socket Wifibot::Connect(), ipAddress=%s, portNo=d\n", ipAddress, portNo);
#endif
        return false; //Couldn't create the socket
    }  

    //Try connecting...

    if (connect(wifibotSocket, (SOCKADDR *)&target, sizeof(target)) == SOCKET_ERROR)
    {
#ifdef WIFIBOT_TRACE_ON
		printf("!!! Cannot create socket Wifibot::Connect(), ipAddress=%s, portNo=d\n", ipAddress, portNo);
#endif
        return false; //Couldn't connect
    }

#ifdef WIFIBOT_TRACE_ON
		printf("Connection to Wifibot Ok!, ipAddress=%s, portNo=%d\n", ipAddress, portNo);
#endif

	return WIFIBOT_ERR_NONE;
}

//--------------------------------------------------

void Wifibot::Disconnect( )
{
//Close the socket if it exists
    if (wifibotSocket)
        closesocket(wifibotSocket);

    WSACleanup(); //Clean up Winsock
}

//--------------------------------------------------

void Wifibot::SetWifibotSpeed(int leftSpeed, int rightSpeed)
{
#ifdef USE_PTHREAD_MUTEX
	pthread_mutex_lock (&mutexWifibotData);
#endif
	this->data.leftSpeed = leftSpeed;
	this->data.rightSpeed = rightSpeed;
#ifdef USE_PTHREAD_MUTEX
	pthread_mutex_unlock (&mutexWifibotData);
#endif
}

//--------------------------------------------------

void Wifibot::GetWifibotData( wifibot_data *new_data)
{
#ifdef USE_PTHREAD_MUTEX
	pthread_mutex_lock (&mutexWifibotData);
#endif
	*new_data = this->data;
#ifdef USE_PTHREAD_MUTEX
	pthread_mutex_unlock (&mutexWifibotData);
#endif
}

//--------------------------------------------------

bool Wifibot::Update(wifibot_data *new_data )
{
#ifdef WIFIBOT_TRACE_ON
	printf("Wifibot::Update( ), leftSpeed: %d, rightSpeed: %d\n", new_data->leftSpeed, new_data->rightSpeed);
#endif

	// On commence d'abord par envoyer une trame au Wifibot, mise à jour dans sbuf
	/*
	Char 1 is 255
	Char2 is size (here is 0x07)
	Char 3-4 is the left speed 0 -> 240 tics max
	Char 5-6 is the right speed 0 -> 240 tics max
	Char 7 is the Left / Right speed command flag : Forward / Backward and speed control left & right ON/ OFF.
	Char 7 is decomposed as follow (1 byte char -> 8 bits):
		(128) Bit 7 Left Side Closed Loop Speed control :: 1 -> ON / 0 -> OFF
		(64) Bit 6 Left Side Forward / Backward speed flag :: 1 -> Forward / 0 -> Reverse
		(32) Bit 5 Right Side Closed Loop Speed control :: 1 -> ON / 0 -> OFF
		(16) Bit 4 Right Side Forward / Backward speed flag :: 1 -> Forward / 0 -> Reverse
		(8) Bit 3 Relay 4 On/Off (DSUB15 POWER Pin 13 and 14 )
		(4) Bit 2 Relay 3 On/Off (DSUB15 POWER Pin 11 and 12 )
		(2) Bit 1 Relay 2 On/Off (DSUB15 POWER Pin 4 and 5)
		(1) Bit 0 Relay 1 for Sensors. On/Off: 0 is OFF 1 is ON (DSUB15 POWER Pin 3)
	*/
		if (new_data->leftSpeed != current_left_speed)
		{
#ifdef WIFIBOT_TRACE_ON
			printf("Left speed has change! LeftSpeed = %d\n", new_data->leftSpeed);
#endif
			current_left_speed = new_data->leftSpeed;
		}
		if (new_data->rightSpeed != current_right_speed)
		{
#ifdef WIFIBOT_TRACE_ON
			printf("Right speed has change! RightSpeed = %d\n", new_data->rightSpeed);
#endif
			current_right_speed = new_data->rightSpeed;
		}
		sbuf[0] = 255;  // Obligatoire
		sbuf[1] = 0x07;  // taille de sbuf en octets
		sbuf[2] = (BYTE)abs(current_left_speed); // Vitesse des roues à gauche (0 à 240 tics)
		sbuf[3] = 0x00;
		sbuf[4] = (BYTE)abs(current_right_speed); // Vitesse des roues à droite (0 à 240 tics)
		sbuf[5] = 0;
		sbuf[6] = 0;
		if (current_left_speed > 0)
			sbuf[6]+=64;
		if (current_right_speed > 0)
			sbuf[6] += 16; // 0-> les 2 paires de roues vont en arrière, 80-> les 2 paires de roues vont en avant (voir le fichier Ethernet Wifi Protocol.PDF)
		sbuf[6] += 128 + 32;
		short mycrcsend = Crc16((unsigned char*)sbuf+1,6);

		sbuf[7] = (BYTE)mycrcsend;
		sbuf[8] = (BYTE)(mycrcsend >> 8);

		send(wifibotSocket,(char*)sbuf,9,0); // On envoie la trame de 9 octets au Wifibot
	
		Sleep(10);
		int rcvnbr = recv(wifibotSocket,(char *)rcvbuf,21,0); // On attend la réponse du Wifibot et on récupère les données dans rcvbuf (voir le fichier Ethernet Wifi Protocol.PDF)
		short mycrcrcv = (short)((rcvbuf[20] << 8) + rcvbuf[19]);
		mycrcsend = Crc16(rcvbuf,19);
		if (mycrcrcv == mycrcsend)
		{
#ifdef WIFIBOT_TRACE_ON
			printf("Checksum Ok\n");
#endif
			BYTE buffso_send[17];
			buffso_send[0]=rcvbuf[2];//GetADC(hUSB,0x48); not speed but batery level
			
			int myspeedL=(int)((rcvbuf[1] << 8) + rcvbuf[0]);
			if (myspeedL > 32767) myspeedL=myspeedL-65536;
			myspeedL=myspeedL/5;
			buffso_send[1]=myspeedL;
			
			buffso_send[2]=rcvbuf[17];
			
			int myspeedR=(int)((rcvbuf[10] << 8) + rcvbuf[9]);
			if (myspeedR > 32767) myspeedR=myspeedR-65536;
			myspeedR=myspeedR/5;
			buffso_send[3]=(char)myspeedR;

			long odoL = ((((long)rcvbuf[8] << 24))+(((long)rcvbuf[7] << 16))+(((long)rcvbuf[6] << 8))+((long)rcvbuf[5]));
			long odoR = ((((long)rcvbuf[16] << 24))+(((long)rcvbuf[15] << 16))+(((long)rcvbuf[14] << 8))+((long)rcvbuf[13]));
			
					
			buffso_send[4]=(char)(rcvbuf[17]);
			buffso_send[5]=(char)rcvbuf[3];
			buffso_send[6]=rcvbuf[11];
			buffso_send[7]=(unsigned char)odoL;
			buffso_send[8]=(unsigned char)(odoL >> 8);
			buffso_send[9]=(unsigned char)(odoL >> 16);
			buffso_send[10]=(unsigned char)(odoL >> 24);
			buffso_send[11]=(unsigned char)odoR;
			buffso_send[12]=(odoR >> 8);
			buffso_send[13]=(odoR >> 16);
			buffso_send[14]=(odoR >> 24);
			buffso_send[15]=rcvbuf[4];
			buffso_send[16]=rcvbuf[12];
#ifdef USE_PTHREAD_MUTEX
			pthread_mutex_lock (&mutexWifibotData);
#endif
			this->data.batVoltage=buffso_send[0];
			this->data.speedFrontLeft=(int)buffso_send[1];
			this->data.speedRearLeft=0;
			this->data.speedFrontRight=(int)buffso_send[3];
			this->data.speedRearRight=0;
			this->data.irLeft=(int)buffso_send[5];
			this->data.irRight=(int)buffso_send[6];
			this->data.irLeft2=(int)buffso_send[15];
			this->data.irRight2=(int)buffso_send[16];
			this->data.odometryLeft = *(long *)(buffso_send+7);
			this->data.odometryRight = *(long *)(buffso_send+11);
			this->data.leftSpeed = new_data->leftSpeed;
			this->data.rightSpeed = new_data->rightSpeed;
#ifdef USE_PTHREAD_MUTEX
			pthread_mutex_unlock (&mutexWifibotData);
#endif
			GetWifibotData( new_data);
		}
		else
		{
			new_data = NULL;
			printf("Bad checksum. Nothing done o((\n"); // Mauvaise transmission
			return false;
		}

		Sleep(10);

		return WIFIBOT_ERR_NONE;
}

bool Wifibot::SendData(wifibot_data * new_data)
{
	if (new_data->leftSpeed != current_left_speed)
	{
#ifdef WIFIBOT_TRACE_ON
		printf("Left speed has change! LeftSpeed = %d\n", new_data->leftSpeed);
#endif
		current_left_speed = new_data->leftSpeed;
	}
	if (new_data->rightSpeed != current_right_speed)
	{
#ifdef WIFIBOT_TRACE_ON
		printf("Right speed has change! RightSpeed = %d\n", new_data->rightSpeed);
#endif
		current_right_speed = new_data->rightSpeed;
	}
	sbuf[0] = 255;  // Obligatoire
	sbuf[1] = 0x07;  // taille de sbuf en octets
	sbuf[2] = (BYTE)abs(current_left_speed); // Vitesse des roues à gauche (0 à 240 tics)
	sbuf[3] = 0x00;
	sbuf[4] = (BYTE)abs(current_right_speed); // Vitesse des roues à droite (0 à 240 tics)
	sbuf[5] = 0;
	sbuf[6] = 0;
	if (current_left_speed > 0)
		sbuf[6] += 64;
	if (current_right_speed > 0)
		sbuf[6] += 16; // 0-> les 2 paires de roues vont en arrière, 80-> les 2 paires de roues vont en avant (voir le fichier Ethernet Wifi Protocol.PDF)
	sbuf[6] += 128 + 32;
	short mycrcsend = Crc16((unsigned char*)sbuf + 1, 6);

	sbuf[7] = (BYTE)mycrcsend;
	sbuf[8] = (BYTE)(mycrcsend >> 8);

	send(wifibotSocket, (char*)sbuf, 9, 0); // On envoie la trame de 9 octets au Wifibot

	Sleep(10);
	return WIFIBOT_ERR_NONE;
}

bool Wifibot::ReceiveData(wifibot_data * recv_data)
{
	int rcvnbr = recv(wifibotSocket, (char *)rcvbuf, 21, 0); // On attend la réponse du Wifibot et on récupère les données dans rcvbuf (voir le fichier Ethernet Wifi Protocol.PDF)
	short mycrcrcv = (short)((rcvbuf[20] << 8) + rcvbuf[19]);
	short mycrcsend = Crc16(rcvbuf, 19);
	if (mycrcrcv == mycrcsend)
	{
#ifdef WIFIBOT_TRACE_ON
		printf("Checksum Ok\n");
#endif
		BYTE buffso_send[17];
		buffso_send[0] = rcvbuf[2];//GetADC(hUSB,0x48); not speed but batery level

		int myspeedL = (int)((rcvbuf[1] << 8) + rcvbuf[0]);
		if (myspeedL > 32767) myspeedL = myspeedL - 65536;
		myspeedL = myspeedL ;
		buffso_send[1] = myspeedL;

		buffso_send[2] = rcvbuf[17];

		int myspeedR = (int)((rcvbuf[10] << 8) + rcvbuf[9]);
		if (myspeedR > 32767) myspeedR = myspeedR - 65536;
		myspeedR = myspeedR;
		buffso_send[3] = (char)myspeedR;

		long odoL = ((((long)rcvbuf[8] << 24)) + (((long)rcvbuf[7] << 16)) + (((long)rcvbuf[6] << 8)) + ((long)rcvbuf[5]));
		long odoR = ((((long)rcvbuf[16] << 24)) + (((long)rcvbuf[15] << 16)) + (((long)rcvbuf[14] << 8)) + ((long)rcvbuf[13]));


		buffso_send[4] = (char)(rcvbuf[17]);
		buffso_send[5] = (char)rcvbuf[3];
		buffso_send[6] = rcvbuf[11];
		buffso_send[7] = (unsigned char)odoL;
		buffso_send[8] = (unsigned char)(odoL >> 8);
		buffso_send[9] = (unsigned char)(odoL >> 16);
		buffso_send[10] = (unsigned char)(odoL >> 24);
		buffso_send[11] = (unsigned char)odoR;
		buffso_send[12] = (odoR >> 8);
		buffso_send[13] = (odoR >> 16);
		buffso_send[14] = (odoR >> 24);
		buffso_send[15] = rcvbuf[4];
		buffso_send[16] = rcvbuf[12];
#ifdef USE_PTHREAD_MUTEX
		pthread_mutex_lock(&mutexWifibotData);
#endif
		this->data.batVoltage = buffso_send[0];
		this->data.speedFrontLeft = (int)buffso_send[1];
		this->data.speedRearLeft = 0;
		this->data.speedFrontRight = (int)buffso_send[3];
		this->data.speedRearRight = 0;
		this->data.irLeft = (int)buffso_send[5];
		this->data.irRight = (int)buffso_send[6];
		this->data.irLeft2 = (int)buffso_send[15];
		this->data.irRight2 = (int)buffso_send[16];
		this->data.odometryLeft = *(long *)(buffso_send + 7);
		this->data.odometryRight = *(long *)(buffso_send + 11);
		this->data.leftSpeed = this->data.speedFrontLeft;
		this->data.rightSpeed = this->data.speedFrontRight;;
#ifdef USE_PTHREAD_MUTEX
		pthread_mutex_unlock(&mutexWifibotData);
#endif
		GetWifibotData(recv_data);
	}
	else
	{
		recv_data = NULL;
		printf("Bad checksum. Nothing done o((\n"); // Mauvaise transmission
		return false;
	}
	return WIFIBOT_ERR_NONE;
}
