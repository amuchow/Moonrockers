/*


*/

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <time.h>
#include <core.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <semaphore.h>

using namespace std;

/*Hex Bit Checking Define*/
#DEFINE DEVICE_ID_CONTROLLER   0x0001
#DEFINE DEVICE_ID_MOONBRAIN    0x0002
#DEFINE MINING_MODE            0x0004
#DEFINE DUMP_IN                0x0008
#DEFINE CONV_REVERSE           0x0010
#DEFINE CONV_OUT               0x0020
#DEFINE DUMP_OUT               0x0040
#DEFINE CONV_IN                0x0080
#DEFINE CONV_FORWARD           0x0100
#DEFINE LEFT_FORWARD           0x0200
#DEFINE LEFT_REVERSE           0x0400
#DEFINE RIGHT_FORWARD          0x0800
#DEFINE RIGHT_REVERSE          0x1000
#DEFINE START_AUTO             0x2000
#DEFINE END_AUTO               0x4000

/*Output Pin Constants*/
const int LeftFor  =  8;
const int LeftRev  =  7;
const int RightFor = 13;
const int RightRev = 12;
const int ConvFor  =  2;
const int ConvRev  =  3;
const int ConvIn   =  4;
const int ConvOut  =  5;
const int DumpIn   =  6;
const int DumpOut  =  9;
const int MineMode = 10;

/************************************************************************************************/

/*Connection Globals*/
int listenfd = 0, connfd = 0;
int n;
struct sockaddr_in serv_addr;
char sendBuff[1025];
time_t ticks;

/************************************************************************************************/

void setup()
{
	pinMode(LeftFor,  OUTPUT);
	pinMode(LeftRev,  OUTPUT);
	pinMode(RightFor, OUTPUT);
	pinMode(RightRev, OUTPUT);
	pinMode(ConvFor,  OUTPUT);
	pinMode(ConvRev,  OUTPUT);
	pinMode(ConvIn,   OUTPUT);
	pinMode(ConvOut,  OUTPUT);
	pinMode(DumpIn,   OUTPUT);
	pinMode(DumpOut,  OUTPUT);
	pinMode(MineMode, OUTPUT);

	int recBuff;
	int deviceFlag;
	int currDevice = 1;

/************************************************************************************************/

	listenfd = socket(AF_INET, SOCK_STREAM, 0);
	memset(&serv_addr, '0', sizeof(serv_addr));
	memset(sendBuff, '0', sizeof(sendBuff));

	serv_addr.sin_family = AF_INET;
	serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	serv_addr.sin_port = htons(5000);

	bind(listenfd, (struct sockaddr*)&serv_addr, sizeof(serv_addr));

	listen(listenfd, 10);

	connfd = accept(listenfd, (struct sockaddr*)NULL, NULL);

/************************************************************************************************/

}

void loop()
{
	recBuff = 0;
	deviceFlag = 0;

	//Read Socket

/************************************************************************************************/

	read(connfd, sendBuff, strlen(sendBuff));
	recBuff = atoi(sendBuff);

/************************************************************************************************/

	//Set Current Device to Receive Instructions From
	checkAuto(recBuff, currDevice);

	//Find Current Device of Command
	deviceFlag = checkDevice(recBuff);

	//If Current Device and Set Device are the Same Parse Command
	if (deviceFlag == currDevice)
	{
		parseHex(recBuff);
	}
	usleep(50000);
}

void parseHex(int buffer)
{
	/*Mining Mode Might Need Own Function For Toggle Functionality*/
	if (buffer & MINING_MODE)   digitalWrite(MineMode, HIGH);
	else                        digitalWrite(MineMode,  LOW);
	 
	if (buffer & DUMP_IN)       digitalWrite(DumpIn,   HIGH);
	else                        digitalWrite(DumpIn,    LOW);

	if (buffer & CONV_REVERSE)  digitalWrite(ConvRev,  HIGH);
	else                        digitalWrite(ConvRev,   LOW);

	if (buffer & CONV_OUT)      digitalWrite(ConvOut,  HIGH);
	else                        digitalWrite(ConvOut,   LOW);

	if (buffer & DUMP_OUT)      digitalWrite(DumpOut,  HIGH);
	else                        digitalWrite(DumpOut,   LOW);

	if (buffer & CONV_IN)       digitalWrite(ConvIn,   HIGH);
	else                        digitalWrite(ConvIn,    LOW);

	if (buffer & CONV_FORWARD)  digitalWrite(ConvFor,  HIGH);
	else                        digitalWrite(ConvFor,   LOW);

	if (buffer & LEFT_FORWARD)  digitalWrite(LeftFor,  HIGH);
	else                        digitalWrite(LeftFor,   LOW);

	if (buffer & LEFT_REVERSE)  digitalWrite(LeftRev,  HIGH);
	else                        digitalWrite(LeftRev,   LOW);

	if (buffer & RIGHT_FORWARD) digitalWrite(RightFor, HIGH);
	else                        digitalWrite(RightFor,  LOW);

	if (buffer & RIGHT_REVERSE) digitalWrite(RightRev, HIGH);
	else                        digitalWrite(RightRev,  LOW);
}

int checkDevice(int buffer)
{
	if (buffer & DEVICE_ID_CONTROLLER)     return 1;
	else if (buffer & DEVICE_ID_MOONBRAIN) return 2;
	else                                   return 0;
}

void checkAuto(int buffer, int & device)
{
	if (buffer & DEVICE_ID_CONTROLLER)
	{
		if (buffer & START_AUTO)
		{
			device = 2;
		}
		else if (buffer & END_AUTO)
		{
			device = 1;
		}
	}
}

