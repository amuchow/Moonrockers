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

/*Connection Globals*/
int listenfd = 0, connfd = 0;
int n;
struct sockaddr_in serv_addr;
char sendBuff[1025];
time_t ticks;

// Control globals
int recBuff;
int deviceFlag;
int currDevice = 1;
int deviceController = 1;
int deviceAutonomy = 2;

void *connection_handler(void *);
int checkDevice(int buffer);
void checkAuto(int buffer, int & device);

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
}
void loop()
{

    int socket_desc , client_sock , c , *new_sock;
    struct sockaddr_in server , client;

    listenfd = socket(AF_INET, SOCK_STREAM, 0);
    memset(&serv_addr, '0', sizeof(serv_addr));
    memset(sendBuff, '0', sizeof(sendBuff));

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    serv_addr.sin_port = htons(5000);

    bind(listenfd, (struct sockaddr*)&serv_addr, sizeof(serv_addr));

    listen(listenfd, 10);
    while(client_sock=accept(socket_desc,(struct sockaddr*)&client,(socklen_t*)&c))
    {
        puts("Connection accepted");

        pthread_t sniffer_thread;
        new_sock = (int *)malloc(1);
        *new_sock = client_sock;

        if( pthread_create( &sniffer_thread , NULL ,  connection_handler , (void*) new_sock) < 0)
        {
            perror("could not create thread");
        }

        puts("Handler assigned");
    }

    connfd = accept(listenfd, (struct sockaddr*)NULL, NULL);

}

void *connection_handler(void *)
{
    while(1)
    {
        recBuff = 0;
        deviceFlag = 0;

        //Read Socket

        read(connfd, sendBuff, strlen(sendBuff));
        recBuff = atoi(sendBuff);

        //Set Current Device to Receive Instructions From
        checkAuto(recBuff, currDevice);

        //Find Current Device of Command
        deviceFlag = checkDevice(recBuff);

        //If Current Device and Set Device are the Same Parse Command
        if (deviceFlag == currDevice)
        {
             if(sendBuff[6]=='1')           digitalWrite(LeftFor, HIGH);
              else                             digitalWrite(LeftFor, LOW);
              if(sendBuff[7]=='1')           digitalWrite(LeftRev, HIGH);
              else                             digitalWrite(LeftRev, LOW);
              if(sendBuff[8]=='1')           digitalWrite(RightFor, HIGH);
              else                             digitalWrite(RightFor, LOW);
              if(sendBuff[9]=='1')           digitalWrite(RightRev, HIGH);
              else                             digitalWrite(RightRev, LOW);
              if(sendBuff[5]=='1')           digitalWrite(ConvFor, HIGH);
              else                             digitalWrite(ConvFor, LOW);
              if(sendBuff[1]=='1')           digitalWrite(ConvRev, HIGH);
              else                             digitalWrite(ConvRev, LOW);
              if(sendBuff[4]=='1')           digitalWrite(ConvIn, HIGH);
              else                             digitalWrite(ConvIn, LOW);
              if(sendBuff[2]=='1')           digitalWrite(ConvOut, HIGH);
              else                             digitalWrite(ConvOut, LOW);
              if(sendBuff[0]=='1')           digitalWrite(DumpIn, HIGH);
              else                             digitalWrite(DumpIn, LOW);
              if(sendBuff[3]=='1')           digitalWrite(DumpOut, HIGH);
              else                             digitalWrite(DumpOut, LOW);
              if(sendBuff[10] == '1')        digitalWrite(MineMode, HIGH);    
              else if(sendBuff[10] == '2')     digitalWrite(MineMode, LOW);

        }

        int  i = 0;
        while(i < 9)
        {
             sendBuff[i] = '0';
             i++;
        }
        usleep(50000);
    }
}


int checkDevice(int buffer)
{
    if (buffer == deviceController)    return 1;
    else if (buffer == deviceAutonomy)  return 2;
    else                               return 0;
}

void checkAuto(int buffer, int & device)
{
    if (buffer == deviceController)
    {
        if (buffer == deviceController)
        {
            device = 2;
        }
        else if (buffer == deviceAutonomy)
        {
            device = 1;
        }
    }
}
