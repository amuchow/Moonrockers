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

/*Global Variables*/
int LeftFor = 8;
int LeftRev = 7;
int RightFor = 13;
int RightRev = 12;
int ConvFor = 2;
int ConvRev = 3;
int ConvIn = 4;
int ConvOut = 5;
int DumpIn = 6;
int DumpOut = 9;
int MineMode = 10;

/*Connection Globals*/
int listenfd = 0, connfd = 0;
int n;
struct sockaddr_in serv_addr;
char sendBuff[1025];
time_t ticks;



void setup()
{
pinMode(LeftFor, OUTPUT);
pinMode(LeftRev, OUTPUT);
pinMode(RightFor, OUTPUT);
pinMode(RightRev, OUTPUT);
pinMode(ConvFor, OUTPUT);
pinMode(ConvRev, OUTPUT);
pinMode(ConvIn, OUTPUT);
pinMode(ConvOut, OUTPUT);
pinMode(DumpIn, OUTPUT);
pinMode(DumpOut, OUTPUT);
pinMode(MineMode, OUTPUT);

    listenfd = socket(AF_INET, SOCK_STREAM, 0);
    memset(&serv_addr, '0', sizeof(serv_addr));
    memset(sendBuff, '0', sizeof(sendBuff));

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    serv_addr.sin_port = htons(5000);

    bind(listenfd, (struct sockaddr*)&serv_addr, sizeof(serv_addr));

    listen(listenfd, 10);

    connfd = accept(listenfd, (struct sockaddr*)NULL, NULL);

}

void loop()
{
     n = read(connfd, sendBuff, strlen(sendBuff) );

     if(n>0)
    {
      if(sendBuff[6]=='1')           zzdigitalWrite(LeftFor, HIGH);
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

    
    
    int  i = 0;
    while(i < 9)
    {
         sendBuff[i] = '0';
         i++;
    }
     
    usleep(50000);
}
  //    close(connfd);
  //   sleep(1);
}
