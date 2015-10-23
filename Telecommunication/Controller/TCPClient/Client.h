#pragma once
#include <WinSock2.h>
#include <WS2tcpip.h>
#include <iostream>
#include <string>

#pragma comment(lib, "Ws2_32.lib")
#define WIN32_LEAN_AND_MEAN
#define DEFAULT_PORT "5000"
#define DEFAULT_BUFFER_LENGTH 512

class Client
{
public:
	Client(char * server);
	bool Start();
	void Stop();
	bool Send(char * smsg);
	bool Recv();
	~Client();
private:
	char * servername;
	SOCKET ConnectSocket;
};

