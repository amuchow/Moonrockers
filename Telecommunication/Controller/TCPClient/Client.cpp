#include "Client.h"


Client::Client(char * server)
{
	servername = server;
	ConnectSocket = INVALID_SOCKET;
}
//Initialize Connection
bool Client::Start()
{
	WSADATA wsaData;

	//Init Winsock
	int iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
	if (iResult != 0)
	{
		std::cout << "WSAStartup has failed: " << iResult << "\n";
		return false;
	}
	struct addrinfo * result = NULL,
		            * ptr = NULL,
		              hints;
	
	ZeroMemory(&hints, sizeof(hints));
	hints.ai_family = AF_UNSPEC;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_protocol = IPPROTO_TCP;

	//Resolve Server Address and Port
	iResult = getaddrinfo(servername, DEFAULT_PORT, &hints, &result);
	if (iResult != 0)
	{
		std::cout << "getaddrinfo has failed: " << iResult << "\n";
		WSACleanup();
		return false;
	}
	ptr = result;

	ConnectSocket = socket(ptr->ai_family, ptr->ai_socktype, ptr->ai_protocol);
	if (ConnectSocket == INVALID_SOCKET)
	{
		std::cout << "Error Creating Socket(): " << WSAGetLastError() << "\n";
		freeaddrinfo(result);
		WSACleanup();
		return false;
	}

	//Connect to Server
	iResult = connect(ConnectSocket, ptr->ai_addr, (int)ptr->ai_addrlen);
	if (iResult == SOCKET_ERROR)
	{
		closesocket(ConnectSocket);
		ConnectSocket = INVALID_SOCKET;
	}

	freeaddrinfo(result);

	if (ConnectSocket == INVALID_SOCKET)
	{
		std::cout << "Unable to Connect to Server\n";
		WSACleanup();
		return false;
	}
	return true;
}


//Free Resources
void Client::Stop()
{
	int iResult = shutdown(ConnectSocket, SD_SEND);

	if (iResult == SOCKET_ERROR)
	{
		std::cout << "Shutdown has failed: " << WSAGetLastError() << "\n";
	}

	closesocket(ConnectSocket);
	WSACleanup();
}

bool Client::Send(char * smsg)
{
	int iResult = send(ConnectSocket, smsg, strlen(smsg), 0);

	if (iResult == SOCKET_ERROR)
	{
		std::cout << "Sending Message has failed: " << WSAGetLastError() << "\n";
		Stop();
		return false;
	}
	return true;
}

bool Client::Recv()
{
	char recbuffer[DEFAULT_BUFFER_LENGTH];
	int iResult = recv(ConnectSocket, recbuffer, DEFAULT_BUFFER_LENGTH, 0);

	if (iResult > 0)
	{
		char rmsg[DEFAULT_BUFFER_LENGTH];
		memset(&rmsg, 0, sizeof(rmsg));
		strncpy_s(rmsg, recbuffer, iResult);
		std::cout << "Received: " << rmsg << "\n";
		return true;
	}
	return false;
}

Client::~Client()
{
	Stop();
}
