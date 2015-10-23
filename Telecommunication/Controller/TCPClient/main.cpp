#include "Client.h"

using namespace std;

int main()
{
	string msg;
	string addy;
	cout << "Server Address: ";
	cin >> addy;

	Client client((char *)addy.c_str());

	if (!client.Start())
		return 1;

	while (true)
	{
		cout << "Send: ";
		cin >> msg;

		if (msg.compare("Close") == 0)
		{
			break;
		}
		
		client.Send((char *)msg.c_str());
		client.Recv();
	}

	client.Stop();

	return 0;
}