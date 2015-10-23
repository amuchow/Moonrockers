#include "Client.h"
#include "CXBOXController.h"
#include <Windows.h>
#include <iostream>
#include <string>

bool connectPrompt();
void disconnect();
bool CheckControllerInput();

Client* client;
CXBOXController* Player1;
std::string command;


int main()
{
	bool flag;
	Player1 = new CXBOXController(1);
	if (!connectPrompt())
	{
		delete (Player1);
		delete (client);
	}

	/*std::cout << "Instructions:\n";
	std::cout << "[A] Vibrate Left Only\n";
	std::cout << "[B] Vibrate Right Only\n";
	std::cout << "[X] Vibrate Both\n";
	std::cout << "[Y] Vibrate Neither\n";
	std::cout << "[BACK] Exit\n";
	*/

	while (true)
	{
		command.clear();
		if (!CheckControllerInput())
		{
			delete (Player1);
			disconnect();
			return -1;
		}
		if (!client->Send((char *)command.c_str()))
		{
			delete (Player1);
			disconnect();
			return -2;
		}
	    
		Sleep(50);
	}	
	delete (Player1);
	disconnect();
	return 0;
} //End Main

bool connectPrompt()
{
	std::string msg;
	std::string addy;

	std::cout << "Robot Address: ";
	std::cin >> addy;

	client = new Client((char*)addy.c_str());
	
	if (!client->Start())
		return false;

	std::cout << "Connected\n";
	return true;
}

void disconnect()
{
    client->Stop();
	delete(client);
	return;
}

bool CheckControllerInput()
{
	if (Player1->IsConnected())
	{
		Player1->Update();

		/*Dump In*/
		if (Player1->State._buttons[GamePad_Button_A] == TRUE)
		{
			command += '1';
		}
		else
			command += '0';
		/*Conveyor Backward*/
		if (Player1->State._buttons[GamePad_Button_B] == TRUE)
		{
			command += '1';
		}
		else
			command += '0';
		/*Conveyor Out*/
		if (Player1->State._buttons[GamePad_Button_X] == TRUE)
		{
			command += '1';
		}
		else
			command += '0';
		/*Dump Out*/
		if (Player1->State._buttons[GamePad_Button_Y] == TRUE)
		{
			command += '1';
		}
		else
			command += '0';
		/*Conveyor In*/
		if (Player1->State._buttons[GamePad_Button_LEFT_SHOULDER] == TRUE)
		{
			command += '1';
		}
		else
			command += '0';
		/*Conveyor Forward*/
		if (Player1->State._buttons[GamePad_Button_RIGHT_SHOULDER] == TRUE)
		{
			command += '1';
		}
		else
			command += '0';
		/*Left Forward*/
		if (Player1->State._left_thumb.y > 0)
		{
			command += '1';
		}
		else
			command += '0';
		/*Left Reverse*/
		if (Player1->State._left_thumb.y < 0)
		{
			command += '1';
		}
		else
			command += '0';
		/*Right Forward*/
		if (Player1->State._right_thumb.y > 0)
		{
			command += '1';
		}
		else
			command += '0';
		/*Right Reverse*/
		if (Player1->State._right_thumb.y < 0)
		{
			command += '1';
		}
		else
			command += '0';
	}
	else
	{
		std::cout << "\n\tERROR! PLAYER 1 - XBOX 360 Controller Not Found!\n";
		std::cout << "Press Any Key To Exit.";
		std::cin.get();
		return false;
	}
	return true;
}