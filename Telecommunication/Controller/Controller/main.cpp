#include "CXBOXController.h"
#include <iostream>



CXBOXController* Player1;
int main(int argc, char* argv[])
{
	Player1 = new CXBOXController(1);

	std::cout << "Instructions:\n";
	std::cout << "[A] Vibrate Left Only\n";
	std::cout << "[B] Vibrate Right Only\n";
	std::cout << "[X] Vibrate Both\n";
	std::cout << "[Y] Vibrate Neither\n";
	std::cout << "[BACK] Exit\n";

	while (true)
	{
		if (Player1->IsConnected())
		{
			Player1->Update();
			/*if (Player1->GetState().Gamepad.wButtons & XINPUT_GAMEPAD_A)
			{
			Player1->Vibrate(65535, 0);
			}

			if (Player1->GetState().Gamepad.wButtons & XINPUT_GAMEPAD_B)
			{
			Player1->Vibrate(0, 65535);
			}

			if (Player1->GetState().Gamepad.wButtons & XINPUT_GAMEPAD_X)
			{
			Player1->Vibrate(65535, 65535);
			}

			if (Player1->GetState().Gamepad.wButtons & XINPUT_GAMEPAD_Y)
			{
			Player1->Vibrate();
			}

			if (Player1->GetState().Gamepad.wButtons & XINPUT_GAMEPAD_BACK)
			{
			break;
			}
			}*/
			if (Player1->State._buttons[GamePad_Button_A] == TRUE)
			{
				Player1->Vibrate(65535, 0);
			}
			if (Player1->State._buttons[GamePad_Button_B] == TRUE)
			{
				Player1->Vibrate(0, 65535);
			}
			if (Player1->State._buttons[GamePad_Button_X] == TRUE)
			{
				Player1->Vibrate(65535, 65535);
			}
			if (Player1->State._buttons[GamePad_Button_Y] == TRUE)
			{
				Player1->Vibrate(0, 0);
			}
			if (Player1->State._buttons[GamePad_Button_BACK] == TRUE)
			{
				break;
			}
			if (Player1->State._buttons[GamePad_Button_LEFT_SHOULDER] == TRUE)
			{
				Player1->Vibrate(65535, 0);
			}
			if (Player1->State._buttons[GamePad_Button_RIGHT_SHOULDER] == TRUE)
			{
				Player1->Vibrate(0, 65535);
			}
			if (Player1->State._left_trigger > 0.2f)
			{
				Player1->Vibrate(65535, 0);
				std::cout << "Left: " << Player1->State._left_trigger << "\n";
			}
			if (Player1->State._right_trigger > 0.2f)
			{
				Player1->Vibrate(0, 65535);
				std::cout << "Right: " << Player1->State._right_trigger << "\n";
			}
			if (Player1->State._left_thumb.y > 0)
			{
				Player1->Vibrate(65535, 0);
				std::cout << "Left: " << Player1->State._left_thumb.y << "\n";
			}
			if (Player1->State._right_thumb.y > 0)
			{
				Player1->Vibrate(0, 65535);
				std::cout << "Right: " << Player1->State._right_thumb.y << "\n";
			}
		}
		else
		{
			std::cout << "\n\tERROR! PLAYER 1 - XBOX 360 Controller Not Found!\n";
			std::cout << "Press Any Key To Exit.";
			std::cin.get();
			break;
		}
	}

	delete(Player1);
	return(0);
}
