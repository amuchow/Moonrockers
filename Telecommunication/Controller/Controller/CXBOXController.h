#ifndef _XBOX_CONTROLLER_H_
#define _XBOX_CONTROLLER_H_

// No MFC
#define WIN32_LEAN_AND_MEAN

// We need the Windows Header and the XInput Header
#include <windows.h>
#include <XInput.h>

// Now, the XInput Library
// NOTE: COMMENT THIS OUT IF YOU ARE NOT USING A COMPILER THAT SUPPORTS THIS METHOD OF LINKING LIBRARIES
#pragma comment(lib, "XInput9_1_0.lib")

//Deadzone Declarations
#define XINPUT_GAMEPAD_RIGHT_THUMB_DEADZONE 8689
#define XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE  8689
#define XINPUT_GAMEPAD_TRIGGER_THRESHOLD    100
//Button Definitions
typedef enum
{
	GamePad_Button_DPAD_UP = 0,
	GamePad_Button_DPAD_DOWN = 1,
	GamePad_Button_DPAD_LEFT = 2,
	GamePad_Button_DPAD_RIGHT = 3,
	GamePad_Button_START = 4,
	GamePad_Button_BACK = 5,
	GamePad_Button_LEFT_THUMB = 6,
	GamePad_Button_RIGHT_THUMB = 7,
	GamePad_Button_LEFT_SHOULDER = 8,
	GamePad_Button_RIGHT_SHOULDER = 9,
	GamePad_Button_A = 10,
	GamePad_Button_B = 11,
	GamePad_Button_X = 12,
	GamePad_Button_Y = 13,
	GamePad_Button_MAX = 14
}PadButton;

//XY Coordinate Structure
struct XYCoord
{
	float x = 0;
	float y = 0;
};

//State Structure
struct PadState
{
	bool _buttons[GamePad_Button_MAX];
	XYCoord _left_thumb;
	XYCoord _right_thumb;
	float _left_trigger;
	float _right_trigger;

	void reset()
	{
		for (int i = 0; i < GamePad_Button_MAX; i++) _buttons[i] = false;
		_left_thumb.x = 0;
		_left_thumb.y = 0;
		_right_thumb.x = 0;
		_right_thumb.y = 0;
	}
};

// XBOX Controller Class Definition
class CXBOXController
{

public:
	CXBOXController(int playerNumber);
	virtual ~CXBOXController(void);
	XINPUT_STATE GetState();
    bool IsConnected();
	void Vibrate(int leftVal = 0, int rightVal = 0);
	void Update();
public:
	PadState State;
private:
	XINPUT_STATE _controllerState;
	int _controllerNum;
};

#endif
