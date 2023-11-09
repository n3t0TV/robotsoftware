#ifndef __ESTADO_TELEOPS__
#define __ESTADO_TELEOPS__

using namespace std;

enum EstadoScooter
{
	INACTIVE = 0,
	COMMAND = 1,
	DRIVE = 2,
	PARKED = 3,
	VENDING = 4,
	TELEOP = 5,
	JOYSTICK = 6,
	NOT_REGISTERED=7
};

#endif
