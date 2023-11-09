#ifndef __ESTADO_TELEOPS__
#define __ESTADO_TELEOPS__

using namespace std;

enum EstadoTeleops
{
	TELEOPS_STATE_IDLE,
	TELEOPS_STATE_START,
	TELEOPS_STATE_DO,
	TELEOPS_STATE_END,
	TELEOPS_STATE_DRIVE
};

//scooter from server and local states
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

struct teleoperacion{
	int sensores;
	int video;
	int control;
	int latency;
	string servidor;
	int estatus;//scoter from server
	int id;
};

enum EstadoSocket
{
	SOCKET_STOPPED,
	SOCKET_STARTING,
	SOCKET_STARTED,
	SOCKET_STOPPING
};





#endif
