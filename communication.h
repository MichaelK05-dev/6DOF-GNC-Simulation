#pragma once
#include <WinSock2.h>
#include "DataStructures.h"

class Communication {
public:
	Communication();
	~Communication();

	bool receiveData(SensorData& data);
	void sendCommands(const ActuatorCommands& commands);
private:
	SOCKET sock; // UDP-Socket
	sockaddr_in simulink_address;
};