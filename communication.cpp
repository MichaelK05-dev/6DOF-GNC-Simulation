#include "communication.h"
#include <iostream>
#include <WS2tcpip.h>

Communication::Communication() {
	WSADATA wsaData;
	if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
		throw std::runtime_error("WSAStartup failed");
	}

	sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (sock == INVALID_SOCKET) {
		WSACleanup();
		throw std::runtime_error("Socket creation failed");
	}

	//input = port 25000
	sockaddr_in local_address;
	local_address.sin_family = AF_INET;
	local_address.sin_port = htons(25000);
	local_address.sin_addr.s_addr = INADDR_ANY;

	if (bind(sock, (sockaddr*)&local_address, sizeof(local_address)) == SOCKET_ERROR) {
		closesocket(sock);
		WSACleanup();
		throw std::runtime_error("Bind failed on port 25000");
	}

	simulink_address.sin_family = AF_INET;
	simulink_address.sin_port = htons(25001);
	inet_pton(AF_INET, "127.0.0.1", &simulink_address.sin_addr);

	std::cout << "Communications ran\n";
}

Communication::~Communication() {
	closesocket(sock);
	WSACleanup();
	std::cout << "Communications closed\n";
}

bool Communication::receiveData(SensorData& data) {
	int bytes_received = recvfrom(sock, (char*)&data, sizeof(SensorData), 0, NULL, NULL);
	if (bytes_received == sizeof(SensorData)) {
		return true;
	}
	return false;
}

void Communication::sendCommands(const ActuatorCommands& commands) {
	sendto(sock, (char*)&commands, sizeof(ActuatorCommands), 0, (sockaddr*)&simulink_address, sizeof(simulink_address));
}