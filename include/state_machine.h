#pragma once
#include "DataStructures.h"

enum class FlightState {
	PAD_IDLE,
	POWERED_ASCENT,
	COASTING_TO_APOGEE,
	DESCENT,
};

class StateMachine {
public:
	StateMachine();
	void update(const SensorData& sensor_data);

	FlightState getCurrentState() const;

private: 
	FlightState currentState;
};