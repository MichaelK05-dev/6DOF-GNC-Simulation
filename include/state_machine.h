#pragma once
#include "DataStructures.h"
#include "Navigation.h"

enum class FlightState {
	PAD_IDLE,
	POWERED_ASCENT,
	COASTING_TO_APOGEE,
	DESCENT,
};

class StateMachine {
public:
	StateMachine();
	void update(const Navigation::StateEstimate& estimated_state);

	FlightState getCurrentState() const;

private: 
	FlightState currentState;
};