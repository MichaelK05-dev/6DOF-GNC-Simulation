#include "state_machine.h"
#include  <iostream>

StateMachine::StateMachine() {
	currentState = FlightState::PAD_IDLE;
}

FlightState StateMachine::getCurrentState() const {
	return currentState;
}

void StateMachine::update(const Navigation::StateEstimate& estimated_state) {
	switch (currentState) {
	case FlightState::PAD_IDLE:
		if (estimated_state.vel_z < 0) { // -z = up
			currentState = FlightState::POWERED_ASCENT;
		    std::cout << "PAD_IDLE -> POWERED_ASCENT\n";
		}
		std::cout << "velocity: " << estimated_state.vel_z;
		break;
	case FlightState::POWERED_ASCENT:
		if (engine_status == false) { 
			currentState = FlightState::COASTING_TO_APOGEE;
			std::cout << "POWERED_ASCENT -> COASTING_TO_APOGEE";
		}
		break;
	case FlightState::COASTING_TO_APOGEE:
		// TO DO: Check Vz to detect descent
		break;
	case FlightState::DESCENT:
		break;
	}
}