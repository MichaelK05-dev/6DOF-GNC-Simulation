#include "state_machine.h"
#include  <iostream>

StateMachine::StateMachine() {
	currentState = FlightState::PAD_IDLE;
}

FlightState StateMachine::getCurrentState() const {
	return currentState;
}

void StateMachine::update(const SensorData& sensor_data) {
	switch (currentState) {
	case FlightState::PAD_IDLE:
		if (sensor_data.sim_time > 2.0) {
			currentState = FlightState::POWERED_ASCENT; // TO DO: Switch to velocity based state change
		    std::cout << "PAD_IDLE -> POWERED_ASCENT\n";
		}
		break;
	case FlightState::POWERED_ASCENT:
		if (sensor_data.sim_time > 100.0) { // TO DO: Detect engine shutdown
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