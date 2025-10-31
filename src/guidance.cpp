#include "guidance.h"

Guidance::Guidance() {
	currentTarget = {};
}

Guidance::AttitudeTarget Guidance::getTarget() {
	return currentTarget;
}

// TO DO: Add different flight configs, add initial roll
void Guidance::update(FlightState current_flight_state, Navigation::StateEstimate& current_state) {
	switch (current_flight_state) {

	case FlightState::PAD_IDLE:
		currentTarget.roll = 0.0;
		currentTarget.pitch = 0.0;
		currentTarget.yaw = 0.0;
		break;
	case FlightState::POWERED_ASCENT:
		currentTarget.roll = 0.0;
		currentTarget.pitch = 0.0;
		currentTarget.yaw = 0.0;
		break;
	case FlightState::COASTING_TO_APOGEE:
		currentTarget.roll = 0.0;
		currentTarget.pitch = 0.0;
		currentTarget.yaw = 0.0;
		break;
	default:
		currentTarget = {};
		break;
	}
}