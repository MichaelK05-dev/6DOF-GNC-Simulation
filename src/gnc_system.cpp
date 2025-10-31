#include "gnc_system.h"

GncSystem::GncSystem() {

}

ActuatorCommands GncSystem::run(SensorData& raw_sensors, double dt) {
	navigation.update(raw_sensors);
	Navigation::StateEstimate current_state = navigation.getState();

	state_machine.update(current_state);
	FlightState current_flight_state = state_machine.getCurrentState();

	guidance.update(current_flight_state, current_state);
	Guidance::AttitudeTarget target_attitude = guidance.getTarget();

	if (current_flight_state == FlightState::POWERED_ASCENT) {
		control.run(target_attitude, current_state, dt);
	}
	else {
		control.reset();
	}

	return control.getCommands();

}