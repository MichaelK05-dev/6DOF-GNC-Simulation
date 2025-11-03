#include "gnc_system.h"

GncSystem::GncSystem() {
	
}

ActuatorCommands GncSystem::run(SensorData& raw_sensors, double dt) {
	navigation.update(raw_sensors);
	Navigation::StateEstimate current_state = navigation.getState();
	double current_altitude = -current_state.pos_z;
	double current_time = current_state.sim_time;

	const PIDGains aggressive_gains = { 0.8, 0.02, 2.8 }; // Not used for now
	const PIDGains gentle_gains = { 0.8, 0.01, 1.5 };

	state_machine.update(current_state);
	FlightState current_flight_state = state_machine.getCurrentState();

	guidance.update(current_flight_state, current_state);
	Guidance::AttitudeTarget target_attitude = guidance.getTarget();

	if (current_altitude <= 5000) {
		control.setPitchGains(aggressive_gains);
	//	control.setYawGains(aggressive_gains);
	}
	else {
		control.setPitchGains(gentle_gains);
		//control.setYawGains(gentle_gains);
	}

	if (current_flight_state == FlightState::POWERED_ASCENT || current_flight_state == FlightState::COASTING_TO_APOGEE) {
		control.run(target_attitude, current_state, dt);
	}
	else {
		control.reset();
	}

	return control.getCommands();

}