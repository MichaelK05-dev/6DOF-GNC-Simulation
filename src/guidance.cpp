#define _USE_MATH_DEFINES
#include "guidance.h"
#include <algorithm>
#include <cmath>
#include <iostream>
double complete_time = 0;
Guidance::Guidance() {
	currentTarget = {};
}
Guidance::AttitudeTarget Guidance::getTarget() {
	return currentTarget;
}

// TO DO: Add different flight configs
// Current config: Polar orbit, no initial roll required
void Guidance::update(FlightState current_flight_state, Navigation::StateEstimate& current_state) {
	
	switch (current_flight_state) {

	case FlightState::PAD_IDLE:
		
		currentTarget.roll = 0.0;
		currentTarget.pitch = M_PI/2.0; // 90 deg
		currentTarget.yaw = 0.0;
		break;

	case FlightState::POWERED_ASCENT: { 
		if (-current_state.vel_z < 50) {
			currentTarget.pitch = M_PI / 2.0;
		}

		else if (!pitch_over_complete) {
			const double pitch_over_target = 88.0 * (M_PI / 180.0);
			currentTarget.pitch = pitch_over_target;
			std::cout << "Pitch target: " << currentTarget.pitch * (180.0 / M_PI);

			if (current_state.pitch <= pitch_over_target) { 
				pitch_over_complete = true;
				complete_time = current_state.sim_time;
				std::cout << "Pitch over COMPLETE after " << complete_time << "s. Switching to flight path tracking.\n";
				
			}
		}

		else {
			currentTarget.pitch = std::min(current_state.flight_path_angle, 88.0 * (M_PI / 180.0)); // gravity turn
		}

		

		break;
	}
		
		
		
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