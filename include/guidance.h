#pragma once
#include "navigation.h"
#include "state_machine.h"

class Guidance {
public:
	struct AttitudeTarget {
		double roll = 0.0;
		double pitch = 0.0;
		double yaw = 0.0;
	};

	Guidance();
	void update(FlightState current_flight_state, Navigation::StateEstimate& current_state);
	AttitudeTarget getTarget();
private: 
	AttitudeTarget currentTarget;
};