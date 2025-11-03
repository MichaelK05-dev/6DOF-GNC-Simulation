#pragma once
#include "DataStructures.h"

class Navigation {
public: 
	struct StateEstimate {
		double sim_time;
		double pos_x, pos_y, pos_z;
		double roll, pitch, yaw;
		double p, q, r;
		double vel_x, vel_y, vel_z;
		double flight_path_angle;
	};

	Navigation();
	void update(SensorData& raw_sensors);
	StateEstimate getState();
private:
	StateEstimate estimated_state;
};