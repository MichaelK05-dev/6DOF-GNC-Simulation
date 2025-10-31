#pragma once
class Navigation {
public: 
	struct StateEstimate {
		double sim_time;
		double pos_x, pos_y, pos_z;
		double roll, pitch, yaw;
		double p, q, r;
	};
};