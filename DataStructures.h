#pragma once

struct SensorData {
	double sim_time;
	double pos_x, pos_y, pos_z;
	double roll, pitch, yaw;
	double p, q, r;
};

struct ActuatorCommands {
	double gimbal_pitch_cmd;
	double gimbal_yaw_cmd;
	double roll_thrust_cmd;
};
