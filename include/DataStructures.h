#pragma once

struct SensorData {
	double sim_time;
	double pos_x, pos_y, pos_z;
	double roll, pitch, yaw;
	double p, q, r;
	double vel_x, vel_y, vel_z;
	double flight_path_angle;
};

struct ActuatorCommands {
	double gimbal_pitch_cmd;
	double gimbal_yaw_cmd;
	double roll_thrust_cmd;
	double engine_status; // on/off
};

extern double burntime;
extern bool engine_status;
extern double complete_time;