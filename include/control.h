#pragma once
#include "DataStructures.h"
#include "pid_controller.h"
#include "guidance.h"
#include "navigation.h"

class Control {
public:
	Control();

	void run(); // TO DO: Add parameters
	ActuatorCommands getCommands() const;
	void reset();
private:
	double normalize_angle(double angle_radiants);
	PIDController pitch_pid;
	PIDController yaw_pid;
	PIDController roll_pid;

	ActuatorCommands current_commands;
};