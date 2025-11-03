#pragma once
#include "DataStructures.h"
#include "pid_controller.h"
#include "guidance.h"
#include "navigation.h"
struct PIDGains {
	double kp, ki, kd;
};

class Control {
public:
	Control();

	void run(Guidance::AttitudeTarget& target, Navigation::StateEstimate& current, double dt); // TO DO: Add parameters
	ActuatorCommands getCommands() const;
	
	void setPitchGains(const PIDGains& gains);
	void setYawGains(const PIDGains& gains);
	void setRollGains(const PIDGains& gains);
	void reset();
private:
	double normalize_angle(double angle_radiants);
	PIDController pitch_pid;
	PIDController yaw_pid;
	PIDController roll_pid;

	ActuatorCommands current_commands;
};