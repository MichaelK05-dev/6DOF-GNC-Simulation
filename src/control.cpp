#define _USE_MATH_DEFINES
#include "control.h"
#include <algorithm>
#include <cmath>
#include <iostream>

Control::Control() {

	pitch_pid.setGains(0.8, 0.01, 1.5);
	yaw_pid.setGains(2, 0.7, 2.7);
	roll_pid.setGains(10.0, 1, 30.0);
	reset();
}


void Control::setPitchGains(const PIDGains& gains) {
	pitch_pid.setGains(gains.kp, gains.ki, gains.kd);
}
void Control::setYawGains(const PIDGains& gains) {
	yaw_pid.setGains(gains.kp, gains.ki, gains.kd);
}
void Control::setRollGains(const PIDGains& gains) {
	roll_pid.setGains(gains.kp, gains.ki, gains.kd);
}


void Control::reset() {
	pitch_pid.reset();
	yaw_pid.reset();
	roll_pid.reset();
	current_commands = {};

}

ActuatorCommands Control::getCommands() const {
	return current_commands;
}

void Control::run(Guidance::AttitudeTarget& target, Navigation::StateEstimate& current, double dt) {

	double pitch_error = normalize_angle(target.pitch - current.pitch);
	double yaw_error = normalize_angle(target.yaw - current.yaw);
	double roll_error = normalize_angle(target.roll - current.roll);
	std::cout << "Current pitch error: " << pitch_error;
	std::cout << "Current yaw error: " << yaw_error;
	double pitch_cmd = pitch_pid.calculate(pitch_error, -current.q, dt, true);
	double yaw_cmd = yaw_pid.calculate(yaw_error, -current.r, dt, false);
	double roll_cmd = roll_pid.calculate(roll_error, -current.p, dt, true);

	const double max_gimbal_angle = 6.0 * (M_PI / 180.0);
	current_commands.gimbal_pitch_cmd = std::clamp(pitch_cmd, -max_gimbal_angle, max_gimbal_angle);
	current_commands.gimbal_yaw_cmd = std::clamp(yaw_cmd, -max_gimbal_angle, max_gimbal_angle);
	std::cout << "current gimbal1: " << current_commands.gimbal_pitch_cmd << "und " << pitch_cmd;
	std::cout << "current gimbal2: " << current_commands.gimbal_yaw_cmd << "und " << yaw_cmd;

	current_commands.roll_thrust_cmd = std::clamp(roll_cmd, -1.0, 1.0);
	if (current.sim_time > burntime) {
		current_commands.engine_status = false;
		std::cout << "MECO";
	}

}

double Control::normalize_angle(double angle_rad) {
	return std::atan2(std::sin(angle_rad), std::cos(angle_rad));
}
