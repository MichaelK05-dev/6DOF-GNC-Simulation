#define _USE_MATH_DEFINES
#include "control.h"
#include <algorithm>
#include <cmath>

Control::Control() {
	// Not tuned yet, just placeholder values

	pitch_pid.setGains(0.5, 0.1, 0.2);
	yaw_pid.setGains(0.5, 0.1, 0.2);

	roll_pid.setGains(1.0, 0.2, 0.3);

	reset();
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

	double pitch_cmd = pitch_pid.calculate(pitch_error, -current.q, dt);
	double yaw_cmd = yaw_pid.calculate(yaw_error, -current.r, dt);
	double roll_cmd = roll_pid.calculate(roll_error, -current.p, dt);

	const double max_gimbal_angle = 6.0 * (M_PI / 180.0);
	current_commands.gimbal_pitch_cmd = std::clamp(pitch_cmd, -max_gimbal_angle, max_gimbal_angle);
	current_commands.gimbal_yaw_cmd = std::clamp(yaw_cmd, -max_gimbal_angle, max_gimbal_angle);

	current_commands.roll_thrust_cmd == std::clamp(roll_cmd, -1.0, 1.0);

}

double Control::normalize_angle(double angle_rad) {
	return std::atan2(std::sin(angle_rad), std::cos(angle_rad));
}
