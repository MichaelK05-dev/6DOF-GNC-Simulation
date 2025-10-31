#include "pid_controller.h"
#define _USE_MATH_DEFINES
#include <cmath>

PIDController::PIDController() : kp(0.0), ki(0.0), kd(0.0), integral(0.0) {}

void PIDController::setGains(double p, double i, double d) {
	kp = p;
	ki = i;
	kd = d;
}

void PIDController::reset() {
	integral = 0.0;
}

double PIDController::calculate(double error, double derivative, double dt) {
	double p_out = kp * error;

	integral += error * dt;
	double i_out = ki * integral;

	double d_out = kd * derivative;

	double output = p_out + i_out + d_out;

	return output;

}