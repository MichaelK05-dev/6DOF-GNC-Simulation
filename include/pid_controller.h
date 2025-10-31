#pragma once

class PIDController {
public:
	PIDController();

	void setGains(double kp, double ki, double kd);
	double calculate(double error, double deriviate);

	void reset();
private:
	double kp;
	double ki;
	double kd;

	double integral;
};