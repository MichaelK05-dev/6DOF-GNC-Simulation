#pragma once
#include "DataStructures.h"
#include "state_machine.h"
#include "navigation.h"
#include "guidance.h"
#include "control.h"

class GncSystem {
public: 
	GncSystem();

	ActuatorCommands run(SensorData& raw_sensors, double dt);

private:
	StateMachine state_machine;
	Navigation navigation;
	Guidance guidance;
	Control control;
};