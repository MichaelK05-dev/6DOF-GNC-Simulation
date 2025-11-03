#include "navigation.h"

Navigation::Navigation() {
	estimated_state = {};
}

Navigation::StateEstimate Navigation::getState() {
	return estimated_state;
}
// just copying all the data for now, TO DO: Add filters and artificial noise to the simulation
void Navigation::update(SensorData& raw_sensors) {
    estimated_state.sim_time = raw_sensors.sim_time;
    estimated_state.pos_x = raw_sensors.pos_x;
    estimated_state.pos_y = raw_sensors.pos_y;
    estimated_state.pos_z = raw_sensors.pos_z;
    estimated_state.roll = raw_sensors.roll;
    estimated_state.pitch = raw_sensors.pitch;
    estimated_state.yaw = raw_sensors.yaw;
    estimated_state.p = raw_sensors.p;
    estimated_state.q = raw_sensors.q;
    estimated_state.r = raw_sensors.r;
    estimated_state.vel_x = raw_sensors.vel_x;
    estimated_state.vel_y = raw_sensors.vel_y;
    estimated_state.vel_z = raw_sensors.vel_z;
    estimated_state.flight_path_angle = raw_sensors.flight_path_angle;

}