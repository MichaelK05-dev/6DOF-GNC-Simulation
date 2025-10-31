#define _USE_MATH_DEFINES
#include <iostream>
#include "communication.h"
#include "DataStructures.h"
#include <cmath>

int main()
{
    try {
        Communication comm;
        SensorData sensor_data;
        ActuatorCommands actuator_commands = {};

        while (true) {
            if (comm.receiveData(sensor_data)) {
                std::cout << "Time = " << sensor_data.sim_time << "s\n"
                    << "Altitude = " << -sensor_data.pos_z << "m\n"
                    << "Pitch = " << sensor_data.pitch * 180.0 / M_PI << "deg\n";

                comm.sendCommands(actuator_commands);
            }
        }
    }
    catch (const std::exception& e) {
        std::cerr << "An error occured: " << e.what() << "\n";
        return 1;
    }
    return 0;
}


