#define _USE_MATH_DEFINES
#include <iostream>
#include "communication.h"
#include "gnc_system.h"
#include <cmath>

double burntime = 160;
bool engine_status = true;

int main()
{
    try {
        Communication comm;
        GncSystem gnc;
        
        double last_time = 0.0;

        while (true) {
            SensorData sensor_data;
            if (comm.receiveData(sensor_data)) {
                double dt = sensor_data.sim_time - last_time;
                last_time = sensor_data.sim_time;
                ActuatorCommands actuator_commands = gnc.run(sensor_data, dt);
                comm.sendCommands(actuator_commands);

               
                std::cout << "Time = " << sensor_data.sim_time << "s\n"
                    << "Altitude = " << -sensor_data.pos_z << "m\n"
                    << "Pitch = " << sensor_data.pitch * 180.0 / M_PI << "deg\n";

                
            }
        }
    }
    catch (const std::exception& e) {
        std::cerr << "An error occured: " << e.what() << "\n";
        return 1;
    }
    return 0;
}


