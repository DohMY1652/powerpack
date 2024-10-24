// PWM.cpp
#include <iostream>

#include "PWM.h"

PWM::PWM(std::shared_ptr<DatabaseConfig> &databaseconfig)
 :  databaseconfig(databaseconfig),
    n_channel(databaseconfig->get_n_pid_channel() 
            + 3 * (databaseconfig->get_n_pos_channel()
            + databaseconfig->get_n_neg_channel())) {

        std::vector<double> parameters = databaseconfig->get_sensor_parameters();
        frequency = (int)parameters[0];
        pid_pos_index = (int)parameters[1];
        pid_neg_index = (int)parameters[2];
        data.resize(n_channel);
}

PWM::~PWM() {
}

// void PWM::set_data(unsigned int value, int PWM_idx) {
//     if (PWM_idx >= 0 && PWM_idx < n_channel) {
//         data[PWM_idx] = value;
//     }
// }

// void PWM::update(std::vector<unsigned int> control_signal) {
//     int idx = 0;
//     for (const double& value : control_signal) {
//         data[idx] = value;
//         idx++;
//     }
// }

// void PWM::print_all_data() {
//     std::cout << "PWM control signal" << std::endl;
//     for (const double& value : data) {
//         std::cout << value << " ";
//     }
//     std::cout << std::endl;
// }

// int PWM::get_n_channel() {
//     return n_channel;
// }

// std::vector<unsigned int> PWM::get_control_signal() {
//     return data;
// }
