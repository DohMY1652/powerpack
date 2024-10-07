// PWM.cpp
#include "PWM.h"
#include <iostream>

PWM::PWM(int n_channel) : n_channel(n_channel) {
    data.resize(n_channel);
    // std::cout << "PWM generated" << std::endl;
}

PWM::~PWM() {
}

void PWM::set_data(unsigned int value, int PWM_idx) {
    if (PWM_idx >= 0 && PWM_idx < n_channel) {
        data[PWM_idx] = value;
    }
}

void PWM::update(std::vector<double> control_signal) {
    int idx = 0;
    for (const double& value : control_signal) {
        data[idx] = value;
        idx++;
    }
}

void PWM::print_all_data() {
    std::cout << "PWM control signal" << std::endl;
    for (const double& value : data) {
        std::cout << value << " ";
    }
    std::cout << std::endl;
}

int PWM::get_n_channel() {
    return n_channel;
}

std::vector<unsigned int> PWM::get_control_signal() {
    return data;
}