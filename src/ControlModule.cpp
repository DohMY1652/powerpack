#include <iostream>
#include <ros/ros.h>

#include "ControlModule.h"

ControlModule::ControlModule(Sensor& sensor, PWM& pwm, ReferenceGoverner& referencegoverner, bool is_positive, int sensor_idx, std::vector<int> pwm_idx,  std::vector<double> gains) :
    sensor(sensor),
    pwm(pwm),
    referencegoverner(referencegoverner),
    is_positive(is_positive),
    sensor_idx(sensor_idx),
    pwm_idx(pwm_idx) {
        control.resize(pwm_idx.size());
        controller = new MPCController(is_positive, gains);
        
}

ControlModule::~ControlModule() {
}

void ControlModule::get_channel_info() const {
    std::cout << "========================" << std::endl;
    if (is_positive) {
        std::cout << "channel type : positive" << std::endl;    
    }
    else {
        std::cout << "channel type : negative" << std::endl;    
    }
    std::cout << "sensor index : " << sensor_idx << std::endl;
    std::cout << "pwm index : ";
    for (const int& value : pwm_idx) {
        std::cout << value;
    }
    std::cout << std::endl;
}

bool ControlModule::get_channel_type() const {
    return is_positive;
}

void ControlModule::calculate_control_signal(double now, double P_micro, double P_macro, double target) {
    controller->set_now_state(now, P_micro, P_macro);
    controller->set_now_target_trajectory(target);
    controller->calculate_control();
    control = controller->get_control_signal();
}

std::vector<unsigned int> ControlModule::get_control_signal() const {
    return control;
}


std::string ControlModule::get_controller_type() const {
    return controller->get_controller_type();
}