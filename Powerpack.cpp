#include <iostream>

#include "Powerpack.h"


Powerpack::Powerpack() {

    YAML::Node config = YAML::LoadFile("config.yaml");
    n_pos_channel = config["positive_channel"]["n_channel"].as<int>();
    n_neg_channel = config["negative_channel"]["n_channel"].as<int>();

    sensor = new Sensor(n_pos_channel+n_neg_channel);
    pwm = new PWM((3*(n_pos_channel+n_neg_channel)));
    referencegoverner = new ReferenceGoverner((n_pos_channel+n_neg_channel), 0);

    for (int i = 0; i < n_pos_channel; i++) {
        ControlModule tmp_module(*sensor, *pwm, *referencegoverner, positive,i, {3*i, 3*i+1, 3*i+2});
        modules.push_back(tmp_module);
    }
    for (int i = n_pos_channel; i < n_pos_channel+n_neg_channel; i++) {
        ControlModule tmp_module(*sensor, *pwm, *referencegoverner, negative, i, {3*i, 3*i+1, 3*i+2});
        modules.push_back(tmp_module);
    }

    std::cout << "Powerpack generated" << std::endl;
    print_powerpack_info();

}

Powerpack::~Powerpack() {

}

void Powerpack::get_pos_powerpack_info() {
    for (ControlModule& tmp_module : modules) {
        if (tmp_module.get_channel_type() == true) {
            tmp_module.get_channel_info();
        }
    }
}

void Powerpack::get_neg_powerpack_info() {
    for (ControlModule& tmp_module : modules) {
        if (tmp_module.get_channel_type() == false) {
            tmp_module.get_channel_info();
        }
    }
}

void Powerpack::print_powerpack_info() {
    std::cout << "====================" << std::endl;
    std::cout << "channel info" << std::endl;
    std::cout << "number of positive channel : " << n_pos_channel << std::endl;
    std::cout << "number of negative channel : " << n_neg_channel << std::endl;
    std::cout << "--------------------" << std::endl;
    std::cout << "controller type" << std::endl;
    for (ControlModule& tmp_module : modules) {
        std::cout << tmp_module.get_controller_type() << ", ";
    }
    std::cout << std::endl;
    std::cout << "--------------------" << std::endl;
    std::cout << "Sensor info" << std::endl;
    std::cout << "number of sensor channel : " << sensor->get_n_channel() << std::endl;
    std::cout << "--------------------" << std::endl;
    std::cout << "PWM info" << std::endl;
    std::cout << "number of pwm channel : " << pwm->get_n_channel() << std::endl;
    
    std::cout << "====================" << std::endl;

}

void Powerpack::update_sensor() {
    sensor->update();
}

void Powerpack::update_reference() {
    referencegoverner->update();
}

void Powerpack::update_pwm(){
    set_all_control_signal();
}

void Powerpack::calculate_control_signal() {
    for (int i = 0; i<(n_pos_channel+n_neg_channel); i++) {
        modules[i].calculate_control_signal(sensor->get_data(i), referencegoverner->get_data(i));
    }
    
}

void Powerpack::set_all_control_signal() {
    std::vector<double> control_signal;
     for (int i = 0; i<((n_pos_channel+n_neg_channel)); i++) {
        for (const double& value : modules[i].get_control_signal()) {
            control_signal.push_back(value);
        }
    }
    pwm->update(control_signal);
}


void Powerpack::run() {
    update_sensor();
    update_reference();
    calculate_control_signal();
    update_pwm();
    

    // get_all_powerpack_info();
    sensor->print_all_data();
    referencegoverner->print_all_data();
    pwm->print_all_data();
}

